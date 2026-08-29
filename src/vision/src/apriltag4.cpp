#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/global_definitions.hpp"
#include "ros2_utils/help_logger.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

extern "C"
{
#include "apriltag/apriltag.h"
#include "apriltag/apriltag_pose.h"
#include "apriltag/common/getopt.h"
#include "apriltag/tag36h11.h"
}

using namespace std;
using namespace cv;

typedef struct
{
    int id;
    float pose_R[3];
    float pose_t[3];
} apriltag_iris_t;

class AprilTag4 : public rclcpp::Node
{
  private:
    bool tf_is_initialized = false;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    geometry_msgs::msg::TransformStamped tf_camera_base;
    HelpLogger logger;

    rclcpp::CallbackGroup::SharedPtr sub_callback_group_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_frame;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_array;
    rclcpp::TimerBase::SharedPtr frame_timer_;

    cv::Mat image_bgr;
    cv::Mat image_gray;
    cv::VideoCapture cap;

    apriltag_family_t *tf = NULL;
    apriltag_detector_t *td = NULL;

    sensor_msgs::msg::CameraInfo::SharedPtr camera_info;

    float param_tag_size = 0.88;
    float param_quad_decimate = 2.0;
    float param_blur = 0.0;
    int param_nthreads = 1;
    bool param_debug = false;
    bool param_refine_edges = true;

  public:
    AprilTag4() : Node("AprilTag4"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }

        this->declare_parameter<float>("apriltag_size", 0.88);
        this->get_parameter("apriltag_size", param_tag_size);
        this->declare_parameter<float>("apriltag_quad_decimate", 2.0);
        this->get_parameter("apriltag_quad_decimate", param_quad_decimate);
        this->declare_parameter<float>("apriltag_blur", 0.0);
        this->get_parameter("apriltag_blur", param_blur);
        this->declare_parameter<int>("apriltag_nthreads", 1);
        this->get_parameter("apriltag_nthreads", param_nthreads);
        this->declare_parameter<bool>("apriltag_debug", false);
        this->get_parameter("apriltag_debug", param_debug);
        this->declare_parameter<bool>("apriltag_refine_edges", true);
        this->get_parameter("apriltag_refine_edges", param_refine_edges);

        tf = tag36h11_create();
        td = apriltag_detector_create();
        apriltag_detector_add_family(td, tf);
        td->quad_decimate = param_quad_decimate;
        td->quad_sigma = param_blur;
        td->nthreads = param_nthreads;
        td->debug = param_debug;
        td->refine_edges = param_refine_edges;

        sub_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        pub_debug_frame = this->create_publisher<sensor_msgs::msg::Image>("/sign/image", 1);
        pub_marker_array = this->create_publisher<visualization_msgs::msg::MarkerArray>("/apriltag/markers", 1);

        while (!tf_is_initialized)
        {
            rclcpp::sleep_for(std::chrono::seconds(1));
            try
            {
                tf_camera_base = tf_buffer_.lookupTransform("base_link", "camera_color_optical_frame", tf2::TimePointZero);
                tf_is_initialized = true;
            }
            catch (const tf2::TransformException &ex)
            {
                logger.warn("TF not ready: %s", ex.what());
                rclcpp::sleep_for(std::chrono::milliseconds(100));
            }
        }

        camera_info = std::make_shared<sensor_msgs::msg::CameraInfo>();
        camera_info->k = {600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0};

        std::string camera_path = "/dev/v4l/by-id/usb-e-con_systems_See3CAM_CU55_14205401-video-index0";
        cap.open(camera_path);
        if (!cap.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open webcam at %s", camera_path.c_str());
            rclcpp::shutdown();
        }

        frame_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&AprilTag4::process_camera_frame, this));
    }

    void process_camera_frame()
    {
        if (!tf_is_initialized)
        {
            logger.warn("TF not initialized, skipping frame processing");
            return;
        }

        cap >> image_bgr;
        if (image_bgr.empty())
        {
            logger.warn("Captured empty frame");
            return;
        }

        cv::cvtColor(image_bgr, image_gray, cv::COLOR_BGR2GRAY);
        image_u8_t im = {image_gray.cols, image_gray.rows, image_gray.cols, image_gray.data};
        zarray_t *detections = apriltag_detector_detect(td, &im);

        float minimal_distance = 1000.0f;
        apriltag_iris_t tag_filtered;
        tag_filtered.id = -1;

        for (int i = 0; i < zarray_size(detections); i++)
        {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

            line(image_bgr, Point(det->p[0][0], det->p[0][1]), Point(det->p[1][0], det->p[1][1]), Scalar(0, 0xff, 0), 2);
            line(image_bgr, Point(det->p[1][0], det->p[1][1]), Point(det->p[2][0], det->p[2][1]), Scalar(0xff, 0, 0), 2);
            line(image_bgr, Point(det->p[2][0], det->p[2][1]), Point(det->p[3][0], det->p[3][1]), Scalar(0xff, 0, 0), 2);
            line(image_bgr, Point(det->p[3][0], det->p[3][1]), Point(det->p[0][0], det->p[0][1]), Scalar(0, 0, 0xff), 2);

            apriltag_detection_info_t info;
            info.det = det;
            info.tagsize = param_tag_size;
            info.fx = camera_info->k[0];
            info.fy = camera_info->k[4];
            info.cx = camera_info->k[2];
            info.cy = camera_info->k[5];

            apriltag_pose_t pose;
            double err = estimate_tag_pose(&info, &pose);

            float distance = sqrt(pose.t->data[0] * pose.t->data[0] + pose.t->data[1] * pose.t->data[1] + pose.t->data[2] * pose.t->data[2]);
            if (distance < minimal_distance)
            {
                tag_filtered.id = det->id;
                tag_filtered.pose_R[0] = pose.R->data[0];
                tag_filtered.pose_R[1] = pose.R->data[1];
                tag_filtered.pose_R[2] = pose.R->data[2];
                tag_filtered.pose_t[0] = pose.t->data[0] * 0.1;
                tag_filtered.pose_t[1] = pose.t->data[1] * 0.1;
                tag_filtered.pose_t[2] = pose.t->data[2] * 0.1;
                minimal_distance = distance;
            }
        }

        apriltag_detections_destroy(detections);

        std::string text = "ID: " + std::to_string(tag_filtered.id);
        cv::putText(image_bgr, text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);

        sensor_msgs::msg::Image debug_frame_msg;
        cv_bridge::CvImage debug_frame_cv;
        debug_frame_cv.header.stamp = this->now();
        debug_frame_cv.header.frame_id = "camera_color_optical_frame";
        debug_frame_cv.encoding = sensor_msgs::image_encodings::BGR8;
        debug_frame_cv.image = image_bgr;
        debug_frame_msg = *debug_frame_cv.toImageMsg();
        pub_debug_frame->publish(debug_frame_msg);

        visualization_msgs::msg::MarkerArray marker_array_msg;

        if (tag_filtered.id != -1)
        {
            try
            {
                geometry_msgs::msg::Point point_camera;
                point_camera.x = tag_filtered.pose_t[0];
                point_camera.y = tag_filtered.pose_t[1];
                point_camera.z = tag_filtered.pose_t[2];

                geometry_msgs::msg::Point point_base;
                tf2::doTransform(point_camera, point_base, tf_camera_base);

                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "base_link";
                marker.header.stamp = this->now();
                marker.ns = "apriltag_markers";
                marker.id = tag_filtered.id;
                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose.position.x = point_base.x;
                marker.pose.position.y = point_base.y;
                marker.pose.position.z = point_base.z;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.scale.z = 0.1;
                marker.color.r = 1.0;
                marker.color.a = 1.0;
                marker.lifetime = rclcpp::Duration::from_seconds(0.5);
                marker_array_msg.markers.push_back(marker);
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_ERROR(this->get_logger(), "TF lookup failed: %s", ex.what());
            }
        }

        pub_marker_array->publish(marker_array_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto apriltag_node = std::make_shared<AprilTag4>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(apriltag_node);
    executor.spin();
    return 0;
}
