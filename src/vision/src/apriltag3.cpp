#include <iomanip>
#include <iostream>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "opencv2/opencv.hpp"

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

extern "C" {
#include "apriltag/apriltag.h"
#include "apriltag/apriltag_pose.h"
#include "apriltag/common/getopt.h"
#include "apriltag/tag16h5.h"
#include "apriltag/tag25h9.h"
#include "apriltag/tag36h11.h"
#include "apriltag/tagCircle21h7.h"
#include "apriltag/tagCircle49h12.h"
#include "apriltag/tagCustom48h12.h"
#include "apriltag/tagStandard41h12.h"
#include "apriltag/tagStandard52h13.h"
}

using namespace std;
using namespace cv;

typedef struct {
    int id; // Tag ID
    float pose_R[3]; // Rotation matrix for pose estimation
    float pose_t[3]; // Translation vector for pose estimation
} apriltag_iris_t;

class AprilTag3 : public rclcpp::Node {
private:
    // -------------------------------------------------
    // Transform
    // -------------------------------------------------
    bool tf_is_initialized = false;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    geometry_msgs::msg::TransformStamped tf_camera_base;
    // -------------------------------------------------
    // Logging
    // -------------------------------------------------
    HelpLogger logger;
    // -------------------------------------------------
    // Subscribers Callback Groups
    // -------------------------------------------------
    rclcpp::CallbackGroup::SharedPtr sub_callback_group_;
    // -------------------------------------------------
    // Timer Callback Group
    // -------------------------------------------------

    // -------------------------------------------------
    // Subscribers
    // -------------------------------------------------
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_bgr_rs;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_image_bgr_info;
    // -------------------------------------------------
    // Publishers
    // -------------------------------------------------
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_frame;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_array;
    // -------------------------------------------------
    // Timer
    // -------------------------------------------------

    // -------------------------------------------------
    // Global Variables
    // -------------------------------------------------
    cv::Mat image_bgr;
    cv::Mat image_gray;

    apriltag_family_t* tf = NULL;
    apriltag_detector_t* td = NULL;

    sensor_msgs::msg::CameraInfo::SharedPtr camera_info;
    // -------------------------------------------------
    // Parameters
    // -------------------------------------------------
    float param_tag_size = 0.88;
    float param_quad_decimate = 2.0;
    float param_blur = 0.0;
    int param_nthreads = 1;
    bool param_debug = false;
    bool param_refine_edges = true;

public:
    AprilTag3()
        : Node("AprilTag3")
        , tf_buffer_(this->get_clock())
        , tf_listener_(tf_buffer_)
    {
        if (!logger.init()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }

        // --------------------------------------------------
        // Get parameters
        // --------------------------------------------------
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

        //?==================================================
        //?                   APRILTAG
        //?==================================================
        tf = tag36h11_create();
        td = apriltag_detector_create();
        apriltag_detector_add_family(td, tf);

        td->quad_decimate = param_quad_decimate;
        td->quad_sigma = param_blur;
        td->nthreads = param_nthreads;
        td->debug = param_debug;
        td->refine_edges = param_refine_edges;
        //?==================================================

        // --------------------------------------------------
        // Create subscribers callback groups
        // --------------------------------------------------
        sub_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        // --------------------------------------------------
        // Create timer callback groups
        // @param MutuallyExclusive: Ensures that only one callback from this group can run at a time
        // @param Reentrant: Allows multiple callbacks from this group to run concurrently
        // --------------------------------------------------

        // --------------------------------------------------
        // Create subscriber options and assign callback groups
        // --------------------------------------------------
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = sub_callback_group_;
        // --------------------------------------------------
        // Create subscribers
        // --------------------------------------------------
        sub_image_bgr_rs = this->create_subscription<sensor_msgs::msg::Image>(
            "/vision/color_image", 1,
            std::bind(&AprilTag3::callback_sub_image_bgr_rs, this, std::placeholders::_1), sub_options);

        sub_image_bgr_info = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/vision/camera_info", 1,
            std::bind(&AprilTag3::callback_sub_image_bgr_info, this, std::placeholders::_1), sub_options);
        // --------------------------------------------------
        // Create publishers
        // --------------------------------------------------
        pub_debug_frame = this->create_publisher<sensor_msgs::msg::Image>(
            "/apriltag/image/debug", 1);
        pub_marker_array = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/apriltag/markers", 1);

        while (!tf_is_initialized) {
            rclcpp::sleep_for(std::chrono::seconds(1));
            try {
                tf_camera_base = tf_buffer_.lookupTransform("base_link", "camera_color_optical_frame", tf2::TimePointZero);
                tf_is_initialized = true;
            } catch (const tf2::TransformException& ex) {
                logger.warn("TF not ready: %s", ex.what());
                rclcpp::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }

    void callback_sub_image_bgr_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        // logger.info("Received camera info with size: %dx%d", msg->width, msg->height);
        camera_info = msg;
    }

    void callback_sub_image_bgr_rs(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // logger.info("Received image with size: %dx%d", msg->width, msg->height);
        if (!tf_is_initialized) {
            logger.warn("TF not initialized, skipping point cloud processing");
            return;
        }

        try {
            auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            image_bgr = cv_ptr->image.clone();
            cv::cvtColor(image_bgr, image_gray, cv::COLOR_BGR2GRAY);

            image_u8_t im = { image_gray.cols, image_gray.rows, image_gray.cols, image_gray.data };
            zarray_t* detections = apriltag_detector_detect(td, &im);

            if (errno == EAGAIN) {
                printf("Unable to create the %d threads requested.\n", td->nthreads);
                return;
            }

            std::vector<Point3d> tag_points;
            float minimal_distance = 1000.0f;

            apriltag_iris_t tag_filtered;
            tag_filtered.id = -1; // Initialize with an invalid ID
            // Modify the detection loop to include pose estimation
            for (int i = 0; i < zarray_size(detections); i++) {
                apriltag_detection_t* det;
                zarray_get(detections, i, &det);

                // Draw detection outlines (existing code)
                line(image_bgr, Point(det->p[0][0], det->p[0][1]),
                    Point(det->p[1][0], det->p[1][1]),
                    Scalar(0, 0xff, 0), 2);
                line(image_bgr, Point(det->p[0][0], det->p[0][1]),
                    Point(det->p[3][0], det->p[3][1]),
                    Scalar(0, 0, 0xff), 2);
                line(image_bgr, Point(det->p[1][0], det->p[1][1]),
                    Point(det->p[2][0], det->p[2][1]),
                    Scalar(0xff, 0, 0), 2);
                line(image_bgr, Point(det->p[2][0], det->p[2][1]),
                    Point(det->p[3][0], det->p[3][1]),
                    Scalar(0xff, 0, 0), 2);

                // Pose estimation
                apriltag_detection_info_t info;
                info.det = det;
                info.tagsize = param_tag_size;

                info.fx = camera_info->k[0]; // Focal length x
                info.fy = camera_info->k[4]; // Focal length y
                info.cx = camera_info->k[2]; // Principal point x
                info.cy = camera_info->k[5]; // Principal point y

                apriltag_pose_t pose;
                double err = estimate_tag_pose(&info, &pose);

                // Print pose information
                std::string pose_info = "Pose: ";
                pose_info += "x: " + std::to_string(pose.t->data[0] * 0.1) + ", ";
                pose_info += "y: " + std::to_string(pose.t->data[1] * 0.1) + ", ";
                pose_info += "z: " + std::to_string(pose.t->data[2] * 0.1) + ", ";
                pose_info += "roll: " + std::to_string(pose.R->data[0]) + ", ";
                pose_info += "pitch: " + std::to_string(pose.R->data[1]) + ", ";
                pose_info += "yaw: " + std::to_string(pose.R->data[2]) + ", ";
                pose_info += "error: " + std::to_string(err);
                cv::putText(image_bgr, pose_info, Point(10, 30 + i * 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);

                float distance = sqrt(pose.t->data[0] * pose.t->data[0] + pose.t->data[1] * pose.t->data[1] + pose.t->data[2] * pose.t->data[2]);
                if (distance < minimal_distance) {
                    tag_filtered.id = det->id;
                    tag_filtered.pose_R[0] = pose.R->data[0];
                    tag_filtered.pose_R[1] = pose.R->data[1];
                    tag_filtered.pose_R[2] = pose.R->data[2];
                    tag_filtered.pose_t[0] = pose.t->data[0] * 0.1; // Convert to meters
                    tag_filtered.pose_t[1] = pose.t->data[1] * 0.1; // Convert to meters
                    tag_filtered.pose_t[2] = pose.t->data[2] * 0.1; // Convert to meters
                    minimal_distance = distance;
                }

                // Store tag points for visualization
                tag_points.push_back(Point3d(pose.t->data[0] * 0.1, pose.t->data[1] * 0.1, pose.t->data[2] * 0.1));
            }

            // logger.info("Detected %d AprilTags", zarray_size(detections));

            apriltag_detections_destroy(detections);

            sensor_msgs::msg::Image debug_frame_msg;
            cv_bridge::CvImage debug_frame_cv;
            debug_frame_cv.header.stamp = this->now();
            debug_frame_cv.header.frame_id = "camera_color_optical_frame";
            debug_frame_cv.encoding = sensor_msgs::image_encodings::BGR8;
            debug_frame_cv.image = image_bgr;
            debug_frame_msg = *debug_frame_cv.toImageMsg();
            pub_debug_frame->publish(debug_frame_msg);

            visualization_msgs::msg::MarkerArray marker_array_msg;

            // Always publish marker array - either with tag or empty to clear previous markers
            if (tag_filtered.id != -1) {
                // Get transform from camera to base_link and apply it
                geometry_msgs::msg::TransformStamped transform_stamped;
                try {
                    transform_stamped = tf_buffer_.lookupTransform("base_link", "camera_color_optical_frame", tf2::TimePointZero);

                    // Create a geometry_msgs::msg::Point in camera frame
                    geometry_msgs::msg::Point point_camera;
                    point_camera.x = tag_filtered.pose_t[0];
                    point_camera.y = tag_filtered.pose_t[1];
                    point_camera.z = tag_filtered.pose_t[2];

                    // Transform the point from camera frame to base_link frame
                    geometry_msgs::msg::Point point_base;
                    tf2::doTransform(point_camera, point_base, transform_stamped);

                    visualization_msgs::msg::Marker marker;
                    marker.header.frame_id = "base_link"; // Changed to base_link
                    marker.header.stamp = this->now();
                    marker.ns = "apriltag_markers";
                    marker.id = tag_filtered.id;
                    marker.type = visualization_msgs::msg::Marker::SPHERE;
                    marker.action = visualization_msgs::msg::Marker::ADD;
                    marker.pose.position.x = point_base.x; // Use transformed coordinates
                    marker.pose.position.y = point_base.y;
                    marker.pose.position.z = point_base.z;
                    marker.pose.orientation.x = 0.0;
                    marker.pose.orientation.y = 0.0;
                    marker.pose.orientation.z = 0.0;
                    marker.pose.orientation.w = 1.0;
                    marker.scale.x = 0.1;
                    marker.scale.y = 0.1;
                    marker.scale.z = 0.1;
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 0.0;
                    marker.color.a = 1.0;
                    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
                    marker_array_msg.markers.push_back(marker);

                } catch (const tf2::TransformException& ex) {
                    RCLCPP_ERROR(this->get_logger(), "TF lookup failed: %s", ex.what());
                }
            } else {
                // try {
                //     visualization_msgs::msg::Marker marker;
                //     marker.header.frame_id = "base_link"; // Changed to base_link
                //     marker.header.stamp = this->now();
                //     marker.ns = "apriltag_markers";
                //     marker.id = tag_filtered.id;
                //     marker.type = visualization_msgs::msg::Marker::SPHERE;
                //     marker.action = visualization_msgs::msg::Marker::ADD;
                //     marker.pose.position.x = FLT_MAX; // Use transformed coordinates
                //     marker.pose.position.y = 0;
                //     marker.pose.position.z = 0;
                //     marker.pose.orientation.x = 0.0;
                //     marker.pose.orientation.y = 0.0;
                //     marker.pose.orientation.z = 0.0;
                //     marker.pose.orientation.w = 1.0;
                //     marker.scale.x = 0.1;
                //     marker.scale.y = 0.1;
                //     marker.scale.z = 0.1;
                //     marker.color.r = 1.0;
                //     marker.color.g = 0.0;
                //     marker.color.b = 0.0;
                //     marker.color.a = 1.0;
                //     marker.lifetime = rclcpp::Duration::from_seconds(0.5);
                //     marker_array_msg.markers.push_back(marker);

                // } catch (const tf2::TransformException& ex) {
                //     RCLCPP_ERROR(this->get_logger(), "TF lookup failed: %s", ex.what());
                // }
            }

            // Always publish marker array regardless of detection status
            pub_marker_array->publish(marker_array_msg);

        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
};
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto apriltag_node = std::make_shared<AprilTag3>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(apriltag_node);
    executor.spin();

    return 0;
}