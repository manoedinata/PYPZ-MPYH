#ifndef VISION_CAPTURE4_HPP
#define VISION_CAPTURE4_HPP
// C++ Standard Library
#include <algorithm>
#include <chrono>
#include <iostream>
#include <map>
#include <mutex>
#include <thread>

// C Standard Library
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <math.h>
#include <signal.h>
#include <sstream>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

// Third-party Libraries
#include "cv_bridge/cv_bridge.h"
#include "pcl_ros/transforms.hpp"
#include <librealsense2/rs.hpp>
#include <mlpack/methods/dbscan/dbscan.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>

// ROS 2 Libraries
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/global_definitions.hpp"
#include "ros2_utils/help_logger.hpp"
#include "ros2_utils/pid.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/int8.hpp"
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#define DASHED_REFERENCE 0
#define EDGE_REFERENCE 1

#define LEFT_LANE 0
#define RIGHT_LANE 1
#define CENTER_LANE 2

typedef pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
typedef point_cloud::Ptr cloud_pointer;

class VisionCapture4 : public rclcpp::Node
{
  private:
    // -------------------------------------------------
    // Transform
    // -------------------------------------------------
    bool tf_is_initialized = false;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    geometry_msgs::msg::TransformStamped tf_camera_base_;
    geometry_msgs::msg::TransformStamped tf_base_camera_;
    // -------------------------------------------------
    // Logging
    // -------------------------------------------------
    HelpLogger logger;
    // -------------------------------------------------
    // Subscribers Callback Groups
    // -------------------------------------------------
    rclcpp::CallbackGroup::SharedPtr sub_camera_bgr_rs_group_;
    rclcpp::CallbackGroup::SharedPtr sub_camera_depth_rs_group_;
    rclcpp::CallbackGroup::SharedPtr sub_camera_info_rs_group_;

    rclcpp::CallbackGroup::SharedPtr sub_callback_group_;
    // -------------------------------------------------
    // Timer Callback Group
    // -------------------------------------------------
    rclcpp::CallbackGroup::SharedPtr tim_routine_group_;
    // -------------------------------------------------
    // Subscribers
    // -------------------------------------------------
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_camera_bgr_rs_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_camera_depth_rs_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info_rs_;

    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sub_controlbox_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_vision_config_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_initial_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_enc_meter;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_target_speed;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_lane_used_web_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_button_;

    // -------------------------------------------------
    // Publishers
    // -------------------------------------------------
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_color_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_color_hull_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_camera_info_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_color_depth_overlay_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_filtered_binary_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_road_binary_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_binary_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_binary_2_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_binary_3_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cleaned_pointcloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_yuv_pointcloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered_points_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_sign_points_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_imagecloud_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_laserscan_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_slope_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_target_velocity_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_controlbox_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_config_vision_;
    // -------------------------------------------------
    // Timer
    // -------------------------------------------------
    rclcpp::TimerBase::SharedPtr timer_routine_;
    rclcpp::TimerBase::SharedPtr timer_img_routine_;
    rclcpp::TimerBase::SharedPtr timer_pointcloud_routine_;
    // -------------------------------------------------
    // Global Variables
    // -------------------------------------------------
    cv::Mat color_image_;
    cv::Mat depth_image_;
    rs2_intrinsics color_intrinsics_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_;

    std::mutex image_mutex_;
    std::mutex depth_mutex_;
    std::mutex camera_info_mutex_;
    std::mutex point_cloud_mutex_;

    uint8_t img_sync_ = 0;
    uint8_t pointcloud_sync_ = 0;

    uint8_t image_received_ = 0;
    uint8_t depth_received_ = 0;
    uint8_t camera_info_received_ = 0;

    int16_t center_cam_x_ = 0;
    int16_t center_cam_y_ = 0;

    cv::Point robot_position_ = {0, 0};
    int16_t bev_height_ = 0;
    int16_t bev_width_ = 0;

    float final_angle_used_for_steering_ = 0;

    float lookahead_far_meter_ = 0.5;
    float lookahead_near_meter_ = 0.3;
    int lookahead_far_pixel_ = 140;
    int lookahead_near_pixel_ = 100;
    float meter_to_pixel_ = 226.0869565f;
    float wheelbase_ = 0.27f;
    int used_lookahead = 100;
    float used_lookahead_meter = 0.5;

    float max_steering_deg_ = 35.0;
    float target_velocity_ = 0.8;
    float target_final_angle_ = 0.0;

    float line_length_min_ = 30.0f; // Minimum length of the dashed line segments
    float line_length_max_ = 45.0f;

    float line_length_edge_min_ = 30.0f; // Minimum length of the edge line segments
    float line_length_edge_max_ = 45.0f;

    float line_length_left_ = 30.0f; // Minimum length of the dashed line segments
    float line_length_right_ = 45.0f;

    float line_length_edge_left_ = 30.0f; // Minimum length of the edge line segments
    float line_length_edge_right_ = 45.0f;

    double min_dist_jarak_hindar_ = 0.5;
    double min_dist_jarak_keluar_ = 1.0;
    double constant_speed_belok_ = 5.0;

    uint8_t used_lane_ = 0;
    uint8_t origin_used_lane_ = 0;
    uint8_t origin_lane_dalam_ = 0;
    uint8_t prev_used_lane = 0;

    uint8_t used_reference_ = 0;

    int8_t ada_belokan_ = 0;
    int8_t ada_obs_ = 0;
    int8_t ada_obs_kanan_ = 0;
    int8_t ada_obs_lane_dalam_ = 0;
    int8_t ada_obs_slow_down_ = 0;
    int8_t prev_ada_obs_ = 0;
    int8_t perpindahan_lane_ = 0;
    int8_t prev_perpindahan_lane_ = 0;
    float pos_ada_belokan = 0;
    float jarak_hindar = 0;
    double counter_switch_lane = 0;
    float out_duration = 2000;
    float first_enc_see_obs = 0;
    float dist_to_robot_meter = 0;
    float dist_to_robot = 0;
    float jarak_actual_hindar = 0;
    float pos_enc_hindar = 0;
    double awal_center_lane = 0;
    uint8_t keluar_enc_ = 0;

    float kp_steering_ = 1.0;
    float ki_steering_ = 0.0;
    float kd_steering_ = 0.0;

    int valid_center_left_ = 0;
    int valid_center_right_ = 0;
    int valid_up_ = 0;
    int valid_down_ = 0;
    int cropping_distance_ = 0;

    float enc_meter;
    float enc_speed;
    float speed_motor;

    float jarak_hindar_meter_ = 0.7;

    float out_duration_belokan_ = 3000.0f;       // Duration to switch lane in milliseconds
    float out_duration_normal_ = 1500.0f;        // Duration to switch lane in milliseconds
    float offset_out_duration_center_ = 6000.0f; // Offset to adjust the duration

    float max_enc_meter_obs_ = 0.40f;       // Maximum encoder meter when obstacle is detected
    float max_enc_meter_obs_center_ = 0.70; // Maximum encoder meter when obstacle is detected

    float offset_camera_to_bev_height_ = 0.07f; // Offset from camera to BEV height in meters

    int8_t controlbox_size = 19;
    int16_t controlbox_data[50];

    rclcpp::Time sync_time_;

    cv::Point robot_position_pixel_real_;
    cv::Point robot_position_pixel_camera_;
    cv::Point robot_position_meter_real_;

    // angle between robot position and closest point in look_ahead_far and look_ahead_near
    float angle_used_ = 0.0f;
    float angle_used_kiri_ = 0.0f;
    float angle_used_kanan_ = 0.0f;
    float angle_used_edge_kiri_ = 0.0f;
    float angle_used_edge_kanan_ = 0.0f;

    std::vector<cv::Point> prev_saved_dashed_centroid_;

    int16_t dashed_filter_area_ = 200;
    float segment_speed_1 = 0.9f;
    float segment_speed_2 = 0.1f;
    float segment_speed_3 = 0.2f;
    float segment_speed_4 = 0.2f;
    int16_t road_segment_threshold_area = 5500;
    float constant_transient_speed = 0.8f;
    float scaller_speed = 1.0f;
    float offset_kiri = 2;

    float dash_kiri_default = 0.0f;
    float dash_kanan_default = 0.0f;
    float dash_kiri_anomali = 0.0f;
    float dash_kanan_anomali = 0.0f;
    float edge_kiri_default = 0.0f;
    float edge_kanan_default = 0.0f;
    float edge_kiri_anomali = 0.0f;
    float edge_kanan_anomali = 0.0f;

    int8_t button_1 = 0;
    int8_t button_2 = 0;
    int8_t toogle_button_1 = 0;
    int8_t toogle_button_2 = 0;

    int8_t use_realsense_ros = 0;

    // -------------------------------------------------
    // Fuzzy Variables
    // -------------------------------------------------
    float straight_center_ = 0.0; // STRAIGHT_PARAMS[0]
    float straight_sigma_ = 7.0;  // STRAIGHT_PARAMS[1]

    float wiggle_center_ = 17.5; // WIGGLE_PARAMS[0]
    float wiggle_sigma_ = 7.0;   // WIGGLE_PARAMS[1]

    float curve_center_ = 35.0; // CURVE_PARAMS[0]
    float curve_sigma_ = 7.0;   // CURVE_PARAMS[1]

    float speed_straight_ = 1.2; // SPEED_STRAIGHT
    float speed_wiggle_ = 0.95;  // SPEED_WIGGLE
    float speed_curve_ = 0.7;    // SPEED_CURVE

    float lookahead_straight_distance_ = 0.0f;
    float lookahead_wiggle_distance_ = 0.0f;
    float lookahead_curve_distance_ = 0.0f;

    // -------------------------------------------------
    // RealSense Variables
    // -------------------------------------------------
    rs2::pipeline pipe_;
    rs2::config cfg_;
    bool pipeline_started_ = false;
    std::mutex pipeline_mutex_;
    rs2::align align_to_color{RS2_STREAM_COLOR};
    rs2::pointcloud pc_;
    rs2::rates_printer printer_;
    // -------------------------------------------------
    // Shutdown handling
    // -------------------------------------------------
    std::atomic<bool> shutdown_requested_{false};

  public:
    VisionCapture4();
    ~VisionCapture4();

  private:
    void callback_sub_enc_meter(const std_msgs::msg::Float32::SharedPtr msg);
    void callback_sub_target_speed(const std_msgs::msg::Float32::SharedPtr msg);
    void callback_sub_initial(const std_msgs::msg::Int8::SharedPtr msg);
    void callback_sub_vision_config(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void callback_sub_controlbox(const std_msgs::msg::Int16MultiArray::SharedPtr msg);
    void callback_sub_lane_used_web(const std_msgs::msg::Int16::SharedPtr msg);
    void callback_sub_button(const std_msgs::msg::Int8::SharedPtr msg);

    void callback_tim_img_routine();

    void set_realsense_config(char *_param, int _value)
    {
        char cmd[128];
        sprintf(cmd, "ros2 param set /camera %s %d", _param, _value);
        system(cmd);
    }

    void set_realsense_config(char *_param, double _value)
    {
        char cmd[128];
        sprintf(cmd, "ros2 param set /camera %s %lf", _param, _value);
        system(cmd);
    }

    rs2_intrinsics camera_info_to_intrinsics(const sensor_msgs::msg::CameraInfo &info)
    {
        rs2_intrinsics intr;
        intr.width = info.width;
        intr.height = info.height;
        intr.fx = info.k[0];
        intr.fy = info.k[4];
        intr.ppx = info.k[2];
        intr.ppy = info.k[5];

        if (info.distortion_model == "plumb_bob")
            intr.model = RS2_DISTORTION_BROWN_CONRADY;
        else
            intr.model = RS2_DISTORTION_NONE;

        for (int i = 0; i < 5; i++)
            intr.coeffs[i] = (i < info.d.size()) ? static_cast<float>(info.d[i]) : 0.0f;

        return intr;
    }

    void callback_sub_camera_bgr_rs(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            std::lock_guard<std::mutex> lock(image_mutex_);
            auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat buffer = cv_ptr->image.clone();

            if (buffer.cols == 640 && buffer.rows == 360)
                color_image_ = cv_ptr->image.clone();
            else
                logger.error("Color image size is not correct %d %d", buffer.cols, buffer.rows);

            image_received_ = 1;
            sync_time_ = this->now();

            // ==================================================================
            //                        DEBUG VISION CAPTURE
            // ==================================================================
            double start_time = this->now().seconds();
            static double last_time = start_time;
            double elapsed_time = start_time - last_time;
            last_time = start_time;
            // logger.info("Color image elapsed time: %.4f seconds -> %.2f hz", elapsed_time, 1.0 / elapsed_time);
            // ==================================================================
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void callback_sub_camera_depth_rs(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            std::lock_guard<std::mutex> lock(depth_mutex_);
            auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            cv::Mat buffer = cv_ptr->image.clone();

            if (buffer.cols == 640 && buffer.rows == 360)
                depth_image_ = cv_ptr->image.clone();
            else
                logger.error("Depth image size is not correct %d %d", buffer.cols, buffer.rows);

            depth_received_ = 1;

            // ==================================================================
            //                        DEBUG VISION CAPTURE
            // ==================================================================
            double start_time = this->now().seconds();
            static double last_time = start_time;
            double elapsed_time = start_time - last_time;
            last_time = start_time;
            // logger.info("Depth image elapsed time: %.4f seconds -> %.2f hz", elapsed_time, 1.0 / elapsed_time);
            // ==================================================================
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void callback_sub_camera_info_rs(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> img_lock(camera_info_mutex_);

            color_intrinsics_ = camera_info_to_intrinsics(*msg);
            camera_info_received_ = 1;
        }
    }

    void callback_tim_pointcloud_routine()
    {
        // ==================================================================
        //                      WAIT TF TO BE INTIALIZED
        // ==================================================================
        if (!tf_is_initialized)
        {
            logger.warn("TF not initialized, skipping point cloud processing");
            return;
        }
        // ==================================================================
        //                    AVOID EXECUTION IF NOT NEEDED
        // ==================================================================
        if (pointcloud_sync_ <= 2)
            return;
        // ==================================================================
        //                        DEBUG VISION CAPTURE
        // ==================================================================
        double start_time = this->now().seconds();
        static double last_time = start_time;
        double elapsed_time = start_time - last_time;
        last_time = start_time;
        // logger.info("Timer pointcloud routine elapsed time: %.4f seconds", elapsed_time);
        // ==================================================================
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud;

        // Get the latest point cloud from the RealSense camera
        {
            std::lock_guard<std::mutex> lock(point_cloud_mutex_);

            if (!point_cloud_)
            {
                logger.warn("Point cloud not yet available, skipping publishing");
                return;
            }

            point_cloud = point_cloud_;
            // logger.info("==> Retrieved point cloud from global variables.");

            pointcloud_sync_ = 0;
        }

        pcl::PointCloud<pcl::PointXYZ> points_camera2base;
        pcl_ros::transformPointCloud(*point_cloud, points_camera2base, tf_camera_base_);

        //?==================================================================
        //?                   MAIN POINT CLOUD PROCESSING
        //?==================================================================
        pcl::PointCloud<pcl::PointXYZ> points_camera2base_filtered;
        pcl::PointCloud<pcl::PointXYZ> points_camera2base_filtered_for_sign;
        // Filter: keep only points above 2 cm from ground

        // Crop region in front of the robot (X = forward, Y = side-to-side, Z already filtered)
        static pcl::CropBox<pcl::PointXYZ> crop_box;
        crop_box.setInputCloud(points_camera2base.makeShared());
        crop_box.setMin(Eigen::Vector4f(0.2, -0.35, 0.15, 1.0)); // In front of robot
        crop_box.setMax(Eigen::Vector4f(2.0, 0.35, 1.0, 1.0));   // 2m ahead, ±1m wide, max 2m tall
        crop_box.setNegative(false);                             // Keep only points inside the box
        crop_box.filter(points_camera2base_filtered);

        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setInputCloud(points_camera2base_filtered.makeShared());
        voxel_grid.setLeafSize(0.008f, 0.008f, 0.008f); // 2cm voxel size
        voxel_grid.filter(points_camera2base_filtered);

        //? =============== ROAD SIGN DETECTION ===============

        // Crop region in front of the robot (X = forward, Y = side-to-side, Z already filtered)
        static pcl::CropBox<pcl::PointXYZ> crop_box_sign;
        crop_box_sign.setInputCloud(points_camera2base.makeShared());
        crop_box_sign.setMin(Eigen::Vector4f(0.01, -0.5, 0.07, 1.0)); // min x, y, z
        crop_box_sign.setMax(Eigen::Vector4f(1.8, 0.2, 0.2, 1.0));    // max x, y, z
        crop_box_sign.setNegative(false);
        crop_box_sign.filter(points_camera2base_filtered_for_sign);

        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_sign;
        voxel_grid_sign.setInputCloud(points_camera2base_filtered_for_sign.makeShared());
        voxel_grid_sign.setLeafSize(0.008f, 0.008f, 0.008f); // 2cm voxel size
        voxel_grid_sign.filter(points_camera2base_filtered_for_sign);

        //?==================================================================
        //?                 PUBLISH
        //?==================================================================
        // Publish the filtered point cloud
        sensor_msgs::msg::PointCloud2 msg_filtered_camera;
        pcl::toROSMsg(points_camera2base_filtered, msg_filtered_camera);
        msg_filtered_camera.header.stamp = sync_time_;
        msg_filtered_camera.header.frame_id = "base_link"; // transformed frame
        pub_filtered_points_->publish(msg_filtered_camera);

        // Publish the point cloud in base frame
        // sensor_msgs::msg::PointCloud2 msg_filtered_camera_for_sign;
        // pcl::toROSMsg(points_camera2base_filtered_for_sign, msg_filtered_camera_for_sign);
        // msg_filtered_camera_for_sign.header.stamp = sync_time_;
        // msg_filtered_camera_for_sign.header.frame_id = "base_link"; // transformed frame
        // pub_sign_points_->publish(msg_filtered_camera_for_sign);
        //?==================================================================
        // Convert PCL point cloud to ROS message
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*point_cloud, cloud_msg);
        cloud_msg.header.stamp = sync_time_;
        cloud_msg.header.frame_id = "camera_color_optical_frame";

        // Publish the point cloud
        pub_pointcloud_->publish(cloud_msg);
    }

    void callback_tim_routine()
    {
        if (shutdown_requested_)
            return;

        std::lock_guard<std::mutex> lock(pipeline_mutex_);

        if (!pipeline_started_)
            return;

        try
        {
            // Wait for frames with timeout
            rs2::frameset frameset = pipe_.wait_for_frames();
            if (!frameset)
            {
                logger.error("Failed to get frames from the camera.");
                return;
            }

            frameset = align_to_color.process(frameset);

            if (frameset)
            {
                rs2::video_frame color_frame = frameset.get_color_frame();
                rs2::depth_frame depth_frame = frameset.get_depth_frame();

                if (!color_frame || !depth_frame)
                {
                    logger.error("Failed to get color or depth frame.");
                    return;
                }

                // Convert to OpenCV format
                cv::Mat color_image(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);
                cv::Mat depth_image(cv::Size(depth_frame.get_width(), depth_frame.get_height()), CV_16U, (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);

                // Get camera intrinsics
                rs2_intrinsics intrinsics = color_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

                // Buffer into Global Variable
                {
                    std::lock_guard<std::mutex> img_lock(image_mutex_);
                    color_image_ = color_image.clone();
                    depth_image_ = depth_image.clone();
                    color_intrinsics_ = intrinsics;
                    // logger.info("<== Buffered color and depth images into global variables.");

                    img_sync_ = 1;
                }

                // Create point cloud from depth frame
                pc_.map_to(color_frame);
                auto points = pc_.calculate(depth_frame);
                auto sp = points.get_profile().as<rs2::video_stream_profile>();

                // Create a PCL point cloud object
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
                cloud->header.frame_id = "camera_color_optical_frame";
                cloud->header.stamp = this->now().nanoseconds();
                cloud->width = static_cast<uint32_t>(sp.width());
                cloud->height = static_cast<uint32_t>(sp.height());
                cloud->is_dense = false;
                cloud->points.resize(points.size());
                auto Vertex = points.get_vertices();

                // Iterate through all points and set XYZ coordinates
                for (size_t i = 0; i < points.size(); ++i)
                {
                    cloud->points[i].x = Vertex[i].x;
                    cloud->points[i].y = Vertex[i].y;
                    cloud->points[i].z = Vertex[i].z;
                }

                if (!cloud)
                {
                    logger.error("Failed to convert RealSense points to PCL cloud.");
                    return;
                }

                // Check if the point cloud is empty
                if (cloud->empty())
                {
                    logger.error("Point cloud is empty.");
                    return;
                }

                // Buffer into Global Variable
                {
                    std::lock_guard<std::mutex> pc_lock(point_cloud_mutex_);
                    point_cloud_ = cloud;
                    // logger.info("<== Buffered point cloud into global variable.");

                    if (pointcloud_sync_ < 255)
                        pointcloud_sync_ += 1;
                }

                sync_time_ = this->now();
                // ==================================================================
                //                       PUBLISH VISION CAPTURE
                // ==================================================================
                // -- Publish the color image
                sensor_msgs::msg::Image color_msg;
                cv_bridge::CvImage color_cv;
                color_cv.header.stamp = sync_time_;
                color_cv.header.frame_id = "camera_color_optical_frame";
                color_cv.encoding = sensor_msgs::image_encodings::BGR8;
                color_cv.image = color_image;
                color_msg = *color_cv.toImageMsg();
                pub_color_->publish(color_msg);

                // -- Publish the depth image
                sensor_msgs::msg::Image depth_msg;
                cv_bridge::CvImage depth_cv;
                depth_cv.header.stamp = sync_time_;
                depth_cv.header.frame_id = "camera_color_optical_frame";
                depth_cv.encoding = sensor_msgs::image_encodings::MONO16;
                depth_cv.image = depth_image;
                depth_msg = *depth_cv.toImageMsg();
                pub_depth_->publish(depth_msg);

                // -- Publish camera info
                sensor_msgs::msg::CameraInfo camera_info_msg;
                camera_info_msg.header.stamp = sync_time_;
                camera_info_msg.header.frame_id = "camera_color_optical_frame";
                camera_info_msg.width = color_image.cols;
                camera_info_msg.height = color_image.rows;
                camera_info_msg.distortion_model = "plumb_bob";

                // Camera intrinsic matrix K
                camera_info_msg.k[0] = intrinsics.fx;
                camera_info_msg.k[1] = 0.0;
                camera_info_msg.k[2] = intrinsics.ppx;
                camera_info_msg.k[3] = 0.0;
                camera_info_msg.k[4] = intrinsics.fy;
                camera_info_msg.k[5] = intrinsics.ppy;
                camera_info_msg.k[6] = 0.0;
                camera_info_msg.k[7] = 0.0;
                camera_info_msg.k[8] = 1.0;

                // Distortion coefficients
                camera_info_msg.d.resize(5);
                camera_info_msg.d[0] = intrinsics.coeffs[0]; // k1
                camera_info_msg.d[1] = intrinsics.coeffs[1]; // k2
                camera_info_msg.d[2] = intrinsics.coeffs[2]; // p1
                camera_info_msg.d[3] = intrinsics.coeffs[3]; // p2
                camera_info_msg.d[4] = intrinsics.coeffs[4]; // k3

                // Projection matrix P
                camera_info_msg.p[0] = intrinsics.fx;
                camera_info_msg.p[1] = 0.0;
                camera_info_msg.p[2] = intrinsics.ppx;
                camera_info_msg.p[3] = 0.0;
                camera_info_msg.p[4] = 0.0;
                camera_info_msg.p[5] = intrinsics.fy;
                camera_info_msg.p[6] = intrinsics.ppy;
                camera_info_msg.p[7] = 0.0;
                camera_info_msg.p[8] = 0.0;
                camera_info_msg.p[9] = 0.0;
                camera_info_msg.p[10] = 1.0;
                camera_info_msg.p[11] = 0.0;

                // Rectification matrix R (identity for monocular)
                camera_info_msg.r[0] = 1.0;
                camera_info_msg.r[1] = 0.0;
                camera_info_msg.r[2] = 0.0;
                camera_info_msg.r[3] = 0.0;
                camera_info_msg.r[4] = 1.0;
                camera_info_msg.r[5] = 0.0;
                camera_info_msg.r[6] = 0.0;
                camera_info_msg.r[7] = 0.0;
                camera_info_msg.r[8] = 1.0;

                pub_camera_info_->publish(camera_info_msg);
                // ==================================================================
                //                        DEBUG VISION CAPTURE
                // ==================================================================
                double start_time = this->now().seconds();
                static double last_time = start_time;
                double elapsed_time = start_time - last_time;
                last_time = start_time;
                // logger.info("Timer routine elapsed time: %.4f seconds", elapsed_time);
                // ==================================================================
            }
        }
        catch (const rs2::error &e)
        {
            if (!shutdown_requested_)
                RCLCPP_ERROR(this->get_logger(), "RealSense error in callback: %s", e.what());
        }
        catch (const std::exception &e)
        {
            if (!shutdown_requested_)
                RCLCPP_ERROR(this->get_logger(), "Unexpected error in callback: %s", e.what());
        }
    }

    void centroid_lookahead_used(cv::Mat lookahead_used, cv::Mat dashed_line, cv::Point &closest_point, float &angle_used)
    {
        cv::Mat intersection_lookahed;
        cv::bitwise_and(lookahead_used, dashed_line, intersection_lookahed);
        static cv::Point closest_point_used(robot_position_.x, robot_position_.y - 20);

        cv::Moments m_used = cv::moments(intersection_lookahed);
        if (m_used.m00 != 0)
        {
            closest_point.x = static_cast<int>(m_used.m10 / m_used.m00);
            closest_point.y = static_cast<int>(m_used.m01 / m_used.m00);
            // logger.info("Centroid lookahead used (x, y): (%d, %d)", closest_point.x, closest_point.y);
        }
        else
        {
            // logger.warn("No centroid found in lookahead used, using previous point (%d, %d)", closest_point_used.x, closest_point_used.y);
        }

        if (closest_point.x > 0 && closest_point.y > 0)
            angle_used = computeAngle(closest_point, robot_position_);
    }

    cv::Point get_robot_position_pixel_camera(cv::Point robot_position_)
    {
        int pixel_x = static_cast<int>(robot_position_.x);
        int pixel_y = static_cast<int>((robot_position_.y + (((wheelbase_ * 0.5) + offset_camera_to_bev_height_) * meter_to_pixel_))); // Invert Y axis for image coordinates
        return cv::Point(pixel_x, pixel_y);
    }

    cv::Point get_robot_position_pixel_real(cv::Point robot_position_)
    {
        int pixel_x = static_cast<int>(robot_position_.x);
        int pixel_y = static_cast<int>((robot_position_.y + ((wheelbase_ + offset_camera_to_bev_height_) * meter_to_pixel_))); // Invert Y axis for image coordinates
        return cv::Point(pixel_x, pixel_y);
    }

    cv::Point get_robot_position_meter_real(cv::Point robot_position_)
    {
        int meter_x = static_cast<int>(robot_position_.x / meter_to_pixel_);
        int meter_y = static_cast<int>((robot_position_.y + ((wheelbase_ + offset_camera_to_bev_height_) * meter_to_pixel_)) / meter_to_pixel_); // Invert Y axis for image coordinates
        return cv::Point(meter_x, meter_y);
    }

    cv::Point find_closest_black_point_square(const cv::Mat &image, cv::Point center, int radius, uint8_t is_left = 0)
    {
        cv::Point origin_center(center.x, center.y);
        cv::Point closest = center;
        double minDist = std::numeric_limits<double>::max();
        bool found = false;

        // if (is_left == 1) {
        //     center.x -= radius;
        // } else if (is_left == 0) {
        //     center.x += radius;
        // }

        for (int dy = -radius; dy <= radius; dy++)
        {
            for (int dx = -radius; dx <= radius; dx++)
            {
                cv::Point candidate(center.x + dx, center.y + dy);

                if (candidate.x >= 0 && candidate.x < image.cols && candidate.y >= 0 && candidate.y < image.rows && (dx * dx + dy * dy) <= radius * radius)
                {

                    if (image.at<uchar>(candidate.y, candidate.x) == 0)
                    {
                        double dist = cv::norm(candidate - origin_center);
                        if (dist < minDist)
                        {
                            minDist = dist;
                            closest = candidate;
                            found = true;
                        }
                    }
                }
            }
        }

        return found ? closest : cv::Point(-1, -1); // Return invalid point if no black pixel found
    }

    void adaptive_flood_fill(int16_t offset_x, int16_t offset_y, int16_t radius, cv::Mat bev_binary, cv::Mat &bev_color_image, cv::Mat &bev_cleaned_binary, std::vector<cv::Point> &saved_dashed_centroid)
    {
        static cv::Point center = {robot_position_.x, robot_position_.y};
        cv::Point start_point_kanan(center.x + offset_x, center.y - offset_y);
        cv::Point start_point_kiri(center.x - offset_x, center.y - offset_y);

        cv::Point centroid_closest_from_robot = {0, 0};
        for (size_t i = 0; i < saved_dashed_centroid.size(); i++)
        {
            cv::Point &point = saved_dashed_centroid[i];
            if (point.x < 0 || point.y < 0 || point.x >= bev_binary.cols || point.y >= bev_binary.rows)
                continue; // Skip points outside the image bounds

            if (i == 0)
                centroid_closest_from_robot = point;

            cv::circle(bev_color_image, point, 10, cv::Scalar(0, 255, 0), -1);
            // cv::putText(bev_color_image, std::to_string(i), point + cv::Point(5, 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        }

        cv::circle(bev_color_image, center, 20, cv::Scalar(255, 255, 0), -1);

        cv::Point polygon_points[1][4];
        polygon_points[0][0] = cv::Point(0, bev_height_);
        polygon_points[0][1] = cv::Point(valid_center_left_, bev_height_);
        polygon_points[0][2] = cv::Point(0, valid_up_);
        polygon_points[0][3] = cv::Point(0, bev_height_);
        const cv::Point *ppt[1] = {polygon_points[0]};
        int npt[] = {4};
        cv::fillPoly(bev_binary, ppt, npt, 1, cv::Scalar(255));

        polygon_points[0][0] = cv::Point(bev_width_, bev_height_);
        polygon_points[0][1] = cv::Point(valid_center_right_, bev_height_);
        polygon_points[0][2] = cv::Point(bev_width_, valid_up_);
        polygon_points[0][3] = cv::Point(bev_width_, bev_height_);
        cv::fillPoly(bev_binary, ppt, npt, 1, cv::Scalar(255));

        cv::Point closest_left_point(0, 0);
        cv::Point closest_right_point(0, 0);

        float n = 10;
        int8_t start = 5;
        cv::Mat bev_flood_fill_right = cv::Mat::zeros(bev_binary.size(), CV_8UC1);
        cv::Mat bev_flood_fill_left = cv::Mat::zeros(bev_binary.size(), CV_8UC1);
        cv::Mat last_bev_binary_left = cv::Mat::zeros(bev_binary.size(), CV_8UC1);
        cv::Mat last_bev_binary_right = cv::Mat::zeros(bev_binary.size(), CV_8UC1);
        for (int8_t i = start; i <= n; i++)
        {
            cv::Mat bev_circle_validate = cv::Mat::zeros(bev_binary.size(), CV_8UC1);
            cv::Mat bev_flood_fill_right_buffer = bev_binary.clone();
            cv::Mat bev_flood_fill_left_buffer = bev_binary.clone();
            iterate_flood_fill((float)(i / n), start_point_kanan, radius, closest_right_point, bev_color_image, bev_flood_fill_right_buffer, 0);
            iterate_flood_fill((float)(i / n), start_point_kiri, radius, closest_left_point, bev_color_image, bev_flood_fill_left_buffer, 1);

            if (i > start)
            {
                cv::Mat last_bev_not_binary_left = last_bev_binary_left.clone();
                cv::Mat last_bev_not_binary_right = last_bev_binary_right.clone();
                cv::Mat left_check = bev_flood_fill_left_buffer.clone();
                cv::Mat right_check = bev_flood_fill_right_buffer.clone();

                cv::bitwise_not(last_bev_not_binary_left, last_bev_not_binary_left);
                cv::bitwise_not(last_bev_not_binary_right, last_bev_not_binary_right);

                cv::bitwise_and(bev_flood_fill_left_buffer, last_bev_not_binary_left, left_check);
                cv::bitwise_and(bev_flood_fill_right_buffer, last_bev_not_binary_right, right_check);

                cv::erode(left_check, left_check, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)), cv::Point(-1, -1), 1);
                cv::erode(right_check, right_check, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)), cv::Point(-1, -1), 1);

                cv::circle(bev_circle_validate, robot_position_, (bev_height_ - cropping_distance_) * (float)((i - 1) / n) - 2, cv::Scalar(255), -1);

                cv::bitwise_and(bev_circle_validate, left_check, left_check);
                cv::bitwise_and(bev_circle_validate, right_check, right_check);

                if (cv::countNonZero(left_check) > 0 || cv::countNonZero(right_check) > 0)
                {
                    printf("Flood fill bocor %d\n", i);
                    cv::circle(bev_color_image, cv::Point(bev_width_ / 2, bev_height_ - (bev_height_ - cropping_distance_) * (float)((i) / n)), 5, cv::Scalar(0, 0, 255), -1);
                    break;
                }
                else
                {
                    cv::circle(bev_circle_validate, robot_position_, (bev_height_ - cropping_distance_) * (float)(i / n), cv::Scalar(255), -1);
                    cv::circle(bev_circle_validate, robot_position_, (bev_height_ - cropping_distance_) * (float)((i - 1) / n), cv::Scalar(0), -1);
                    cv::bitwise_and(bev_circle_validate, bev_flood_fill_left_buffer, bev_flood_fill_left_buffer);
                    cv::bitwise_and(bev_circle_validate, bev_flood_fill_right_buffer, bev_flood_fill_right_buffer);

                    cv::bitwise_or(bev_flood_fill_left, bev_flood_fill_left_buffer, bev_flood_fill_left);
                    cv::bitwise_or(bev_flood_fill_right, bev_flood_fill_right_buffer, bev_flood_fill_right);
                }
            }
            else
            {
                cv::bitwise_or(bev_flood_fill_left, bev_flood_fill_left_buffer, bev_flood_fill_left);
                cv::bitwise_or(bev_flood_fill_right, bev_flood_fill_right_buffer, bev_flood_fill_right);
            }

            last_bev_binary_left = bev_flood_fill_left.clone();
            last_bev_binary_right = bev_flood_fill_right.clone();
        }

        cv::morphologyEx(bev_flood_fill_right, bev_flood_fill_right, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)), cv::Point(-1, -1), 2);
        cv::morphologyEx(bev_flood_fill_left, bev_flood_fill_left, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)), cv::Point(-1, -1), 2);

        // check findcontours in flood fill kanan and kiri
        std::vector<std::vector<cv::Point>> flood_fill_right_contours;
        std::vector<std::vector<cv::Point>> flood_fill_left_contours;
        cv::findContours(bev_flood_fill_right, flood_fill_right_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::findContours(bev_flood_fill_left, flood_fill_left_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // check width and height of flood fill kanan and kiri
        int flood_fill_right_width = 0;
        int flood_fill_right_height = 0;
        int flood_fill_left_width = 0;
        int flood_fill_left_height = 0;
        if (!flood_fill_right_contours.empty())
        {
            flood_fill_right_width = cv::boundingRect(flood_fill_right_contours[0]).width;
            flood_fill_right_height = cv::boundingRect(flood_fill_right_contours[0]).height;
        }
        if (!flood_fill_left_contours.empty())
        {
            flood_fill_left_width = cv::boundingRect(flood_fill_left_contours[0]).width;
            flood_fill_left_height = cv::boundingRect(flood_fill_left_contours[0]).height;
        }

        static int flood_fill_right_width_prev = 0;
        static int flood_fill_left_width_prev = 0;
        static int flood_fill_right_height_prev = 0;
        static int flood_fill_left_height_prev = 0;

        bev_cleaned_binary = cv::Mat::zeros(bev_binary.size(), CV_8UC1);
        if ((abs(flood_fill_right_width - flood_fill_left_width) > 0 || abs(flood_fill_right_height - flood_fill_left_height) > 0))
        {

            // logger.warn("Flood fill right and left contours are not similar, skipping flood fill optimization");
            int error_right = abs(flood_fill_right_width - flood_fill_right_width_prev) + abs(flood_fill_right_height - flood_fill_right_height_prev);
            int error_left = abs(flood_fill_left_width - flood_fill_left_width_prev) + abs(flood_fill_left_height - flood_fill_left_height_prev);

            // logger.info("right width %d %d height %d %d left width %d %d height %d %d",
            //     flood_fill_right_width, flood_fill_right_width_prev, flood_fill_right_height, flood_fill_right_height_prev,
            //     flood_fill_left_width, flood_fill_left_width_prev, flood_fill_left_height, flood_fill_left_height_prev);

            if (error_right < error_left)
            {
                // logger.info("Using flood fill right as primary contour");
                bev_cleaned_binary = bev_flood_fill_right.clone();
                flood_fill_right_height_prev = flood_fill_right_height;
                flood_fill_right_width_prev = flood_fill_right_width;
                // center.x = closest_right_point.x;
                center.x = centroid_closest_from_robot.x;
                cv::putText(bev_color_image, "Right Flood Fill", cv::Point(10, 210), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
            }
            else
            {
                // logger.info("Using flood fill left as primary contour");
                bev_cleaned_binary = bev_flood_fill_left.clone();
                flood_fill_left_height_prev = flood_fill_left_height;
                flood_fill_left_width_prev = flood_fill_left_width;
                // center.x = closest_left_point.x;
                center.x = centroid_closest_from_robot.x;
                cv::putText(bev_color_image, "Left Flood Fill", cv::Point(10, 210), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
            }
        }
        else
        {
            cv::bitwise_or(bev_flood_fill_right, bev_flood_fill_left, bev_cleaned_binary);
            flood_fill_right_width_prev = flood_fill_right_width;
            flood_fill_left_width_prev = flood_fill_left_width;
            flood_fill_right_height_prev = flood_fill_right_height;
            flood_fill_left_height_prev = flood_fill_left_height;

            if (bev_cleaned_binary.at<uchar>(robot_position_.y - 20, robot_position_.x) == 255)
            {
                // logger.info("Flood fill cleaned binary at robot position is 255, setting center to robot position");
                // center.x = robot_position_.x;
                cv::putText(bev_color_image, "Flood Fill Both Reset", cv::Point(10, 210), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
            }
            else
            {
                cv::putText(bev_color_image, "Flood Fill Both", cv::Point(10, 210), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
            }
        }
    }

    void iterate_flood_fill(float scaling_unit, cv::Point start_point, int radius, cv::Point &closest_point, cv::Mat &bev_color_image, cv::Mat &bev_flood_fill, uint8_t is_left)
    {
        cv::Mat bev_binary = bev_flood_fill.clone();
        cv::circle(bev_binary, robot_position_, (bev_height_ - cropping_distance_) * scaling_unit, cv::Scalar(255), 5);
        cv::circle(bev_color_image, robot_position_, (bev_height_ - cropping_distance_) * scaling_unit, cv::Scalar(255 * scaling_unit, 0, 0), 2);
        bev_flood_fill = bev_binary.clone();
        closest_flood_fill(start_point, radius, closest_point, bev_color_image, bev_flood_fill, is_left);
    }

    void closest_flood_fill(cv::Point start_point, int radius, cv::Point &closest_point, cv::Mat &bev_color_image, cv::Mat &bev_flood_fill, uint8_t is_left)
    {
        closest_point = start_point;
        if (start_point.x >= 0 && start_point.x < bev_flood_fill.cols && start_point.y >= 0 && start_point.y < bev_flood_fill.rows)
        {
            if (bev_flood_fill.at<uchar>(start_point.y, start_point.x) == 255)
            {
                cv::Point new_start = find_closest_black_point_square(bev_flood_fill, start_point, radius, is_left);
                if (new_start.x != -1 && new_start.y != -1)
                    start_point = new_start;
            }

            if (bev_flood_fill.at<uchar>(start_point.y, start_point.x) == 0)
            {
                cv::Mat mask = cv::Mat::zeros(bev_flood_fill.rows + 2, bev_flood_fill.cols + 2, CV_8UC1);
                cv::floodFill(bev_flood_fill, mask, start_point,
                              cv::Scalar(255), nullptr, cv::Scalar(0), cv::Scalar(0), 4);

                // Extract just the mask (remove the 1-pixel border)
                cv::Mat flood_mask = mask(cv::Rect(1, 1, bev_flood_fill.cols, bev_flood_fill.rows));
                bev_flood_fill = flood_mask * 255;
                cv::circle(bev_color_image, start_point, 5, cv::Scalar(0, 0, 0), -1);
                closest_point = start_point;
            }
        }

        cv::circle(bev_color_image, start_point, radius, cv::Scalar(0, 255, 0), 1);

        if (is_left)
        {
            // Draw a rectangle for the search area (left)
            cv::rectangle(
                bev_color_image,
                cv::Point(start_point.x - 2 * radius, start_point.y - radius),
                cv::Point(start_point.x, start_point.y + radius),
                cv::Scalar(255, 0, 0), 1);
        }
        else
        {
            // Draw a rectangle for the search area (right)
            cv::rectangle(
                bev_color_image,
                cv::Point(start_point.x, start_point.y - radius),
                cv::Point(start_point.x + 2 * radius, start_point.y + radius),
                cv::Scalar(0, 0, 255), 1);
        }
    }

    void dynamic_flood_fill(int16_t offset_x, int16_t offset_y, int16_t radius, cv::Mat bev_binary, cv::Mat &bev_color_image, cv::Mat &bev_cleaned_binary)
    {
        static cv::Point center = {robot_position_.x, robot_position_.y};
        cv::Point start_point_kanan(center.x + offset_x, center.y - offset_y);
        cv::Point start_point_kiri(center.x - offset_x, center.y - offset_y);

        if (button_1)
            center = cv::Point(robot_position_.x, robot_position_.y);

        cv::Point polygon_points[1][4];
        polygon_points[0][0] = cv::Point(0, bev_height_);
        polygon_points[0][1] = cv::Point(valid_center_left_, bev_height_);
        polygon_points[0][2] = cv::Point(0, valid_up_);
        polygon_points[0][3] = cv::Point(0, bev_height_);
        const cv::Point *ppt[1] = {polygon_points[0]};
        int npt[] = {4};
        cv::fillPoly(bev_binary, ppt, npt, 1, cv::Scalar(255));

        polygon_points[0][0] = cv::Point(bev_width_, bev_height_);
        polygon_points[0][1] = cv::Point(valid_center_right_, bev_height_);
        polygon_points[0][2] = cv::Point(bev_width_, valid_up_);
        polygon_points[0][3] = cv::Point(bev_width_, bev_height_);
        cv::fillPoly(bev_binary, ppt, npt, 1, cv::Scalar(255));

        cv::Mat bev_flood_fill_right = bev_binary.clone();
        cv::Mat bev_flood_fill_left = bev_binary.clone();

        cv::Point closest_left_point(0, 0);
        cv::Point closest_right_point(0, 0);

        if (start_point_kanan.x >= 0 && start_point_kanan.x < bev_flood_fill_right.cols && start_point_kanan.y >= 0 && start_point_kanan.y < bev_flood_fill_right.rows)
        {
            if (bev_flood_fill_right.at<uchar>(start_point_kanan.y, start_point_kanan.x) == 255)
            {
                cv::Point new_start = find_closest_black_point_square(bev_flood_fill_right, start_point_kanan, radius, 0);
                if (new_start.x != -1 && new_start.y != -1)
                    start_point_kanan = new_start;
            }

            if (bev_flood_fill_right.at<uchar>(start_point_kanan.y, start_point_kanan.x) == 0)
            {
                cv::Mat mask_kanan = cv::Mat::zeros(bev_flood_fill_right.rows + 2, bev_flood_fill_right.cols + 2, CV_8UC1);
                cv::floodFill(bev_flood_fill_right, mask_kanan, start_point_kanan,
                              cv::Scalar(255), nullptr, cv::Scalar(0), cv::Scalar(0), 4);

                // Extract just the mask (remove the 1-pixel border)
                cv::Mat flood_mask_kanan = mask_kanan(cv::Rect(1, 1, bev_flood_fill_right.cols, bev_flood_fill_right.rows));
                bev_flood_fill_right = flood_mask_kanan * 255;
                cv::circle(bev_color_image, start_point_kanan, 5, cv::Scalar(0, 0, 255), -1);
                closest_right_point = start_point_kanan;
            }
        }

        if (start_point_kiri.x >= 0 && start_point_kiri.x < bev_flood_fill_left.cols && start_point_kiri.y >= 0 && start_point_kiri.y < bev_flood_fill_left.rows)
        {
            if (bev_flood_fill_left.at<uchar>(start_point_kiri.y, start_point_kiri.x) == 255)
            {
                cv::Point new_start = find_closest_black_point_square(bev_flood_fill_left, start_point_kiri, radius, 1);
                if (new_start.x != -1 && new_start.y != -1)
                    start_point_kiri = new_start;
            }

            if (bev_flood_fill_left.at<uchar>(start_point_kiri.y, start_point_kiri.x) == 0)
            {
                cv::Mat mask_kiri = cv::Mat::zeros(bev_flood_fill_left.rows + 2, bev_flood_fill_left.cols + 2, CV_8UC1);
                cv::floodFill(bev_flood_fill_left, mask_kiri, start_point_kiri,
                              cv::Scalar(255), nullptr, cv::Scalar(0), cv::Scalar(0), 4);

                // Extract just the mask (remove the 1-pixel border)
                cv::Mat flood_mask_kiri = mask_kiri(cv::Rect(1, 1, bev_flood_fill_left.cols, bev_flood_fill_left.rows));
                bev_flood_fill_left = flood_mask_kiri * 255;
                cv::circle(bev_color_image, start_point_kiri, 5, cv::Scalar(255, 0, 0), -1);
                closest_left_point = start_point_kiri;
            }
        }

        // check findcontours in flood fill kanan and kiri
        std::vector<std::vector<cv::Point>> flood_fill_right_contours;
        std::vector<std::vector<cv::Point>> flood_fill_left_contours;
        cv::findContours(bev_flood_fill_right, flood_fill_right_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::findContours(bev_flood_fill_left, flood_fill_left_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // check width and height of flood fill kanan and kiri
        int flood_fill_right_width = 0;
        int flood_fill_right_height = 0;
        int flood_fill_left_width = 0;
        int flood_fill_left_height = 0;
        if (!flood_fill_right_contours.empty())
        {
            flood_fill_right_width = cv::boundingRect(flood_fill_right_contours[0]).width;
            flood_fill_right_height = cv::boundingRect(flood_fill_right_contours[0]).height;
        }
        if (!flood_fill_left_contours.empty())
        {
            flood_fill_left_width = cv::boundingRect(flood_fill_left_contours[0]).width;
            flood_fill_left_height = cv::boundingRect(flood_fill_left_contours[0]).height;
        }

        static int flood_fill_right_width_prev = 0;
        static int flood_fill_left_width_prev = 0;
        static int flood_fill_right_height_prev = 0;
        static int flood_fill_left_height_prev = 0;

        double time_now_switch = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        static double last_time_switch = time_now_switch;

        bev_cleaned_binary = cv::Mat::zeros(bev_binary.size(), CV_8UC1);
        if ((abs(flood_fill_right_width - flood_fill_left_width) > 10 || abs(flood_fill_right_height - flood_fill_left_height) > 10))
        {

            // logger.warn("Flood fill right and left contours are not similar, skipping flood fill optimization");
            int error_right = abs(flood_fill_right_width - flood_fill_right_width_prev) + abs(flood_fill_right_height - flood_fill_right_height_prev);
            int error_left = abs(flood_fill_left_width - flood_fill_left_width_prev) + abs(flood_fill_left_height - flood_fill_left_height_prev);

            // logger.info("right width %d %d height %d %d left width %d %d height %d %d",
            //     flood_fill_right_width, flood_fill_right_width_prev, flood_fill_right_height, flood_fill_right_height_prev,
            //     flood_fill_left_width, flood_fill_left_width_prev, flood_fill_left_height, flood_fill_left_height_prev);

            if (error_right < error_left)
            {
                // logger.info("Using flood fill right as primary contour");
                bev_cleaned_binary = bev_flood_fill_right.clone();
                flood_fill_right_height_prev = flood_fill_right_height;
                flood_fill_right_width_prev = flood_fill_right_width;
                center.x = closest_right_point.x;
            }
            else
            {
                // logger.info("Using flood fill left as primary contour");
                bev_cleaned_binary = bev_flood_fill_left.clone();
                flood_fill_left_height_prev = flood_fill_left_height;
                flood_fill_left_width_prev = flood_fill_left_width;
                center.x = closest_left_point.x;
            }
        }
        else
        {
            cv::bitwise_or(bev_flood_fill_right, bev_flood_fill_left, bev_cleaned_binary);
            flood_fill_right_width_prev = flood_fill_right_width;
            flood_fill_left_width_prev = flood_fill_left_width;
            flood_fill_right_height_prev = flood_fill_right_height;
            flood_fill_left_height_prev = flood_fill_left_height;

            if (bev_cleaned_binary.at<uchar>(robot_position_.y - 20, robot_position_.x) == 255)
            {
                // logger.info("Flood fill cleaned binary at robot position is 255, setting center to robot position");
                center.x = robot_position_.x;
            }
        }
    }

    int8_t edge_reference_detection(cv::Mat &bev_cleaned_binary, cv::Mat &bev_color_image, std::vector<cv::Point> &left_flood_fill_points, std::vector<cv::Point> &right_flood_fill_points)
    {
        left_flood_fill_points.clear();
        right_flood_fill_points.clear();

        cv::Mat line_valid_region = cv::Mat::zeros(bev_cleaned_binary.size(), CV_8UC1);

        // make polygon valid region from (0, bev_height), (valid_center_left, bev_height), (0, valid_up)
        cv::Point polygon_points[1][4];
        polygon_points[0][0] = cv::Point(0, bev_height_);
        polygon_points[0][1] = cv::Point(valid_center_left_, bev_height_);
        polygon_points[0][2] = cv::Point(0, valid_up_);
        polygon_points[0][3] = cv::Point(0, bev_height_);
        const cv::Point *ppt[1] = {polygon_points[0]};
        int npt[] = {4};
        cv::fillPoly(line_valid_region, ppt, npt, 1, cv::Scalar(255));
        cv::fillPoly(bev_color_image, ppt, npt, 1, cv::Scalar(0, 255, 0));

        // make polygon valid region from (bev_width, bev_height), (valid_center_right, bev_height), (bev_width, valid_up)
        polygon_points[0][0] = cv::Point(bev_width_, bev_height_);
        polygon_points[0][1] = cv::Point(valid_center_right_, bev_height_);
        polygon_points[0][2] = cv::Point(bev_width_, valid_up_);
        polygon_points[0][3] = cv::Point(bev_width_, bev_height_);
        cv::fillPoly(line_valid_region, ppt, npt, 1, cv::Scalar(255));
        cv::fillPoly(bev_color_image, ppt, npt, 1, cv::Scalar(0, 255, 0));

        // Draw a black rectangle in the center region between left and right valid centers
        // cv::Point rect_points[1][4];
        // rect_points[0][0] = cv::Point(static_cast<int>(valid_center_left_ * 0.5), bev_height_);
        // rect_points[0][1] = cv::Point(static_cast<int>((bev_width_ + valid_center_right_) * 0.5), bev_height_);
        // rect_points[0][2] = cv::Point(static_cast<int>((bev_width_ + valid_center_right_) * 0.5), valid_up_);
        // rect_points[0][3] = cv::Point(static_cast<int>(valid_center_left_ * 0.5), valid_up_);
        // const cv::Point* rect_ppt[1] = { rect_points[0] };
        // int rect_npt[] = { 4 };
        // cv::fillPoly(line_valid_region, rect_ppt, rect_npt, 1, cv::Scalar(0));
        // cv::fillPoly(bev_color_image, rect_ppt, rect_npt, 1, cv::Scalar(0, 0, 0));

        uint8_t stop_left = 0;
        uint8_t stop_right = 0;

        uint8_t left_valid = 0;
        uint8_t right_valid = 0;

        for (int i = 0; i < bev_height_ - cropping_distance_; i += 20)
        {
            cv::Point potential_rising = cv::Point(-1, -1);
            cv::Point potential_falling = cv::Point(-1, -1);
            for (float j = 0.1; j < 180; j += 0.1)
            {
                float distance = i;
                float angle = j * M_PI / 180.0f;
                float angle_last = (j - 0.1) * M_PI / 180.0f;
                int16_t x = static_cast<int16_t>(robot_position_.x + distance * cos(angle));
                int16_t y = static_cast<int16_t>(robot_position_.y - distance * sin(angle));
                int16_t x_last = static_cast<int16_t>(robot_position_.x + distance * cos(angle_last));
                int16_t y_last = static_cast<int16_t>(robot_position_.y - distance * sin(angle_last));
                // cv::circle(bev_color_image_raw, cv::Point(x, y), 2, cv::Scalar(0, 0, 255), -1);

                if (x >= 0 && x < bev_width_ && y >= 0 && y < bev_height_)
                {
                    int16_t pixel_value_dot = bev_cleaned_binary.at<uchar>(y, x) - bev_cleaned_binary.at<uchar>(y_last, x_last);
                    if (pixel_value_dot > 0 && line_valid_region.at<uchar>(y, x) == 0 && stop_right == 0)
                    {
                        potential_rising = cv::Point(x, y);
                        right_flood_fill_points.push_back(potential_rising);
                        break;
                    }
                    else if (pixel_value_dot > 0 && line_valid_region.at<uchar>(y, x) == 255)
                    {
                        cv::circle(bev_color_image, potential_rising, 5, cv::Scalar(0, 0, 255), -1);
                        stop_right = 1;
                    }
                    else if (pixel_value_dot < 0)
                    {
                        break;
                    }
                }
            }

            for (float j = 179.9; j > 0; j -= 0.1)
            {
                float distance = i;
                float angle = j * M_PI / 180.0f;
                float angle_last = (j + 0.1) * M_PI / 180.0f;
                int16_t x = static_cast<int16_t>(robot_position_.x + distance * cos(angle));
                int16_t y = static_cast<int16_t>(robot_position_.y - distance * sin(angle));
                int16_t x_last = static_cast<int16_t>(robot_position_.x + distance * cos(angle_last));
                int16_t y_last = static_cast<int16_t>(robot_position_.y - distance * sin(angle_last));
                // cv::circle(bev_color_image_raw, cv::Point(x, y), 2, cv::Scalar(255, 0, 0), -1);

                if (x >= 0 && x < bev_width_ && y >= 0 && y < bev_height_)
                {
                    int16_t pixel_value_dot = bev_cleaned_binary.at<uchar>(y, x) - bev_cleaned_binary.at<uchar>(y_last, x_last);
                    if (pixel_value_dot > 0 && line_valid_region.at<uchar>(y, x) == 0 && stop_left == 0)
                    {
                        potential_falling = cv::Point(x, y);
                        left_flood_fill_points.push_back(potential_falling);
                        break;
                    }
                    else if (pixel_value_dot > 0 && line_valid_region.at<uchar>(y, x) == 255)
                    {
                        cv::circle(bev_color_image, potential_falling, 3, cv::Scalar(255, 0, 0), -1);
                        stop_left = 1;
                    }
                    else if (pixel_value_dot < 0)
                    {
                        break;
                    }
                }
            }
        }

        for (int i = 0; i < right_flood_fill_points.size(); i++)
            cv::circle(bev_color_image, right_flood_fill_points[i], 5, cv::Scalar(0, 0, 255), -1);

        for (int i = 0; i < left_flood_fill_points.size(); i++)
            cv::circle(bev_color_image, left_flood_fill_points[i], 5, cv::Scalar(255, 0, 0), -1);

        if (right_flood_fill_points.size() > 3)
            right_valid = 1;

        if (left_flood_fill_points.size() > 3)
            left_valid = 1;

        if (left_valid && right_valid)
            return 3;
        else if (right_valid)
            return 2;
        else if (left_valid)
            return 1;
        else
            return 0;
    }

    void road_segment(cv::Mat bev_cleaned_binary, cv::Mat &bev_color_image, uint8_t num_segments, uint16_t threshold_area, float *segment_speed, uint8_t debug = 0)
    {
        cv::Mat segmented_region = cv::Mat::zeros(bev_cleaned_binary.size(), CV_8UC1);

        int max_radius = bev_height_ - cropping_distance_;
        int segment_radius = max_radius / num_segments;

        std::vector<cv::Scalar> segment_colors = {
            cv::Scalar(255, 0, 0),  // Blue
            cv::Scalar(0, 255, 0),  // Green
            cv::Scalar(0, 0, 255),  // Red
            cv::Scalar(0, 255, 255) // Yellow
        };
        cv::Mat segmented_color = cv::Mat::zeros(bev_cleaned_binary.size(), CV_8UC3);

        for (int seg = 0; seg < num_segments; ++seg)
        {
            int inner_radius = seg * segment_radius;
            int outer_radius = (seg + 1) * segment_radius;

            cv::Mat mask = cv::Mat::zeros(bev_cleaned_binary.size(), CV_8UC1);
            cv::circle(mask, robot_position_, outer_radius, cv::Scalar(255), -1);
            if (inner_radius > 0)
                cv::circle(mask, robot_position_, inner_radius, cv::Scalar(0), -1);
            cv::Mat segment_mask;
            cv::bitwise_and(mask, bev_cleaned_binary, segment_mask);

            cv::Mat color_mask;
            cv::cvtColor(segment_mask, color_mask, cv::COLOR_GRAY2BGR);
            segmented_color.setTo(segment_colors[seg % segment_colors.size()], segment_mask);

            int segment_area = cv::countNonZero(segment_mask);

            if (segment_area < threshold_area && seg != 0)
                segment_speed[seg] = 0.0;

            cv::Moments m = cv::moments(segment_mask, true);
            if (m.m00 != 0)
            {
                cv::Point centroid(static_cast<int>(m.m10 / m.m00), static_cast<int>(m.m01 / m.m00));
                cv::circle(segmented_color, centroid, 10, cv::Scalar(255, 255, 255), -1);
                if (debug)
                {
                    std::string text = std::to_string(segment_area) + " px";
                    cv::putText(bev_color_image, text, centroid, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
                }
            }
        }

        cv::addWeighted(bev_color_image, 0.7, segmented_color, 0.3, 0, bev_color_image);
    }

    void transform_point(float *point_in, float *point_out)
    {
        geometry_msgs::msg::PointStamped point_base, point_camera;
        point_base.header.frame_id = "base_link";
        point_base.point.x = point_in[0];
        point_base.point.y = point_in[1];
        point_base.point.z = point_in[2];

        try
        {
            tf2::doTransform(point_base, point_camera, tf_base_camera_);
            point_out[0] = point_camera.point.x;
            point_out[1] = point_camera.point.y;
            point_out[2] = point_camera.point.z;
        }
        catch (const tf2::TransformException &ex)
        {
            logger.warn("Failed to transform point: %s", ex.what());
            point_out[0] = point_in[0];
            point_out[1] = point_in[1];
            point_out[2] = point_in[2];
        }
    }

    std::vector<cv::Point> scanObstaclesPolar(const cv::Mat &obstacle_bev,
                                              const cv::Point &robot_bev,
                                              int num_angles = 360)
    {
        std::vector<cv::Point> detected_points;

        int W = obstacle_bev.cols, H = obstacle_bev.rows;
        float max_radius = std::hypot(std::max(robot_bev.x, W - robot_bev.x), std::max(robot_bev.y, H - robot_bev.y));

        for (int angle = 0; angle < num_angles; ++angle)
        {
            float theta = angle * CV_PI / 180.0f; // convert to radians

            // Hitung titik ujung pada garis arah "theta"
            int x2 = int(robot_bev.x + max_radius * std::cos(theta));
            int y2 = int(robot_bev.y + max_radius * std::sin(theta));

            cv::LineIterator it(obstacle_bev, robot_bev, cv::Point(x2, y2));
            bool found = false;
            for (int i = 0; i < it.count; ++i, ++it)
            {
                uchar val = **it;
                if (val > 0)
                { // obstacle detected
                    detected_points.push_back(it.pos());
                    found = true;
                    break; // ambil titik terdekat
                }
            }
            if (!found)
                detected_points.push_back(cv::Point(-1, -1)); // tidak ada obstacle pada arah ini
        }
        return detected_points;
    }

    void segmentRoadDBSCANOptimized(cv::Mat &road_mask_bev,
                                    std::vector<cv::Point> &left_lane,
                                    std::vector<cv::Point> &right_lane,
                                    double scale = 0.2,   // Skala downsample (misal 0.3)
                                    double epsilon = 7.0, // Radius cluster pada resolusi kecil (pixel)
                                    size_t minPoints = 12 // Minimum anggota cluster
    )
    {

        left_lane.clear();
        right_lane.clear();

        // hapus kontour yang terlalu besar
        std::vector<std::vector<cv::Point>> start_contours;
        cv::findContours(road_mask_bev, start_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (auto &contour : start_contours)
        {
            // erase the contour that too big
            if (cv::contourArea(contour) > 5000) // misal 10000 pixel
            {
                // remove the contour from the mask
                // logger.info("Contour area: %.2f", cv::contourArea(contour));
                cv::drawContours(road_mask_bev, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0), cv::FILLED);
            }
        }

        // 1. Downscale mask untuk mempercepat DBSCAN
        cv::Mat small_mask;
        cv::resize(road_mask_bev, small_mask, cv::Size(), scale, scale, cv::INTER_NEAREST);

        // cari 2 contour terbesar
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(small_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // logger.info("Found %zu contours in small mask", contours.size());

        if (contours.size() < 2)
            return; // Tidak ada cukup kontur
        // Urutkan kontur berdasarkan area
        std::sort(contours.begin(), contours.end(), [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b)
                  { return cv::boundingRect(a).area() > cv::boundingRect(b).area(); });
        // Ambil 2 kontur terbesar
        if (contours.size() > 2)
            contours.resize(2);

        std::sort(contours.begin(), contours.end(), [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b)
                  { return cv::boundingRect(a).y < cv::boundingRect(b).y; });
        // Buat mask dari 2 kontur terbesar
        small_mask = cv::Mat::zeros(small_mask.size(), CV_8U);
        for (const auto &contour : contours)
            cv::drawContours(small_mask, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(255), cv::FILLED);

        // 2. Ekstrak koordinat jalan pada mask kecil
        std::vector<cv::Point> small_points;
        for (int y = 0; y < small_mask.rows; ++y)
            for (int x = 0; x < small_mask.cols; ++x)
                if (small_mask.at<uchar>(y, x) > 0)
                    small_points.emplace_back(x, y);

        if (small_points.size() < minPoints * 2)
            return; // Not enough points

        // 3. Siapkan data untuk DBSCAN
        arma::mat data(2, small_points.size());
        for (size_t i = 0; i < small_points.size(); ++i)
        {
            data(0, i) = small_points[i].x;
            data(1, i) = small_points[i].y;
        }

        // 4. Jalankan DBSCAN
        arma::Row<size_t> assignments;
        mlpack::dbscan::DBSCAN<> dbscan(epsilon, minPoints);
        dbscan.Cluster(data, assignments);

        // 5. Kelompokkan hasil cluster
        std::map<size_t, std::vector<cv::Point>> clusters;
        for (size_t i = 0; i < small_points.size(); ++i)
        {
            size_t label = assignments[i];
            if (label == SIZE_MAX)
                continue; // noise
            clusters[label].push_back(small_points[i]);
        }

        if (clusters.size() < 2)
            return; // Tidak ada dua cluster

        // 6. Tentukan cluster kiri & kanan berdasarkan rata-rata X
        std::vector<std::pair<double, size_t>> mean_x_per_cluster;
        for (const auto &kv : clusters)
        {
            double mean_x = 0;
            for (auto &pt : kv.second)
                mean_x += pt.x;
            mean_x /= kv.second.size();
            mean_x_per_cluster.emplace_back(mean_x, kv.first);
        }
        std::sort(mean_x_per_cluster.begin(), mean_x_per_cluster.end());

        // 7. Upscale koordinat ke resolusi asli BEV
        left_lane.clear();
        right_lane.clear();
        size_t label_left = mean_x_per_cluster.front().second;
        size_t label_right = mean_x_per_cluster.back().second;

        for (const auto &pt : clusters[label_left])
            left_lane.emplace_back(int(pt.x / scale), int(pt.y / scale));
        for (const auto &pt : clusters[label_right])
            right_lane.emplace_back(int(pt.x / scale), int(pt.y / scale));
    }

    void findPointCloud(const cv::Mat &image_thres,
                        cv::Mat &new_thres,
                        cv::Mat &depth_image,
                        int center_cam_x,
                        int center_cam_y,
                        const rs2_intrinsics &intrinsics)
    {
        new_thres = cv::Mat::zeros(new_thres.size(), CV_8U);

        // 1. Ekstrak titik dari mask dan convert ke PCL format 2D
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_2d(new pcl::PointCloud<pcl::PointXYZ>());

        for (int rows = 0; rows < image_thres.rows; rows += 2)
        {
            for (int cols = 0; cols < image_thres.cols; cols += 2)
            {
                if (image_thres.at<uint8_t>(rows, cols) > 0)
                {
                    pcl::PointXYZ point;
                    point.x = cols - center_cam_x;
                    point.y = center_cam_y - rows;
                    point.z = 0.0f; // 2D point, so Z = 0
                    point_cloud_2d->push_back(point);
                }
            }
        }

        if (point_cloud_2d->empty() || point_cloud_2d->size() < 5)
        {
            // logger.warn("No points found in the point cloud from image thresholding");
            return;
        }

        // voxel point_cloud_2d into 1 cm each
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_2d;
        voxel_grid_2d.setInputCloud(point_cloud_2d);
        voxel_grid_2d.setLeafSize(0.02f, 0.02f, 0.02f);
        voxel_grid_2d.filter(*point_cloud_2d);

        if (point_cloud_2d->empty() || point_cloud_2d->size() < 5)
        {
            // logger.warn("No points found in the point cloud from image thresholding");
            return;
        }

        float fx = intrinsics.fx;
        float fy = intrinsics.fy;
        float cx = intrinsics.ppx;
        float cy = intrinsics.ppy;

        if (fx <= 0 || fy <= 0 || cx <= 0 || cy <= 0)
        {
            // logger.error("Invalid camera intrinsics: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", fx, fy, cx, cy);
            return;
        }

        pcl::PointCloud<pcl::PointXYZ> point_cloud_3d;
        // Get transformation from 2D into 3D using standard RealSense axis convention
        for (const auto &point : *point_cloud_2d)
        {
            float pixel_x = static_cast<float>(point.x + center_cam_x);
            float pixel_y = static_cast<float>(center_cam_y - point.y);

            // Access aligned depth image to get actual depth value
            if (pixel_x < 0 || pixel_x >= depth_image.cols || pixel_y < 0 || pixel_y >= depth_image.rows)
                continue;

            uint16_t depth_value = depth_image.at<uint16_t>(static_cast<int>(pixel_y), static_cast<int>(pixel_x));
            if (depth_value == 0)
                continue;

            float depth_meters = depth_value * 0.001f; // Convert from mm to meters

            pcl::PointXYZ world_point;
            world_point.x = ((pixel_x - cx) * depth_meters) / fx;
            world_point.y = ((pixel_y - cy) * depth_meters) / fy;
            world_point.z = depth_meters;

            point_cloud_3d.push_back(world_point);
        }

        if (point_cloud_3d.empty() || point_cloud_3d.size() < 5)
        {
            // logger.warn("No 3D points found in the point cloud from image thresholding");
            return;
        }

        //! transform image_cloud_world_ptr to base link frame
        pcl::PointCloud<pcl::PointXYZ> points_camera2base;
        pcl_ros::transformPointCloud(point_cloud_3d, points_camera2base, tf_camera_base_);

        sensor_msgs::msg::PointCloud2 msg_filtered_camera_for_sign;
        pcl::toROSMsg(points_camera2base, msg_filtered_camera_for_sign);
        msg_filtered_camera_for_sign.header.stamp = sync_time_;
        msg_filtered_camera_for_sign.header.frame_id = "base_link"; // transformed frame
        pub_sign_points_->publish(msg_filtered_camera_for_sign);

        if (points_camera2base.empty() || points_camera2base.size() < 5)
        {
            // logger.warn("No points found in the point cloud after transformation to camera frame");
            return;
        }

        // 4. VoxelGrid
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setInputCloud(points_camera2base.makeShared());
        voxel_grid.setLeafSize(0.02f, 0.02f, 0.02f);
        voxel_grid.filter(points_camera2base); // <-- Diperbaiki: pakai dereference

        pcl::CropBox<pcl::PointXYZ> crop_box;
        crop_box.setInputCloud(points_camera2base.makeShared());
        crop_box.setMin(Eigen::Vector4f(0.05, -1.0, 0.05, 1.0)); // In front of robot
        crop_box.setMax(Eigen::Vector4f(2.0, 1.0, 0.1, 1.0));    // 2m ahead, ±1m wide, max 2m tall
        crop_box.setNegative(false);                             // Keep only points inside the box
        crop_box.filter(points_camera2base);

        if (points_camera2base.empty() || points_camera2base.size() < 5)
        {
            // logger.warn("No points found in the point cloud after transformation to camera frame");
            return;
        }

        pcl::PointCloud<pcl::PointXYZ> points_base2camera;
        pcl_ros::transformPointCloud(points_camera2base, points_base2camera, tf_base_camera_);

        if (points_base2camera.empty() || points_base2camera.size() < 5)
        {
            // logger.warn("No points found in the point cloud after transformation to base frame");
            return;
        }

        // 5. Proyeksikan hasil PCL ke BEV
        std::vector<cv::Point2f> bev_points;
        for (const auto &img_point : points_base2camera)
        {
            // Project 3D world point ke pixel kamera
            float point_3d[3] = {img_point.x, img_point.y, img_point.z};
            float pixel_proj[2];
            rs2_project_point_to_pixel(pixel_proj, &intrinsics, point_3d);

            cv::circle(new_thres, cv::Point2f(pixel_proj[0], pixel_proj[1]), 5, cv::Scalar(255), -1);
        }
    }

    std::vector<cv::Point> sliding_windows(const cv::Mat &binary_image, int width, int height,
                                           int min_pixels, cv::Mat &output_image)
    {
        std::vector<cv::Point> detected_points;
        cv::Scalar color;

        if (width > height)
            color = cv::Scalar(255, 0, 0);
        else
            color = cv::Scalar(0, 0, 255);

        // Create a sliding window
        cv::Rect window(0, 0, width, height);
        cv::Mat roi = binary_image(window);

        for (int row = 0; row < binary_image.rows; row += window.height)
        {
            for (int col = 0; col < binary_image.cols; col += window.width)
            {
                // Update the sliding window position

                window.x = col;
                window.y = row;

                // Ensure the window does not go out of bounds
                if (window.x + window.width > binary_image.cols)
                    window.width = binary_image.cols - window.x;
                if (window.y + window.height > binary_image.rows)
                    window.height = binary_image.rows - window.y;

                // Extract the region of interest
                roi = binary_image(window);

                // Check contours in the sliding window
                std::vector<std::vector<cv::Point>> contours;
                cv::findContours(roi, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

                // Remove small contours
                for (size_t i = 0; i < contours.size(); i++)
                {
                    if (cv::contourArea(contours[i]) < min_pixels)
                    {
                        contours.erase(contours.begin() + i);
                        i--;
                    }
                }

                // Draw centroid of each contour and store it
                for (const auto &contour : contours)
                {
                    cv::Moments m = cv::moments(contour);
                    int cx = int(m.m10 / m.m00) + col; // Adjust centroid position relative to the full image
                    int cy = int(m.m01 / m.m00) + row;

                    // Store the detected point
                    detected_points.emplace_back(cx, cy);

                    // Draw the centroid
                    // cv::circle(output_image, cv::Point(cx, cy), 2, color, -1);
                }

                // Draw the window
                // cv::rectangle(output_image, window, color, 1);
            }
        }

        return detected_points;
    }

    float computeAngle(const cv::Point &from, const cv::Point &to)
    {
        cv::Point2f vec(to.x - from.x, to.y - from.y);
        return std::atan2(vec.y, vec.x) * 180.0f / CV_PI - 90.0f;
    }

    cv::Point getCentroid(const std::vector<cv::Point> &contour)
    {
        cv::Moments m = cv::moments(contour);
        if (m.m00 == 0)
            return {-1, -1};
        return {static_cast<int>(m.m10 / m.m00), static_cast<int>(m.m01 / m.m00)};
    }

    /**
     * Gaussian membership function
     * @param x: input value
     * @param mean: center of the Gaussian
     * @param sigma: standard deviation (spread)
     * @return membership degree [0, 1]
     */
    float gaussianMF(float x, float mean, float sigma)
    {
        return exp(-0.5 * pow((x - mean) / sigma, 2));
    }

    /**
     * Curvature membership functions
     * @param curvature: input curvature value
     * @return [straight_fd, wiggle_fd, curve_fd]
     */
    std::vector<float> curvatureMF(float curvature)
    {
        float straight_fd = gaussianMF(curvature, straight_center_, straight_sigma_);
        float wiggle_fd = gaussianMF(curvature, wiggle_center_, wiggle_sigma_);
        float curve_fd = gaussianMF(curvature, curve_center_, curve_sigma_);

        return {straight_fd, wiggle_fd, curve_fd};
    }

    /**
     * Main fuzzy inference function
     * @param curvature: input curvature value [0, 35]
     * @return crisp speed output
     */
    float getSpeed(float curvature)
    {
        // Clamp input to valid range
        if (curvature < 0)
            curvature = 0;
        if (curvature > max_steering_deg_)
            curvature = max_steering_deg_;

        // Step 1: Fuzzification
        std::vector<float> mf_values = curvatureMF(curvature);
        float straight_fd = mf_values[0];
        float wiggle_fd = mf_values[1];
        float curve_fd = mf_values[2];

        // Step 2: Rule evaluation and aggregation
        float numerator = 0.0;
        float denominator = 0.0;

        // Rule 1: IF curvature is straight THEN speed is fast
        numerator += straight_fd * speed_straight_;
        denominator += straight_fd;

        // Rule 2: IF curvature is wiggle THEN speed is medium
        numerator += wiggle_fd * speed_wiggle_;
        denominator += wiggle_fd;

        // Rule 3: IF curvature is curve THEN speed is slow
        numerator += curve_fd * speed_curve_;
        denominator += curve_fd;

        // Step 3: Defuzzification (weighted average)
        float crisp_speed = (denominator > 0) ? (numerator / denominator) : 0.0;

        return crisp_speed;
    }

    float getLookAhead(float curvature)
    {
        // Clamp input to valid range
        if (curvature < 0)
            curvature = 0;
        if (curvature > max_steering_deg_)
            curvature = max_steering_deg_;

        // Step 1: Fuzzification
        std::vector<float> mf_values = curvatureMF(curvature);
        float straight_fd = mf_values[0];
        float wiggle_fd = mf_values[1];
        float curve_fd = mf_values[2];

        // Step 2: Rule evaluation and aggregation
        float numerator = 0.0;
        float denominator = 0.0;

        // Rule 1: IF curvature is straight THEN speed is fast
        numerator += straight_fd * lookahead_straight_distance_;
        denominator += straight_fd;

        // Rule 2: IF curvature is wiggle THEN speed is medium
        numerator += wiggle_fd * lookahead_wiggle_distance_;
        denominator += wiggle_fd;

        // Rule 3: IF curvature is curve THEN speed is slow
        numerator += curve_fd * lookahead_curve_distance_;
        denominator += curve_fd;

        // Step 3: Defuzzification (weighted average)
        float crisp_speed = (denominator > 0) ? (numerator / denominator) : 0.0;

        return crisp_speed;
    }

    // Fungsi ray casting
    cv::Point2f scan_line(cv::Point2f start, float angle_deg, const cv::Mat &mask)
    {
        float angle_rad = angle_deg * CV_PI / 180.0f;
        float dx = std::cos(angle_rad);
        float dy = std::sin(angle_rad);
        float x = start.x, y = start.y;

        for (int step = 0; step < 100; ++step)
        { // 100 pixel max
            int ix = cvRound(x);
            int iy = cvRound(y);
            if (ix < 0 || iy < 0 || ix >= mask.cols || iy >= mask.rows)
                break;
            if (mask.at<uchar>(iy, ix) == 0)
                break;
            x += dx;
            y += dy;
        }
        return cv::Point2f(x, y);
    }

    std::vector<cv::Point2f> guided_center_scan(
        cv::Point2f start,
        float theta_init,
        float offset,
        const cv::Mat &mask)
    {
        std::vector<cv::Point2f> centers;
        cv::Point2f curr = start;
        float theta = theta_init;

        for (int i = 0; i < 500; ++i)
        { // batasi max step
            // Scan kiri & kanan
            cv::Point2f left = scan_line(curr, theta - offset, mask);
            cv::Point2f right = scan_line(curr, theta + offset, mask);
            // Center
            cv::Point2f center((left.x + right.x) / 2.0f, (left.y + right.y) / 2.0f);
            centers.push_back(center);
            // Update arah
            float dtheta = std::atan2(center.y - curr.y, center.x - curr.x);
            theta = dtheta * 180.0f / CV_PI;
            // Next
            curr = center;
            // Stop condition
            int cx = cvRound(center.x);
            int cy = cvRound(center.y);
            if (cx < 0 || cy < 0 || cx >= mask.cols || cy >= mask.rows)
                break;
            if (mask.at<uchar>(cy, cx) == 0)
                break;
        }
        return centers;
    }

    double speed_to_lookahead(double x)
    {
        if (x < 0.9)
            x = 0.9;
        if (x > 0.9)
            x = 0.9;

        double terms[] = {
            -3.0935334416666588e-001,
            5.7330001149999921e-001};

        double t = 1;
        double r = 0;
        for (double c : terms)
        {
            r += c * t;
            t *= x;
        }

        if (r < lookahead_near_meter_)
            r = lookahead_near_meter_;

        return r;
    }

    double offset_speed_to_dist(double x)
    {
        double terms[] = {
            7.9999999999988969e-003,
            2.5628571428571562e-001,
            8.8571428571428204e-002};

        size_t csz = sizeof terms / sizeof *terms;

        double t = 1;
        double r = 0;
        for (int i = 0; i < csz; i++)
        {
            r += terms[i] * t;
            t *= x;
        }
        return 0;
    }

    void reset_state()
    {
        used_lane_ = origin_used_lane_;
        prev_used_lane = origin_used_lane_;
        used_reference_ = DASHED_REFERENCE;
        ada_belokan_ = 0;
        ada_obs_ = 0;
        ada_obs_kanan_ = 0;
        ada_obs_lane_dalam_ = 0;
        ada_obs_slow_down_ = 0;
        prev_ada_obs_ = 0;
        perpindahan_lane_ = 0;
        prev_perpindahan_lane_ = 0;
        pos_ada_belokan = 0;
        jarak_hindar = 0;
        counter_switch_lane = 0;
        out_duration = 2000;
        first_enc_see_obs = 0;
        dist_to_robot_meter = 0;
        dist_to_robot = 0;
        jarak_actual_hindar = 0;
        pos_enc_hindar = 0;
        awal_center_lane = 0;
        keluar_enc_ = 0;
    }

    void setup_signal_handlers()
    {
        std::signal(SIGINT, [](int)
                    {
            RCLCPP_INFO(rclcpp::get_logger("vision_capture4"), "Received SIGINT, shutting down gracefully...");
            rclcpp::shutdown(); });

        std::signal(SIGTERM, [](int)
                    {
            RCLCPP_INFO(rclcpp::get_logger("vision_capture4"), "Received SIGTERM, shutting down gracefully...");
            rclcpp::shutdown(); });
    }

    void cleanup_realsense()
    {
        shutdown_requested_ = true;
        logger.info("Cleaning up RealSense resources...");
        std::lock_guard<std::mutex> lock(pipeline_mutex_);

        if (pipeline_started_)
        {
            try
            {
                logger.info("Stopping RealSense pipeline...");
                pipe_.stop();
                pipeline_started_ = false;
                logger.info("RealSense pipeline stopped successfully");
            }
            catch (const rs2::error &e)
            {
                logger.error("Error stopping RealSense pipeline: %s", e.what());
            }
            catch (const std::exception &e)
            {
                logger.error("Unexpected error stopping RealSense pipeline: %s", e.what());
            }
        }
    }

    void check_error(rs2_error *e)
    {
        if (e)
        {
            fprintf(stderr, "RealSense error calling %s(%s):\n    %s\n",
                    rs2_get_failed_function(e),
                    rs2_get_failed_args(e),
                    rs2_get_error_message(e));
            rs2_free_error(e);
            exit(EXIT_FAILURE);
        }
    }

    void print_device_info(const rs2::device &dev)
    {
        // Query all sensors (depth, color, motion, etc.)
        for (rs2::sensor sensor : dev.query_sensors())
        {
            std::cout << "\nSensor: " << sensor.get_info(RS2_CAMERA_INFO_NAME) << std::endl;

            std::vector<rs2::stream_profile> profiles = sensor.get_stream_profiles();

            for (const rs2::stream_profile &profile : profiles)
            {
                int fps = profile.fps();
                if (fps < 60)
                    continue;

                if (profile.is<rs2::video_stream_profile>())
                {
                    auto vprof = profile.as<rs2::video_stream_profile>();
                    std::cout << "  Stream: " << rs2_stream_to_string(vprof.stream_type())
                              << " " << vprof.width() << "x" << vprof.height()
                              << " @ " << vprof.fps()
                              << " Format: " << rs2_format_to_string(vprof.format())
                              << std::endl;
                }
                else
                {
                    std::cout << "  Stream: " << rs2_stream_to_string(profile.stream_type())
                              << " @ " << profile.fps()
                              << " Format: " << rs2_format_to_string(profile.format())
                              << std::endl;
                }
            }
            printf("\n");
        }

        std::cout << "\nUsing device: " << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
        std::cout << "    Serial number: " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
        std::cout << "    Firmware version: " << dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << std::endl
                  << std::endl;
    }

    void loadConfig()
    {
        logger.info("Loading configuration from YAML file...");

        char config_file[100];
        sprintf(config_file, "/home/iris/Fira/46-49-52-41/dynamic_conf_vision.yaml");
        // sprintf(config_file, "/home/her/Documents/robot/FIRA25/dynamic_conf_vision.yaml");

        YAML::Node config = YAML::LoadFile(config_file);

        for (int i = 0; i < controlbox_size; ++i)
            controlbox_data[i] = config["Config"]["slider_" + std::to_string(i + 1)].as<int>();

        kp_steering_ = config["Config"]["kp_steering"].as<float>();
        ki_steering_ = config["Config"]["ki_steering"].as<float>();
        kd_steering_ = config["Config"]["kd_steering"].as<float>();
        // lookahead_far_meter_ = config["Config"]["lookahead_far_meter"].as<float>();
        // lookahead_near_meter_ = config["Config"]["lookahead_near_meter"].as<float>();
        meter_to_pixel_ = config["Config"]["meter_to_pixel"].as<float>();
        wheelbase_ = config["Config"]["wheelbase"].as<float>();
        max_steering_deg_ = config["Config"]["max_steering_deg"].as<float>();
        line_length_min_ = config["Config"]["line_length_min"].as<float>();
        line_length_max_ = config["Config"]["line_length_max"].as<float>();
        line_length_edge_min_ = config["Config"]["line_length_edge_min"].as<float>();
        line_length_edge_max_ = config["Config"]["line_length_edge_max"].as<float>();
        speed_straight_ = config["Config"]["speed_straight"].as<float>();
        speed_wiggle_ = config["Config"]["speed_wiggle"].as<float>();
        speed_curve_ = config["Config"]["speed_curve"].as<float>();
        lookahead_straight_distance_ = config["Config"]["lookahead_straight_distance"].as<float>();
        lookahead_wiggle_distance_ = config["Config"]["lookahead_wiggle_distance"].as<float>();
        lookahead_curve_distance_ = config["Config"]["lookahead_curve_distance"].as<float>();
        // used_lane_ = config["Config"]["used_lane"].as<int>();
        valid_center_left_ = config["Config"]["valid_center_left"].as<int>();
        valid_center_right_ = config["Config"]["valid_center_right"].as<int>();
        valid_up_ = config["Config"]["valid_up"].as<int>();
        valid_down_ = config["Config"]["valid_down"].as<int>();
        cropping_distance_ = config["Config"]["cropping_distance"].as<int>();
        jarak_hindar_meter_ = config["Config"]["jarak_hindar_meter"].as<float>();
        out_duration_belokan_ = config["Config"]["out_duration_belokan"].as<float>();
        out_duration_normal_ = config["Config"]["out_duration_normal"].as<float>();
        offset_out_duration_center_ = config["Config"]["offset_out_duration_center"].as<float>();
        max_enc_meter_obs_ = config["Config"]["max_enc_meter_obs"].as<float>();
        max_enc_meter_obs_center_ = config["Config"]["max_enc_meter_obs_center"].as<float>();
        min_dist_jarak_hindar_ = config["Config"]["min_dist_jarak_hindar"].as<float>();
        min_dist_jarak_keluar_ = config["Config"]["min_dist_jarak_keluar"].as<float>();
        constant_speed_belok_ = config["Config"]["constant_speed_belok"].as<float>();
        dashed_filter_area_ = config["Config"]["dashed_filter_area"].as<int16_t>();
        segment_speed_1 = config["Config"]["segment_speed_1"].as<float>();
        segment_speed_2 = config["Config"]["segment_speed_2"].as<float>();
        segment_speed_3 = config["Config"]["segment_speed_3"].as<float>();
        segment_speed_4 = config["Config"]["segment_speed_4"].as<float>();
        road_segment_threshold_area = config["Config"]["road_segment_threshold_area"].as<int16_t>();
        constant_transient_speed = config["Config"]["constant_transient_speed"].as<float>();
        scaller_speed = config["Config"]["scaller_speed"].as<float>();
        offset_kiri = config["Config"]["offset_kiri"].as<float>();

        dash_kiri_default = config["Config"]["dash_kiri_default"].as<float>();
        dash_kanan_default = config["Config"]["dash_kanan_default"].as<float>();
        dash_kiri_anomali = config["Config"]["dash_kiri_anomali"].as<float>();
        dash_kanan_anomali = config["Config"]["dash_kanan_anomali"].as<float>();
        edge_kiri_default = config["Config"]["edge_kiri_default"].as<float>();
        edge_kanan_default = config["Config"]["edge_kanan_default"].as<float>();
        edge_kiri_anomali = config["Config"]["edge_kiri_anomali"].as<float>();
        edge_kanan_anomali = config["Config"]["edge_kanan_anomali"].as<float>();

        lookahead_far_meter_ = controlbox_data[12] * 0.00196;  // Convert from cm to m
        lookahead_near_meter_ = controlbox_data[13] * 0.00196; // Convert from cm to m

        lookahead_far_pixel_ = static_cast<int>(lookahead_far_meter_ * meter_to_pixel_);
        lookahead_near_pixel_ = static_cast<int>(lookahead_near_meter_ * meter_to_pixel_);

        logger.info("YAML configuration loaded successfully.");

        // publish the controlbox data
        std_msgs::msg::Int16MultiArray controlbox_msg;
        controlbox_msg.data.resize(controlbox_size);
        for (int i = 0; i < controlbox_size; ++i)
            controlbox_msg.data[i] = controlbox_data[i];
        pub_controlbox_->publish(controlbox_msg);

        std_msgs::msg::Float32MultiArray config_vision_msg;
        config_vision_msg.data = {
            3.0354,
            kp_steering_,
            ki_steering_,
            kd_steering_,
            lookahead_far_meter_,
            lookahead_near_meter_,
            meter_to_pixel_,
            wheelbase_,
            max_steering_deg_,
            line_length_min_,
            line_length_max_,
            line_length_edge_min_,
            line_length_edge_max_,
            speed_straight_,
            speed_wiggle_,
            speed_curve_,
            lookahead_straight_distance_,
            lookahead_wiggle_distance_,
            lookahead_curve_distance_,
            static_cast<float>(used_lane_),
            static_cast<float>(valid_center_left_),
            static_cast<float>(valid_center_right_),
            static_cast<float>(valid_up_),
            static_cast<float>(valid_down_),
            static_cast<float>(cropping_distance_),
            999.0f,
            jarak_hindar_meter_,
            out_duration_belokan_,
            out_duration_normal_,
            offset_out_duration_center_,
            max_enc_meter_obs_,
            max_enc_meter_obs_center_,
            min_dist_jarak_hindar_,
            min_dist_jarak_keluar_,
            constant_speed_belok_,
            dashed_filter_area_,
            segment_speed_1,
            segment_speed_2,
            segment_speed_3,
            segment_speed_4,
            road_segment_threshold_area,
            constant_transient_speed,
            scaller_speed,
            offset_kiri,
            dash_kiri_default,
            dash_kanan_default,
            dash_kiri_anomali,
            dash_kanan_anomali,
            edge_kiri_default,
            edge_kanan_default,
            edge_kiri_anomali,
            edge_kanan_anomali

        };
        pub_config_vision_->publish(config_vision_msg);
    }

    void saveConfig()
    {
        logger.info("Saving configuration to YAML file...");

        char config_file[100];
        sprintf(config_file, "/home/iris/Fira/46-49-52-41/dynamic_conf_vision.yaml");
        // sprintf(config_file, "/home/her/Documents/robot/FIRA25/dynamic_conf_vision.yaml");

        YAML::Node config;
        config["Config"]["slider_size"] = controlbox_size;
        for (size_t i = 0; i < controlbox_size; ++i)
            config["Config"]["slider_" + std::to_string(i + 1)] = controlbox_data[i];

        lookahead_far_meter_ = controlbox_data[12] * 0.00196;  // Convert from cm to m
        lookahead_near_meter_ = controlbox_data[13] * 0.00196; // Convert from cm to m

        config["Config"]["kp_steering"] = kp_steering_;
        config["Config"]["ki_steering"] = ki_steering_;
        config["Config"]["kd_steering"] = kd_steering_;
        config["Config"]["lookahead_far_meter"] = lookahead_far_meter_;
        config["Config"]["lookahead_near_meter"] = lookahead_near_meter_;
        config["Config"]["meter_to_pixel"] = meter_to_pixel_;
        config["Config"]["wheelbase"] = wheelbase_;
        config["Config"]["max_steering_deg"] = max_steering_deg_;
        config["Config"]["line_length_min"] = line_length_min_;
        config["Config"]["line_length_max"] = line_length_max_;
        config["Config"]["line_length_edge_min"] = line_length_edge_min_;
        config["Config"]["line_length_edge_max"] = line_length_edge_max_;
        config["Config"]["speed_straight"] = speed_straight_;
        config["Config"]["speed_wiggle"] = speed_wiggle_;
        config["Config"]["speed_curve"] = speed_curve_;
        config["Config"]["lookahead_straight_distance"] = lookahead_straight_distance_;
        config["Config"]["lookahead_wiggle_distance"] = lookahead_wiggle_distance_;
        config["Config"]["lookahead_curve_distance"] = lookahead_curve_distance_;
        config["Config"]["used_lane"] = used_lane_;
        config["Config"]["valid_center_left"] = valid_center_left_;
        config["Config"]["valid_center_right"] = valid_center_right_;
        config["Config"]["valid_up"] = valid_up_;
        config["Config"]["valid_down"] = valid_down_;
        config["Config"]["cropping_distance"] = cropping_distance_;
        config["Config"]["jarak_hindar_meter"] = jarak_hindar_meter_;
        config["Config"]["out_duration_belokan"] = out_duration_belokan_;
        config["Config"]["out_duration_normal"] = out_duration_normal_;
        config["Config"]["offset_out_duration_center"] = offset_out_duration_center_;
        config["Config"]["max_enc_meter_obs"] = max_enc_meter_obs_;
        config["Config"]["max_enc_meter_obs_center"] = max_enc_meter_obs_center_;
        config["Config"]["min_dist_jarak_hindar"] = min_dist_jarak_hindar_;
        config["Config"]["min_dist_jarak_keluar"] = min_dist_jarak_keluar_;
        config["Config"]["constant_speed_belok"] = constant_speed_belok_;
        config["Config"]["dashed_filter_area"] = dashed_filter_area_;
        config["Config"]["segment_speed_1"] = segment_speed_1;
        config["Config"]["segment_speed_2"] = segment_speed_2;
        config["Config"]["segment_speed_3"] = segment_speed_3;
        config["Config"]["segment_speed_4"] = segment_speed_4;
        config["Config"]["road_segment_threshold_area"] = road_segment_threshold_area;
        config["Config"]["constant_transient_speed"] = constant_transient_speed;
        config["Config"]["scaller_speed"] = scaller_speed;
        config["Config"]["offset_kiri"] = offset_kiri;
        config["Config"]["dash_kiri_default"] = dash_kiri_default;
        config["Config"]["dash_kanan_default"] = dash_kanan_default;
        config["Config"]["dash_kiri_anomali"] = dash_kiri_anomali;
        config["Config"]["dash_kanan_anomali"] = dash_kanan_anomali;
        config["Config"]["edge_kiri_default"] = edge_kiri_default;
        config["Config"]["edge_kanan_default"] = edge_kanan_default;
        config["Config"]["edge_kiri_anomali"] = edge_kiri_anomali;
        config["Config"]["edge_kanan_anomali"] = edge_kanan_anomali;
        // Save the configuration to file

        lookahead_far_pixel_ = static_cast<int>(lookahead_far_meter_ * meter_to_pixel_);
        lookahead_near_pixel_ = static_cast<int>(lookahead_near_meter_ * meter_to_pixel_);

        std::ofstream fout(config_file);
        fout << config;
        fout.close();

        logger.info("YAML configuration saved successfully.");
    }
};
#endif // VISION_CAPTURE4_HPP