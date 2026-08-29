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
#include <string>
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

// extern "C" {
// #include "apriltag/apriltag.h"
// #include "apriltag/apriltag_pose.h"
// #include "apriltag/common/getopt.h"
// #include "apriltag/tag16h5.h"
// #include "apriltag/tag25h9.h"
// #include "apriltag/tag36h11.h"
// #include "apriltag/tagCircle21h7.h"
// #include "apriltag/tagCircle49h12.h"
// #include "apriltag/tagCustom48h12.h"
// #include "apriltag/tagStandard41h12.h"
// #include "apriltag/tagStandard52h13.h"
// }

// ROS 2 Libraries
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_interface/msg/vision_urban.hpp"
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
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int8.hpp"
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

enum TrafficSign
{
    NO_ENTRY = 0,
    DEAD_END = 1,
    RIGHT = 2,
    LEFT = 3,
    FORWARD = 4,
    STOP = 5
};

#define DASHED_REFERENCE 0
#define EDGE_REFERENCE 1

#define LEFT_LANE 0
#define RIGHT_LANE 1

typedef pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
typedef point_cloud::Ptr cloud_pointer;

class VisionCapture3 : public rclcpp::Node
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

    // -------------------------------------------------
    // Timer Callback Group
    // -------------------------------------------------
    rclcpp::CallbackGroup::SharedPtr tim_routine_group_;
    // -------------------------------------------------
    // Subscribers
    // -------------------------------------------------
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sub_controlbox_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_vision_config_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_sign_picture_id_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_initial_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_used_threshold_;
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
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_config_master_;
    rclcpp::Publisher<ros2_interface::msg::VisionUrban>::SharedPtr pub_vision_urban_;

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
    std::mutex point_cloud_mutex_;

    uint8_t img_sync_ = 0;
    uint8_t pointcloud_sync_ = 0;

    int16_t center_cam_x_ = 0;
    int16_t center_cam_y_ = 0;

    int sign_id_ = 0;

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

    float line_length_min_ = 30.0f; // Minimum length of the dashed line segments
    float line_length_max_ = 45.0f;

    float line_length_edge_min_ = 30.0f; // Minimum length of the edge line segments
    float line_length_edge_max_ = 45.0f;

    uint8_t used_lane_ = 0;
    int8_t jalan_berkelok_ = 0; // 0: tidak berkelok, 1: berkelok kanan, 2: berkelok kiri
    int8_t mask_jalan_bocor_ = 0;

    int16_t used_threshold_ = 1;

    float kp_steering_ = 1.0;
    float ki_steering_ = 0.0;
    float kd_steering_ = 0.0;

    int valid_center_left_ = 0;
    int valid_center_right_ = 0;
    int valid_up_ = 0;
    int valid_down_ = 0;
    int cropping_distance_ = 0;

    float length_titik_putih_ = 60.0f;
    float length_titik_hitam_ = 80.0f;

    uint8_t record_vision_ = 0;
    std::string record_path_ = "/tmp/vision_recording";

    int8_t controlbox_size = 19;
    int16_t controlbox_data[50];

    float derajat_steering_kanan_ = 0;
    float derajat_steering_kiri_ = 0;

    float encoder_belok_kanan_ = 0;
    float encoder_belok_kiri_ = 0;

    float encoder_maju_kanan_ = 0;
    float encoder_maju_kiri_ = 0;
    float encoder_maju_lurus_ = 0;

    float encoder_maju_dead_end_ = 0.015; // Default value for dead end maneuver

    float jarak_ke_putih_ = 0;

    float derajat_gyro_kanan_ = 0;
    float derajat_gyro_kiri_ = 0;

    float jarak_ke_zebracros_ = 0;

    float min_jarak_putih_kanan_ = 0.3;
    float min_jarak_putih_kiri_ = 0.3;
    float min_jarak_putih_lurus_ = 0.3;

    float offset_jarak_sign_pole_ = 0.25; // Offset distance from sign pole
    float last_gyro_angle_ = 78.0f;
    float min_vel_belokan_ = 0.2;
    float jarak_ke_sign_pole_ = 0.25; // Offset distance to sign pole

    float velocity_jalan_otomatis = 0.315127;
    float cntr_jalan_lurus_ = 50;

    int final_sign_id = -1;

    rclcpp::Time sync_time_;

    // angle between robot position and closest point in look_ahead_far and look_ahead_near
    float angle_used_ = 0.0f;
    float angle_used_floodfill_ = 0.0f;
    float angle_used_kiri_ = 0.0f;
    float angle_used_kanan_ = 0.0f;
    float angle_used_edge_kiri_ = 0.0f;
    float angle_used_edge_kanan_ = 0.0f;
    float angle_used_edge_kiri_2_ = 0.0f;
    float angle_used_edge_kanan_2_ = 0.0f;
    float angle_used_edge_kiri_3_ = 0.0f;
    float angle_used_edge_kanan_3_ = 0.0f;

    // point cloud centroid
    Eigen::Vector4f sign_centroid_ = Eigen::Vector4f::Zero();

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

    uint8_t used_reference_ = 0;

    int bev_width = 400;
    int bev_height = 600;

    cv::Point robot_position_;
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

    // Video recording
    cv::VideoWriter video_writer_color_;
    cv::VideoWriter video_writer_bev_color_color_;

  public:
    VisionCapture3()
        : Node("vision_capture3"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }
        // --------------------------------------------------
        // Setup signal handlers for graceful shutdown
        // --------------------------------------------------
        setup_signal_handlers();
        // --------------------------------------------------
        // Get parameters
        // --------------------------------------------------

        this->declare_parameter<float>("lookahead_far_meter", 0.5);
        this->get_parameter("lookahead_far_meter", lookahead_far_meter_);
        this->declare_parameter<float>("lookahead_near_meter", 0.3);
        this->get_parameter("lookahead_near_meter", lookahead_near_meter_);
        this->declare_parameter<float>("meter_to_pixel", 226.0869565);
        this->get_parameter("meter_to_pixel", meter_to_pixel_);
        this->declare_parameter<float>("wheelbase", 0.27);
        this->get_parameter("wheelbase", wheelbase_);
        this->declare_parameter<float>("max_steering_deg", 35.0);
        this->get_parameter("max_steering_deg", max_steering_deg_);
        this->declare_parameter<float>("line_length_min", 30.0);
        this->get_parameter("line_length_min", line_length_min_);
        this->declare_parameter<float>("line_length_max", 45.0);
        this->get_parameter("line_length_max", line_length_max_);
        this->declare_parameter<float>("line_length_edge_min", 30.0);
        this->get_parameter("line_length_edge_min", line_length_edge_min_);
        this->declare_parameter<float>("line_length_edge_max", 45.0);
        this->get_parameter("line_length_edge_max", line_length_edge_max_);
        this->declare_parameter<float>("speed_straight", 1.2);
        this->get_parameter("speed_straight", speed_straight_);
        this->declare_parameter<float>("speed_wiggle", 0.95);
        this->get_parameter("speed_wiggle", speed_wiggle_);
        this->declare_parameter<float>("speed_curve", 0.7);
        this->get_parameter("speed_curve", speed_curve_);
        this->declare_parameter<float>("lookahead_straight", 150.0);
        this->get_parameter("lookahead_straight", lookahead_straight_distance_);
        this->declare_parameter<float>("lookahead_wiggle", 100.0);
        this->get_parameter("lookahead_wiggle", lookahead_wiggle_distance_);
        this->declare_parameter<float>("lookahead_curve", 50.0);
        this->get_parameter("lookahead_curve", lookahead_curve_distance_);
        this->declare_parameter<uint8_t>("used_lane", 1);
        this->get_parameter("used_lane", used_lane_);
        this->declare_parameter<float>("kp_steering", 1.0);
        this->get_parameter("kp_steering", kp_steering_);
        this->declare_parameter<float>("ki_steering", 0.0);
        this->get_parameter("ki_steering", ki_steering_);
        this->declare_parameter<float>("kd_steering", 0.0);
        this->get_parameter("kd_steering", kd_steering_);
        this->declare_parameter<int>("valid_center_left", 0);
        this->get_parameter("valid_center_left", valid_center_left_);
        this->declare_parameter<int>("valid_center_right", 0);
        this->get_parameter("valid_center_right", valid_center_right_);
        this->declare_parameter<int>("valid_up", 0);
        this->get_parameter("valid_up", valid_up_);
        this->declare_parameter<int>("valid_down", 0);
        this->get_parameter("valid_down", valid_down_);
        this->declare_parameter<int>("cropping_distance", 0);
        this->get_parameter("cropping_distance", cropping_distance_);
        this->declare_parameter<uint8_t>("record_vision", 0);
        this->get_parameter("record_vision", record_vision_);

        lookahead_far_pixel_ = static_cast<int>(lookahead_far_meter_ * meter_to_pixel_);
        lookahead_near_pixel_ = static_cast<int>(lookahead_near_meter_ * meter_to_pixel_);

        logger.info("Lookahead %.2f m (%d px) far, %.2f m (%d px) near",
                    lookahead_far_meter_, lookahead_far_pixel_,
                    lookahead_near_meter_, lookahead_near_pixel_);

        // --------------------------------------------------
        // Initialize RealSense context and check for devices
        // --------------------------------------------------
        rs2::context ctx;
        rs2::device_list devices = ctx.query_devices();
        if (devices.size() == 0)
        {
            std::cerr << "No RealSense devices found." << std::endl;
            return;
        }

        for (const auto &dev : devices)
            print_device_info(dev);
        // --------------------------------------------------
        // Configure and start RealSense pipeline
        // --------------------------------------------------
        try
        {
            cfg_.enable_stream(RS2_STREAM_COLOR, 640, 360, RS2_FORMAT_BGR8, 60);
            cfg_.enable_stream(RS2_STREAM_DEPTH, 640, 360, RS2_FORMAT_Z16, 60);

            pipe_.start(cfg_);

            auto device = pipe_.get_active_profile().get_device();
            auto color_sensor = device.first<rs2::color_sensor>();
            if (color_sensor)
            {
                // First disable auto white balance
                // color_sensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, 0.0f);

                // Wait a moment for the setting to take effect
                // std::this_thread::sleep_for(std::chrono::milliseconds(100));
                // Set all camera options to default values
                // color_sensor.set_option(RS2_OPTION_BACKLIGHT_COMPENSATION, 0.0f);
                // color_sensor.set_option(RS2_OPTION_BRIGHTNESS, 0.0f);
                // color_sensor.set_option(RS2_OPTION_CONTRAST, 50.0f);
                // color_sensor.set_option(RS2_OPTION_EXPOSURE, 166.0f);
                // color_sensor.set_option(RS2_OPTION_GAIN, 64.0f);
                // color_sensor.set_option(RS2_OPTION_GAMMA, 300.0f);
                // color_sensor.set_option(RS2_OPTION_HUE, 0.0f);
                // color_sensor.set_option(RS2_OPTION_SATURATION, 64.0f);
                // color_sensor.set_option(RS2_OPTION_SHARPNESS, 50.0f);
                // color_sensor.set_option(RS2_OPTION_WHITE_BALANCE, 4600.0f);
                color_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1.0f);
                color_sensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, 1.0f);
                // color_sensor.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, 16.0f);
                // color_sensor.set_option(RS2_OPTION_POWER_LINE_FREQUENCY, 3.0f);
                // color_sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_PRIORITY, 0.0f);
                // color_sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1.0f);

                // Verify the settings
                // logger.info("RealSense camera settings set to defaults:");
                // logger.info("Backlight Compensation: %.2f", color_sensor.get_option(RS2_OPTION_BACKLIGHT_COMPENSATION));
                // logger.info("Brightness: %.2f", color_sensor.get_option(RS2_OPTION_BRIGHTNESS));
                // logger.info("Contrast: %.2f", color_sensor.get_option(RS2_OPTION_CONTRAST));
                // logger.info("Exposure: %.2f", color_sensor.get_option(RS2_OPTION_EXPOSURE));
                // logger.info("Gain: %.2f", color_sensor.get_option(RS2_OPTION_GAIN));
                // logger.info("Gamma: %.2f", color_sensor.get_option(RS2_OPTION_GAMMA));
                // logger.info("Hue: %.2f", color_sensor.get_option(RS2_OPTION_HUE));
                // logger.info("Saturation: %.2f", color_sensor.get_option(RS2_OPTION_SATURATION));
                // logger.info("Sharpness: %.2f", color_sensor.get_option(RS2_OPTION_SHARPNESS));
                // logger.info("White Balance: %.2f", color_sensor.get_option(RS2_OPTION_WHITE_BALANCE));
                logger.info("Auto Exposure: %s", color_sensor.get_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE) ? "Enabled" : "Disabled");
                logger.info("Auto White Balance: %s", color_sensor.get_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE) ? "Enabled" : "Disabled");
            }

            pipeline_started_ = true;

            logger.info("RealSense pipeline started successfully");
        }
        catch (const rs2::error &e)
        {
            logger.error("Failed to start RealSense pipeline: %s", e.what());
            return;
        }
        // --------------------------------------------------
        // Create subscribers callback groups
        // --------------------------------------------------

        // --------------------------------------------------
        // Create timer callback groups
        // @param MutuallyExclusive: Ensures that only one callback from this group can run at a time
        // @param Reentrant: Allows multiple callbacks from this group to run concurrently
        // --------------------------------------------------
        tim_routine_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        // --------------------------------------------------
        // Create subscriber options and assign callback groups
        // --------------------------------------------------

        // --------------------------------------------------
        // Create subscribers
        // --------------------------------------------------
        sub_controlbox_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "/web/slider", 1, std::bind(&VisionCapture3::callback_sub_controlbox, this, std::placeholders::_1));
        sub_initial_ = this->create_subscription<std_msgs::msg::Int8>(
            "/web/initial", 1, std::bind(&VisionCapture3::callback_sub_initial, this, std::placeholders::_1));
        sub_vision_config_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/web/vision/configuration", 1, std::bind(&VisionCapture3::callback_sub_vision_config, this, std::placeholders::_1));
        sub_sign_picture_id_ = this->create_subscription<std_msgs::msg::Int32>(
            "/sign/picture/id", 1, std::bind(&VisionCapture3::callback_sub_sign_picture_id, this, std::placeholders::_1));
        sub_used_threshold_ = this->create_subscription<std_msgs::msg::Int16>(
            "/web/used_threshold", 1, std::bind(&VisionCapture3::callback_sub_used_threshold, this, std::placeholders::_1));
        // --------------------------------------------------
        // Create publishers
        // --------------------------------------------------
        pub_color_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/vision/color_image", 10);
        pub_color_hull_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/vision/color_hull_image", 10);
        pub_depth_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/vision/depth_image", 10);
        pub_camera_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "/vision/camera_info", 10);
        pub_color_depth_overlay_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/vision/color_depth_overlay", 1);
        pub_filtered_binary_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/vision/filtered_binary", 1);
        pub_road_binary_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/vision/road_binary", 1);
        pub_debug_binary_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/vision/debug_binary", 1);
        pub_debug_binary_2_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/vision/debug_binary2", 1);
        pub_debug_binary_3_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/vision/debug_binary3", 1);
        pub_pointcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/vision/pointcloud", 1);
        pub_filtered_points_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/vision/filtered_points", 1);
        pub_sign_points_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/vision/sign_points", 1);
        pub_imagecloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/vision/imagecloud", 1);
        pub_cleaned_pointcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/vision/cleaned_pointcloud", 1);
        pub_yuv_pointcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/vision/yuv_pointcloud", 1);
        pub_laserscan_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/vision/laserscan", 1);
        pub_slope_ = this->create_publisher<std_msgs::msg::Float32>(
            "/vision/slope", 1);
        pub_target_velocity_ = this->create_publisher<std_msgs::msg::Float32>(
            "/vision/velocity", 1);
        pub_controlbox_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
            "/vision/controlbox", 1);
        pub_config_vision_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/vision/web/configuration_init", 1);
        pub_config_master_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/vision/master_config", 1);
        pub_vision_urban_ = this->create_publisher<ros2_interface::msg::VisionUrban>(
            "/vision/urban_data", 1);

        // --------------------------------------------------
        // Create timer
        // --------------------------------------------------
        timer_routine_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&VisionCapture3::callback_tim_routine, this),
            tim_routine_group_);
        timer_img_routine_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&VisionCapture3::callback_tim_img_routine, this),
            tim_routine_group_);
        // timer_pointcloud_routine_ = this->create_wall_timer(
        //     std::chrono::milliseconds(1),
        //     std::bind(&VisionCapture3::callback_tim_pointcloud_routine, this),
        //     tim_routine_group_);
        // --------------------------------------------------
        // Wait for TF to be initialized
        // --------------------------------------------------
        while (!tf_is_initialized)
        {
            rclcpp::sleep_for(std::chrono::seconds(1));
            try
            {
                tf_camera_base_ = tf_buffer_.lookupTransform("base_link", "camera_color_optical_frame", tf2::TimePointZero);
                tf_base_camera_ = tf_buffer_.lookupTransform("camera_color_optical_frame", "base_link", tf2::TimePointZero);
                tf_is_initialized = true;
            }
            catch (const tf2::TransformException &ex)
            {
                logger.warn("TF not ready: %s", ex.what());
                rclcpp::sleep_for(std::chrono::milliseconds(100));
            }
        }

        logger.info("VisionCapture3 node initialized with multithreading");

        loadConfig(); //
    }

    ~VisionCapture3()
    {
        cleanup_realsense();
    }

  private:
    void callback_tim_img_routine()
    {
        cv::Mat color_image_raw;
        cv::Mat bev_color_image_raw;
        // ==================================================================
        //                    AVOID EXECUTION IF NOT NEEDED
        // ==================================================================
        if (img_sync_ == 0)
            return;
        // ==================================================================
        //                        DEBUG VISION CAPTURE
        // ==================================================================
        double start_time = this->now().seconds();
        static double last_time = start_time;
        double elapsed_time = start_time - last_time;
        last_time = start_time;
        // logger.info("Timer img routine elapsed time: %.4f seconds -> %.2f hz", elapsed_time, 1.0 / elapsed_time);
        // ==================================================================
        cv::Mat color_image;
        cv::Mat depth_image;
        rs2_intrinsics intrinsics;

        // Get the latest images from the RealSense camera
        {
            std::lock_guard<std::mutex> lock(image_mutex_);

            if (color_image_.empty() || depth_image_.empty())
            {
                logger.warn("Images not yet available, skipping overlay generation");
                return;
            }

            color_image = color_image_.clone();
            depth_image = depth_image_.clone();
            intrinsics = color_intrinsics_;
            // logger.info("==> Retrieved color and depth images from global variables.");

            img_sync_ = 0;
        }

        //?==================================================================
        //?                     PROCESSING IMAGES
        //?==================================================================
        color_image_raw = color_image.clone();

        // Adjust contrast and brightness

        //? ==================================================
        //?                Prepocess Image
        //? ==================================================
        cv::Mat frame_matrix(3, 3, CV_64F);
        cv::Mat frame_coeffs(1, 5, CV_64F);
        // Fill the camera matrix with intrinsics
        frame_matrix.at<double>(0, 0) = intrinsics.fx;
        frame_matrix.at<double>(0, 2) = intrinsics.ppx;
        frame_matrix.at<double>(1, 1) = intrinsics.fy;
        frame_matrix.at<double>(1, 2) = intrinsics.ppy;
        frame_matrix.at<double>(2, 2) = 1.0;
        // Fill the distortion coefficients
        frame_coeffs.at<double>(0) = intrinsics.coeffs[0]; // k1
        frame_coeffs.at<double>(1) = intrinsics.coeffs[1]; // k2
        frame_coeffs.at<double>(2) = intrinsics.coeffs[2]; // p1
        frame_coeffs.at<double>(3) = intrinsics.coeffs[3]; // p2
        frame_coeffs.at<double>(4) = intrinsics.coeffs[4]; // k3

        center_cam_x_ = color_image.cols / 2;
        center_cam_y_ = color_image.rows - 1;

        //? ==================================================
        //?                 Process Image
        //? ==================================================

        // Transform points in 3d to matrix image using project realsense function
        // Define ground plane corners in world coordinates (base_link frame)
        // These points define a rectangular area on the ground that will be transformed to BEV
        float ground_width = 1.0f;  // 1 meter wide (-0.5 to +0.5)
        float ground_length = 2.0f; // 2 meters deep (0.5 to 2.5)
        float ground_height = 0.0f; // Ground level

        // 3D points defining the ground plane rectangle in base_link coordinates
        float point_3d_left_bottom[3] = {0.5f, -ground_width / 2, ground_height};              // Close left
        float point_3d_right_bottom[3] = {0.5f, ground_width / 2, ground_height};              // Close right
        float point_3d_left_top[3] = {0.5f + ground_length, -ground_width / 2, ground_height}; // Far left
        float point_3d_right_top[3] = {0.5f + ground_length, ground_width / 2, ground_height}; // Far right

        float point_3d_look_ahead_far[3] = {1.0f, 0.0f, ground_height};  // Look ahead point
        float point_3d_look_ahead_near[3] = {0.5f, 0.0f, ground_height}; // Look ahead point

        float camera_point_left_bottom[3], camera_point_right_bottom[3];
        float camera_point_left_top[3], camera_point_right_top[3];

        float camera_point_look_ahead_far[3];
        float camera_point_look_ahead_near[3];

        transform_point(point_3d_left_bottom, camera_point_left_bottom);
        transform_point(point_3d_right_bottom, camera_point_right_bottom);
        transform_point(point_3d_left_top, camera_point_left_top);
        transform_point(point_3d_right_top, camera_point_right_top);

        transform_point(point_3d_look_ahead_far, camera_point_look_ahead_far);
        transform_point(point_3d_look_ahead_near, camera_point_look_ahead_near);

        float pixel_left_bottom[2];
        float pixel_right_bottom[2];
        float pixel_left_top[2];
        float pixel_right_top[2];

        rs2_project_point_to_pixel(pixel_left_bottom, &intrinsics, camera_point_left_bottom);
        rs2_project_point_to_pixel(pixel_right_bottom, &intrinsics, camera_point_right_bottom);
        rs2_project_point_to_pixel(pixel_left_top, &intrinsics, camera_point_left_top);
        rs2_project_point_to_pixel(pixel_right_top, &intrinsics, camera_point_right_top);

        cv::Point left_bottom(static_cast<int>(pixel_left_bottom[0]), static_cast<int>(pixel_left_bottom[1]));
        cv::Point right_bottom(static_cast<int>(pixel_right_bottom[0]), static_cast<int>(pixel_right_bottom[1]));
        cv::Point left_top(static_cast<int>(pixel_left_top[0]), static_cast<int>(pixel_left_top[1]));
        cv::Point right_top(static_cast<int>(pixel_right_top[0]), static_cast<int>(pixel_right_top[1]));

        cv::Mat realsense_projection = cv::Mat::zeros(color_image.size(), CV_8UC1);

        // Draw rectangle to show camera coverage area
        cv::Rect camera_rect(0, 0, color_image.cols, color_image.rows);
        cv::rectangle(realsense_projection, camera_rect, cv::Scalar(100), 2);

        // Add text to show camera dimensions
        std::string camera_size = "Camera: " + std::to_string(color_image.cols) + "x" + std::to_string(color_image.rows);
        cv::putText(realsense_projection, camera_size, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(200), 2);

        // Draw filled polygon instead of lines
        std::vector<cv::Point> polygon_points = {left_bottom, right_bottom, right_top, left_top};
        cv::fillPoly(realsense_projection, std::vector<std::vector<cv::Point>>{polygon_points}, cv::Scalar(255));

        // Define source points (corners of the projected ground rectangle)
        std::vector<cv::Point2f> src_points;
        src_points.push_back(cv::Point2f(left_bottom.x, left_bottom.y));
        src_points.push_back(cv::Point2f(right_bottom.x, right_bottom.y));
        src_points.push_back(cv::Point2f(left_top.x, left_top.y));
        src_points.push_back(cv::Point2f(right_top.x, right_top.y));

        // Define destination points for Bird's Eye View (BEV)
        std::vector<cv::Point2f> dst_points;
        dst_points.push_back(cv::Point2f(300, bev_height - 50)); // left_bottom -> bottom_right
        dst_points.push_back(cv::Point2f(100, bev_height - 50)); // right_bottom -> bottom_left
        dst_points.push_back(cv::Point2f(300, 50));              // left_top -> top_right
        dst_points.push_back(cv::Point2f(100, 50));              // right_top -> top_left

        // Calculate perspective transformation matrix
        cv::Mat perspective_matrix = cv::getPerspectiveTransform(src_points, dst_points);

        // Apply perspective transformation to the color image
        cv::Mat bev_color_image;
        cv::warpPerspective(color_image, bev_color_image, perspective_matrix, cv::Size(bev_width, bev_height));
        // Convert BEV image to HSV for color-based thresholding
        cv::Mat bev_hsv_frame;
        cv::cvtColor(bev_color_image, bev_hsv_frame, cv::COLOR_BGR2HSV);

        cv::Mat bev_gray_frame;
        cv::warpPerspective(color_image_raw, bev_gray_frame, perspective_matrix, cv::Size(bev_width, bev_height));
        cv::cvtColor(bev_gray_frame, bev_gray_frame, cv::COLOR_BGR2GRAY);

        cv::Mat bev_binary;
        cv::Mat bev_binary_raw;
        cv::Mat bev_binary_adaptive;

        // Apply HSV threshold to detect white/bright areas (likely lane markings)
        // cv::inRange(bev_hsv_frame, cv::Scalar(0, 0, 200), cv::Scalar(180, 255, 255), bev_binary);

        // threshold yuv

        cv::Mat yuv_frame;
        cv::cvtColor(color_image, yuv_frame, cv::COLOR_BGR2HSV);

        cv::Mat thres_yuv = cv::Mat::zeros(yuv_frame.size(), CV_8UC1);
        cv::Mat depth_thres = cv::Mat::zeros(thres_yuv.size(), CV_8UC1);
        cv::Mat depth_thres_bev = cv::Mat::zeros(bev_color_image.size(), CV_8UC1);
        cv::Mat obs_thres_bev = cv::Mat::zeros(bev_color_image.size(), CV_8UC1);

        // untuk sign
        { //*   THRESHOLDING YUV IMAGE  *//
            if (controlbox_data[6] > controlbox_data[9])
                std::swap(controlbox_data[6], controlbox_data[9]);
            if (controlbox_data[7] > controlbox_data[10])
                std::swap(controlbox_data[7], controlbox_data[10]);
            if (controlbox_data[8] > controlbox_data[11])
                std::swap(controlbox_data[8], controlbox_data[11]);

            cv::inRange(yuv_frame, cv::Scalar(controlbox_data[6], controlbox_data[7], controlbox_data[8]),
                        cv::Scalar(controlbox_data[9], controlbox_data[10], controlbox_data[11]), thres_yuv);
        }

        // filter small noise
        // Remove small blobs using morphological opening and contour filtering
        cv::morphologyEx(thres_yuv, thres_yuv, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)), cv::Point(-1, -1), 1);

        // Remove small connected components (area < 50 px)
        std::vector<std::vector<cv::Point>> contours_yuv;
        cv::findContours(thres_yuv.clone(), contours_yuv, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (size_t i = 0; i < contours_yuv.size(); ++i)
            if (cv::contourArea(contours_yuv[i]) < 50)
                cv::drawContours(thres_yuv, contours_yuv, static_cast<int>(i), cv::Scalar(0), cv::FILLED);

        findPointCloudSign(thres_yuv, depth_thres, depth_image, center_cam_x_, center_cam_y_, intrinsics);
        cv::warpPerspective(depth_thres, depth_thres_bev, perspective_matrix, cv::Size(bev_width, bev_height));

        cv::warpPerspective(thres_yuv, obs_thres_bev, perspective_matrix, cv::Size(bev_width, bev_height));
        cv::dilate(obs_thres_bev, obs_thres_bev, cv::Mat(), cv::Point(-1, -1), 7);

        //! ==============================================================
        //!     DETECTED OBS
        //! ==============================================================

        cv::Mat obs_frame;
        cv::cvtColor(color_image, obs_frame, cv::COLOR_BGR2YUV);

        cv::Mat real_obs_thres_yuv = cv::Mat::zeros(yuv_frame.size(), CV_8UC1);
        cv::Mat real_obs_depth_thres = cv::Mat::zeros(thres_yuv.size(), CV_8UC1);
        cv::Mat real_obs_depth_thres_bev = cv::Mat::zeros(bev_color_image.size(), CV_8UC1);
        cv::Mat real_obs_thres_bev = cv::Mat::zeros(bev_color_image.size(), CV_8UC1);

        // untuk obstacle
        { //*   THRESHOLDING YUV IMAGE  *//
            if (controlbox_data[12] > controlbox_data[15])
                std::swap(controlbox_data[12], controlbox_data[15]);
            if (controlbox_data[13] > controlbox_data[16])
                std::swap(controlbox_data[13], controlbox_data[16]);
            if (controlbox_data[14] > controlbox_data[17])
                std::swap(controlbox_data[14], controlbox_data[17]);

            cv::inRange(obs_frame, cv::Scalar(controlbox_data[12], controlbox_data[13], controlbox_data[14]),
                        cv::Scalar(controlbox_data[15], controlbox_data[16], controlbox_data[17]), real_obs_thres_yuv);
        }

        // perbesaran obstacle asli untuk masking sign
        cv::Mat bev_obs_raw = cv::Mat::zeros(bev_color_image.size(), CV_8UC1);
        cv::warpPerspective(real_obs_thres_yuv, bev_obs_raw, perspective_matrix, cv::Size(bev_width, bev_height));
        cv::dilate(bev_obs_raw, bev_obs_raw, cv::Mat(), cv::Point(-1, -1), 8);
        cv::bitwise_not(bev_obs_raw, bev_obs_raw);

        // masking sign dengan obstacle
        cv::bitwise_and(bev_obs_raw, obs_thres_bev, obs_thres_bev);

        findPointCloud(real_obs_thres_yuv, real_obs_depth_thres, depth_image, center_cam_x_, center_cam_y_, intrinsics);
        cv::warpPerspective(real_obs_depth_thres, real_obs_depth_thres_bev, perspective_matrix, cv::Size(bev_width, bev_height));
        cv::dilate(real_obs_thres_bev, real_obs_thres_bev, cv::Mat(), cv::Point(-1, -1), 5);

        cv::warpPerspective(real_obs_thres_yuv, real_obs_thres_bev, perspective_matrix, cv::Size(bev_width, bev_height));

        // Convert threshold matrices to color images for visualization
        cv::Mat sign_depth_color = cv::Mat::zeros(depth_thres_bev.size(), CV_8UC3);
        cv::Mat obs_depth_color = cv::Mat::zeros(real_obs_depth_thres_bev.size(), CV_8UC3);

        // Set sign area to blue
        sign_depth_color.setTo(cv::Scalar(255, 0, 0), depth_thres_bev == 255);

        // Set obstacle area to red
        obs_depth_color.setTo(cv::Scalar(0, 0, 255), real_obs_depth_thres_bev == 255);

        // Combine both color images into one frame
        cv::Mat bev_obs_sign;
        cv::addWeighted(sign_depth_color, 0.5, obs_depth_color, 0.5, 0, bev_obs_sign);

        //? FIND POSITION OF OBS FROM real_obs_depth_thres_bev

        // findPointCloud(thres_yuv, depth_thres, depth_image, center_cam_x_, center_cam_y_, intrinsics);
        // cv::warpPerspective(depth_thres, depth_thres_bev, perspective_matrix, cv::Size(bev_width, bev_height));

        // cv::warpPerspective(thres_yuv, obs_thres_bev, perspective_matrix, cv::Size(bev_width, bev_height));
        // cv::dilate(obs_thres_bev, obs_thres_bev, cv::Mat(), cv::Point(-1, -1), 7);

        //! ================================================================

        { //*   THRESHOLDING BEV HSV IMAGE  *//
            if (controlbox_data[0] > controlbox_data[3])
                std::swap(controlbox_data[0], controlbox_data[3]);
            if (controlbox_data[1] > controlbox_data[4])
                std::swap(controlbox_data[1], controlbox_data[4]);
            if (controlbox_data[2] > controlbox_data[5])
                std::swap(controlbox_data[2], controlbox_data[5]);

            cv::inRange(bev_hsv_frame, cv::Scalar(0, 0, controlbox_data[2]),
                        cv::Scalar(controlbox_data[3], controlbox_data[4], controlbox_data[5]), bev_binary_raw);
        }
        {
            int bs = controlbox_data[0] % 2 == 0 ? controlbox_data[0] + 1 : controlbox_data[0];
            bs = std::max(bs, 3);
            int c = -controlbox_data[1];

            cv::adaptiveThreshold(
                bev_gray_frame,
                bev_binary_adaptive,
                255,
                cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                cv::THRESH_BINARY,
                bs,
                c);
        }

        if (used_threshold_)
            bev_binary = bev_binary_adaptive.clone();
        else
            bev_binary = bev_binary_raw.clone();

        cv::rectangle(bev_binary, cv::Point(0, 0),
                      cv::Point(bev_width, bev_height * 1 / 2),
                      cv::Scalar(0), -1);

        cv::bitwise_or(bev_binary, obs_thres_bev, bev_binary);
        cv::bitwise_or(bev_binary, real_obs_thres_bev, bev_binary);

        cv::morphologyEx(bev_binary, bev_binary, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1, -1), 1);

        //! ==================================================
        //!        Draw Robot and look ahead distance
        //! ==================================================
        cv::Mat bev_debug_binary = cv::Mat::zeros(bev_binary.size(), CV_8UC1);
        cv::Point robot_position(bev_width * 0.5, bev_height - 1); // Center of the bottom edge
        robot_position_ = robot_position;
        cv::Mat look_ahead_used = cv::Mat::zeros(bev_binary.size(), CV_8UC1);

        // for (int y = 0; y < bev_height; y += 100) {
        //     cv::line(bev_color_image, cv::Point(0, y), cv::Point(bev_width, y), cv::Scalar(255), 1);
        // }

        cv::line(bev_color_image, cv::Point(0, valid_up_), cv::Point(bev_color_image.cols, valid_up_), cv::Scalar(255), 1);
        cv::line(bev_color_image, cv::Point(0, valid_down_), cv::Point(bev_color_image.cols, valid_down_), cv::Scalar(255), 1);
        cv::line(bev_color_image, cv::Point(valid_center_left_, 0), cv::Point(valid_center_left_, bev_color_image.rows), cv::Scalar(255), 1);
        cv::line(bev_color_image, cv::Point(valid_center_right_, 0), cv::Point(valid_center_right_, bev_color_image.rows), cv::Scalar(255), 1);
        cv::line(bev_color_image, cv::Point(0, cropping_distance_), cv::Point(bev_color_image.cols, cropping_distance_), cv::Scalar(255), 1);

        cv::line(bev_color_image, robot_position, cv::Point(robot_position.x, 0), cv::Scalar(255, 100, 100), 1);

        // Draw robot position using rectangle
        // cv::rectangle(bev_color_image, cv::Point(robot_position.x - 20, robot_position.y - 20),
        //     cv::Point(robot_position.x + 20, robot_position.y + 20),
        //     cv::Scalar(0, 255, 0), -1);

        // draw max and min target steering
        float target_x_max = cos((max_steering_deg_ + 90) * M_PI / 180) * lookahead_far_pixel_ + robot_position.x;
        float target_y_max = robot_position.y - sin((max_steering_deg_ + 90) * M_PI / 180) * lookahead_far_pixel_;
        float target_x_min = cos((-max_steering_deg_ + 90) * M_PI / 180) * lookahead_far_pixel_ + robot_position.x;
        float target_y_min = robot_position.y - sin((-max_steering_deg_ + 90) * M_PI / 180) * lookahead_far_pixel_;

        cv::line(bev_color_image, cv::Point(robot_position.x, robot_position.y),
                 cv::Point(target_x_max, target_y_max), cv::Scalar(0, 255, 255), 2);
        cv::line(bev_color_image, cv::Point(robot_position.x, robot_position.y),
                 cv::Point(target_x_min, target_y_min), cv::Scalar(0, 255, 255), 2);

        //! ==================================================
        //!      Draw Robot and look ahead distance end
        //! ==================================================

        // Find contours in the thresholded image
        std::vector<std::vector<cv::Point>> contours;
        cv::Mat bev_binary_copy = bev_binary.clone();
        cv::erode(bev_binary_copy, bev_binary_copy, cv::Mat(), cv::Point(-1, -1), 1);
        cv::findContours(bev_binary_copy, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<cv::Point> detected_points;

        //? =======================================================
        //? connected component labelling with 4 neighbourhood
        //? =======================================================
        // cv::Mat labels, stats, centroids;
        // cv::connectedComponentsWithStats(bev_binary_copy,
        //     labels,
        //     stats,
        //     centroids,
        //     4,
        //     CV_32S,
        //     cv::CCL_DEFAULT);

        // // Draw the connected components
        // for (int i = 1; i < stats.rows; i++) {
        //     int x = stats.at<int>(i, cv::CC_STAT_LEFT);
        //     int y = stats.at<int>(i, cv::CC_STAT_TOP);
        //     int width = stats.at<int>(i, cv::CC_STAT_WIDTH);
        //     int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);
        //     int area = stats.at<int>(i, cv::CC_STAT_AREA);

        //     if (width < 40 && height < 40 && width > 3 && height > 3) { // Filter small components
        //         cv::rectangle(bev_color_image, cv::Rect(x, y, width, height), cv::Scalar(0, 255, 0), 2);
        //         detected_points.emplace_back(x + width / 2, y + height / 2);
        //     }
        // }
        //? =======================================================
        //?                  Rotated Rect
        //? =======================================================
        cv::Mat rectangle = cv::Mat::zeros(bev_color_image.size(), CV_8UC1);
        // Process each contour to find rotated rectangles
        for (const auto &contour : contours)
        {
            if (cv::contourArea(contour) > 50)
            { // Filter small contours
                // Get minimum area rotated rectangle
                cv::RotatedRect rotated_rect = cv::minAreaRect(contour);

                // Calculate confidence based on area ratio
                double contour_area = cv::contourArea(contour);
                double rect_area = rotated_rect.size.width * rotated_rect.size.height;

                double confidence = (contour_area / rect_area) * 100.0;

                // Only process rectangles with 80% confidence or higher
                if (confidence >= 50.0 && (rotated_rect.size.width < 60 && rotated_rect.size.height < 60))
                {
                    // Get the 4 corner points of the rotated rectangle
                    cv::Point2f vertices[4];
                    rotated_rect.points(vertices);

                    // Draw the rotated rectangle
                    for (int i = 0; i < 4; i++)
                        cv::line(bev_color_image, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
                    cv::fillPoly(rectangle, std::vector<cv::Point>{cv::Point(vertices[0]), cv::Point(vertices[1]), cv::Point(vertices[2]), cv::Point(vertices[3])}, cv::Scalar(255));

                    // Draw center point and angle
                    // cv::circle(bev_color_image, rotated_rect.center, 5, cv::Scalar(255, 0, 0), -1);
                    // cv::putText(bev_color_image, std::to_string(static_cast<int>(confidence)) + "%",
                    //     rotated_rect.center, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);

                    detected_points.push_back(rotated_rect.center);
                }
            }
        }

        cv::dilate(rectangle, rectangle, cv::Mat(), cv::Point(-1, -1), 1);
        cv::bitwise_not(rectangle, rectangle);
        cv::bitwise_and(bev_binary, rectangle, bev_binary);
        //? =======================================================

        // cv::Mat removed_line = bev_binary.clone();
        // cv::erode(removed_line, removed_line, cv::Mat(), cv::Point(-1, -1), 1);
        // cv::bitwise_not(removed_line, removed_line);
        // cv::bitwise_and(removed_line, bev_binary, bev_binary);

        // DBSCAN-like clustering implementation
        std::vector<std::vector<cv::Point>> clusters;
        std::vector<cv::Point> clusters_centroid;
        std::vector<bool> visited(detected_points.size(), false);

        float eps = 30.0f; // Maximum distance between points in same cluster
        int minPts = 2;    // Minimum points required to form cluster

        for (size_t i = 0; i < detected_points.size(); i++)
        {
            if (visited[i])
                continue;

            std::vector<int> neighbors;
            // Find neighbors within eps distance
            for (size_t j = 0; j < detected_points.size(); j++)
            {
                float dist = cv::norm(detected_points[i] - detected_points[j]);
                if (dist <= eps)
                    neighbors.push_back(j);
            }

            // If enough neighbors, create cluster
            if (neighbors.size() >= minPts)
            {
                std::vector<cv::Point> cluster;
                std::queue<int> seeds(std::deque<int>(neighbors.begin(), neighbors.end()));

                while (!seeds.empty())
                {
                    int current = seeds.front();
                    seeds.pop();

                    if (!visited[current])
                    {
                        visited[current] = true;
                        cluster.push_back(detected_points[current]);

                        // Find neighbors of current point
                        std::vector<int> currentNeighbors;
                        for (size_t k = 0; k < detected_points.size(); k++)
                        {
                            float dist = cv::norm(detected_points[current] - detected_points[k]);
                            if (dist <= eps)
                                currentNeighbors.push_back(k);
                        }

                        // Add new neighbors to seeds if enough points
                        if (currentNeighbors.size() >= minPts)
                        {
                            for (int neighbor : currentNeighbors)
                                if (!visited[neighbor])
                                    seeds.push(neighbor);
                        }
                    }
                }
                clusters.push_back(cluster);
            }
        }

        //     for(size_t i = 0; i < clusters.size();i++){
        //         float total_x = 0;
        //         float total_y = 0;
        //         for(size_t j = 0; j < clusters[i].size(); j++){
        //             total_x += clusters[i][j].x;
        //             total_y += clusters[i][j].y;
        //         }

        //         if(clusters[i].size() > 0){
        //             float average_x = total_x /  clusters[i].size();
        //             float average_y = total_y /  clusters[i].size();
        //             cv::circle(bev_color_image, cv::Point((int)average_x, (int)average_y), 8, cv::Scalar(255,0,0), 3);
        //             logger.info("average %.2f %.2f", average_x, average_y);
        //             cv::Point cntroid = {average_x, average_y};

        //             clusters_centroid.push_back(cntroid);
        //         }
        //     }

        //     float thresh_distance_z = 100;

        //     for(size_t i = 0; i < clusters_centroid.size(); i++){
        //         for(size_t j = 0; j < clusters_centroid.size(); j++){
        //             if(i != j){
        //                 float distance = sqrt((clusters_centroid[i].x - clusters_centroid[j].x)*(clusters_centroid[i].x - clusters_centroid[j].x) + (clusters_centroid[i].y - clusters_centroid[j].y)* (clusters_centroid[i].y - clusters_centroid[j].y));
        //                 logger.info("================================distance: %.2f", distance);
        //                 if(distance < thresh_distance_z){
        //                     clusters[i].insert(clusters[i].end(), clusters[j].begin(), clusters[j].end());
        //                 }
        //             }
        //         }
        //     }

        //     for(auto &cluster : clusters){
        //         std::sort(cluster.begin(), cluster.end(), [](const cv::Point &a, const cv::Point &b)
        //         {
        //             return (a.x < b.x) || (a.x == b.x && a.y < b.y);
        //         });
        //         cluster.erase(std::unique(cluster.begin(), cluster.end()), cluster.end());
        //     }

        // //         std::vector<cv::Point> all_points;
        // // all_points.insert(all_points.end(), horizontal_points.begin(), horizontal_points.end());
        // // all_points.insert(all_points.end(), vertical_points.begin(), vertical_points.end());

        // Sort clusters by distance from robot position
        std::sort(clusters.begin(), clusters.end(),
                  [&robot_position](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b)
                  {
                      // Calculate average distance for cluster a
                      float totalDistA = 0.0f;
                      for (const auto &point : a)
                          totalDistA += cv::norm(point - robot_position);
                      float avgDistA = totalDistA / a.size();

                      // Calculate average distance for cluster b
                      float totalDistB = 0.0f;
                      for (const auto &point : b)
                          totalDistB += cv::norm(point - robot_position);
                      float avgDistB = totalDistB / b.size();

                      return avgDistA < avgDistB;
                  });

        // Limit to 4 closest clusters
        if (clusters.size() > 4)
            clusters.resize(4);

        // Draw clusters with different colors
        cv::Scalar colors[] = {cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0),
                               cv::Scalar(0, 0, 255), cv::Scalar(255, 255, 0),
                               cv::Scalar(255, 0, 255), cv::Scalar(0, 255, 255)};

        float offset_angle_zebracross = 0.0f;
        uint8_t is_offset_angle_zebracross = 0;
        float offset_angle_lane = 0.0f;
        uint8_t is_offset_angle_lane = 0;
        // Separate vertical and horizontal angles
        static float dist_to_near_cluster = 0;
        std::vector<float> vertical_angles;
        std::vector<float> horizontal_angles;
        std::vector<std::vector<cv::Point>> vertical_clusters;
        std::vector<std::vector<cv::Point>> horizontal_clusters;
        std::vector<cv::Point> centroid_cluster_vertical;
        std::vector<cv::Point> centroid_cluster_horizontal;

        for (size_t i = 0; i < clusters.size(); i++)
        {
            // Skip small clusters
            if (clusters[i].size() < 4)
            {
                clusters.erase(clusters.begin() + i);
                i--;
                continue;
            }

            cv::Point centroid_cluster_ = {0, 0};
            centroid_cluster_.x = std::accumulate(clusters[i].begin(), clusters[i].end(), 0, [](int sum, const cv::Point &p)
                                                  { return sum + p.x; }) /
                                  clusters[i].size();
            centroid_cluster_.y = std::accumulate(clusters[i].begin(), clusters[i].end(), 0, [](int sum, const cv::Point &p)
                                                  { return sum + p.y; }) /
                                  clusters[i].size();

            cv::Scalar color = colors[i % 6];
            float cluster_angle = 0.0f;
            for (int j = 0; j < clusters[i].size(); j++)
            {
                // cv::circle(bev_color_image, clusters[i][j], 8, color, 3);
                cv::line(bev_color_image, clusters[i][j], clusters[i][(j + 1) % clusters[i].size()], color, 5);
            }

            // check cluster angle between points by calculating mode (often show angle)
            std::vector<float> angles;
            for (size_t j = 0; j < clusters[i].size() - 1; j++)
            {
                float angle = atan2(clusters[i][j + 1].y - clusters[i][j].y,
                                    clusters[i][j + 1].x - clusters[i][j].x) *
                              180.0 / CV_PI;
                angles.push_back(angle);
            }

            // Filter noise by removing outliers (angles far from median)
            if (!angles.empty())
            {
                std::vector<float> sorted_angles = angles;
                std::sort(sorted_angles.begin(), sorted_angles.end());

                float median = sorted_angles[sorted_angles.size() / 2];
                float threshold = 30.0f; // degrees

                std::vector<float> filtered_angles;
                for (float angle : angles)
                    if (std::abs(angle - median) <= threshold)
                        filtered_angles.push_back(angle);

                // Calculate average angle from filtered data
                if (!filtered_angles.empty())
                    cluster_angle = std::accumulate(filtered_angles.begin(), filtered_angles.end(), 0.0f) / filtered_angles.size();
                else
                    cluster_angle = median; // Use median if all filtered out
            }
            else
            {
                cluster_angle = 0.0f;
            }

            cluster_angle *= -1;

            while (cluster_angle < -180.0f)
                cluster_angle += 360.0f;
            while (cluster_angle > 180.0f)
                cluster_angle -= 360.0f;

            cv::putText(bev_color_image, std::to_string(static_cast<int>(cluster_angle)) + " deg",
                        cv::Point(clusters[i][0].x + 10, clusters[i][0].y - 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);

            if (abs(cluster_angle - 90) < 30 || abs(cluster_angle + 90) < 30)
            {
                vertical_clusters.push_back(clusters[i]);
                centroid_cluster_vertical.push_back(centroid_cluster_);
                vertical_angles.push_back(cluster_angle);
                // logger.info("Cluster %zu is vertical with angle: %.2f | %.2f", i, cluster_angle, abs(cluster_angle - 90));
            }
            else if (abs(cluster_angle) < 45 || abs(cluster_angle - 180) < 45 || abs(cluster_angle + 180) < 45)
            {
                horizontal_clusters.push_back(clusters[i]);
                centroid_cluster_horizontal.push_back(centroid_cluster_);
                horizontal_angles.push_back(cluster_angle + 90);
                // logger.info("Cluster %zu is horizontal with angle: %.2f", i, cluster_angle);
            }
        }

        std::vector<float> adjusted_angles;
        for (float angle : vertical_angles)
        {
            // logger.info("v before adjust: %.2f deg", angle);
            if (angle < 0)
                angle += 180.0f; // Adjust negative angles to positive

            // logger.info("v after adjust: %.2f deg", angle);
            adjusted_angles.push_back(angle);
        }

        for (float angle : horizontal_angles)
        {
            // logger.info("h before adjust: %.2f deg", angle);
            while (angle <= 45)
                angle += 90.0f;

            while (angle >= 135)
                angle -= 90.0f;
            // logger.info("h after adjust: %.2f deg", angle);

            adjusted_angles.push_back(angle);
        }

        if (adjusted_angles.size() > 0)
        {
            float average_vertical_angle = 0;
            for (int i = 0; i < adjusted_angles.size(); i++)
                average_vertical_angle += adjusted_angles[i];
            average_vertical_angle /= adjusted_angles.size();
            offset_angle_zebracross = average_vertical_angle;
            is_offset_angle_zebracross = 1;
            // logger.info("Offset angle zebracross: %.2f deg", offset_angle_zebracross);
        }

        float dist_to_near_horizontal_cluster = std::numeric_limits<float>::max();
        float dist_to_near_vertical_cluster = std::numeric_limits<float>::max();

        if (!centroid_cluster_horizontal.empty())
            dist_to_near_horizontal_cluster = cv::norm(centroid_cluster_horizontal[0] - robot_position);
        if (!centroid_cluster_vertical.empty())
            dist_to_near_vertical_cluster = cv::norm(centroid_cluster_vertical[0] - robot_position);

        dist_to_near_cluster = std::min(dist_to_near_horizontal_cluster, dist_to_near_vertical_cluster);

        uint8_t is_close_horizontal = 0;
        static uint8_t berhenti = 0;

        uint8_t belok_kanan = 0;
        uint8_t belok_kiri = 0;

        if (sign_id_ == RIGHT)
            belok_kanan = 1;
        else if (sign_id_ == LEFT)
            belok_kiri = 1;

        // check distance closest horizontal cluster to robot then log it
        static uint8_t counter = 0;
        if (!horizontal_clusters.empty() && clusters.size() > 1)
        {
            static uint8_t prev_is_close_horizontal = 0;
            float closest_distance = std::numeric_limits<float>::max();
            cv::Point robot_gate_upper = {robot_position.x, robot_position.y - 0.4 * meter_to_pixel_};
            cv::Point robot_gate_bottom = {robot_position.x, robot_position.y - 0.2 * meter_to_pixel_};

            cv::line(bev_color_image, robot_gate_upper, robot_gate_bottom, cv::Scalar(0, 255, 0), 2);

            if (centroid_cluster_horizontal[0].y > robot_gate_upper.y)
            {
                if (centroid_cluster_horizontal[0].y > robot_gate_bottom.y)
                {
                    is_close_horizontal = 1;
                    berhenti = 1;
                }
            }
        }
        else
        {
            // logger.info("Horizontal Clusters is empty");
        }

        cv::Point titik_putih = {-1, -1};
        cv::Point titik_hitam = {-1, -1};
        float dist_near_zebracross_vertical_kiri = 99999;
        float dist_near_zebracross_vertical_kanan = 99999;

        if (vertical_clusters.size())
        {
            for (size_t i = 0; i < vertical_clusters.size(); i++)
            {
                float angle = -computeAngle(centroid_cluster_vertical[i], robot_position);
                cv::Point center_circle;

                if (angle > 0)
                {
                    center_circle.x = centroid_cluster_vertical[i].x + length_titik_hitam_ * sin(vertical_angles[i] * M_PI / 180);
                    center_circle.y = centroid_cluster_vertical[i].y + length_titik_hitam_ * cos(vertical_angles[i] * M_PI / 180);

                    dist_near_zebracross_vertical_kiri = cv::norm(centroid_cluster_vertical[i] - robot_position);

                    // cv::circle(bev_color_image, center_circle, 8, cv::Scalar(0, 0, 0), -1);
                    titik_hitam = center_circle;
                }
                else
                {
                    center_circle.x = centroid_cluster_vertical[i].x - length_titik_putih_ * sin(vertical_angles[i] * M_PI / 180);
                    center_circle.y = centroid_cluster_vertical[i].y - length_titik_putih_ * cos(vertical_angles[i] * M_PI / 180);

                    dist_near_zebracross_vertical_kanan = cv::norm(centroid_cluster_vertical[i] - robot_position);

                    titik_putih = center_circle;
                }
            }
        }
        if (titik_putih.x == -1 && titik_putih.y == -1)
            titik_putih = titik_hitam;

        cv::circle(bev_color_image, titik_putih, 8, cv::Scalar(255, 255, 255), -1);

        //? ==================================================
        //?               Flood Fill Dinamis
        //? ==================================================
        cv::Point start_point_kanan(robot_position.x + 0, robot_position.y - 0);
        cv::Point start_point_kiri(robot_position.x - 0, robot_position.y - 20);

        //! ==================================================
        //!            OPTIMALIZATION FLOOD FILL
        //! ==================================================
        // Create mask to remove transformation artifacts (black regions)
        cv::Mat valid_region_mask = cv::Mat::zeros(bev_binary.size(), CV_8UC1);
        cv::Mat line_valid_region = cv::Mat::zeros(bev_binary.size(), CV_8UC1);

        cv::Point points[1][6];
        points[0][0] = cv::Point(0, cropping_distance_);
        points[0][1] = cv::Point(0, valid_up_);
        points[0][2] = cv::Point(valid_center_left_, valid_down_);
        points[0][3] = cv::Point(valid_center_right_, valid_down_);
        points[0][4] = cv::Point(bev_width, valid_up_);
        points[0][5] = cv::Point(bev_width, cropping_distance_);
        const cv::Point *ppt[1] = {points[0]};
        int npt[] = {6};
        cv::fillPoly(valid_region_mask, ppt, npt, 1, cv::Scalar(255));

        // make line valid region from 0,0 -> 0,330 -> 155,590 -> 245,590 -> 400,330 -> 400,0 with thickness 5
        cv::line(line_valid_region, cv::Point(0, cropping_distance_), cv::Point(0, valid_up_), cv::Scalar(255), 5);
        cv::line(line_valid_region, cv::Point(0, valid_up_), cv::Point(valid_center_left_, valid_down_), cv::Scalar(255), 5);
        cv::line(line_valid_region, cv::Point(valid_center_left_, valid_down_), cv::Point(valid_center_right_, valid_down_), cv::Scalar(255), 40);
        cv::line(line_valid_region, cv::Point(valid_center_right_, valid_down_), cv::Point(bev_width, valid_up_), cv::Scalar(255), 5);
        cv::line(line_valid_region, cv::Point(bev_width, valid_up_), cv::Point(bev_width, cropping_distance_), cv::Scalar(255), 5);
        cv::line(line_valid_region, cv::Point(bev_width, cropping_distance_), cv::Point(0, cropping_distance_), cv::Scalar(255), 5);
        cv::circle(line_valid_region, robot_position, bev_height - cropping_distance_, cv::Scalar(255, 0, 0), 5);

        // cv::imshow("Valid Region Mask", valid_region_mask);
        // cv::imshow("Line Valid Region", line_valid_region);

        cv::bitwise_or(bev_binary, line_valid_region, bev_binary);

        // Function to find closest black pixel within radius
        auto findClosestBlackPixel = [](const cv::Mat &image, cv::Point center, int radius) -> cv::Point
        {
            cv::Point closest = center;
            double minDist = std::numeric_limits<double>::max();
            bool found = false;

            for (int dy = -radius; dy <= radius; dy++)
            {
                for (int dx = -radius; dx <= radius; dx++)
                {
                    cv::Point candidate(center.x + dx, center.y + dy);

                    // Check if point is within image bounds and within circle
                    if (candidate.x >= 0 && candidate.x < image.cols && candidate.y >= 0 && candidate.y < image.rows && (dx * dx + dy * dy) <= radius * radius)
                    {

                        // Check if pixel is black (0)
                        if (image.at<uchar>(candidate.y, candidate.x) == 0)
                        {
                            double dist = sqrt(dx * dx + dy * dy);
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
        };

        // Check if starting points are white (255), if so find closest black pixel
        int search_radius = 50;

        // control maju kanan kiri. penutupan jalan
        if (final_sign_id == FORWARD)
        {
            for (size_t i = 0; i < vertical_clusters.size(); i++)
            {
                cv::Point start_point = vertical_clusters[i][0];
                cv::Point end_point;
                end_point.x = start_point.x + 500 * cos((vertical_angles[i]) * CV_PI / 180.0);
                end_point.y = start_point.y + 500 * -sin((vertical_angles[i]) * CV_PI / 180.0);
                cv::line(bev_binary, start_point, end_point, cv::Scalar(255), 30);
                // cv::line(bev_color_image, start_point, end_point, cv::Scalar(255, 255, 0), 2);

                end_point.x = start_point.x - 500 * cos((vertical_angles[i]) * CV_PI / 180.0);
                end_point.y = start_point.y - 500 * -sin((vertical_angles[i]) * CV_PI / 180.0);
                cv::line(bev_binary, start_point, end_point, cv::Scalar(255), 30);
                // cv::line(bev_color_image, start_point, end_point, cv::Scalar(255, 255, 0), 2);
            }
        }

        cv::Mat bev_flood_fill_kanan = bev_binary.clone();
        cv::Mat bev_flood_fill_kiri = bev_binary.clone();

        if (start_point_kanan.x >= 0 && start_point_kanan.x < bev_flood_fill_kanan.cols && start_point_kanan.y >= 0 && start_point_kanan.y < bev_flood_fill_kanan.rows)
        {
            if (bev_flood_fill_kanan.at<uchar>(start_point_kanan.y, start_point_kanan.x) == 255)
            {
                cv::Point new_start = findClosestBlackPixel(bev_flood_fill_kanan, start_point_kanan, search_radius);
                if (new_start.x != -1 && new_start.y != -1)
                    start_point_kanan = new_start;
            }

            if (bev_flood_fill_kanan.at<uchar>(start_point_kanan.y, start_point_kanan.x) == 0)
            {
                cv::Mat mask_kanan = cv::Mat::zeros(bev_flood_fill_kanan.rows + 2, bev_flood_fill_kanan.cols + 2, CV_8UC1);
                cv::floodFill(bev_flood_fill_kanan, mask_kanan, start_point_kanan,
                              cv::Scalar(255), nullptr, cv::Scalar(0), cv::Scalar(0), 4);
                // Extract just the mask (remove the 1-pixel border)
                cv::Mat flood_mask_kanan = mask_kanan(cv::Rect(1, 1, bev_flood_fill_kanan.cols, bev_flood_fill_kanan.rows));
                bev_flood_fill_kanan = flood_mask_kanan * 255;
                cv::circle(bev_color_image, start_point_kanan, 5, cv::Scalar(0, 0, 255), -1);
            }
        }

        if (start_point_kiri.x >= 0 && start_point_kiri.x < bev_flood_fill_kiri.cols && start_point_kiri.y >= 0 && start_point_kiri.y < bev_flood_fill_kiri.rows)
        {
            if (bev_flood_fill_kiri.at<uchar>(start_point_kiri.y, start_point_kiri.x) == 255)
            {
                cv::Point new_start = findClosestBlackPixel(bev_flood_fill_kiri, start_point_kiri, search_radius);
                if (new_start.x != -1 && new_start.y != -1)
                    start_point_kiri = new_start;
            }

            if (bev_flood_fill_kiri.at<uchar>(start_point_kiri.y, start_point_kiri.x) == 0)
            {
                cv::Mat mask_kiri = cv::Mat::zeros(bev_flood_fill_kiri.rows + 2, bev_flood_fill_kiri.cols + 2, CV_8UC1);
                cv::floodFill(bev_flood_fill_kiri, mask_kiri, start_point_kiri,
                              cv::Scalar(255), nullptr, cv::Scalar(0), cv::Scalar(0), 4);
                // Extract just the mask (remove the 1-pixel border)
                cv::Mat flood_mask_kiri = mask_kiri(cv::Rect(1, 1, bev_flood_fill_kiri.cols, bev_flood_fill_kiri.rows));
                bev_flood_fill_kiri = flood_mask_kiri * 255;
                cv::circle(bev_color_image, start_point_kiri, 5, cv::Scalar(255, 0, 0), -1);
            }
        }

        cv::Mat bev_cleaned_binary = cv::Mat::zeros(bev_color_image.size(), CV_8UC1);
        cv::bitwise_or(bev_flood_fill_kanan, bev_flood_fill_kiri, bev_cleaned_binary);
        cv::morphologyEx(bev_cleaned_binary, bev_cleaned_binary, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7)), cv::Point(-1, -1), 2);

        //* ==================================================
        cv::Mat valid_region_mask_copy = valid_region_mask.clone();
        cv::erode(valid_region_mask_copy, valid_region_mask_copy, cv::Mat(), cv::Point(-1, -1), 10);
        cv::line(valid_region_mask_copy, cv::Point(valid_center_left_, valid_down_), cv::Point(valid_center_right_, valid_down_), cv::Scalar(0), 20);

        cv::Mat bev_binary_dashed = bev_binary.clone();
        cv::bitwise_and(bev_binary_dashed, valid_region_mask_copy, bev_binary_dashed);

        cv::dilate(bev_binary_dashed, bev_binary_dashed, cv::Mat(), cv::Point(-1, -1), 6);
        cv::erode(bev_binary_dashed, bev_binary_dashed, cv::Mat(), cv::Point(-1, -1), 6);
        cv::morphologyEx(bev_binary_dashed, bev_binary_dashed, cv::MORPH_OPEN, cv::Mat(), cv::Point(-1, -1), 1);

        cv::erode(bev_cleaned_binary, bev_cleaned_binary, cv::Mat(), cv::Point(-1, -1), 1);
        cv::bitwise_and(bev_cleaned_binary, bev_binary_dashed, bev_binary_dashed);

        std::vector<std::vector<cv::Point>> dashed_contours;
        cv::findContours(bev_binary_dashed, dashed_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (auto it = dashed_contours.begin(); it != dashed_contours.end();)
            if (cv::contourArea(*it) < 20 || cv::contourArea(*it) > 200)
                it = dashed_contours.erase(it); // Remove small contours
            else
                ++it;

        bev_binary_dashed = cv::Mat::zeros(bev_binary_dashed.size(), CV_8UC1);
        for (size_t i = 0; i < dashed_contours.size(); i++)
            cv::drawContours(bev_binary_dashed, dashed_contours, static_cast<int>(i), cv::Scalar(255), -1);

        for (size_t i = 0; i < dashed_contours.size(); i++)
        {
            cv::RotatedRect rotated_rect = cv::minAreaRect(dashed_contours[i]);
            if (rotated_rect.size.width < 200 && rotated_rect.size.height < 200)
            {
                cv::Point2f vertices[4];
                rotated_rect.points(vertices);
                for (int j = 0; j < 4; j++)
                    cv::line(bev_color_image, vertices[j], vertices[(j + 1) % 4], cv::Scalar(255), 2);
            }
        }
        //* ==================================================
        cv::findContours(bev_binary_dashed, dashed_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<std::vector<cv::Point>> filtered_dashed_contours;

        std::vector<cv::Point> filtered_point_right;
        std::vector<cv::Point> filtered_point_left;

        cv::Mat dashed_line_cleaned = cv::Mat::zeros(rectangle.size(), CV_8UC1);

        static float final_used_line_length = 0.0f;
        static float last_angle = 0.0f;
        static float total_angle = 0.0f;

        // Calculate average area with safety check
        float average_area = 0.0f;

        cv::Mat dashed_line_filtered = cv::Mat::zeros(bev_binary.size(), CV_8UC1);
        cv::Mat dashed_line_filtered_left = cv::Mat::zeros(bev_binary.size(), CV_8UC1);
        cv::Mat dashed_line_filtered_right = cv::Mat::zeros(bev_binary.size(), CV_8UC1);
        cv::Mat dashed_line_filtered_edge_left = cv::Mat::zeros(bev_color_image.size(), CV_8UC1);
        cv::Mat dashed_line_filtered_edge_right = cv::Mat::zeros(bev_color_image.size(), CV_8UC1);
        cv::Mat edge_line_filtered_right = cv::Mat::zeros(bev_color_image.size(), CV_8UC1);

        if (!dashed_contours.empty())
        { // ← PENTING: Check kosong!

            for (const auto &contour : dashed_contours)
                average_area += cv::contourArea(contour);
            average_area /= static_cast<float>(dashed_contours.size());

            // filter contours based on area
            for (size_t i = 0; i < dashed_contours.size(); i++)
            {
                float area = cv::contourArea(dashed_contours[i]);
                // logger.info("Contour area: %.2f || %d", area, i);
                if (area < 65 || area > 180)
                {
                    dashed_contours.erase(dashed_contours.begin() + i);
                    i--; // Adjust index after removal
                }
            }

            // sort contours by y position
            std::sort(dashed_contours.begin(), dashed_contours.end(),
                      [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b)
                      {
                          return cv::boundingRect(a).y > cv::boundingRect(b).y;
                      });

            float mean_angle = 0.0f;
            for (size_t i = 0; i < dashed_contours.size(); i++)
            {
                // check angle for each contour
                cv::Point centroid = getCentroid(dashed_contours[i]);

                float angle = computeAngle(centroid, robot_position);

                if (angle > 10)
                {
                    // remove contour if angle is too high
                    dashed_contours.erase(dashed_contours.begin() + i);
                    i--; // Adjust index after removal
                }
            }

            // check the distance between contours
            for (size_t i = 0; i < dashed_contours.size(); i++)
            {

                // Check if the contour is too close to the previous one
                if (i > 0)
                {
                    cv::Rect prev_bounding_rect = cv::boundingRect(dashed_contours[i - 1]);
                    float dist = sqrt(pow(cv::boundingRect(dashed_contours[i]).x - prev_bounding_rect.x, 2) + pow(cv::boundingRect(dashed_contours[i]).y - prev_bounding_rect.y, 2));
                    // logger.info("Distance between contours %d and %d: %.2f", i - 1, i, dist);
                    if (dist > 80) // Adjust threshold as needed
                    {
                        dashed_contours.erase(dashed_contours.begin() + i);
                        i--; // Adjust index after removal
                    }
                }
            }

            // put text id
            for (size_t i = 0; i < dashed_contours.size(); i++)
            {
                cv::Rect bounding_rect = cv::boundingRect(dashed_contours[i]);
                cv::putText(bev_color_image, std::to_string(i), cv::Point(bounding_rect.x, bounding_rect.y - 10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
            }

            for (const auto &contour : dashed_contours)
                cv::drawContours(dashed_line_cleaned, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(255), cv::FILLED);

            std::vector<std::pair<cv::Point, float>> saved_dashed_centroid;
            // draw contours on the cleaned binary image

            // draw line from centroid contour to next centroid contour
            if (dashed_contours.size() >= 1)
            {                                    // ← PENTING: Check minimal 2 contours!
                cv::Point prev_centroid(-1, -1); // Track previous centroid for angle calculation

                for (size_t i = 0; i < dashed_contours.size() - 1; i++)
                {

                    cv::Point centroid_a = getCentroid(dashed_contours[i]);
                    cv::Point centroid_b = getCentroid(dashed_contours[i + 1]);

                    // Check if points are valid
                    if (centroid_a.x > 0 && centroid_a.y > 0 && centroid_b.x > 0 && centroid_b.y > 0)
                    {

                        // Check angle with previous line segment if exists
                        bool skip_line = false;

                        if (prev_centroid != cv::Point(-1, -1))
                        {
                            // Hitung sudut antara dua segmen: prev → A dan A → B
                            cv::Point2f vec1 = centroid_a - prev_centroid;
                            cv::Point2f vec2 = centroid_b - centroid_a;

                            float len1 = cv::norm(vec1);
                            float len2 = cv::norm(vec2);

                            if (len1 > 0 && len2 > 0)
                            {
                                float cos_theta = vec1.dot(vec2) / (len1 * len2);
                                cos_theta = std::max(-1.0f, std::min(cos_theta, 1.0f)); // Hindari NaN
                                float angle_between = std::acos(cos_theta) * 180.0f / CV_PI;

                                if (angle_between > 180.0f)
                                    angle_between -= 360.0f; // Normalisasi sudut
                                else if (angle_between < -180.0f)
                                    angle_between += 360.0f; // Normalisasi sudut

                                if (fabs(angle_between) > 45.0f)
                                    skip_line = true;
                            }
                        }

                        // Jika tidak di-skip, simpan centroid + arah
                        if (!skip_line)
                        {
                            float angle_deg = computeAngle(centroid_a, centroid_b);

                            saved_dashed_centroid.emplace_back(centroid_a, angle_deg);
                            if (i == dashed_contours.size() - 2)
                                saved_dashed_centroid.emplace_back(centroid_b, angle_deg);

                            prev_centroid = centroid_a; // Update previous centroid
                        }
                    }
                }
            }

            // push to first element
            if (saved_dashed_centroid.size() > 0)
            {
                float angle_robot_to_first = computeAngle(robot_position, saved_dashed_centroid[0].first);
                saved_dashed_centroid.insert(saved_dashed_centroid.begin(), std::make_pair(robot_position, angle_robot_to_first));
            }

            //* ==================================================
            float line_length = 30.0f; // Length of the dashed line segments
            final_used_line_length = line_length;

            cv::Point prev_first_point(robot_position.x, robot_position.y);
            cv::Point prev_second_point(robot_position.x, robot_position.y);

            total_angle = 0.0f;
            for (int i = 0; i < saved_dashed_centroid.size(); i++)
            {

                cv::Point second_point;
                cv::Point first_point;

                cv::Point last_second_point;
                cv::Point last_first_point;

                //? ===================================================
                //? CALCULATE ANGLE BETWEEN TWO POINTS
                //? ===================================================

                if (i == 0)
                {
                    // Use robot position for the first point
                    second_point = cv::Point(robot_position.x, robot_position.y);
                    first_point = cv::Point(robot_position.x, robot_position.y);
                }
                else
                {
                    // second_point = saved_dashed_centroid[i].first;
                    // first_point = saved_dashed_centroid[i - 1].first;
                    second_point.x = static_cast<int>(saved_dashed_centroid[i].first.x + (final_used_line_length - 5) * std::cos(saved_dashed_centroid[i].second * M_PI / 180.0f));
                    second_point.y = static_cast<int>(saved_dashed_centroid[i].first.y + (final_used_line_length - 5) * std::sin(saved_dashed_centroid[i].second * M_PI / 180.0f));

                    first_point.x = static_cast<int>(saved_dashed_centroid[i].first.x - final_used_line_length * std::cos(saved_dashed_centroid[i].second * M_PI / 180.0f));
                    first_point.y = static_cast<int>(saved_dashed_centroid[i].first.y - final_used_line_length * std::sin(saved_dashed_centroid[i].second * M_PI / 180.0f));
                }

                if (i == saved_dashed_centroid.size() - 1)
                {
                    cv::Point last_point(0, 0);

                    last_point.x = static_cast<int>(saved_dashed_centroid[i].first.x + (final_used_line_length + 30) * std::cos((saved_dashed_centroid[i].second + 90) * M_PI / 180.0f));
                    last_point.y = static_cast<int>(saved_dashed_centroid[i].first.y + (final_used_line_length + 30) * std::sin((saved_dashed_centroid[i].second + 90) * M_PI / 180.0f));

                    last_second_point.x = static_cast<int>(last_point.x + (final_used_line_length - 5) * std::cos(saved_dashed_centroid[i].second * M_PI / 180.0f));
                    last_second_point.y = static_cast<int>(last_point.y + (final_used_line_length - 5) * std::sin(saved_dashed_centroid[i].second * M_PI / 180.0f));

                    last_first_point.x = static_cast<int>(last_point.x - final_used_line_length * std::cos(saved_dashed_centroid[i].second * M_PI / 180.0f));
                    last_first_point.y = static_cast<int>(last_point.y - final_used_line_length * std::sin(saved_dashed_centroid[i].second * M_PI / 180.0f));

                    cv::line(bev_color_image, second_point, last_second_point, cv::Scalar(255, 255, 0), 2);
                    cv::line(bev_color_image, first_point, last_first_point, cv::Scalar(255, 0, 255), 2);

                    cv::line(dashed_line_filtered_left, second_point, last_second_point, cv::Scalar(255), 5);
                    cv::line(dashed_line_filtered_right, first_point, last_first_point, cv::Scalar(255), 5);
                }

                cv::line(bev_color_image, first_point, second_point, cv::Scalar(0, 255, 0), 2);

                if (i < saved_dashed_centroid.size() - 1)
                {
                    cv::line(bev_color_image, saved_dashed_centroid[i].first, saved_dashed_centroid[i + 1].first, cv::Scalar(0, 0, 255), 2);
                    cv::line(dashed_line_filtered, saved_dashed_centroid[i].first, saved_dashed_centroid[i + 1].first, cv::Scalar(255), 5);

                    if (i != 0)
                    {
                        total_angle += fabs(saved_dashed_centroid[i].second - saved_dashed_centroid[i + 1].second);
                        // logger.info("Angle between points: %.2f", (saved_dashed_centroid[i].second - saved_dashed_centroid[i + 1].second));
                    }
                }

                //? ===================================================
                //?            DRAW LINE IN BINARY IMAGE
                //? ===================================================
                cv::line(dashed_line_filtered_left, second_point, prev_second_point, cv::Scalar(255), 5);
                cv::line(dashed_line_filtered_right, first_point, prev_first_point, cv::Scalar(255), 5);

                //? ===================================================
                //?           DRAW LINE FOR VISUALIZATION
                //? ===================================================
                cv::line(bev_color_image, second_point, prev_second_point, cv::Scalar(255, 255, 0), 2);
                cv::line(bev_color_image, first_point, prev_first_point, cv::Scalar(255, 0, 255), 2);

                prev_first_point = first_point;
                prev_second_point = second_point;
            }
        }

        //* ==================================================

        // laser scan
        cv::Mat valid_ungu = cv::Mat::ones(bev_color_image.size(), CV_8UC1) * 255;
        cv::circle(valid_ungu, robot_position, bev_height - cropping_distance_ - 20, cv::Scalar(0), -1);

        std::vector<cv::Point> laser_scan_points_ungu;
        for (float angle = 0; angle < 90; angle += 1.5)
        {
            for (int distance = 1; distance < bev_color_image.cols; distance += 1)
            {
                int x = static_cast<int>(robot_position.x + distance * cos(angle * CV_PI / 180.0));
                int y = static_cast<int>(robot_position.y - distance * sin(angle * CV_PI / 180.0));
                int x_last = static_cast<int>(robot_position.x + (distance - 1) * cos(angle * CV_PI / 180.0));
                int y_last = static_cast<int>(robot_position.y - (distance - 1) * sin(angle * CV_PI / 180.0));

                if (x > 0 && x < bev_color_image.cols && y > 0 && y < bev_color_image.rows)
                {
                    int pixel_value_dot = bev_cleaned_binary.at<uchar>(y, x) - bev_cleaned_binary.at<uchar>(y_last, x_last);
                    if (pixel_value_dot < 0 && valid_ungu.at<uchar>(y, x) == 0)
                    {
                        laser_scan_points_ungu.push_back(cv::Point(x, y));
                        // cv::circle(bev_color_image, cv::Point(x, y), 10, cv::Scalar(255, 0, 255), -1);
                    }
                    else if (pixel_value_dot < 0 && valid_ungu.at<uchar>(y, x) == 255)
                    {
                        break;
                    }
                }
            }
        }

        if (laser_scan_points_ungu.size() < 2)
            logger.error("Laser Scan is Empty");

        // for (int i = bev_binary.rows - 1; i > 0; i--) {
        //     for (int j = bev_binary.cols - 2; j > 0; j--) {
        //         int x = j;
        //         int y = i;
        //         int x_last = j + 1;
        //         int y_last = i;
        //         int pixel_value_dot = bev_cleaned_binary.at<uchar>(y, x) - bev_cleaned_binary.at<uchar>(y_last, x_last);
        //         if (pixel_value_dot > 0) {
        //             cv::circle(edge_line_filtered_right, cv::Point(x - 32, y), 2, cv::Scalar(255), -1);
        //             cv::circle(bev_color_image, cv::Point(x - 32, y), 2, cv::Scalar(255, 0, 255), -1);
        //             laser_scan_points.push_back(cv::Point(x - 32, y));
        //         }
        //     }
        // }

        std::vector<cv::Point> left_flood_fill_points;
        std::vector<cv::Point> right_flood_fill_points;

        uint8_t left_valid = 0;
        uint8_t right_valid = 0;

        uint8_t return_edge = 0;

        return_edge = edge_reference_detection(bev_cleaned_binary, bev_color_image, left_flood_fill_points, right_flood_fill_points);

        if (return_edge == 1)
        {
            left_valid = 1;
        }
        else if (return_edge == 2)
        {
            right_valid = 1;
        }
        else if (return_edge == 3)
        {
            left_valid = 1;
            right_valid = 1;
        }
        else
        {
            left_valid = 0;
            right_valid = 0;
        }

        static float most_bigger_angle_right = 0.0f;
        static float prev_most_bigger_angle_right = 0.0f;
        static uint16_t deteksi_jalan_bocor = 0;

        int8_t ada_pertigaan = 0;
        float jarak_ke_pertigaan = 0;

        static float last_angle_left_flood_fill_points = 0;
        static float last_angle_right_flood_fill_points = 0;

        // logger.info("last angle: %.2f %.2f", last_angle_left_flood_fill_points, last_angle_right_flood_fill_points);

        if (!left_flood_fill_points.empty())
        {
            last_angle_left_flood_fill_points = computeAngle(left_flood_fill_points[left_flood_fill_points.size() - 1], robot_position_);
            float prev_deg = 0;
            float total_angle = 0;
            float line_length_ref_baru_ = line_length_max_;
            cv::Point prev_second_point = robot_position;

            for (size_t i = 0; i < left_flood_fill_points.size() - 1; i++)
            {
                cv::circle(bev_color_image, left_flood_fill_points[i], 2, cv::Scalar(255, 0, 0), -1);

                float angle_deg = computeAngle(left_flood_fill_points[i], left_flood_fill_points[i + 1]);

                if (i != 0)
                {
                    total_angle += fabs(prev_deg - angle_deg);
                    // logger.info("angle between: %.2f", (prev_deg - angle_deg));
                }

                cv::Point second_point;

                second_point.x = static_cast<int>(left_flood_fill_points[i].x - (line_length_ref_baru_ + 10) * std::cos(angle_deg * M_PI / 180.0f));
                second_point.y = static_cast<int>(left_flood_fill_points[i].y - (line_length_ref_baru_ + 10) * std::sin(angle_deg * M_PI / 180.0f));

                // cv::line(bev_color_image, second_point, prev_second_point, cv::Scalar(255, 255, 0), 2);
                cv::line(dashed_line_filtered_edge_left, second_point, prev_second_point, cv::Scalar(255), 5);

                prev_second_point = second_point;
                prev_deg = angle_deg;
            }
        }

        float total_angle_edge = 0.0f;
        int8_t ada_patahan = 0;
        static int16_t cntr_jalan_lurus = 0;
        static int8_t status_jalan_berkelok = 0;

        if (jalan_berkelok_)
        {
            status_jalan_berkelok = 1;
            cntr_jalan_lurus = 0;
        }
        else
        {
            cntr_jalan_lurus++;
            if (cntr_jalan_lurus > cntr_jalan_lurus_)
            {
                status_jalan_berkelok = 0;
                cntr_jalan_lurus = cntr_jalan_lurus_;
            }
        }

        if (!right_flood_fill_points.empty())
        {
            last_angle_right_flood_fill_points = computeAngle(right_flood_fill_points[right_flood_fill_points.size() - 1], robot_position_);
            float prev_deg = 0;
            float total_angle = 0;
            float line_length_ref_baru_ = line_length_min_;
            float prev_angle_to_robot = 0.0f;
            cv::Point prev_second_point = robot_position;

            if (status_jalan_berkelok)
                line_length_ref_baru_ = line_length_max_;

            size_t total_points = fminf(right_flood_fill_points.size() - 1, 8);

            for (size_t i = 0; i < total_points; i++)
            {
                cv::circle(bev_color_image, right_flood_fill_points[i], 2, cv::Scalar(0, 0, 255), -1);

                float angle_deg = computeAngle(right_flood_fill_points[i + 1], right_flood_fill_points[i]);

                cv::Point second_point;

                second_point.x = static_cast<int>(right_flood_fill_points[i].x - (line_length_ref_baru_)*std::cos(angle_deg * M_PI / 180.0f));
                second_point.y = static_cast<int>(right_flood_fill_points[i].y - (line_length_ref_baru_)*std::sin(angle_deg * M_PI / 180.0f));

                float angle_to_robot = computeAngle(right_flood_fill_points[i], robot_position);

                // logger.info("Angle to robot: %.2f || %zu", angle_to_robot, i);
                if (i != 0)
                {
                    if (fabs(prev_deg - angle_deg) > 15.0f)
                    {
                        ada_patahan = 1;
                    }
                    else
                    {
                        // logger.info("Angle to robot: %.2f || %zu", fabs(prev_deg - angle_deg), i);
                        total_angle += fabs(prev_deg - angle_deg);
                    }

                    // logger.info("angle between: %.2f", (prev_deg - angle_deg));
                } // //  // //

                if (i == 0)
                    most_bigger_angle_right = fabs(angle_to_robot);
                else if (fabs(angle_to_robot) > most_bigger_angle_right)
                    most_bigger_angle_right = fabs(angle_to_robot);

                cv::line(bev_color_image, second_point, prev_second_point, cv::Scalar(0, 255, 255), 2);
                cv::line(dashed_line_filtered_edge_right, second_point, prev_second_point, cv::Scalar(255), 5);

                prev_second_point = second_point;
                prev_angle_to_robot = angle_to_robot;
                prev_deg = angle_deg;
            }
            total_angle_edge = total_angle;
        }

        if (last_angle_left_flood_fill_points < -50 && last_angle_right_flood_fill_points > 50)
        {
            ada_pertigaan = 1;

            jarak_ke_pertigaan = robot_position_.y - right_flood_fill_points[right_flood_fill_points.size() - 1].y;
            logger.info("%.2f", jarak_ke_pertigaan);
        }

        jalan_berkelok_ = (total_angle_edge > 40.0f);
        // jalan_berkelok_ = (most_bigger_angle_right > 20.0f);

        mask_jalan_bocor_ = (fabs(prev_most_bigger_angle_right - most_bigger_angle_right) > 20.0f);

        if (mask_jalan_bocor_)
            deteksi_jalan_bocor += 1;

        // logger.info("Most bigger angle right: %.2f || %.2f -> %d | %d", most_bigger_angle_right, fabs(prev_most_bigger_angle_right - most_bigger_angle_right), mask_jalan_bocor_, deteksi_jalan_bocor);

        prev_most_bigger_angle_right = most_bigger_angle_right;

        // for (size_t i = 1; i < laser_scan_points.size(); i++) {
        //     cv::circle(bev_color_image, cv::Point(laser_scan_points[i].x - 30, laser_scan_points[i].y), 5, cv::Scalar(255, 0, 255), -1);
        //     cv::circle(edge_line_filtered_right, cv::Point(laser_scan_points[i].x - 30, laser_scan_points[i].y), 5, cv::Scalar(255), -1);
        // }

        // cv::line(edge_line_filtered_right, cv::Point(laser_scan_points.front().x - 30, laser_scan_points.front().y), robot_position, cv::Scalar(255), 1);

        //! ==================================================
        //!           OPTIMALIZATION FLOOD FILL END
        //! ==================================================

        //==================================================

        // black rectangle from 0,0 until cropping distance
        cv::rectangle(bev_cleaned_binary, cv::Point(0, 0), cv::Point(bev_cleaned_binary.cols, cropping_distance_ + 30), cv::Scalar(0), -1);

        int max_x_distance = 0;
        int min_x_distance = INT_MAX;

        // First pass: collect all row sums
        std::vector<int> row_sums;
        for (int y = 0; y < bev_cleaned_binary.rows; ++y)
        {
            int total = 0;
            for (int x = 0; x < bev_cleaned_binary.cols; ++x)
                total += bev_cleaned_binary.at<uchar>(y, x);
            if (total > 0)
            { // Only consider rows with white pixels
                row_sums.push_back(total / 255);
            }
        }

        // Second pass: find actual min and max
        if (!row_sums.empty())
        {
            max_x_distance = *std::max_element(row_sums.begin(), row_sums.end());
            min_x_distance = *std::min_element(row_sums.begin(), row_sums.end());
        }

        cv::putText(bev_color_image, "Max X Distance: " + std::to_string(max_x_distance),
                    cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        cv::putText(bev_color_image, "Min X Distance: " + std::to_string(min_x_distance),
                    cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);

        int error = abs(max_x_distance - min_x_distance);

        std::vector<cv::Point> vertical_points = sliding_windows(bev_cleaned_binary, bev_cleaned_binary.cols, 10,
                                                                 10, bev_color_image);

        std::vector<float> angles_vertical;

        if (vertical_points.size() > 2)
        {
            for (size_t i = 0; i < vertical_points.size() - 1; i++)
            {
                cv::Point p1 = vertical_points[i];
                cv::Point p2 = vertical_points[i + 1];
                float angle = atan2(p2.y - p1.y, p2.x - p1.x) * 180.0 / M_PI;
                angles_vertical.push_back(angle);
                // cv::putText(frame, std::to_string(static_cast<int>(angle)), p1, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
            }

            std::map<float, int> angle_count_vertical;
            for (float angle : angles_vertical)
                angle_count_vertical[angle]++;

            float mode_angle_vertical = std::max_element(angle_count_vertical.begin(), angle_count_vertical.end(),
                                                         [](const std::pair<float, int> &a, const std::pair<float, int> &b)
                                                         {
                                                             return a.second < b.second;
                                                         })
                                            ->first;

            cv::Point vertical_end(robot_position.x + 100 * cos((180 - mode_angle_vertical) * M_PI / 180.0),
                                   robot_position.y - 100 * sin((180 - mode_angle_vertical) * M_PI / 180.0));

            if (error < 8)
            {
                cv::line(bev_color_image, robot_position, vertical_end, cv::Scalar(0, 255, 0), 2);
                is_offset_angle_lane = 1;
                offset_angle_lane = 90 + (90 - mode_angle_vertical);
                // logger.info("Offset angle lane: %.2f deg", offset_angle_lane);
            }
        }

        //===================================================

        //? ============================================================
        //?         NORMALISASI LOOKAHEAD DISTANCE
        //? ============================================================
        // logger.info("Total angle: %.2f", total_angle);
        // if (total_angle > max_steering_deg_)
        //     total_angle = max_steering_deg_;

        // float norm_angle = (max_steering_deg_ - total_angle) / max_steering_deg_;
        // float delta = lookahead_far_pixel_ - lookahead_near_pixel_;

        // used_lookahead = lookahead_near_pixel_ + (norm_angle * delta);
        // target_velocity_ = speed_curve_ + (norm_angle * (speed_straight_ - speed_curve_));

        if (status_jalan_berkelok)
            used_lookahead = lookahead_near_pixel_ * 0.5;
        else
            used_lookahead = lookahead_near_pixel_;
        //? ============================================================
        //?         CREATE LOOKAHEAD MAT
        //? ============================================================

        used_lookahead_meter = used_lookahead / meter_to_pixel_;

        // Draw look-ahead distance far from robot position
        cv::circle(bev_color_image, robot_position, static_cast<int>(lookahead_far_pixel_),
                   cv::Scalar(255, 0, 0), 2);

        // Draw look-ahead distance near from robot position
        cv::circle(bev_color_image, robot_position, static_cast<int>(lookahead_near_pixel_),
                   cv::Scalar(0, 0, 255), 2);

        // Draw look-ahead distance used from robot position
        cv::circle(bev_color_image, robot_position, static_cast<int>(used_lookahead),
                   cv::Scalar(255, 0, 255), 2);

        // Draw look-ahead for centroid calculation
        cv::circle(look_ahead_used, robot_position, static_cast<int>(used_lookahead),
                   cv::Scalar(255), 2);

        //? ============================================================
        //?         GET CENTROID OF LOOKAHEAD USED
        //? ============================================================

        cv::Mat lookahead_used_kanan_dashed = cv::Mat::zeros(bev_color_image.size(), CV_8UC1);
        cv::Mat lookahead_used_kanan_edge = cv::Mat::zeros(bev_color_image.size(), CV_8UC1);

        cv::bitwise_and(look_ahead_used, dashed_line_filtered_edge_right, lookahead_used_kanan_edge);
        cv::bitwise_and(look_ahead_used, dashed_line_filtered_right, lookahead_used_kanan_dashed);

        // from robot position get closest point in look_ahead_far and look_ahead_near
        cv::Point closest_point_used_kanan_edge(robot_position.x, robot_position.y - 20);
        cv::Point closest_point_used_kanan_dashed(robot_position.x, robot_position.y - 20);

        // get centroid of look_ahead_far
        cv::Moments m_used_kanan_edge = cv::moments(lookahead_used_kanan_edge);
        cv::Moments m_used_kanan_dashed = cv::moments(lookahead_used_kanan_dashed);

        if (m_used_kanan_edge.m00 != 0)
        {
            closest_point_used_kanan_edge.x = static_cast<int>(m_used_kanan_edge.m10 / m_used_kanan_edge.m00);
            closest_point_used_kanan_edge.y = static_cast<int>(m_used_kanan_edge.m01 / m_used_kanan_edge.m00);
            cv::circle(bev_color_image, closest_point_used_kanan_edge, 5, cv::Scalar(255, 0, 255), 2);
        }

        if (m_used_kanan_dashed.m00 != 0)
        {
            closest_point_used_kanan_dashed.x = static_cast<int>(m_used_kanan_dashed.m10 / m_used_kanan_dashed.m00);
            closest_point_used_kanan_dashed.y = static_cast<int>(m_used_kanan_dashed.m01 / m_used_kanan_dashed.m00);
            cv::circle(bev_color_image, closest_point_used_kanan_dashed, 5, cv::Scalar(255, 0, 255), 2);
        }

        if (closest_point_used_kanan_edge.x > 0 && closest_point_used_kanan_edge.y > 0)
        {
            angle_used_kanan_ = computeAngle(closest_point_used_kanan_edge, robot_position);
            // logger.info("angle edge: %.2f", angle_used_kanan_);
        }

        if (closest_point_used_kanan_dashed.x > 0 && closest_point_used_kanan_dashed.y > 0 && (cv::norm(closest_point_used_kanan_dashed - robot_position) > used_lookahead - 1))
        {
            angle_used_kanan_ = computeAngle(closest_point_used_kanan_dashed, robot_position);
            // logger.info("closest_point_used_kanan_dashed.x: %d %d ", closest_point_used_kanan_dashed.x, closest_point_used_kanan_dashed.y);
            // logger.info("angle dash: %.2f", angle_used_kanan_);
        }

        final_angle_used_for_steering_ = angle_used_kanan_;
        cv::line(bev_color_image, robot_position, closest_point_used_kanan_edge, cv::Scalar(255, 0, 0), 2);
        cv::line(bev_color_image, robot_position, closest_point_used_kanan_dashed, cv::Scalar(0, 0, 255), 2);

        //? ==================================================
        //?             Find Real Obs Point Cloud
        //? ==================================================

        // get contour of depth_thres
        std::vector<std::vector<cv::Point>> contours_real_obs;
        cv::findContours(real_obs_depth_thres_bev, contours_real_obs, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // transform depth thres to bev

        std::vector<cv::Point> real_obs_points;
        cv::Point real_obs_centroid_pixels = {0, 0};
        float real_obs_centroid_meter[2] = {0, 0};

        // sort contours_real_obs by distance to robot position
        std::sort(contours_real_obs.begin(), contours_real_obs.end(),
                  [&robot_position](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b)
                  {
                      cv::Point centroid_a = cv::Point(static_cast<int>(std::accumulate(a.begin(), a.end(), 0, [](int sum, const cv::Point &p)
                                                                                        { return sum + p.x; }) /
                                                                        a.size()),
                                                       static_cast<int>(std::accumulate(a.begin(), a.end(), 0, [](int sum, const cv::Point &p)
                                                                                        { return sum + p.y; }) /
                                                                        a.size()));
                      cv::Point centroid_b = cv::Point(static_cast<int>(std::accumulate(b.begin(), b.end(), 0, [](int sum, const cv::Point &p)
                                                                                        { return sum + p.x; }) /
                                                                        b.size()),
                                                       static_cast<int>(std::accumulate(b.begin(), b.end(), 0, [](int sum, const cv::Point &p)
                                                                                        { return sum + p.y; }) /
                                                                        b.size()));
                      return cv::norm(centroid_a - robot_position) < cv::norm(centroid_b - robot_position);
                  });
        if (!contours_real_obs.empty())
        {
            // draw closest contours_real_obs on bev_color_image
            real_obs_centroid_pixels = cv::Point(static_cast<int>(std::accumulate(contours_real_obs[0].begin(), contours_real_obs[0].end(), 0, [](int sum, const cv::Point &p)
                                                                                  { return sum + p.x; }) /
                                                                  contours_real_obs[0].size()),
                                                 static_cast<int>(std::accumulate(contours_real_obs[0].begin(), contours_real_obs[0].end(), 0, [](int sum, const cv::Point &p)
                                                                                  { return sum + p.y; }) /
                                                                  contours_real_obs[0].size()));

            cv::circle(bev_color_image, real_obs_centroid_pixels, 15, cv::Scalar(255, 0, 0), 3);
            real_obs_centroid_meter[0] = (robot_position.x - real_obs_centroid_pixels.x) / meter_to_pixel_;
            real_obs_centroid_meter[1] = (robot_position.y - real_obs_centroid_pixels.y) / meter_to_pixel_;
        }

        std::string real_ob_centroid_string = "Sign Pos: (" + std::to_string(real_obs_centroid_meter[0]) + ", " + std::to_string(real_obs_centroid_meter[1]) + ")";

        cv::putText(bev_color_image, real_ob_centroid_string, cv::Point(10, 180), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 2);
        cv::circle(bev_color_image, real_obs_centroid_pixels, 10, cv::Scalar(0, 0, 255), -1);

        //? ================================================

        //? ==================================================
        //?             Find Sign Point Cloud
        //? ==================================================
        cv::Mat sign_pointcloud_image = cv::Mat::zeros(color_image.size(), CV_8UC1);

        // get contour of depth_thres
        std::vector<std::vector<cv::Point>> contours_sign;
        cv::findContours(depth_thres_bev, contours_sign, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // transform depth thres to bev

        std::vector<cv::Point> sign_points;
        cv::Point sign_centroid_pixels = {0, 0};
        float sign_centroid_meter[2] = {0, 0};

        // sort contours_sign by distance to robot position
        std::sort(contours_sign.begin(), contours_sign.end(),
                  [&robot_position](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b)
                  {
                      cv::Point centroid_a = cv::Point(static_cast<int>(std::accumulate(a.begin(), a.end(), 0, [](int sum, const cv::Point &p)
                                                                                        { return sum + p.x; }) /
                                                                        a.size()),
                                                       static_cast<int>(std::accumulate(a.begin(), a.end(), 0, [](int sum, const cv::Point &p)
                                                                                        { return sum + p.y; }) /
                                                                        a.size()));
                      cv::Point centroid_b = cv::Point(static_cast<int>(std::accumulate(b.begin(), b.end(), 0, [](int sum, const cv::Point &p)
                                                                                        { return sum + p.x; }) /
                                                                        b.size()),
                                                       static_cast<int>(std::accumulate(b.begin(), b.end(), 0, [](int sum, const cv::Point &p)
                                                                                        { return sum + p.y; }) /
                                                                        b.size()));
                      return cv::norm(centroid_a - robot_position) < cv::norm(centroid_b - robot_position);
                  });
        if (!contours_sign.empty())
        {
            // draw closest contours_sign on bev_color_image
            sign_centroid_pixels = cv::Point(static_cast<int>(std::accumulate(contours_sign[0].begin(), contours_sign[0].end(), 0, [](int sum, const cv::Point &p)
                                                                              { return sum + p.x; }) /
                                                              contours_sign[0].size()),
                                             static_cast<int>(std::accumulate(contours_sign[0].begin(), contours_sign[0].end(), 0, [](int sum, const cv::Point &p)
                                                                              { return sum + p.y; }) /
                                                              contours_sign[0].size()));

            cv::circle(bev_color_image, sign_centroid_pixels, 15, cv::Scalar(0, 200, 255), 3);
            sign_centroid_meter[0] = (robot_position.x - sign_centroid_pixels.x) / meter_to_pixel_;
            sign_centroid_meter[1] = (robot_position.y - sign_centroid_pixels.y) / meter_to_pixel_;
        }

        std::string sign_centroid_string = "Sign Pos: (" + std::to_string(sign_centroid_meter[0]) + ", " + std::to_string(sign_centroid_meter[1]) + ")";
        //? ================================================

        static float dist_to_putih;
        dist_to_putih = cv::norm(titik_putih - robot_position);

        cv::putText(bev_color_image, sign_centroid_string, cv::Point(10, 150), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 2);

        //? ================================================================
        //? publish final angle near in pub_slope_
        //? ================================================================
        std_msgs::msg::Float32 msg_angle_used;
        msg_angle_used.data = -final_angle_used_for_steering_;
        pub_slope_->publish(msg_angle_used);

        std_msgs::msg::Float32 msg_target_velocity;
        msg_target_velocity.data = target_velocity_;
        pub_target_velocity_->publish(msg_target_velocity);

        //? ================================================================
        //? Publish vision urban data
        //? ================================================================
        // int8 berhenti

        // int16 pos_target_px_x
        // int16 pos_target_px_y

        // int16 pos_robot_px_x
        // int16 pos_robot_px_y

        // float32 dist_putih_meter
        // float32 dist_near_zebracross

        // float32 target_angle_ungu
        // float32 target_angle_putih

        ros2_interface::msg::VisionUrban vision_urban_msg;
        vision_urban_msg.berhenti = berhenti;
        vision_urban_msg.pos_target_px_x = titik_putih.x;
        vision_urban_msg.pos_target_px_y = titik_putih.y;
        vision_urban_msg.pos_robot_px_x = robot_position.x;
        vision_urban_msg.pos_robot_px_y = robot_position.y;
        vision_urban_msg.dist_putih_meter = dist_to_putih / meter_to_pixel_;
        vision_urban_msg.dist_near_zebracross = dist_to_near_cluster / meter_to_pixel_;
        vision_urban_msg.target_angle_ungu = -computeAngle(closest_point_used_kanan_edge, robot_position);
        vision_urban_msg.target_angle_putih = -computeAngle(titik_putih, robot_position);
        vision_urban_msg.meter_to_pixel = meter_to_pixel_;
        vision_urban_msg.offset_angle_zebracross = offset_angle_zebracross;
        vision_urban_msg.offset_angle_lane = offset_angle_lane;
        vision_urban_msg.dist_near_zebracross_vertical = dist_to_near_vertical_cluster / meter_to_pixel_;
        vision_urban_msg.dist_near_zebracross_horizontal = dist_to_near_horizontal_cluster / meter_to_pixel_;
        vision_urban_msg.dist_near_zebracross_vertical_kiri = dist_near_zebracross_vertical_kiri / meter_to_pixel_;
        vision_urban_msg.dist_near_zebracross_vertical_kanan = dist_near_zebracross_vertical_kanan / meter_to_pixel_;
        vision_urban_msg.jalan_berkelok = jalan_berkelok_;
        vision_urban_msg.mask_jalan_bocor = mask_jalan_bocor_;
        vision_urban_msg.centroid_obs_x = real_obs_centroid_meter[0];
        vision_urban_msg.centroid_obs_y = real_obs_centroid_meter[1];
        vision_urban_msg.centroid_sign_x = sign_centroid_meter[0];
        vision_urban_msg.centroid_sign_y = sign_centroid_meter[1];
        vision_urban_msg.ada_pertigaan = ada_pertigaan;
        vision_urban_msg.jarak_ke_pertigaan = jarak_ke_pertigaan / meter_to_pixel_;

        pub_vision_urban_->publish(vision_urban_msg);

        static int8_t counter_publish = 0;

        if (counter_publish++ > 5)
        {
            counter_publish = 0;

            // -- Convert depth image to 8-bit for visualization
            cv::Mat depth_image_8bit;
            depth_image.convertTo(depth_image_8bit, CV_8U, 255.0 / 10000.0); // Scale depth values to 0-255
            cv::applyColorMap(depth_image_8bit, depth_image_8bit, cv::COLORMAP_JET);

            cv::Mat weigted_image;
            cv::addWeighted(color_image, 0.5, depth_image_8bit, 0.5, 0, weigted_image);

            // -- Show FPS
            double fps = 1.0 / elapsed_time;
            cv::putText(weigted_image, "FPS: " + std::to_string(fps), cv::Point(10, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);

            // cv::Mat showed_image;
            // cv::cvtColor(display_thres_bev, display_thres_bev, cv::COLOR_GRAY2BGR);
            // cv::cvtColor(bev_cleaned_binary, bev_cleaned_binary, cv::COLOR_GRAY2BGR);
            // cv::cvtColor(dashed_line, dashed_line, cv::COLOR_GRAY2BGR);
            // cv::hconcat(bev_color_image, display_thres_bev, showed_image);
            // cv::hconcat(showed_image, bev_cleaned_binary, showed_image);
            // cv::hconcat(showed_image, dashed_line, showed_image);

            //? 000000

            // -- Publish the overlay image
            sensor_msgs::msg::Image overlay_msg;
            cv_bridge::CvImage overlay_cv;
            overlay_cv.header.stamp = sync_time_;
            overlay_cv.header.frame_id = "camera_color_optical_frame";
            overlay_cv.encoding = sensor_msgs::image_encodings::BGR8;
            overlay_cv.image = bev_color_image;
            overlay_msg = *overlay_cv.toImageMsg();
            pub_color_depth_overlay_->publish(overlay_msg);

            // -- Publish the filtered binary image
            sensor_msgs::msg::Image filtered_debug2_msg;
            cv_bridge::CvImage filtered_debug2_cv;
            filtered_debug2_cv.header.stamp = sync_time_;
            filtered_debug2_cv.header.frame_id = "camera_color_optical_frame";
            filtered_debug2_cv.encoding = sensor_msgs::image_encodings::MONO8;
            filtered_debug2_cv.image = bev_binary;
            filtered_debug2_msg = *filtered_debug2_cv.toImageMsg();
            pub_debug_binary_2_->publish(filtered_debug2_msg);

            // -- Publish the filtered binary image
            sensor_msgs::msg::Image filtered_debug3_msg;
            cv_bridge::CvImage filtered_debug3_cv;
            filtered_debug3_cv.header.stamp = sync_time_;
            filtered_debug3_cv.header.frame_id = "camera_color_optical_frame";
            filtered_debug3_cv.encoding = sensor_msgs::image_encodings::BGR8;
            filtered_debug3_cv.image = bev_obs_sign;
            filtered_debug3_msg = *filtered_debug3_cv.toImageMsg();
            pub_debug_binary_3_->publish(filtered_debug3_msg);

            // -- Publish the filtered binary image
            sensor_msgs::msg::Image filtered_binary_msg;
            cv_bridge::CvImage filtered_binary_cv;
            filtered_binary_cv.header.stamp = sync_time_;
            filtered_binary_cv.header.frame_id = "camera_color_optical_frame";
            filtered_binary_cv.encoding = sensor_msgs::image_encodings::MONO8;
            filtered_binary_cv.image = real_obs_thres_yuv;
            filtered_binary_msg = *filtered_binary_cv.toImageMsg();
            pub_filtered_binary_->publish(filtered_binary_msg);

            // -- Publish the filtered binary image
            sensor_msgs::msg::Image filtered_road_msg;
            cv_bridge::CvImage filtered_road_cv;
            filtered_road_cv.header.stamp = sync_time_;
            filtered_road_cv.header.frame_id = "camera_color_optical_frame";
            filtered_road_cv.encoding = sensor_msgs::image_encodings::MONO8;
            filtered_road_cv.image = thres_yuv;
            filtered_road_msg = *filtered_road_cv.toImageMsg();
            pub_road_binary_->publish(filtered_road_msg);

            // -- Publish the filtered binary image
            sensor_msgs::msg::Image filtered_debug_msg;
            cv_bridge::CvImage filtered_debug_cv;
            filtered_debug_cv.header.stamp = sync_time_;
            filtered_debug_cv.header.frame_id = "camera_color_optical_frame";
            filtered_debug_cv.encoding = sensor_msgs::image_encodings::MONO8;
            filtered_debug_cv.image = bev_binary;
            filtered_debug_msg = *filtered_debug_cv.toImageMsg();
            pub_debug_binary_->publish(filtered_debug_msg);
            // // -- Publsih color hull
            // sensor_msgs::msg::Image color_hull_msg;
            // cv_bridge::CvImage color_hull_cv;
            // color_hull_cv.header.stamp = sync_time_;
            // color_hull_cv.header.frame_id = "camera_color_optical_frame";
            // color_hull_cv.encoding = sensor_msgs::image_encodings::BGR8;
            // color_hull_cv.image = hull_result;
            // color_hull_msg = *color_hull_cv.toImageMsg();
            // pub_color_hull_->publish(color_hull_msg);
        }
    }

    int8_t edge_reference_detection(cv::Mat &bev_cleaned_binary, cv::Mat &bev_color_image, std::vector<cv::Point> &left_flood_fill_points, std::vector<cv::Point> &right_flood_fill_points)
    {
        left_flood_fill_points.clear();
        right_flood_fill_points.clear();

        cv::Mat line_valid_region = cv::Mat::zeros(bev_cleaned_binary.size(), CV_8UC1);

        // make polygon valid region from (0, bev_height), (valid_center_left, bev_height), (0, valid_up)
        cv::Point polygon_points[1][4];
        polygon_points[0][0] = cv::Point(0, bev_height);
        polygon_points[0][1] = cv::Point(valid_center_left_, bev_height);
        polygon_points[0][2] = cv::Point(0, valid_up_);
        polygon_points[0][3] = cv::Point(0, bev_height);
        const cv::Point *ppt[1] = {polygon_points[0]};
        int npt[] = {4};
        cv::fillPoly(line_valid_region, ppt, npt, 1, cv::Scalar(255));
        cv::fillPoly(bev_color_image, ppt, npt, 1, cv::Scalar(0, 255, 0));

        // make polygon valid region from (bev_width, bev_height), (valid_center_right, bev_height), (bev_width, valid_up)
        polygon_points[0][0] = cv::Point(bev_width, bev_height);
        polygon_points[0][1] = cv::Point(valid_center_right_, bev_height);
        polygon_points[0][2] = cv::Point(bev_width, valid_up_);
        polygon_points[0][3] = cv::Point(bev_width, bev_height);
        cv::fillPoly(line_valid_region, ppt, npt, 1, cv::Scalar(255));
        cv::fillPoly(bev_color_image, ppt, npt, 1, cv::Scalar(0, 255, 0));

        // Draw a black rectangle in the center region between left and right valid centers
        // cv::Point rect_points[1][4];
        // rect_points[0][0] = cv::Point(static_cast<int>(valid_center_left_ * 0.5), bev_height);
        // rect_points[0][1] = cv::Point(static_cast<int>((bev_width + valid_center_right_) * 0.5), bev_height);
        // rect_points[0][2] = cv::Point(static_cast<int>((bev_width + valid_center_right_) * 0.5), valid_up_);
        // rect_points[0][3] = cv::Point(static_cast<int>(valid_center_left_ * 0.5), valid_up_);
        // const cv::Point* rect_ppt[1] = { rect_points[0] };
        // int rect_npt[] = { 4 };
        // cv::fillPoly(line_valid_region, rect_ppt, rect_npt, 1, cv::Scalar(0));
        // cv::fillPoly(bev_color_image, rect_ppt, rect_npt, 1, cv::Scalar(0, 0, 0));

        uint8_t stop_left = 0;
        uint8_t stop_right = 0;

        uint8_t left_valid = 0;
        uint8_t right_valid = 0;

        for (int i = 0; i < (bev_height - cropping_distance_) - 10; i += 10)
        {
            cv::Point potential_rising = cv::Point(-1, -1);
            cv::Point potential_falling = cv::Point(-1, -1);
            for (float j = 0.1; j < 180; j += 0.1)
            {
                float distance = i;
                float angle = j * M_PI / 180.0f;
                float angle_last = (j - 0.1) * M_PI / 180.0f;
                int16_t x = static_cast<int16_t>(robot_position_.x + distance * cos(angle));
                int16_t y = static_cast<int16_t>(robot_position_.y - 15 - distance * sin(angle));
                int16_t x_last = static_cast<int16_t>(robot_position_.x + distance * cos(angle_last));
                int16_t y_last = static_cast<int16_t>(robot_position_.y - 15 - distance * sin(angle_last));
                // cv::circle(bev_color_image_raw, cv::Point(x, y), 2, cv::Scalar(0, 0, 255), -1);

                if (x >= 0 && x < bev_width && y >= 0 && y < bev_height - 25)
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
                int16_t y = static_cast<int16_t>(robot_position_.y - 15 - distance * sin(angle));
                int16_t x_last = static_cast<int16_t>(robot_position_.x + distance * cos(angle_last));
                int16_t y_last = static_cast<int16_t>(robot_position_.y - 15 - distance * sin(angle_last));
                // cv::circle(bev_color_image_raw, cv::Point(x, y), 2, cv::Scalar(255, 0, 0), -1);

                if (x >= 0 && x < bev_width && y >= 0 && y < bev_height - 25)
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

        if (right_flood_fill_points.size() > 2)
            right_valid = 1;

        if (left_flood_fill_points.size() > 2)
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

        // pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_camera;
        // voxel_grid_camera.setInputCloud(points_camera2base.makeShared());
        // voxel_grid_camera.setLeafSize(0.008f, 0.008f, 0.008f); // 2cm voxel size
        // voxel_grid_camera.filter(points_camera2base);
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
            rs2::frameset frameset;
            if (!pipe_.try_wait_for_frames(&frameset, 1000))
            { // 1 second timeout
                logger.warn("Failed to get frames from the camera within timeout.");

                pipe_.stop();
                pipeline_started_ = false;
                logger.error("Stopping RealSense pipeline due to timeout.");
                rclcpp::shutdown();
                return;
            }
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
                    logger.error("Failed to get color or depth bev_color_image.");
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

    void findPointCloudSign(const cv::Mat &image_thres,
                            cv::Mat &new_thres,
                            cv::Mat &depth_image,
                            int center_cam_x,
                            int center_cam_y,
                            const rs2_intrinsics &intrinsics)
    {
        // RESET SIGN CENTROID
        sign_centroid_[0] = 0.0f;
        sign_centroid_[1] = 0.0f;

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
        crop_box.setMin(Eigen::Vector4f(0.1, -0.3, 0.04, 1.0)); // In front of robot
        crop_box.setMax(Eigen::Vector4f(1.30, 0.3, 0.2, 1.0));  // (1.10, 0.3, 0.2, 1.0)
        crop_box.setNegative(false);                            // Keep only points inside the box
        crop_box.filter(points_camera2base);

        if (points_camera2base.empty() || points_camera2base.size() < 5)
        {
            // logger.warn("No points found in the point cloud after transformation to camera frame");
            return;
        }

        sensor_msgs::msg::PointCloud2 msg_filtered_camera_for_sign;
        pcl::toROSMsg(points_camera2base, msg_filtered_camera_for_sign);
        msg_filtered_camera_for_sign.header.stamp = sync_time_;
        msg_filtered_camera_for_sign.header.frame_id = "base_link"; // transformed frame
        pub_sign_points_->publish(msg_filtered_camera_for_sign);

        pcl::PointCloud<pcl::PointXYZ> points_base2camera;
        pcl_ros::transformPointCloud(points_camera2base, points_base2camera, tf_base_camera_);

        if (points_base2camera.empty() || points_base2camera.size() < 5)
        {
            // logger.warn("No points found in the point cloud after transformation to base frame");
            return;
        }

        // get centroid of the point cloud
        pcl::compute3DCentroid(points_base2camera, sign_centroid_);

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
        crop_box.setMin(Eigen::Vector4f(0.05, -0.3, 0.25, 1.0)); // In front of robot
        crop_box.setMax(Eigen::Vector4f(1.5, 0.3, 0.35, 1.0));   // 2m ahead, ±1m wide, max 2m tall
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

    void setup_signal_handlers()
    {
        std::signal(SIGINT, [](int)
                    {
            RCLCPP_INFO(rclcpp::get_logger("vision_capture3"), "Received SIGINT, shutting down gracefully...");
            rclcpp::shutdown(); });

        std::signal(SIGTERM, [](int)
                    {
            RCLCPP_INFO(rclcpp::get_logger("vision_capture3"), "Received SIGTERM, shutting down gracefully...");
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
            logger.info("\n");
        }

        std::cout << "\nUsing device: " << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
        std::cout << "    Serial number: " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
        std::cout << "    Firmware version: " << dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << std::endl
                  << std::endl;
    }

    void callback_sub_initial(const std_msgs::msg::Int8::SharedPtr msg)
    {
        // Publish the controlbox data
        std_msgs::msg::Int16MultiArray controlbox_msg;
        controlbox_msg.data.resize(controlbox_size);
        for (size_t i = 0; i < controlbox_size; ++i)
            controlbox_msg.data[i] = controlbox_data[i];
        pub_controlbox_->publish(controlbox_msg);

        save_config_master();
    }

    void save_config_master()
    {
        std_msgs::msg::Float32MultiArray config_vision_msg;
        config_vision_msg.data = {
            99.56,
            kp_steering_,
            ki_steering_,
            kd_steering_,
            lookahead_far_meter_,
            lookahead_near_meter_,
            meter_to_pixel_,
            max_steering_deg_,
            wheelbase_,
            valid_center_left_,
            valid_center_right_,
            valid_up_,
            valid_down_,
            cropping_distance_,
            derajat_steering_kanan_,
            derajat_steering_kiri_,
            encoder_belok_kanan_,
            encoder_belok_kiri_,
            encoder_maju_kanan_,
            encoder_maju_kiri_,
            encoder_maju_lurus_,
            jarak_ke_putih_,
            derajat_gyro_kanan_,
            derajat_gyro_kiri_,
            length_titik_putih_,
            length_titik_hitam_,
            jarak_ke_zebracros_,
            min_jarak_putih_kanan_,
            min_jarak_putih_kiri_,
            min_jarak_putih_lurus_,
            encoder_maju_dead_end_,
            velocity_jalan_otomatis,
            line_length_min_,
            line_length_max_,
            offset_jarak_sign_pole_,
            last_gyro_angle_,
            cntr_jalan_lurus_,
            min_vel_belokan_,
            jarak_ke_sign_pole_,
        };

        lookahead_far_pixel_ = static_cast<int>(lookahead_far_meter_ * meter_to_pixel_);
        lookahead_near_pixel_ = static_cast<int>(lookahead_near_meter_ * meter_to_pixel_);

        pub_config_vision_->publish(config_vision_msg);
        pub_config_master_->publish(config_vision_msg);
    }

    void callback_sub_vision_config(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // Update vision parameters
        kp_steering_ = msg->data[0];
        ki_steering_ = msg->data[1];
        kd_steering_ = msg->data[2];
        lookahead_far_meter_ = msg->data[3];
        lookahead_near_meter_ = msg->data[4];
        meter_to_pixel_ = msg->data[5];
        max_steering_deg_ = msg->data[6];
        wheelbase_ = msg->data[7];
        valid_center_left_ = msg->data[8];
        valid_center_right_ = msg->data[9];
        valid_up_ = msg->data[10];
        valid_down_ = msg->data[11];
        cropping_distance_ = msg->data[12];
        derajat_steering_kanan_ = msg->data[13];
        derajat_steering_kiri_ = msg->data[14];
        encoder_belok_kanan_ = msg->data[15];
        encoder_belok_kiri_ = msg->data[16];
        encoder_maju_kanan_ = msg->data[17];
        encoder_maju_kiri_ = msg->data[18];
        encoder_maju_lurus_ = msg->data[19];
        jarak_ke_putih_ = msg->data[20];
        derajat_gyro_kanan_ = msg->data[21];
        derajat_gyro_kiri_ = msg->data[22];
        length_titik_putih_ = msg->data[23];
        length_titik_hitam_ = msg->data[24];
        jarak_ke_zebracros_ = msg->data[25];
        min_jarak_putih_kanan_ = msg->data[26];
        min_jarak_putih_kiri_ = msg->data[27];
        min_jarak_putih_lurus_ = msg->data[28];
        encoder_maju_dead_end_ = msg->data[29];
        velocity_jalan_otomatis = msg->data[30];
        line_length_min_ = msg->data[31];
        line_length_max_ = msg->data[32];
        offset_jarak_sign_pole_ = msg->data[33];
        last_gyro_angle_ = msg->data[34];
        cntr_jalan_lurus_ = msg->data[35];
        min_vel_belokan_ = msg->data[36];
        jarak_ke_sign_pole_ = msg->data[37];

        save_config_master();

        // Save the configuration to file
        saveConfig();
    }

    void callback_sub_sign_picture_id(const std_msgs::msg::Int32::SharedPtr msg)
    {
        sign_id_ = msg->data;
    }

    void callback_sub_used_threshold(const std_msgs::msg::Int16::SharedPtr msg)
    {
        used_threshold_ = msg->data;
        logger.info("Used threshold updated: %d", used_threshold_);
    }

    // callbak controlbox
    void callback_sub_controlbox(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->data.size(); ++i)
            controlbox_data[i] = msg->data[i];

        // Save the configuration to file
        saveConfig();
    }

    void loadConfig()
    {
        char config_file[100];
        sprintf(config_file, "/home/iris/Fira/46-49-52-41/dynamic_conf_vision2.yaml");

        YAML::Node config = YAML::LoadFile(config_file);

        for (size_t i = 0; i < controlbox_size; ++i)
            controlbox_data[i] = config["Config"]["slider_" + std::to_string(i + 1)].as<int>();

        kp_steering_ = config["Config"]["kp_steering"].as<float>();
        ki_steering_ = config["Config"]["ki_steering"].as<float>();
        kd_steering_ = config["Config"]["kd_steering"].as<float>();
        lookahead_far_meter_ = config["Config"]["lookahead_far_meter"].as<float>();
        lookahead_near_meter_ = config["Config"]["lookahead_near_meter"].as<float>();
        meter_to_pixel_ = config["Config"]["meter_to_pixel"].as<float>();
        max_steering_deg_ = config["Config"]["max_steering_deg"].as<float>();
        wheelbase_ = config["Config"]["wheelbase"].as<float>();
        valid_center_left_ = config["Config"]["valid_center_left"].as<int>();
        valid_center_right_ = config["Config"]["valid_center_right"].as<int>();
        valid_up_ = config["Config"]["valid_up"].as<int>();
        valid_down_ = config["Config"]["valid_down"].as<int>();
        cropping_distance_ = config["Config"]["cropping_distance"].as<int>();
        derajat_steering_kanan_ = config["Config"]["derajat_steering_kanan"].as<float>();
        derajat_steering_kiri_ = config["Config"]["derajat_steering_kiri"].as<float>();
        encoder_belok_kanan_ = config["Config"]["encoder_belok_kanan"].as<float>();
        encoder_belok_kiri_ = config["Config"]["encoder_belok_kiri"].as<float>();
        encoder_maju_kanan_ = config["Config"]["encoder_maju_kanan"].as<float>();
        encoder_maju_kiri_ = config["Config"]["encoder_maju_kiri"].as<float>();
        encoder_maju_lurus_ = config["Config"]["encoder_maju_lurus"].as<float>();
        jarak_ke_putih_ = config["Config"]["jarak_ke_putih"].as<float>();
        derajat_gyro_kanan_ = config["Config"]["derajat_gyro_kanan"].as<float>();
        derajat_gyro_kiri_ = config["Config"]["derajat_gyro_kiri"].as<float>();
        length_titik_putih_ = config["Config"]["length_titik_putih"].as<float>();
        length_titik_hitam_ = config["Config"]["length_titik_hitam"].as<float>();
        jarak_ke_zebracros_ = config["Config"]["jarak_ke_zebracros"].as<float>();
        min_jarak_putih_kanan_ = config["Config"]["min_jarak_putih_kanan"].as<float>();
        min_jarak_putih_kiri_ = config["Config"]["min_jarak_putih_kiri"].as<float>();
        min_jarak_putih_lurus_ = config["Config"]["min_jarak_putih_lurus"].as<float>();
        encoder_maju_dead_end_ = config["Config"]["encoder_maju_dead_end"].as<float>();
        velocity_jalan_otomatis = config["Config"]["velocity_jalan_otomatis"].as<float>();
        line_length_min_ = config["Config"]["line_length_min"].as<float>();
        line_length_max_ = config["Config"]["line_length_max"].as<float>();
        offset_jarak_sign_pole_ = config["Config"]["offset_jarak_sign_pole"].as<float>();
        last_gyro_angle_ = config["Config"]["last_gyro_angle"].as<float>();
        cntr_jalan_lurus_ = config["Config"]["cntr_jalan_lurus"].as<int>();
        min_vel_belokan_ = config["Config"]["min_vel_belokan"].as<float>();
        jarak_ke_sign_pole_ = config["Config"]["jarak_ke_sign_pole"].as<float>();

        logger.info("YAML configuration loaded successfully.");

        // publish the controlbox data
        std_msgs::msg::Int16MultiArray controlbox_msg;
        controlbox_msg.data.resize(controlbox_size);
        for (size_t i = 0; i < controlbox_size; ++i)
            controlbox_msg.data[i] = controlbox_data[i];
        pub_controlbox_->publish(controlbox_msg);

        save_config_master();
    }

    void saveConfig()
    {
        logger.info("Saving configuration to YAML file...");

        char config_file[100];
        sprintf(config_file, "/home/iris/Fira/46-49-52-41/dynamic_conf_vision2.yaml");

        YAML::Node config;
        config["Config"]["slider_size"] = controlbox_size;
        for (size_t i = 0; i < controlbox_size; ++i)
            config["Config"]["slider_" + std::to_string(i + 1)] = controlbox_data[i];

        config["Config"]["kp_steering"] = kp_steering_;
        config["Config"]["ki_steering"] = ki_steering_;
        config["Config"]["kd_steering"] = kd_steering_;
        config["Config"]["lookahead_far_meter"] = lookahead_far_meter_;
        config["Config"]["lookahead_near_meter"] = lookahead_near_meter_;
        config["Config"]["meter_to_pixel"] = meter_to_pixel_;
        config["Config"]["max_steering_deg"] = max_steering_deg_;
        config["Config"]["wheelbase"] = wheelbase_;
        config["Config"]["valid_center_left"] = valid_center_left_;
        config["Config"]["valid_center_right"] = valid_center_right_;
        config["Config"]["valid_up"] = valid_up_;
        config["Config"]["valid_down"] = valid_down_;
        config["Config"]["cropping_distance"] = cropping_distance_;
        config["Config"]["derajat_steering_kanan"] = derajat_steering_kanan_;
        config["Config"]["derajat_steering_kiri"] = derajat_steering_kiri_;
        config["Config"]["encoder_belok_kanan"] = encoder_belok_kanan_;
        config["Config"]["encoder_belok_kiri"] = encoder_belok_kiri_;
        config["Config"]["encoder_maju_kanan"] = encoder_maju_kanan_;
        config["Config"]["encoder_maju_kiri"] = encoder_maju_kiri_;
        config["Config"]["encoder_maju_lurus"] = encoder_maju_lurus_;
        config["Config"]["jarak_ke_putih"] = jarak_ke_putih_;
        config["Config"]["derajat_gyro_kanan"] = derajat_gyro_kanan_;
        config["Config"]["derajat_gyro_kiri"] = derajat_gyro_kiri_;
        config["Config"]["length_titik_putih"] = length_titik_putih_;
        config["Config"]["length_titik_hitam"] = length_titik_hitam_;
        config["Config"]["jarak_ke_zebracros"] = jarak_ke_zebracros_;
        config["Config"]["min_jarak_putih_kanan"] = min_jarak_putih_kanan_;
        config["Config"]["min_jarak_putih_kiri"] = min_jarak_putih_kiri_;
        config["Config"]["min_jarak_putih_lurus"] = min_jarak_putih_lurus_;
        config["Config"]["encoder_maju_dead_end"] = encoder_maju_dead_end_;
        config["Config"]["velocity_jalan_otomatis"] = velocity_jalan_otomatis;
        config["Config"]["line_length_min"] = line_length_min_;
        config["Config"]["line_length_max"] = line_length_max_;
        config["Config"]["offset_jarak_sign_pole"] = offset_jarak_sign_pole_;
        config["Config"]["last_gyro_angle"] = last_gyro_angle_;
        config["Config"]["cntr_jalan_lurus"] = cntr_jalan_lurus_;
        config["Config"]["min_vel_belokan"] = min_vel_belokan_;
        config["Config"]["jarak_ke_sign_pole"] = jarak_ke_sign_pole_;

        std::ofstream fout(config_file);
        fout << config;
        fout.close();

        logger.info("YAML configuration saved successfully.");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<VisionCapture3> node_VisionCapture2;

    try
    {
        node_VisionCapture2 = std::make_shared<VisionCapture3>();

        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node_VisionCapture2);
        executor.spin();
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vision_capture3"), "Failed to create VisionCapture3 node: %s", e.what());
        rclcpp::shutdown();
    }

    if (node_VisionCapture2)
        node_VisionCapture2.reset();

    rclcpp::shutdown();
    return 0;
}