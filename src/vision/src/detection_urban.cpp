// C++ Standard Library
#include <algorithm>
#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>

// C Standard Library
#include <cmath>
#include <filesystem>
#include <fstream>
#include <signal.h>
#include <yaml-cpp/yaml.h>

// Third-party Libraries
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

// ROS 2 Libraries
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/help_logger.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int8.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

class DetectionUrbanNode : public rclcpp::Node
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
    rclcpp::CallbackGroup::SharedPtr sub_group_;
    // -------------------------------------------------
    // Timer Callback Group
    // -------------------------------------------------
    rclcpp::CallbackGroup::SharedPtr tim_routine_group_;
    // -------------------------------------------------
    // Subscribers
    // -------------------------------------------------
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_color_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sub_controlbox_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_vision_config_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_sign_picture_id_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_initial_;
    // -------------------------------------------------
    // Publishers
    // -------------------------------------------------
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_color_depth_overlay_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_filtered_binary_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_road_binary_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_binary_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_binary_2_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_binary_3_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_slope_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_target_velocity_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_controlbox_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_config_vision_;
    // -------------------------------------------------
    // Timer
    // -------------------------------------------------
    rclcpp::TimerBase::SharedPtr timer_routine_;
    // -------------------------------------------------
    // Global Variables
    // -------------------------------------------------
    cv::Mat color_image_;
    cv::Mat depth_image_;
    sensor_msgs::msg::CameraInfo color_intrinsics_;

    std::mutex image_mutex_;

    int8_t sign_id_ = 0;

    // Vision parameters
    float lookahead_far_meter_ = 0.3;
    float lookahead_near_meter_ = 0.3;
    int lookahead_far_pixel_ = 82;
    int lookahead_near_pixel_ = 82;
    float meter_to_pixel_ = 272.7272727f;
    float wheelbase_ = 0.23f;
    float max_steering_deg_ = 35.0;

    // PID steering control
    float kp_steering_ = 30.0;
    float ki_steering_ = 0.0;
    float kd_steering_ = 0.0;

    // Valid region parameters
    int valid_center_left_ = 120;
    int valid_center_right_ = 280;
    int valid_up_ = 415;
    int valid_down_ = 600;
    int cropping_distance_ = 450;

    // Control box data
    int8_t controlbox_size = 19;
    int16_t controlbox_data[50];

    // -------------------------------------------------
    // Shutdown handling
    // -------------------------------------------------
    std::atomic<bool> shutdown_requested_{false};

  public:
    DetectionUrbanNode()
        : Node("detection_urban_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
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
        this->declare_parameter<float>("lookahead_far_meter", 0.3);
        this->get_parameter("lookahead_far_meter", lookahead_far_meter_);
        this->declare_parameter<float>("lookahead_near_meter", 0.3);
        this->get_parameter("lookahead_near_meter", lookahead_near_meter_);
        this->declare_parameter<float>("meter_to_pixel", 272.7272727);
        this->get_parameter("meter_to_pixel", meter_to_pixel_);
        this->declare_parameter<float>("wheelbase", 0.23);
        this->get_parameter("wheelbase", wheelbase_);
        this->declare_parameter<float>("max_steering_deg", 35.0);
        this->get_parameter("max_steering_deg", max_steering_deg_);
        this->declare_parameter<float>("kp_steering", 30.0);
        this->get_parameter("kp_steering", kp_steering_);
        this->declare_parameter<float>("ki_steering", 0.0);
        this->get_parameter("ki_steering", ki_steering_);
        this->declare_parameter<float>("kd_steering", 0.0);
        this->get_parameter("kd_steering", kd_steering_);
        this->declare_parameter<int>("valid_center_left", 120);
        this->get_parameter("valid_center_left", valid_center_left_);
        this->declare_parameter<int>("valid_center_right", 280);
        this->get_parameter("valid_center_right", valid_center_right_);
        this->declare_parameter<int>("valid_up", 415);
        this->get_parameter("valid_up", valid_up_);
        this->declare_parameter<int>("valid_down", 600);
        this->get_parameter("valid_down", valid_down_);
        this->declare_parameter<int>("cropping_distance", 450);
        this->get_parameter("cropping_distance", cropping_distance_);

        lookahead_far_pixel_ = static_cast<int>(lookahead_far_meter_ * meter_to_pixel_);
        lookahead_near_pixel_ = static_cast<int>(lookahead_near_meter_ * meter_to_pixel_);

        logger.info("Lookahead %.2f m (%d px) far, %.2f m (%d px) near",
                    lookahead_far_meter_, lookahead_far_pixel_,
                    lookahead_near_meter_, lookahead_near_pixel_);

        // --------------------------------------------------
        // Create subscribers callback groups
        // --------------------------------------------------
        sub_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // --------------------------------------------------
        // Create timer callback groups
        // --------------------------------------------------
        tim_routine_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        // --------------------------------------------------
        // Create subscriber options and assign callback groups
        // --------------------------------------------------
        rclcpp::SubscriptionOptions sub_options_color;
        sub_options_color.callback_group = sub_group_;
        rclcpp::SubscriptionOptions sub_options_depth;
        sub_options_depth.callback_group = sub_group_;
        rclcpp::SubscriptionOptions sub_options_camera_info;
        sub_options_camera_info.callback_group = sub_group_;
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = sub_group_;
        // --------------------------------------------------
        // Create subscribers
        // --------------------------------------------------
        //- Image subscribers
        sub_color_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/rs2_cam_main/color/image_raw", 1,
            std::bind(&DetectionUrbanNode::callback_sub_camera_color, this, std::placeholders::_1), sub_options_color);
        sub_depth_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/rs2_cam_main/aligned_depth_to_color/image_raw", 1,
            std::bind(&DetectionUrbanNode::callback_sub_camera_depth, this, std::placeholders::_1), sub_options_depth);
        sub_camera_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/rs2_cam_main/color/camera_info", 1,
            std::bind(&DetectionUrbanNode::callback_sub_camera_info, this, std::placeholders::_1), sub_options_camera_info);

        //- Sign subscribers
        sub_sign_picture_id_ = this->create_subscription<std_msgs::msg::Int32>(
            "/sign/picture/id", 1, std::bind(&DetectionUrbanNode::callback_sub_sign_picture_id, this, std::placeholders::_1), sub_options);

        //- Web UI subscribers
        sub_controlbox_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "/web/slider", 1, std::bind(&DetectionUrbanNode::callback_sub_controlbox, this, std::placeholders::_1), sub_options);
        sub_initial_ = this->create_subscription<std_msgs::msg::Int8>(
            "/web/initial", 1, std::bind(&DetectionUrbanNode::callback_sub_initial, this, std::placeholders::_1), sub_options);
        sub_vision_config_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/web/vision/configuration", 1, std::bind(&DetectionUrbanNode::callback_sub_vision_config, this, std::placeholders::_1), sub_options);
        // --------------------------------------------------
        // Create publishers
        // --------------------------------------------------
        //- Image publishers
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

        //- Data publishers
        pub_slope_ = this->create_publisher<std_msgs::msg::Float32>(
            "/vision/slope", 1);
        pub_target_velocity_ = this->create_publisher<std_msgs::msg::Float32>(
            "/vision/velocity", 1);

        //- Web UI publishers
        pub_controlbox_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
            "/vision/controlbox", 1);
        pub_config_vision_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/vision/web/configuration_init", 1);
        // --------------------------------------------------
        // Create timer
        // --------------------------------------------------
        timer_routine_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&DetectionUrbanNode::callback_tim_routine, this),
            tim_routine_group_);
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

        logger.info("DetectionUrbanNode node initialized with multithreading");

        loadConfig();
    }

    ~DetectionUrbanNode()
    {
    }

  private:
    void callback_tim_routine()
    {
        if (shutdown_requested_)
            return;

        // ==================================================================
        //                        DEBUG DETECTION URBAN
        // ==================================================================
        double start_time = this->now().seconds();
        static double last_time = start_time;
        double elapsed_time = start_time - last_time;
        last_time = start_time;
        // logger.info("Timer routine elapsed time: %.4f seconds -> %.2f hz", elapsed_time, 1 / elapsed_time);
        // ==================================================================
    }

    void callback_sub_camera_color(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(image_mutex_);
        try
        {
            auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            color_image_ = cv_ptr->image.clone();
            // ==================================================================
            //                        DEBUG DETECTION URBAN
            // ==================================================================
            double start_time = this->now().seconds();
            static double last_time = start_time;
            double elapsed_time = start_time - last_time;
            last_time = start_time;
            logger.info("Subcsribe image color elapsed time: %.4f seconds -> %.2f hz", elapsed_time, 1 / elapsed_time);
            // ==================================================================
        }
        catch (const cv_bridge::Exception &e)
        {
            logger.error("Failed to convert camera color image: %s", e.what());
        }
    }

    void callback_sub_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        color_intrinsics_ = *msg;
    }

    void callback_sub_camera_depth(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(image_mutex_);
        try
        {
            auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            depth_image_ = cv_ptr->image.clone();
            // ==================================================================
            //                        DEBUG DETECTION URBAN
            // ==================================================================
            double start_time = this->now().seconds();
            static double last_time = start_time;
            double elapsed_time = start_time - last_time;
            last_time = start_time;
            logger.info("Subcsribe image depth elapsed time: %.4f seconds -> %.2f hz", elapsed_time, 1 / elapsed_time);
            // ==================================================================
        }
        catch (const cv_bridge::Exception &e)
        {
            logger.error("Failed to convert camera depth image: %s", e.what());
        }
    }

    void setup_signal_handlers()
    {
        signal(SIGINT, signal_handler);
        signal(SIGTERM, signal_handler);
        signal(SIGQUIT, signal_handler);
        signal(SIGABRT, signal_handler);
    }

    static void signal_handler(int signum)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Signal %d received, shutting down...", signum);
        rclcpp::shutdown();
    }

    void callback_sub_initial(const std_msgs::msg::Int8::SharedPtr msg)
    {
        // Publish the controlbox data
        std_msgs::msg::Int16MultiArray controlbox_msg;
        controlbox_msg.data.resize(controlbox_size);
        for (size_t i = 0; i < controlbox_size; ++i)
            controlbox_msg.data[i] = controlbox_data[i];
        pub_controlbox_->publish(controlbox_msg);

        std_msgs::msg::Float32MultiArray config_vision_msg;
        config_vision_msg.data = {
            kp_steering_,
            ki_steering_,
            kd_steering_,
        };

        pub_config_vision_->publish(config_vision_msg);
    }

    void callback_sub_vision_config(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // Update vision parameters
        kp_steering_ = msg->data[0];
        ki_steering_ = msg->data[1];
        kd_steering_ = msg->data[2];

        // Save the configuration to file
        saveConfig();
    }

    void callback_sub_sign_picture_id(const std_msgs::msg::Int32::SharedPtr msg)
    {
        sign_id_ = msg->data;
    }

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

        logger.info("YAML configuration loaded successfully.");

        // publish the controlbox data
        std_msgs::msg::Int16MultiArray controlbox_msg;
        controlbox_msg.data.resize(controlbox_size);
        for (size_t i = 0; i < controlbox_size; ++i)
            controlbox_msg.data[i] = controlbox_data[i];
        pub_controlbox_->publish(controlbox_msg);

        std_msgs::msg::Float32MultiArray config_vision_msg;
        config_vision_msg.data = {
            kp_steering_,
            ki_steering_,
            kd_steering_,
        };
        pub_config_vision_->publish(config_vision_msg);
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

        std::ofstream fout(config_file);
        fout << config;
        fout.close();

        logger.info("YAML configuration saved successfully.");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DetectionUrbanNode>();

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 0);
    executor.add_node(node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}