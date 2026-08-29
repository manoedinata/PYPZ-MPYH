#include "vision/vision_capture4.hpp"

VisionCapture4::VisionCapture4()
    : Node("vision_capture4"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
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
    this->get_parameter("used_lane", origin_used_lane_);
    this->declare_parameter<uint8_t>("origin_lane_dalam_", 1);
    this->get_parameter("origin_lane_dalam_", origin_lane_dalam_);
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
    this->declare_parameter<int>("use_realsense_ros", 0);
    this->get_parameter("use_realsense_ros", use_realsense_ros);

    lookahead_far_pixel_ = static_cast<int>(lookahead_far_meter_ * meter_to_pixel_);
    lookahead_near_pixel_ = static_cast<int>(lookahead_near_meter_ * meter_to_pixel_);

    used_lane_ = origin_used_lane_;

    // logger.info("Lookahead %.2f m (%d px) far, %.2f m (%d px) near ==================================================",
    //             lookahead_far_meter_, lookahead_far_pixel_,
    //             lookahead_near_meter_, lookahead_near_pixel_);

    if (!use_realsense_ros)
    {
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
                color_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1.0f);
                color_sensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, 1.0f);
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
    }
    // --------------------------------------------------
    // Create subscribers callback groups
    // --------------------------------------------------
    sub_camera_bgr_rs_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    sub_camera_depth_rs_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    sub_camera_info_rs_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    sub_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    // --------------------------------------------------
    // Create timer callback groups
    // @param MutuallyExclusive: Ensures that only one callback from this group can run at a time
    // @param Reentrant: Allows multiple callbacks from this group to run concurrently
    // --------------------------------------------------
    tim_routine_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    // --------------------------------------------------
    // Create subscriber options and assign callback groups
    // --------------------------------------------------
    rclcpp::SubscriptionOptions sub_img_bgr_options;
    rclcpp::SubscriptionOptions sub_img_depth_rs_options;
    rclcpp::SubscriptionOptions sub_img_info_rs_options;
    rclcpp::SubscriptionOptions sub_options;

    sub_img_bgr_options.callback_group = sub_camera_bgr_rs_group_;
    sub_img_depth_rs_options.callback_group = sub_camera_depth_rs_group_;
    sub_img_info_rs_options.callback_group = sub_camera_info_rs_group_;
    sub_options.callback_group = sub_callback_group_;
    // --------------------------------------------------
    // Create subscribers
    // --------------------------------------------------

    if (use_realsense_ros)
    {
        //!==================================================================
        //!                     REALSENSE ROS SUBSCRIBERS
        //!==================================================================
        //! Penting: 3 callback paralel -> 3 thread
        sub_camera_bgr_rs_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 1,
            std::bind(&VisionCapture4::callback_sub_camera_bgr_rs, this, std::placeholders::_1),
            sub_img_bgr_options);

        sub_camera_depth_rs_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_rect_raw", 1,
            std::bind(&VisionCapture4::callback_sub_camera_depth_rs, this, std::placeholders::_1),
            sub_img_depth_rs_options);

        sub_camera_info_rs_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/color/camera_info", 1,
            std::bind(&VisionCapture4::callback_sub_camera_info_rs, this, std::placeholders::_1),
            sub_img_info_rs_options);
    }

    sub_controlbox_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
        "/web/slider", 1, std::bind(&VisionCapture4::callback_sub_controlbox, this, std::placeholders::_1), sub_options);
    sub_initial_ = this->create_subscription<std_msgs::msg::Int8>(
        "/web/initial", 1, std::bind(&VisionCapture4::callback_sub_initial, this, std::placeholders::_1), sub_options);
    sub_vision_config_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/web/vision/configuration", 1, std::bind(&VisionCapture4::callback_sub_vision_config, this, std::placeholders::_1), sub_options);
    sub_enc_meter = this->create_subscription<std_msgs::msg::Float32>(
        "/motor_main/velocity_feedback", 1, std::bind(&VisionCapture4::callback_sub_enc_meter, this, std::placeholders::_1), sub_options);
    sub_target_speed = this->create_subscription<std_msgs::msg::Float32>(
        "/master/target_speed", 1, std::bind(&VisionCapture4::callback_sub_target_speed, this, std::placeholders::_1), sub_options);
    sub_lane_used_web_ = this->create_subscription<std_msgs::msg::Int16>(
        "/web/used_lane", 1, std::bind(&VisionCapture4::callback_sub_lane_used_web, this, std::placeholders::_1), sub_options);
    sub_button_ = this->create_subscription<std_msgs::msg::Int8>(
        "/hardware/button", 1, std::bind(&VisionCapture4::callback_sub_button, this, std::placeholders::_1), sub_options);
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
    // --------------------------------------------------
    // Create timer
    // --------------------------------------------------
    if (!use_realsense_ros)
    {
        logger.info("Using RealSense SDK for image capture");
        timer_routine_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&VisionCapture4::callback_tim_routine, this),
            tim_routine_group_);

        timer_pointcloud_routine_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&VisionCapture4::callback_tim_pointcloud_routine, this),
            tim_routine_group_);
    }
    else
    {
        logger.info("Using RealSense ROS for image capture");
    }
    timer_img_routine_ = this->create_wall_timer(
        std::chrono::milliseconds(1),
        std::bind(&VisionCapture4::callback_tim_img_routine, this),
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

            logger.info("===== tf_camera_base_ =====");
            logger.info("Header:");
            logger.info("  frame_id: %s", tf_camera_base_.header.frame_id.c_str());
            logger.info("  child_frame_id: %s", tf_camera_base_.child_frame_id.c_str());
            logger.info("  stamp: %u.%u",
                        tf_camera_base_.header.stamp.sec,
                        tf_camera_base_.header.stamp.nanosec);

            logger.info("Translation:");
            logger.info("  x = %.6f", tf_camera_base_.transform.translation.x);
            logger.info("  y = %.6f", tf_camera_base_.transform.translation.y);
            logger.info("  z = %.6f", tf_camera_base_.transform.translation.z);

            logger.info("Rotation (quaternion):");
            logger.info("  x = %.6f", tf_camera_base_.transform.rotation.x);
            logger.info("  y = %.6f", tf_camera_base_.transform.rotation.y);
            logger.info("  z = %.6f", tf_camera_base_.transform.rotation.z);
            logger.info("  w = %.6f", tf_camera_base_.transform.rotation.w);

            logger.info("===== tf_base_camera_ =====");
            logger.info("Header:");
            logger.info("  frame_id: %s", tf_base_camera_.header.frame_id.c_str());
            logger.info("  child_frame_id: %s", tf_base_camera_.child_frame_id.c_str());
            logger.info("  stamp: %u.%u",
                        tf_base_camera_.header.stamp.sec,
                        tf_base_camera_.header.stamp.nanosec);

            logger.info("Translation:");
            logger.info("  x = %.6f", tf_base_camera_.transform.translation.x);
            logger.info("  y = %.6f", tf_base_camera_.transform.translation.y);
            logger.info("  z = %.6f", tf_base_camera_.transform.translation.z);

            logger.info("Rotation (quaternion):");
            logger.info("  x = %.6f", tf_base_camera_.transform.rotation.x);
            logger.info("  y = %.6f", tf_base_camera_.transform.rotation.y);
            logger.info("  z = %.6f", tf_base_camera_.transform.rotation.z);
            logger.info("  w = %.6f", tf_base_camera_.transform.rotation.w);

            // Realsense SDK TF
            // ===== tf_camera_base_ =====
            // Header:
            //   frame_id: base_link
            //   child_frame_id: camera_color_optical_frame
            //   stamp: 0.0
            // Translation:
            //   x = 0.114000
            //   y = 0.000000
            //   z = 0.418000
            // Rotation (quaternion):
            //   x = -0.636590
            //   y = 0.636591
            //   z = -0.307821
            //   w = 0.307818
            // ===== tf_base_camera_ =====
            // Header:
            //   frame_id: camera_color_optical_frame
            //   child_frame_id: base_link
            //   stamp: 0.0
            // Translation:
            //   x = -0.000001
            //   y = 0.398429
            //   z = 0.170218
            // Rotation (quaternion):
            //   x = -0.636590
            //   y = 0.636591
            //   z = -0.307821
            //   w = -0.307818

            if (use_realsense_ros)
            {
                tf_camera_base_.header.frame_id = "base_link";
                tf_camera_base_.transform.translation.x = 0.114000;
                tf_camera_base_.transform.translation.y = 0.000000;
                tf_camera_base_.transform.translation.z = 0.418000;
                tf_camera_base_.transform.rotation.x = -0.636590;
                tf_camera_base_.transform.rotation.y = 0.636591;
                tf_camera_base_.transform.rotation.z = -0.307821;
                tf_camera_base_.transform.rotation.w = 0.307818;

                tf_base_camera_.header.frame_id = "camera_color_optical_frame";
                tf_base_camera_.transform.translation.x = -0.000001;
                tf_base_camera_.transform.translation.y = 0.398429;
                tf_base_camera_.transform.translation.z = 0.170218;
                tf_base_camera_.transform.rotation.x = -0.636590;
                tf_base_camera_.transform.rotation.y = 0.636591;
                tf_base_camera_.transform.rotation.z = -0.307821;
                tf_base_camera_.transform.rotation.w = -0.307818;
            }

            tf_is_initialized = true;
        }
        catch (const tf2::TransformException &ex)
        {
            logger.warn("TF not ready: %s", ex.what());
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
    }

    logger.info("VisionCapture4 node initialized with multithreading");

    loadConfig(); //
}

VisionCapture4::~VisionCapture4()
{
    cleanup_realsense();
}

void VisionCapture4::callback_tim_img_routine()
{
    static uint8_t counter_ada_obstacle = 0;
    static uint8_t prev_counter_ada_obstacle = 0;
    static uint8_t counter_tidak_ada_obstacle = 0;

    cv::Mat color_image_raw;
    cv::Mat bev_color_image_raw;
    // ==================================================================
    //                    AVOID EXECUTION IF NOT NEEDED
    // ==================================================================
    if (use_realsense_ros)
    {
        if (image_received_ && depth_received_)
        {
            image_received_ = 0;
            depth_received_ = 0;
            img_sync_ = 1;
        }
        else
        {
            return;
        }
    }

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

    if (!use_realsense_ros)
    {
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
    }
    else
    {
        if (color_image_.empty() || depth_image_.empty())
            return;

        {
            std::lock_guard<std::mutex> lock(image_mutex_);
            color_image = color_image_.clone();
        }
        {
            std::lock_guard<std::mutex> lock(depth_mutex_);
            depth_image = depth_image_.clone();
        }
    }

    // print semua data intrinsics secara lengkap
    // logger.info("Camera intrinsics: fx=%.2f, fy=%.2f, ppx=%.2f, ppy=%.2f, coeffs=[%.4f, %.4f, %.4f, %.4f, %.4f]",
    //     intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy,
    //     intrinsics.coeffs[0], intrinsics.coeffs[1], intrinsics.coeffs[2], intrinsics.coeffs[3], intrinsics.coeffs[4]);

    // Realsense SDK intrinsics
    // Camera intrinsics: fx=320.78, fy=320.36, ppx=326.78, ppy=180.46, coeffs=[-0.0538, 0.0629, -0.0005, 0.0014, -0.0199]

    if (use_realsense_ros)
    {
        intrinsics.fx = 320.78f;
        intrinsics.fy = 320.36f;
        intrinsics.ppx = 326.78f;
        intrinsics.ppy = 180.46f;
        intrinsics.coeffs[0] = -0.0538f; // k1
        intrinsics.coeffs[1] = 0.0629f;  // k2
        intrinsics.coeffs[2] = -0.0005f; // p1
        intrinsics.coeffs[3] = 0.0014f;  // p2
        intrinsics.coeffs[4] = -0.0199f; // k3
    }

    if (use_realsense_ros)
    {
        static uint8_t has_set_frequency = 0;
        static uint8_t has_set_exposure = 0;
        static uint8_t has_set_white_balance = 0;
        static uint8_t has_set_brightness = 0;
        static uint8_t has_set_contrast = 0;
        static uint8_t has_set_saturation = 0;

        // if (!has_set_frequency) {
        //     set_realsense_config("rgb_camera.power_line_frequency", 2);
        //     has_set_frequency = 1;
        // }

        // if (!has_set_exposure) {
        //     set_realsense_config("rgb_camera.exposure", 170);
        //     has_set_exposure = 1;
        // }

        // if (!has_set_white_balance) {
        //     set_realsense_config("rgb_camera.white_balance", 4000.0);
        //     has_set_white_balance = 1;
        // }

        // if (!has_set_brightness) {
        //     set_realsense_config("rgb_camera.brightness", -64);
        //     has_set_brightness = 1;
        // }

        // if (!has_set_contrast) {
        //     set_realsense_config("rgb_camera.contrast", 100);
        //     has_set_contrast = 1;
        // }

        // if (!has_set_saturation) {
        //     set_realsense_config("rgb_camera.saturation", 50);
        //     has_set_saturation = 1;
        // }
    }

    //?==================================================================
    //?                     PROCESSING IMAGES
    //?==================================================================
    color_image_raw = color_image.clone();
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
    cv::Mat hsv_image;
    cv::Mat yuv_image;
    cv::cvtColor(color_image, hsv_image, cv::COLOR_BGR2HSV);
    cv::cvtColor(color_image, yuv_image, cv::COLOR_BGR2YUV);

    cv::Mat color_thresh1;
    cv::Mat color_thresh2;
    cv::Mat color_thresh3;
    cv::Mat color_thresh4;

    {
        if (controlbox_data[0] > controlbox_data[3])
            std::swap(controlbox_data[0], controlbox_data[3]);
        if (controlbox_data[1] > controlbox_data[4])
            std::swap(controlbox_data[1], controlbox_data[4]);
        if (controlbox_data[2] > controlbox_data[5])
            std::swap(controlbox_data[2], controlbox_data[5]);

        cv::inRange(hsv_image, cv::Scalar(controlbox_data[0], controlbox_data[1], controlbox_data[2]),
                    cv::Scalar(controlbox_data[3], controlbox_data[4], controlbox_data[5]), color_thresh1);
    }
    {
        if (controlbox_data[6] > controlbox_data[9])
            std::swap(controlbox_data[6], controlbox_data[9]);
        if (controlbox_data[7] > controlbox_data[10])
            std::swap(controlbox_data[7], controlbox_data[10]);
        if (controlbox_data[8] > controlbox_data[11])
            std::swap(controlbox_data[8], controlbox_data[11]);

        cv::inRange(hsv_image, cv::Scalar(controlbox_data[6], controlbox_data[7], controlbox_data[8]),
                    cv::Scalar(controlbox_data[9], controlbox_data[10], controlbox_data[11]), color_thresh2);
    }
    {
        if (controlbox_data[12] > controlbox_data[15])
            std::swap(controlbox_data[12], controlbox_data[15]);
        if (controlbox_data[13] > controlbox_data[16])
            std::swap(controlbox_data[13], controlbox_data[16]);
        if (controlbox_data[14] > controlbox_data[17])
            std::swap(controlbox_data[14], controlbox_data[17]);

        cv::inRange(yuv_image, cv::Scalar(controlbox_data[12], controlbox_data[13], controlbox_data[14]),
                    cv::Scalar(controlbox_data[15], controlbox_data[16], controlbox_data[17]), color_thresh3);
    }

    cv::morphologyEx(color_thresh3, color_thresh3, cv::MORPH_OPEN, cv::Mat(), cv::Point(-1, -1), 3);
    cv::bitwise_not(color_thresh3, color_thresh4);

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

    pixel_left_bottom[0] = 609.64f;
    pixel_left_bottom[1] = 229.95f;
    pixel_right_bottom[0] = 46.08f;
    pixel_right_bottom[1] = 229.70f;
    pixel_left_top[0] = 401.47f;
    pixel_left_top[1] = 8.60f;
    pixel_right_top[0] = 252.50f;
    pixel_right_top[1] = 8.82f;

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
    bev_width_ = 400;
    bev_height_ = 600;
    std::vector<cv::Point2f> dst_points;
    dst_points.push_back(cv::Point2f(300, bev_height_ - 50)); // left_bottom -> bottom_right
    dst_points.push_back(cv::Point2f(100, bev_height_ - 50)); // right_bottom -> bottom_left
    dst_points.push_back(cv::Point2f(300, 50));               // left_top -> top_right
    dst_points.push_back(cv::Point2f(100, 50));               // right_top -> top_left

    // Calculate perspective transformation matrix
    cv::Mat perspective_matrix = cv::getPerspectiveTransform(src_points, dst_points);

    // Apply perspective transformation to the color image
    cv::Mat bev_color_image;
    cv::warpPerspective(color_image, bev_color_image, perspective_matrix, cv::Size(bev_width_, bev_height_));

    cv::Mat bev_thresh1;
    cv::Mat bev_thresh2;
    cv::Mat bev_thresh3;
    cv::Mat bev_thresh4;

    cv::warpPerspective(color_thresh1, bev_thresh1, perspective_matrix, cv::Size(bev_width_, bev_height_));
    cv::warpPerspective(color_thresh2, bev_thresh2, perspective_matrix, cv::Size(bev_width_, bev_height_));
    cv::warpPerspective(color_thresh3, bev_thresh3, perspective_matrix, cv::Size(bev_width_, bev_height_));
    cv::warpPerspective(color_thresh4, bev_thresh4, perspective_matrix, cv::Size(bev_width_, bev_height_));

    // std::vector<cv::Point> left_flood_fill_points;
    // std::vector<cv::Point> right_flood_fill_points;
    // edge_reference_detection(color_thresh4, color_image, left_flood_fill_points, right_flood_fill_points);

    //!===============================================================================================

    //? ================================================================
    //? publish final angle near in pub_slope_
    //? ================================================================
    std_msgs::msg::Float32 msg_angle_used;
    msg_angle_used.data = target_final_angle_;
    pub_slope_->publish(msg_angle_used);

    std_msgs::msg::Float32 msg_target_velocity;
    msg_target_velocity.data = target_velocity_;
    pub_target_velocity_->publish(msg_target_velocity);

    static int8_t counter_publish = 0;

    if (counter_publish++ > 5)
    {
        counter_publish = 0;

        // -- Publish the overlay image
        sensor_msgs::msg::Image overlay_msg;
        cv_bridge::CvImage overlay_cv;
        overlay_cv.header.stamp = sync_time_;
        overlay_cv.header.frame_id = "camera_color_optical_frame";
        overlay_cv.encoding = sensor_msgs::image_encodings::BGR8;
        overlay_cv.image = color_image;
        overlay_msg = *overlay_cv.toImageMsg();
        pub_color_depth_overlay_->publish(overlay_msg);

        // -- Publish the filtered binary image
        sensor_msgs::msg::Image filtered_debug2_msg;
        cv_bridge::CvImage filtered_debug2_cv;
        filtered_debug2_cv.header.stamp = sync_time_;
        filtered_debug2_cv.header.frame_id = "camera_color_optical_frame";
        filtered_debug2_cv.encoding = sensor_msgs::image_encodings::MONO8;
        filtered_debug2_cv.image = bev_thresh1;
        filtered_debug2_msg = *filtered_debug2_cv.toImageMsg();
        pub_debug_binary_2_->publish(filtered_debug2_msg);

        // -- Publish the filtered binary image
        sensor_msgs::msg::Image filtered_debug3_msg;
        cv_bridge::CvImage filtered_debug3_cv;
        filtered_debug3_cv.header.stamp = sync_time_;
        filtered_debug3_cv.header.frame_id = "camera_color_optical_frame";
        filtered_debug3_cv.encoding = sensor_msgs::image_encodings::MONO8;
        filtered_debug3_cv.image = bev_thresh2;
        filtered_debug3_msg = *filtered_debug3_cv.toImageMsg();
        pub_debug_binary_3_->publish(filtered_debug3_msg);

        // -- Publish the filtered binary image
        sensor_msgs::msg::Image filtered_binary_msg;
        cv_bridge::CvImage filtered_binary_cv;
        filtered_binary_cv.header.stamp = sync_time_;
        filtered_binary_cv.header.frame_id = "camera_color_optical_frame";
        filtered_binary_cv.encoding = sensor_msgs::image_encodings::MONO8;
        filtered_binary_cv.image = bev_thresh3;
        filtered_binary_msg = *filtered_binary_cv.toImageMsg();
        pub_filtered_binary_->publish(filtered_binary_msg);

        // -- Publish the filtered binary image
        sensor_msgs::msg::Image filtered_road_msg;
        cv_bridge::CvImage filtered_road_cv;
        filtered_road_cv.header.stamp = sync_time_;
        filtered_road_cv.header.frame_id = "camera_color_optical_frame";
        filtered_road_cv.encoding = sensor_msgs::image_encodings::MONO8;
        filtered_road_cv.image = bev_thresh4;
        filtered_road_msg = *filtered_road_cv.toImageMsg();
        pub_road_binary_->publish(filtered_road_msg);

        // -- Publish the filtered binary image
        sensor_msgs::msg::Image filtered_debug_msg;
        cv_bridge::CvImage filtered_debug_cv;
        filtered_debug_cv.header.stamp = sync_time_;
        filtered_debug_cv.header.frame_id = "camera_color_optical_frame";
        filtered_debug_cv.encoding = sensor_msgs::image_encodings::BGR8;
        filtered_debug_cv.image = color_image;
        filtered_debug_msg = *filtered_debug_cv.toImageMsg();
        pub_debug_binary_->publish(filtered_debug_msg);
    }
}

void VisionCapture4::callback_sub_enc_meter(const std_msgs::msg::Float32::SharedPtr msg)
{
    rclcpp::Time enc_time = this->now();
    static rclcpp::Time last_enc_time = enc_time;
    double dt = (enc_time - last_enc_time).seconds();

    last_enc_time = enc_time;
    enc_meter += msg->data * dt;
    enc_speed = msg->data * dt;
    // logger.warn("Encoder data: %.2f, %.2f dt: %.4f", enc_meter, enc_speed, dt);
}

void VisionCapture4::callback_sub_target_speed(const std_msgs::msg::Float32::SharedPtr msg)
{
    speed_motor = msg->data;
}

void VisionCapture4::callback_sub_lane_used_web(const std_msgs::msg::Int16::SharedPtr msg)
{
    origin_used_lane_ = msg->data;
    used_lane_ = origin_used_lane_;

    // logger.info("Used lane updated to: %d", origin_used_lane_);
}

void VisionCapture4::callback_sub_button(const std_msgs::msg::Int8::SharedPtr msg)
{
    button_1 = (msg->data >> 0) & 0x01;
    button_2 = (msg->data >> 1) & 0x01;
    toogle_button_1 = (msg->data >> 2) & 0x01;
    toogle_button_2 = (msg->data >> 3) & 0x01;
}

void VisionCapture4::callback_sub_initial(const std_msgs::msg::Int8::SharedPtr msg)
{
    // Publish the controlbox data
    std_msgs::msg::Int16MultiArray controlbox_msg;
    controlbox_msg.data.resize(controlbox_size);
    for (size_t i = 0; i < controlbox_size; ++i)
        controlbox_msg.data[i] = controlbox_data[i];
    pub_controlbox_->publish(controlbox_msg);

    std_msgs::msg::Float32MultiArray config_vision_msg;
    config_vision_msg.data = {
        3.012,
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
        9999.0f, // reserved for future use
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
        edge_kanan_anomali};

    pub_config_vision_->publish(config_vision_msg);
}

void VisionCapture4::callback_sub_vision_config(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    // Update vision parameters
    kp_steering_ = msg->data[0];
    ki_steering_ = msg->data[1];
    kd_steering_ = msg->data[2];
    // lookahead_far_meter_ = msg->data[3];
    // lookahead_near_meter_ = msg->data[4];
    meter_to_pixel_ = msg->data[5];
    wheelbase_ = msg->data[6];
    max_steering_deg_ = msg->data[7];
    line_length_min_ = msg->data[8];
    line_length_max_ = msg->data[9];
    line_length_edge_min_ = msg->data[10];
    line_length_edge_max_ = msg->data[11];
    speed_straight_ = msg->data[12];
    speed_wiggle_ = msg->data[13];
    speed_curve_ = msg->data[14];
    lookahead_straight_distance_ = msg->data[15];
    lookahead_wiggle_distance_ = msg->data[16];
    lookahead_curve_distance_ = msg->data[17];
    // used_lane_ = static_cast<int>(msg->data[18]);
    valid_center_left_ = static_cast<int>(msg->data[19]);
    valid_center_right_ = static_cast<int>(msg->data[20]);
    valid_up_ = static_cast<int>(msg->data[21]);
    valid_down_ = static_cast<int>(msg->data[22]);
    cropping_distance_ = static_cast<int>(msg->data[23]);

    jarak_hindar_meter_ = msg->data[25];
    out_duration_belokan_ = msg->data[26];
    out_duration_normal_ = msg->data[27];
    offset_out_duration_center_ = msg->data[28];
    max_enc_meter_obs_ = msg->data[29];
    max_enc_meter_obs_center_ = msg->data[30];
    min_dist_jarak_hindar_ = msg->data[31];
    min_dist_jarak_keluar_ = msg->data[32];
    constant_speed_belok_ = msg->data[33];
    dashed_filter_area_ = msg->data[34];
    segment_speed_1 = msg->data[35];
    segment_speed_2 = msg->data[36];
    segment_speed_3 = msg->data[37];
    segment_speed_4 = msg->data[38];
    road_segment_threshold_area = msg->data[39];
    constant_transient_speed = msg->data[40];
    scaller_speed = msg->data[41];
    offset_kiri = msg->data[42];

    dash_kiri_default = msg->data[43];
    dash_kanan_default = msg->data[44];
    dash_kiri_anomali = msg->data[45];
    dash_kanan_anomali = msg->data[46];
    edge_kiri_default = msg->data[47];
    edge_kanan_default = msg->data[48];
    edge_kiri_anomali = msg->data[49];
    edge_kanan_anomali = msg->data[50];

    // lookahead_far_pixel_ = static_cast<int>(lookahead_far_meter_ * meter_to_pixel_);
    // lookahead_near_pixel_ = static_cast<int>(lookahead_near_meter_ * meter_to_pixel_);

    for (size_t i = 0; i < msg->data.size(); ++i)
    {
        // logger.info("Vision parameter %zu updated to: %.2f", i, msg->data[i]);
    }

    // logger.info("Vision configuration updated:");

    // Save the configuration to file
    saveConfig();
}

// callbak controlbox
void VisionCapture4::callback_sub_controlbox(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
{
    for (size_t i = 0; i < msg->data.size(); ++i)
        controlbox_data[i] = msg->data[i];

    lookahead_far_meter_ = controlbox_data[12] * 0.00196;  // Convert from cm to m
    lookahead_near_meter_ = controlbox_data[13] * 0.00196; // Convert from cm to m

    lookahead_far_pixel_ = static_cast<int>(lookahead_far_meter_ * meter_to_pixel_);
    lookahead_near_pixel_ = static_cast<int>(lookahead_near_meter_ * meter_to_pixel_);

    // Save the configuration to file
    saveConfig();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<VisionCapture4> node_VisionCapture4;

    try
    {
        node_VisionCapture4 = std::make_shared<VisionCapture4>();
        rclcpp::executors::MultiThreadedExecutor executor(
            rclcpp::ExecutorOptions(),
            11);
        executor.add_node(node_VisionCapture4);
        executor.spin();
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vision_capture4"), "Failed to create VisionCapture4 node: %s", e.what());
        rclcpp::shutdown();
    }

    if (node_VisionCapture4)
        node_VisionCapture4.reset();

    rclcpp::shutdown();
    return 0;
}