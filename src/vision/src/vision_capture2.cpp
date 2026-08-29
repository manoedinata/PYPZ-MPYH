#include "vision/vision_capture2.hpp"

VisionCapture2::VisionCapture2()
    : Node("vision_capture2"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
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
            std::bind(&VisionCapture2::callback_sub_camera_bgr_rs, this, std::placeholders::_1),
            sub_img_bgr_options);

        sub_camera_depth_rs_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_rect_raw", 1,
            std::bind(&VisionCapture2::callback_sub_camera_depth_rs, this, std::placeholders::_1),
            sub_img_depth_rs_options);

        sub_camera_info_rs_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/color/camera_info", 1,
            std::bind(&VisionCapture2::callback_sub_camera_info_rs, this, std::placeholders::_1),
            sub_img_info_rs_options);
    }

    sub_controlbox_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
        "/web/slider", 1, std::bind(&VisionCapture2::callback_sub_controlbox, this, std::placeholders::_1), sub_options);
    sub_initial_ = this->create_subscription<std_msgs::msg::Int8>(
        "/web/initial", 1, std::bind(&VisionCapture2::callback_sub_initial, this, std::placeholders::_1), sub_options);
    sub_vision_config_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/web/vision/configuration", 1, std::bind(&VisionCapture2::callback_sub_vision_config, this, std::placeholders::_1), sub_options);
    sub_enc_meter = this->create_subscription<std_msgs::msg::Float32>(
        "/motor_main/velocity_feedback", 1, std::bind(&VisionCapture2::callback_sub_enc_meter, this, std::placeholders::_1), sub_options);
    sub_target_speed = this->create_subscription<std_msgs::msg::Float32>(
        "/master/target_speed", 1, std::bind(&VisionCapture2::callback_sub_target_speed, this, std::placeholders::_1), sub_options);
    sub_lane_used_web_ = this->create_subscription<std_msgs::msg::Int16>(
        "/web/used_lane", 1, std::bind(&VisionCapture2::callback_sub_lane_used_web, this, std::placeholders::_1), sub_options);
    sub_button_ = this->create_subscription<std_msgs::msg::Int8>(
        "/hardware/button", 1, std::bind(&VisionCapture2::callback_sub_button, this, std::placeholders::_1), sub_options);
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
            std::bind(&VisionCapture2::callback_tim_routine, this),
            tim_routine_group_);

        timer_pointcloud_routine_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&VisionCapture2::callback_tim_pointcloud_routine, this),
            tim_routine_group_);
    }
    else
    {
        logger.info("Using RealSense ROS for image capture");
    }
    timer_img_routine_ = this->create_wall_timer(
        std::chrono::milliseconds(1),
        std::bind(&VisionCapture2::callback_tim_img_routine, this),
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

    logger.info("VisionCapture2 node initialized with multithreading");

    loadConfig(); //
}

VisionCapture2::~VisionCapture2()
{
    cleanup_realsense();
}

void VisionCapture2::callback_tim_img_routine()
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
    // cv::Mat gray_frame;
    // cv::Mat otsu_binary;
    // cv::Mat filtered_binary = cv::Mat::zeros(color_image.size(), CV_8UC1);
    // cv::Mat debug_frame = color_image.clone();

    // cv::cvtColor(color_image, gray_frame, cv::COLOR_BGR2GRAY);
    // cv::GaussianBlur(gray_frame, gray_frame, cv::Size(31, 31), 0);
    // cv::Canny(gray_frame, otsu_binary, 50, 70, 3);
    // cv::dilate(otsu_binary, otsu_binary, cv::Mat(), cv::Point(-1, -1), 3);

    // hsv color segmentation
    // cv::Mat hsv_frame;
    // cv::cvtColor(color_image, hsv_frame, cv::COLOR_BGR2HSV);
    // cv::GaussianBlur(hsv_frame, hsv_frame, cv::Size(31, 31), 0);
    // cv::inRange(hsv_frame, cv::Scalar(0, 0, 160), cv::Scalar(180, 255, 255), otsu_binary); //! PERLU JADI PARAMETER

    // cv::morphologyEx(otsu_binary, otsu_binary, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1, -1), 2);

    // cv::threshold(gray_frame, otsu_binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    // find largest contour area
    // std::vector<int> removed_contours_idx;
    // std::vector<std::vector<cv::Point>> contours;
    // cv::findContours(otsu_binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // if (contours.empty()) {
    //     logger.warn("No contours found in the image");
    //     return;
    // }

    // std::sort(contours.begin(), contours.end(),
    //     [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
    //         return cv::contourArea(a) > cv::contourArea(b);
    //     });

    // int largest_contour_area = cv::contourArea(contours[0]);

    //!=============
    //! Nanti diubah
    //!=============
    // for (size_t i = 0; i < contours.size(); i++) {
    //     float height = cv::boundingRect(contours[i]).height;
    //     float width = cv::boundingRect(contours[i]).width;

    //     // put text id and area in cnt location
    //     cv::putText(debug_frame, std::to_string(width) + " " + std::to_string(height),
    //         cv::Point(contours[i][0].x, contours[i][0].y - 10),
    //         cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);

    //     // Method 1: Filter contours based on area
    //     // if (cv::contourArea(contours[i]) < largest_contour_area * 0.7) {
    //     //     removed_contours_idx.push_back(i);
    //     //     continue;
    //     // }

    //     // Method 2: Filter contours based on height
    //     if (height < 300) {
    //         removed_contours_idx.push_back(i);
    //         continue;
    //     }
    // }

    // cv::drawContours(filtered_binary, contours, -1, cv::Scalar(255), cv::FILLED);
    // cv::drawContours(debug_frame, contours, -1, cv::Scalar(255), cv::FILLED);

    //?===================================================
    //?                 NAVIS
    //?===================================================
    cv::Mat yuv_frame;
    cv::Mat combined_road_obs;
    cv::cvtColor(color_image, yuv_frame, cv::COLOR_BGR2HSV);
    const float croped_value = 0; // 20% of the height

    // cv::resize(yuv_frame, yuv_frame, cv::Size(640, 480), 0, 0, cv::INTER_LINEAR);

    // cv::Rect roi_yuv(0, (yuv_frame.rows * croped_value), yuv_frame.cols, yuv_frame.rows - (yuv_frame.rows * croped_value));
    // yuv_frame = yuv_frame(roi_yuv);
    cv::Mat thres_yuv = cv::Mat::zeros(yuv_frame.size(), CV_8UC1);

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
    cv::erode(thres_yuv, thres_yuv, cv::Mat(), cv::Point(-1, -1), 5);

    // find contours in the thresholded YUV image
    std::vector<std::vector<cv::Point>> yuv_contours;
    cv::findContours(thres_yuv, yuv_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::sort(yuv_contours.begin(), yuv_contours.end(),
              [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b)
              {
                  return cv::contourArea(a) > cv::contourArea(b);
              });
    // remove contours that are too small, only get 2 longest countours
    if (yuv_contours.size() > 1)
        yuv_contours.resize(1);

    // get centroid of the largest contour
    cv::Point centroid_yuv(0, 0);
    cv::Mat cleaned_yuv = cv::Mat::zeros(thres_yuv.size(), CV_8UC1);
    for (const auto &contour : yuv_contours)
    {
        if (cv::contourArea(contour) < 800) // Filter out small contours
            continue;

        // Temporary mask for this contour
        cv::Mat contour_mask = cv::Mat::zeros(thres_yuv.size(), CV_8UC1);

        // Draw only this contour
        cv::drawContours(contour_mask, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(255), cv::FILLED);

        // Count white pixels (i.e., area in pixels)
        int white_pixel_count = cv::countNonZero(contour_mask);

        if (white_pixel_count < 800)
            continue;
        // logger.info("Contour area in pixels: %d", white_pixel_count);

        // Calculate the centroid of the contour
        cv::Moments m = cv::moments(contour);
        if (m.m00 != 0)
        {
            centroid_yuv.x = static_cast<int>(m.m10 / m.m00);
            centroid_yuv.y = static_cast<int>(m.m01 / m.m00);
        }
        else
        {
            centroid_yuv.x = 0;
            centroid_yuv.y = 0;
        }

        cv::drawContours(cleaned_yuv, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(255), cv::FILLED);
    }

    std::vector<std::vector<cv::Point>> cleaned_yuv_contour;
    cv::findContours(cleaned_yuv, cleaned_yuv_contour, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat cropped_yuv = cleaned_yuv.clone();
    cv::Rect roi_yuv(0, (cropped_yuv.rows * croped_value), cropped_yuv.cols, cropped_yuv.rows - (cropped_yuv.rows * croped_value));
    cropped_yuv = cropped_yuv(roi_yuv);
    //! ==================================================
    //!        Find Obstacles in Depth
    //! ==================================================

    // get centroid_yuv from the cleaned_yuv image
    cv::Mat depth_thres = cv::Mat::zeros(thres_yuv.size(), CV_8UC1);

    // log the time
    // double start_time_obs = this->now().seconds();
    findPointCloud(thres_yuv, depth_thres, depth_image, center_cam_x_, center_cam_y_, intrinsics);
    // double elapsed_time_obs = this->now().seconds() - start_time_obs;
    // logger.info("Timer img routine elapsed time: %.4f seconds", elapsed_time_obs);

    //? ==================================================
    //?     DETEKSI JALAN EDGE
    //? ==================================================

    cv::Mat hsv_frame;
    cv::Mat cleaned_binary;
    cv::Mat thres_hsv;
    cv::cvtColor(color_image, hsv_frame, cv::COLOR_BGR2HSV);

    // resize hsv_frame to 640x480
    // cv::resize(hsv_frame, hsv_frame, cv::Size(640, 480), 0, 0, cv::INTER_LINEAR);

    // crop 200 pixel from top of frame
    cv::Rect roi(0, (hsv_frame.rows * croped_value), hsv_frame.cols, hsv_frame.rows - (hsv_frame.rows * croped_value));
    hsv_frame = hsv_frame(roi);
    thres_hsv = cv::Mat::zeros(hsv_frame.size(), CV_8UC1);

    cv::inRange(hsv_frame, cv::Scalar(0, 0, 200), cv::Scalar(180, 255, 255), thres_hsv); //! PERLU JADI PARAMETER

    //! BITWISE AND OBS DAN EDGE
    cv::Mat obs_yuv_temp = thres_yuv.clone();
    cv::dilate(obs_yuv_temp, obs_yuv_temp, cv::Mat(), cv::Point(-1, -1), 9);
    cv::bitwise_not(obs_yuv_temp, obs_yuv_temp);
    cv::bitwise_and(obs_yuv_temp, thres_hsv, thres_hsv);

    // find countours in the thresholded image
    std::vector<std::vector<cv::Point>> hsv_contours;
    cv::findContours(thres_hsv, hsv_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // sort contours by height
    std::sort(hsv_contours.begin(), hsv_contours.end(),
              [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b)
              {
                  return cv::boundingRect(a).height > cv::boundingRect(b).height;
              });

    //! remove contours that are too small, only get 3 longest countours
    if (hsv_contours.size() > 3)
        hsv_contours.resize(3);

    // draw contours on the cleaned binary image
    cleaned_binary = cv::Mat::zeros(thres_hsv.size(), CV_8UC1);

    std::vector<cv::Point> nearest_pixels = std::vector<cv::Point>();
    std::vector<float> distances = std::vector<float>();

    for (const auto &contour : hsv_contours)
    {
        if (cv::contourArea(contour) < 1200) // Filter out small contours
            continue;

        // Temporary mask for this contour
        cv::Mat contour_mask = cv::Mat::zeros(thres_hsv.size(), CV_8UC1);

        // Draw only this contour
        cv::drawContours(contour_mask, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(255), cv::FILLED);

        // Count white pixels (i.e., area in pixels)
        int white_pixel_count = cv::countNonZero(contour_mask);

        if (white_pixel_count < 1200)
            continue;

        // get the nearest pixel point to the centroid of the yuv contour
        if (!cleaned_yuv_contour.empty())
        {
            cv::Point nearest_point(0, 0);
            double min_distance = std::numeric_limits<double>::max();
            for (const auto &point : contour)
            {
                double distance = cv::norm(point - centroid_yuv);
                if (distance < min_distance)
                {
                    min_distance = distance;
                    nearest_point = point;
                }
            }
            // Draw a circle at the nearest point
            // cv::circle(debug_frame, nearest_point, 5, cv::Scalar(0, 255, 0), -1);
            nearest_pixels.push_back(nearest_point);
            distances.push_back(min_distance);
            // logger.info("min dist: %.2f", min_distance);
        }
        // logger.info("Contour area in pixels: %d", white_pixel_count);

        cv::drawContours(cleaned_binary, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(255), cv::FILLED);
    }

    // combine the cleaned binary image with the thresholded YUV image
    // combined_road_obs = cv::Mat::zeros(cleaned_binary.size(), CV_8UC1);
    // cv::bitwise_or(cleaned_binary, cropped_yuv, combined_road_obs);

    //? ==================================================
    //?                    HERNANDA
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

    // logger.info("Transformed points to camera frame:");
    // logger.info("Left Bottom: (%.2f, %.2f, %.2f)",
    //     camera_point_left_bottom[0], camera_point_left_bottom[1], camera_point_left_bottom[2]);
    // logger.info("Right Bottom: (%.2f, %.2f, %.2f)",
    //     camera_point_right_bottom[0], camera_point_right_bottom[1], camera_point_right_bottom[2]);
    // logger.info("Left Top: (%.2f, %.2f, %.2f)",
    //     camera_point_left_top[0], camera_point_left_top[1], camera_point_left_top[2]);
    // logger.info("Right Top: (%.2f, %.2f, %.2f)",
    //     camera_point_right_top[0], camera_point_right_top[1], camera_point_right_top[2]);

    // Realsense SDK transform points to camera frame
    // Left Bottom: (0.50, 0.09, 0.56)
    // Right Bottom: (-0.50, 0.09, 0.56)
    // Left Top: (0.50, -1.15, 2.13)
    // Right Top: (-0.50, -1.15, 2.13)

    // Realsense ROS transform points to camera frame
    // Left Bottom: (0.44, 0.01, 0.57)
    // Right Bottom: (-0.56, 0.01, 0.57)
    // Left Top: (0.45, -0.94, 2.33)
    // Right Top: (-0.55, -0.95, 2.33)

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

    // logger.info("Projected points: left_bottom=(%.2f, %.2f), right_bottom=(%.2f, %.2f), left_top=(%.2f, %.2f), right_top=(%.2f, %.2f)",
    //             pixel_left_bottom[0], pixel_left_bottom[1],
    //             pixel_right_bottom[0], pixel_right_bottom[1],
    //             pixel_left_top[0], pixel_left_top[1],
    //             pixel_right_top[0], pixel_right_top[1]);

    // Realsense projection image SDK
    // Projected points: left_bottom=(609.64, 229.95), right_bottom=(46.08, 229.70), left_top=(401.47, 8.60), right_top=(252.50, 8.82)

    if (use_realsense_ros)
    {
        pixel_left_bottom[0] = 609.64f;
        pixel_left_bottom[1] = 229.95f;
        pixel_right_bottom[0] = 46.08f;
        pixel_right_bottom[1] = 229.70f;
        pixel_left_top[0] = 401.47f;
        pixel_left_top[1] = 8.60f;
        pixel_right_top[0] = 252.50f;
        pixel_right_top[1] = 8.82f;
    }

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
    bev_color_image_raw = bev_color_image.clone();

    cv::Mat bev_obs_binary;
    cv::warpPerspective(thres_yuv, bev_obs_binary, perspective_matrix, cv::Size(bev_width_, bev_height_));

    cv::Mat bev_filtered_binary;
    cv::warpPerspective(cleaned_binary, bev_filtered_binary, perspective_matrix, cv::Size(bev_width_, bev_height_));

    cv::Mat bev_depth_thres_obs;
    cv::warpPerspective(depth_thres, bev_depth_thres_obs, perspective_matrix, cv::Size(bev_width_, bev_height_));

    // Apply Hough Line detection optimized preprocessing
    cv::Mat bev_gray_frame;
    cv::cvtColor(bev_color_image, bev_gray_frame, cv::COLOR_BGR2GRAY);

    // Enhanced preprocessing for line detection
    // cv::GaussianBlur(bev_gray_frame, bev_gray_frame, cv::Size(15, 15), 0);

    // Apply adaptive threshold for better edge detection
    cv::Mat bev_binary;
    // Convert BEV image to HSV for color-based thresholding
    cv::Mat bev_hsv_frame;
    cv::cvtColor(bev_color_image, bev_hsv_frame, cv::COLOR_BGR2HSV);

    // Apply HSV threshold to detect white/bright areas (likely lane markings)
    // cv::inRange(bev_hsv_frame, cv::Scalar(0, 0, 200), cv::Scalar(180, 255, 255), bev_binary);

    // blur the hsv frame
    // cv::GaussianBlur(bev_hsv_frame, bev_hsv_frame, cv::Size(5, 5), 0);

    { //*   THRESHOLDING BEV HSV IMAGE  *//
        if (controlbox_data[0] > controlbox_data[3])
            std::swap(controlbox_data[0], controlbox_data[3]);
        if (controlbox_data[1] > controlbox_data[4])
            std::swap(controlbox_data[1], controlbox_data[4]);
        if (controlbox_data[2] > controlbox_data[5])
            std::swap(controlbox_data[2], controlbox_data[5]);

        cv::inRange(bev_hsv_frame, cv::Scalar(controlbox_data[0], controlbox_data[1], controlbox_data[2]),
                    cv::Scalar(controlbox_data[3], controlbox_data[4], controlbox_data[5]), bev_binary);
    }

    //? ==================================================================
    //? RESET STATE VARIABLES IF bev_binary IMAGE IS 90% BLACK
    //? ==================================================================
    // double black_pixel_ratio = static_cast<double>(cv::countNonZero(bev_binary)) / (bev_binary.rows * bev_binary.cols);
    // logger.info("Black pixel ratio: %.2f", black_pixel_ratio);
    // if (black_pixel_ratio <= 0.0001) {
    //     reset_state();
    //     logger.warn("=============\nBEV binary image is too dark, resetting state variables\n==========");
    //     return;
    // }

    if (button_1)
    {
        reset_state();
        logger.warn("=============\nBEV binary image is too dark, resetting state variables\n==========");
        return;
    }

    // cv::dilate(bev_binary, bev_binary, cv::Mat(), cv::Point(-1, -1), 5);
    // cv::morphologyEx(bev_binary, bev_binary, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1, -1), 5);
    // cv::erode(bev_binary, bev_binary, cv::Mat(), cv::Point(-1, -1), 5);

    cv::Mat bev_binary_raw = bev_binary.clone();
    cv::Mat bev_binary_copy = bev_binary.clone();

    cv::dilate(bev_obs_binary, bev_obs_binary, cv::Mat(), cv::Point(-1, -1), 5);
    cv::bitwise_or(bev_binary, bev_obs_binary, bev_binary);

    cv::addWeighted(bev_binary_raw, 0.7, bev_binary, 0.3, 0, bev_binary_raw);

    // Connect broken lines using morphological operations
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::dilate(bev_binary, bev_binary, kernel, cv::Point(-1, -1), 1);
    cv::erode(bev_binary, bev_binary, kernel, cv::Point(-1, -1), 1);

    // Close gaps in lines using morphological closing
    cv::Mat line_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 1));
    cv::morphologyEx(bev_binary, bev_binary, cv::MORPH_CLOSE, line_kernel);

    std::vector<cv::Point> horizontal_points = sliding_windows(bev_binary, bev_binary.cols, 5,
                                                               3, bev_color_image);
    std::vector<cv::Point> vertical_points = sliding_windows(bev_binary, 5, bev_binary.rows,
                                                             3, bev_color_image);

    std::vector<cv::Point> all_points;
    all_points.insert(all_points.end(), horizontal_points.begin(), horizontal_points.end());
    all_points.insert(all_points.end(), vertical_points.begin(), vertical_points.end());

    for (const auto &point : all_points)
        cv::circle(bev_binary, point, 4, cv::Scalar(255), -1);

    cv::Mat display_thres_bev = bev_binary.clone();

    //! ==================================================
    //!        Draw Robot and look ahead distance
    //! ==================================================
    // lookahead_far_pixel_ = (speed_to_lookahead(speed_motor)) * meter_to_pixel_ * 1;
    // lookahead_near_pixel_ = (speed_to_lookahead(speed_motor)) * meter_to_pixel_ * 1;

    lookahead_far_pixel_ = lookahead_near_meter_ * meter_to_pixel_;
    lookahead_near_pixel_ = lookahead_near_meter_ * meter_to_pixel_;

    cv::Mat bev_debug_binary = cv::Mat::zeros(bev_binary.size(), CV_8UC1);

    robot_position_.x = static_cast<int>(bev_width_ * 0.5);
    robot_position_.y = static_cast<int>(bev_height_ - 1);

    robot_position_pixel_real_ = get_robot_position_pixel_real(robot_position_);
    robot_position_pixel_camera_ = get_robot_position_pixel_camera(robot_position_);
    robot_position_meter_real_ = get_robot_position_meter_real(robot_position_);

    // for (int y = 0; y < bev_height_; y += 100) {
    //     cv::line(bev_color_image, cv::Point(0, y), cv::Point(bev_width_, y), cv::Scalar(255), 1);
    // }

    cv::line(bev_color_image, cv::Point(0, valid_up_), cv::Point(bev_color_image.cols, valid_up_), cv::Scalar(255), 1);
    cv::line(bev_color_image, cv::Point(0, valid_down_), cv::Point(bev_color_image.cols, valid_down_), cv::Scalar(255), 1);
    cv::line(bev_color_image, cv::Point(valid_center_left_, 0), cv::Point(valid_center_left_, bev_color_image.rows), cv::Scalar(255), 1);
    cv::line(bev_color_image, cv::Point(valid_center_right_, 0), cv::Point(valid_center_right_, bev_color_image.rows), cv::Scalar(255), 1);
    cv::line(bev_color_image, cv::Point(0, cropping_distance_), cv::Point(bev_color_image.cols, cropping_distance_), cv::Scalar(255), 1);

    for (float i = 0.2; i < 1.2; i += 0.1)
        cv::circle(bev_color_image, robot_position_, i * meter_to_pixel_, cv::Scalar(255, 0, 0), 1);

    cv::Mat dashed_line_filtered = cv::Mat::zeros(bev_binary.size(), CV_8UC1);
    cv::Mat dashed_line_filtered_left = cv::Mat::zeros(bev_binary.size(), CV_8UC1);
    cv::Mat dashed_line_filtered_right = cv::Mat::zeros(bev_binary.size(), CV_8UC1);
    cv::Mat dashed_line_filtered_edge_left = cv::Mat::zeros(bev_binary.size(), CV_8UC1);
    cv::Mat dashed_line_filtered_edge_right = cv::Mat::zeros(bev_binary.size(), CV_8UC1);

    cv::line(bev_color_image, robot_position_, cv::Point(robot_position_.x, 0), cv::Scalar(255, 100, 100), 1);

    // Find contours in the thresholded image
    std::vector<std::vector<cv::Point>> contours;
    cv::erode(bev_binary_copy, bev_binary_copy, cv::Mat(), cv::Point(-1, -1), 1);
    cv::findContours(bev_binary_copy, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Point> detected_points;
    //? =======================================================
    //?                  Rotated Rect
    //? =======================================================
    cv::Mat rectangle = cv::Mat::zeros(bev_color_image.size(), CV_8UC1);
    // Process each contour to find rotated rectangles
    for (const auto &contour : contours)
    {
        if (cv::contourArea(contour) > 20)
        { // Filter small contours
            // Get minimum area rotated rectangle
            cv::RotatedRect rotated_rect = cv::minAreaRect(contour);

            // Calculate confidence based on area ratio
            double contour_area = cv::contourArea(contour);
            double rect_area = rotated_rect.size.width * rotated_rect.size.height;

            double confidence = (contour_area / rect_area) * 100.0;

            // Only process rectangles with 80% confidence or higher
            if (confidence >= 5.0 && (rotated_rect.size.width < 100 && rotated_rect.size.height < 100))
            {
                // Get the 4 corner points of the rotated rectangle
                cv::Point2f vertices[4];
                rotated_rect.points(vertices);

                // Draw the rotated rectangle
                for (int i = 0; i < 4; i++)
                {
                    // cv::line(bev_color_image, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
                }
                cv::fillPoly(rectangle, std::vector<cv::Point>{cv::Point(vertices[0]), cv::Point(vertices[1]), cv::Point(vertices[2]), cv::Point(vertices[3])}, cv::Scalar(255));

                // Draw center point and angle
                // cv::circle(bev_color_image, rotated_rect.center, 5, cv::Scalar(255, 0, 0), -1);
                // cv::putText(bev_color_image, std::to_string(static_cast<int>(confidence)) + "%",
                //     rotated_rect.center, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);

                detected_points.push_back(rotated_rect.center);
            }
        }
    }

    cv::dilate(rectangle, rectangle, cv::Mat(), cv::Point(-1, -1), 5);

    cv::bitwise_not(rectangle, rectangle);
    // cv::bitwise_and(bev_binary, rectangle, bev_binary);
    cv::bitwise_not(rectangle, rectangle);
    //? =======================================================

    // Draw robot position using rectangle
    // cv::rectangle(bev_color_image, cv::Point(robot_position_.x - 20, robot_position_.y - 20),
    //     cv::Point(robot_position_.x + 20, robot_position_.y + 20),
    //     cv::Scalar(0, 255, 0), -1);

    cv::circle(bev_color_image, cv::Point(robot_position_.x, robot_position_.y + 20), 5, cv::Scalar(0, 100, 0), -1);

    // draw max and min target steering
    float target_x_max = cos((max_steering_deg_ + 90) * M_PI / 180) * lookahead_far_pixel_ + robot_position_.x;
    float target_y_max = robot_position_.y - sin((max_steering_deg_ + 90) * M_PI / 180) * lookahead_far_pixel_;
    float target_x_min = cos((-max_steering_deg_ + 90) * M_PI / 180) * lookahead_far_pixel_ + robot_position_.x;
    float target_y_min = robot_position_.y - sin((-max_steering_deg_ + 90) * M_PI / 180) * lookahead_far_pixel_;

    cv::line(bev_color_image, cv::Point(robot_position_.x, robot_position_.y),
             cv::Point(target_x_max, target_y_max), cv::Scalar(0, 255, 255), 2);
    cv::line(bev_color_image, cv::Point(robot_position_.x, robot_position_.y),
             cv::Point(target_x_min, target_y_min), cv::Scalar(0, 255, 255), 2);

    //* ==================================================
    //*            OPTIMALIZATION FLOOD FILL
    //* ==================================================
    cv::Mat valid_region_mask = cv::Mat::zeros(bev_binary.size(), CV_8UC1);
    cv::Mat line_valid_region = cv::Mat::zeros(bev_binary.size(), CV_8UC1);

    cv::Point points[1][6];
    points[0][0] = cv::Point(0, cropping_distance_);
    points[0][1] = cv::Point(0, valid_up_);
    points[0][2] = cv::Point(valid_center_left_, valid_down_);
    points[0][3] = cv::Point(valid_center_right_, valid_down_);
    points[0][4] = cv::Point(bev_width_, valid_up_);
    points[0][5] = cv::Point(bev_width_, cropping_distance_);
    const cv::Point *ppt[1] = {points[0]};
    int npt[] = {6};
    cv::fillPoly(valid_region_mask, ppt, npt, 1, cv::Scalar(255));

    cv::line(line_valid_region, cv::Point(0, cropping_distance_), cv::Point(0, valid_up_), cv::Scalar(255), 5);
    cv::line(line_valid_region, cv::Point(0, valid_up_), cv::Point(valid_center_left_, valid_down_), cv::Scalar(255), 5);
    cv::line(line_valid_region, cv::Point(valid_center_left_, valid_down_), cv::Point(valid_center_right_, valid_down_), cv::Scalar(255), 20);
    cv::line(line_valid_region, cv::Point(valid_center_right_, valid_down_), cv::Point(bev_width_, valid_up_), cv::Scalar(255), 5);
    cv::line(line_valid_region, cv::Point(bev_width_, valid_up_), cv::Point(bev_width_, cropping_distance_), cv::Scalar(255), 5);
    cv::line(line_valid_region, cv::Point(bev_width_, cropping_distance_), cv::Point(0, cropping_distance_), cv::Scalar(255), 5);
    cv::circle(line_valid_region, robot_position_, bev_height_ - cropping_distance_, cv::Scalar(255, 0, 0), 5);

    // cv::dilate(bev_binary, bev_binary, cv::Mat(), cv::Point(-1, -1), 10);
    cv::bitwise_or(bev_binary, line_valid_region, bev_binary);
    cv::addWeighted(bev_binary_raw, 0.5, bev_binary, 0.5, 0, bev_binary_raw);
    cv::addWeighted(bev_depth_thres_obs, 0.5, bev_binary_raw, 0.5, 0, bev_binary_raw);

    cv::Mat bev_cleaned_binary = cv::Mat::zeros(bev_binary.size(), CV_8UC1);
    {
        //? ==================================================
        //?                   FLOOD FILL 1
        //? ==================================================
        dynamic_flood_fill(7, 5, 100, bev_binary, bev_color_image, bev_cleaned_binary);
        // cv::erode(bev_binary, bev_binary, cv::Mat(), cv::Point(-1, -1), 10);
        // cv::dilate(bev_cleaned_binary, bev_cleaned_binary, cv::Mat(), cv::Point(-1, -1), 10);
        cv::morphologyEx(bev_cleaned_binary, bev_cleaned_binary, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)), cv::Point(-1, -1), 5);
    }
    {
        //? ==================================================
        //?                    FLOOD FILL 2
        //? ==================================================
        // adaptive_flood_fill(7, 5, 100, bev_binary, bev_color_image, bev_cleaned_binary, prev_saved_dashed_centroid_);

        // for (size_t i = 0; i < prev_saved_dashed_centroid_.size(); i++) {
        //     cv::circle(bev_color_image, prev_saved_dashed_centroid_[i], 5, cv::Scalar(255, 0, 0), -1);
        // }

        // cv::erode(bev_binary, bev_binary, cv::Mat(), cv::Point(-1, -1), 10);
        // cv::dilate(bev_cleaned_binary, bev_cleaned_binary, cv::Mat(), cv::Point(-1, -1), 10);
        // cv::morphologyEx(bev_cleaned_binary, bev_cleaned_binary, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)), cv::Point(-1, -1), 5);
    }
    //? ==================================================
    //?                    SEGMENT ROAD
    //? ==================================================
    float segment_speed_[4] = {segment_speed_1, segment_speed_2, segment_speed_3, segment_speed_4};
    road_segment(bev_cleaned_binary, bev_color_image, 4, road_segment_threshold_area, segment_speed_, 1);

    // for (int i = 0; i < 4; i++) {
    //     logger.info("Segment %d speed: %.2f", i, segment_speed[i]);
    // }
    //? ==================================================
    //?                  SEGMENT ROAD END
    //? ==================================================

    std::vector<std::vector<cv::Point>> filtered_contours;
    cv::findContours(bev_cleaned_binary, filtered_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat fix_bev_cleaned_binary = cv::Mat::zeros(bev_cleaned_binary.size(), CV_8UC1);

    std::sort(filtered_contours.begin(), filtered_contours.end(),
              [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b)
              {
                  return cv::contourArea(a) > cv::contourArea(b);
              });
    if (!filtered_contours.empty())
        cv::drawContours(fix_bev_cleaned_binary, filtered_contours, 0, cv::Scalar(255), cv::FILLED);

    bev_cleaned_binary = fix_bev_cleaned_binary.clone();
    //* ==================================================
    //*           OPTIMALIZATION FLOOD FILL END
    //* ==================================================

    //  ==================================================
    //                   EDGE REFERENCE
    //  ==================================================
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
    //  ==================================================
    //                 EDGE REFERENCE END
    //  ==================================================

    //! ==================================================
    //!            DASHED LINE & OBS CROPPING
    //! ==================================================
    cv::Mat dashed_line = display_thres_bev.clone();
    cv::erode(dashed_line, dashed_line, cv::Mat(), cv::Point(-1, -1), 1);

    cv::bitwise_and(bev_cleaned_binary, dashed_line, dashed_line);
    // cv::bitwise_and(rectangle, dashed_line, dashed_line);

    // Remove left and right detected lane (avoid noise)
    cv::Mat outer_line = bev_filtered_binary.clone();
    cv::dilate(outer_line, outer_line, cv::Mat(), cv::Point(-1, -1), 2);
    cv::bitwise_not(outer_line, outer_line);
    cv::bitwise_and(outer_line, dashed_line, dashed_line);

    cv::Mat bev_obs_binary_cropped = bev_obs_binary.clone();
    cv::dilate(bev_obs_binary_cropped, bev_obs_binary_cropped, cv::Mat(), cv::Point(-1, -1), 3);
    cv::bitwise_and(bev_obs_binary_cropped, bev_cleaned_binary, bev_obs_binary_cropped);
    cv::bitwise_xor(bev_obs_binary_cropped, bev_cleaned_binary, bev_cleaned_binary);
    //! ==================================================
    //!          DASHED LINE & OBS CROPPING END
    //! ==================================================

    //? ===============================================================
    //?                DETEKSI BELOKAN
    //? ===============================================================
    // get width of bounding box of the contours filtered_contours
    int bounding_box_width_ = 0;
    int area_bounding_box = 0;
    float turn_percentage = 0;
    int turn_direction = 0; // 0 = straight, -1 = left, 1 = right

    if (!filtered_contours.empty())
    {
        // Use the top-most contour for turn detection
        size_t top_most_idx = 0;
        int min_y = cv::boundingRect(filtered_contours[0]).y;
        for (size_t i = 1; i < filtered_contours.size(); ++i)
        {
            int y = cv::boundingRect(filtered_contours[i]).y;
            if (y < min_y)
            {
                min_y = y;
                top_most_idx = i;
            }
        }
        cv::Rect bounding_box = cv::boundingRect(filtered_contours[top_most_idx]);
        area_bounding_box = static_cast<int>(cv::contourArea(filtered_contours[top_most_idx]));
        bounding_box_width_ = bounding_box.width;
        turn_percentage = static_cast<float>(bounding_box_width_) / static_cast<float>(bev_width_);

        // Check turn direction based on contour position relative to center
        int contour_center_x = bounding_box.x + bounding_box.width / 2;
        int image_center_x = bev_width_ / 2;

        if (contour_center_x < image_center_x - 10)
            turn_direction = -1; // Turn left
        else if (contour_center_x > image_center_x + 10)
            turn_direction = 1; // Turn right
        else
            turn_direction = 0; // Straight
    }

    if (turn_percentage > 0.5f || area_bounding_box < 22000) //! NGAWUR
    {
        ada_belokan_ = 1;
        pos_ada_belokan = enc_meter;

        // Log turn direction
        if (turn_direction == -1)
        {
            // logger.info("Turn detected: LEFT");
        }
        else if (turn_direction == 1)
        {
            // logger.info("Turn detected: RIGHT");
        }
    }

    //! JADIKAN PARAMETER
    if (enc_meter - pos_ada_belokan > 0.3f && ada_belokan_ == 1)
    {
        ada_belokan_ = 0;
        pos_ada_belokan = 0;
    }

    //? ====================================================================
    //?         Check if there is any obstacle in the road
    //? ====================================================================

    // // find contour obstacles

    //! ====================================================================
    //!          OBS DETECTION
    //! ====================================================================
    used_reference_ = DASHED_REFERENCE;

    std::vector<std::vector<cv::Point>> obs_contours;
    cv::findContours(bev_obs_binary_cropped, obs_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::Point longest_point(robot_position_.x, robot_position_.y); //! unused
    cv::Mat bev_cleaned_binary_obs_cropped = cv::Mat::zeros(bev_binary.size(), CV_8UC1);

    // find the largest contour in bev_depth_thres_obs
    std::vector<std::vector<cv::Point>> filtered_depth_obs_contours;
    cv::findContours(bev_depth_thres_obs, filtered_depth_obs_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<std::vector<cv::Point>> sorted_obs_contours;

    float abs_angle = std::min(fabs(target_final_angle_ * M_PI / 180.0f * constant_speed_belok_), M_PI / 2.0f); // Limit angle to 90 degrees);
    float cos_angle = cos(abs_angle);
    float angle_const = fabs(cos_angle);

    jarak_actual_hindar += enc_speed * angle_const; // in meter
    // logger.info("jarak: %.2f , %.2f", jarak_actual_hindar, enc_speed);
    // logger.info("speed ACTUAL HINDAR: %.2f m || %.3f || angle: %.2f || cos: %.2f", enc_speed * angle_const, angle_const, abs_angle, cos_angle);
    // logger.info("actual hindar: %.2f m || speed real: %.3f m/s || angle: %.2f deg || cos: %.2f",
    // jarak_actual_hindar, enc_speed, abs_angle * 180.0f / M_PI, cos_angle);

    if (!filtered_depth_obs_contours.empty())
    {
        // Sort contours by bounding box area and keep only the largest one
        std::sort(filtered_depth_obs_contours.begin(), filtered_depth_obs_contours.end(),
                  [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b)
                  {
                      return cv::boundingRect(a).area() > cv::boundingRect(b).area();
                  });

        // logger.info("area : %.2f", cv::boundingRect(filtered_depth_obs_contours[0]).area());
        cv::Point centroid_depth_obs = getCentroid(filtered_depth_obs_contours[0]);

        // Draw the largest contour on the cleaned binary image
        bev_depth_thres_obs = cv::Mat::zeros(bev_depth_thres_obs.size(), CV_8UC1);
        cv::drawContours(bev_depth_thres_obs, filtered_depth_obs_contours, 0, cv::Scalar(255), cv::FILLED);

        //? =====================================================================
        //? FIND CLOSEST POINTS FROM OBS CENTROID ON LEFT AND RIGHT LANE
        //? =====================================================================

        float min_dist_left = std::numeric_limits<float>::max();  // jarak robot ke kiri jalan
        float min_dist_right = std::numeric_limits<float>::max(); // jarak robot ke kanan jalan

        // if (((dist_to_robot / meter_to_pixel_) + offset_speed_to_dist(enc_speed)) < 1.0) {
        //     logger.info("Obstacle PELAN");
        // }

        cv::Point closest_point_obs_lane_kiri(0, 0);
        cv::Point closest_point_obs_lane_kanan(0, 0);
        std::vector<cv::Point> valid_points;

        for (const auto &pt : left_flood_fill_points)
        {
            float dist = cv::norm(pt - centroid_depth_obs);
            if (dist < min_dist_left)
            {
                min_dist_left = dist;
                closest_point_obs_lane_kiri = pt;
            }
        }

        for (const auto &pt : right_flood_fill_points)
        {
            float dist = cv::norm(pt - centroid_depth_obs);
            if (dist < min_dist_right)
            {
                min_dist_right = dist;
                closest_point_obs_lane_kanan = pt;
            }
        }

        //* =============================
        //* FIND DIST ROBOT TO OBS
        //* =============================

        if (min_dist_left < min_dist_right)
            valid_points = left_flood_fill_points;
        else
            valid_points = right_flood_fill_points;

        // remove point that to close to obs
        valid_points.erase(std::remove_if(valid_points.begin(), valid_points.end(),
                                          [&](const cv::Point &pt)
                                          {
                                              return cv::norm(pt - centroid_depth_obs) < 100;
                                          }),
                           valid_points.end());

        // remove point that to close to obs
        valid_points.erase(std::remove_if(valid_points.begin(), valid_points.end(),
                                          [&](const cv::Point &pt)
                                          {
                                              return cv::norm(pt - robot_position_) < 100;
                                          }),
                           valid_points.end());

        cv::Point prev_valid_point = robot_position_;

        float total_dist_to_robot = cv::norm(centroid_depth_obs - robot_position_);

        if (!valid_points.empty())
        {

            total_dist_to_robot = 10;

            float prev_deg = 0;
            float total_angle = 0;
            float offset_length = min_dist_left < min_dist_right ? 40 : -40;
            float threshold_jarak = 0;

            cv::Point prev_second_point = robot_position_;

            for (size_t i = 0; i < valid_points.size() - 1; i++)
            {
                cv::circle(bev_color_image, valid_points[i], 2, cv::Scalar(255, 0, 0), -1);

                float angle_deg = computeAngle(valid_points[i], valid_points[i + 1]);

                if (i != 0)
                {
                    total_angle += fabs(prev_deg - angle_deg);
                    // logger.info("angle between: %.2f", (prev_deg - angle_deg));
                }

                cv::Point second_point;

                second_point.x = static_cast<int>(valid_points[i].x - (offset_length)*std::cos(angle_deg * M_PI / 180.0f));
                second_point.y = static_cast<int>(valid_points[i].y - (offset_length)*std::sin(angle_deg * M_PI / 180.0f));

                if (i == 0)
                    threshold_jarak = 150;
                else
                    threshold_jarak = 60;

                if (cv::norm(second_point - prev_second_point) > threshold_jarak)
                    continue; // Skip points that are too far to the previous point

                total_dist_to_robot += cv::norm(second_point - prev_second_point);

                cv::line(bev_color_image, second_point, prev_second_point, cv::Scalar(100, 100, 0), 6);

                prev_second_point = second_point;
                prev_deg = angle_deg;
            }

            total_dist_to_robot += cv::norm(prev_second_point - centroid_depth_obs);
            cv::line(bev_color_image, prev_second_point, centroid_depth_obs, cv::Scalar(100, 100, 0), 6);
        }

        if (cv::norm(centroid_depth_obs - robot_position_) < 100)
            dist_to_robot = cv::norm(centroid_depth_obs - robot_position_); // jarak centroid ke robot
        else
            dist_to_robot = total_dist_to_robot; // jarak centroid ke robot

        dist_to_robot_meter = std::max((dist_to_robot / meter_to_pixel_) - offset_speed_to_dist(speed_motor), 0.0); // jarak centroid ke robot dalam meter

        if (min_dist_right < 50 || min_dist_left < 50)
        {

            ada_obs_ = 1;
            // used_reference_ = EDGE_REFERENCE;

            if (centroid_depth_obs.x > 0 && centroid_depth_obs.y > 0)
            {
                cv::circle(bev_color_image, centroid_depth_obs, 8, cv::Scalar(0, 0, 0), -1);
                cv::circle(bev_color_image, centroid_depth_obs, 7, cv::Scalar(0, 100, 255), -1);
            }

            jarak_hindar = std::max(jarak_hindar_meter_ + offset_speed_to_dist(speed_motor), min_dist_jarak_hindar_);

            uint8_t cek_obs = min_dist_left > min_dist_right;

            //? SWAP VARIABLE
            if (origin_used_lane_ == LEFT_LANE)
            {
                cek_obs = min_dist_left < min_dist_right;
                cv::Point temp = closest_point_obs_lane_kiri;
                closest_point_obs_lane_kiri = closest_point_obs_lane_kanan;
                closest_point_obs_lane_kanan = temp;
            }

            if (cek_obs)
            {
                ada_obs_kanan_ = cek_obs;
                // used_lane_ = CENTER_LANE;

                cv::line(bev_color_image, centroid_depth_obs, closest_point_obs_lane_kanan, cv::Scalar(0, 0, 255), 7);
                cv::circle(bev_color_image, centroid_depth_obs, 15, cv::Scalar(0, 0, 255), -1);

                if ((dist_to_robot_meter < jarak_hindar))
                {

                    // if (prev_ada_obs_ == 0) {
                    //     pos_enc_hindar = jarak_actual_hindar;
                    //     counter_switch_lane = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                    // }
                    used_lane_ = !origin_used_lane_;
                    out_duration = ada_belokan_ ? out_duration_belokan_ : out_duration_normal_;
                }
            }
            else
            {
                ada_obs_kanan_ = cek_obs;
                cv::circle(bev_color_image, centroid_depth_obs, 15, cv::Scalar(255, 0, 0), -1);

                if (prev_ada_obs_ == 0)
                {
                    pos_enc_hindar = jarak_actual_hindar;
                    counter_switch_lane = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                }

                cv::line(bev_color_image, centroid_depth_obs, closest_point_obs_lane_kiri, cv::Scalar(255, 0, 0), 7);
            }
        }
    }

    double time_now_switch = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    // if ((used_lane_ != origin_used_lane_) && (ada_obs_ && ada_obs_kanan_)) {
    //     if (enc_meter - pos_enc_hindar > max_enc_meter_obs_center_ || (time_now_switch - counter_switch_lane > out_duration + offset_out_duration_center_)) {
    //         used_lane_ = origin_used_lane_;
    //         used_reference_ = DASHED_REFERENCE;
    //         ada_obs_ = 0;
    //     }
    // }

    float enc_meter_keluar = std::max(max_enc_meter_obs_ + offset_speed_to_dist(speed_motor), min_dist_jarak_keluar_); // Minimum distance to switch lane

    if (ada_obs_)
    {
        logger.info("=============== Masuk ke kondisi keluar ===============");

        if (jarak_actual_hindar - pos_enc_hindar > enc_meter_keluar && perpindahan_lane_)
        {
            if (ada_obs_kanan_ && keluar_enc_)
            {
                // used_lane_ = CENTER_LANE;
                // keluar_enc_ = 1;
                awal_center_lane = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                used_reference_ = DASHED_REFERENCE;
                used_lane_ = origin_used_lane_;
                ada_obs_ = 0;
                keluar_enc_ = 0;
            }

            if (!ada_obs_kanan_)
            {
                ada_obs_ = 0;
                used_reference_ = DASHED_REFERENCE;
                used_lane_ = origin_used_lane_;
            }

            // if (ada_obs_kanan_ && keluar_enc_) {
            //     if (time_now_switch - awal_center_lane > offset_out_duration_center_) {
            //         used_lane_ = origin_used_lane_;
            //         used_reference_ = DASHED_REFERENCE;
            //         ada_obs_ = 0;
            //         keluar_enc_ = 0;
            //     }
            // }
        }
        else
        {
            used_reference_ = EDGE_REFERENCE;
            if (ada_obs_kanan_)
            {
                // used_reference_ = EDGE_REFERENCE;
                keluar_enc_ = 1;
            }
        }
    }

    // logger.info("%d || %d || %d || %.2f", jarak_actual_hindar - pos_enc_hindar > enc_meter_keluar && (time_now_switch - counter_switch_lane > out_duration), ada_obs_kanan_, keluar_enc_, pos_enc_hindar);

    ada_obs_slow_down_ = (!filtered_depth_obs_contours.empty()) && (dist_to_robot_meter < (jarak_hindar_meter_ * 1.5) + offset_speed_to_dist(speed_motor)) || keluar_enc_ || (ada_obs_kanan_ && ada_obs_);

    std::string offset_speed_str = "OFFSET SPEED: " + std::to_string(offset_speed_to_dist(speed_motor)) + " m";
    std::string jarak_hindar_str = "JARAK HINDAR: " + std::to_string(jarak_hindar) + " m -> " + std::to_string(dist_to_robot / meter_to_pixel_) + " m";
    std::string dist_to_robot_str = "JARAK KELUAR: " + std::to_string(enc_meter_keluar) + " m -> " + std::to_string(jarak_actual_hindar - pos_enc_hindar) + " m";
    std::string ada_obs_str = ada_obs_slow_down_ ? "ADA OBSTACLE" : "TIDAK ADA OBSTACLE";

    cv::putText(bev_color_image, offset_speed_str, cv::Point(10, 130), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2.1); // HIJAU
    cv::putText(bev_color_image, jarak_hindar_str, cv::Point(10, 150), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2.1);   // HIJAU
    cv::putText(bev_color_image, dist_to_robot_str, cv::Point(10, 170), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2.1);  // MERAH
    cv::putText(bev_color_image, ada_obs_str, cv::Point(10, 190), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 2.1);      // KUNING

    //? ===============================================================
    //?                DETEKSI GARIS PUTUS-PUTUS
    //? ===============================================================

    // cv::dilate(bev_filtered_binary, bev_filtered_binary, cv::Mat(), cv::Point(-1, -1), 3);
    // cv::bitwise_not(bev_filtered_binary, bev_filtered_binary);
    // cv::bitwise_and(bev_filtered_binary, bev_cleaned_binary, bev_cleaned _binary);

    // find contour
    std::vector<std::vector<cv::Point>> dashed_contours;
    cv::findContours(dashed_line, dashed_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<cv::Point>> filtered_dashed_contours;

    std::vector<cv::Point> filtered_point_right;
    std::vector<cv::Point> filtered_point_left;

    cv::Mat dashed_line_cleaned = cv::Mat::zeros(dashed_line.size(), CV_8UC1);

    static float final_used_line_length = 0.0f;
    static float last_angle = 0.0f;
    static float total_angle = 0.0f;

    // Calculate average area with safety check
    float average_area = 0.0f;

    if (ada_obs_)
    {
        line_length_left_ = dash_kiri_anomali;
        line_length_right_ = dash_kanan_anomali;
        line_length_edge_left_ = edge_kiri_anomali;
        line_length_edge_right_ = edge_kanan_anomali;
    }
    else
    {
        line_length_left_ = dash_kiri_default;
        line_length_right_ = dash_kanan_default;
        line_length_edge_left_ = edge_kiri_default;
        line_length_edge_right_ = edge_kanan_default;
    }

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
            if (area < 50 || area > dashed_filter_area_)
            {
                dashed_contours.erase(dashed_contours.begin() + i);
                i--; // Adjust index after removal
            }
        }

        // filter contours based on area
        // for (size_t i = 0; i < dashed_contours.size(); i++)
        // {
        //     float area = cv::contourArea(dashed_contours[i]);
        //     if (area < average_area * 0.7f)
        //     {
        //         dashed_contours.erase(dashed_contours.begin() + i);
        //         i--; // Adjust index after removal
        //     }
        // }

        // sort contours by y position
        std::sort(dashed_contours.begin(), dashed_contours.end(),
                  [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b)
                  {
                      return cv::boundingRect(a).y > cv::boundingRect(b).y;
                  });

        // check the distance between contours
        for (size_t i = 0; i < dashed_contours.size(); i++)
        {

            // Check if the contour is too close to the previous one
            if (i > 0)
            {
                cv::Rect prev_bounding_rect = cv::boundingRect(dashed_contours[i - 1]);
                float dist = sqrt(pow(cv::boundingRect(dashed_contours[i]).x - prev_bounding_rect.x, 2) + pow(cv::boundingRect(dashed_contours[i]).y - prev_bounding_rect.y, 2));
                // logger.info("Distance between contours %d and %d: %.2f", i - 1, i, dist);
                if (dist > 100) // Adjust threshold as needed
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
        if (dashed_contours.size() >= 2)
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
            float angle_robot_to_first = computeAngle(robot_position_, saved_dashed_centroid[0].first);
            saved_dashed_centroid.insert(saved_dashed_centroid.begin(), std::make_pair(robot_position_, angle_robot_to_first));
        }

        prev_saved_dashed_centroid_.clear();
        for (const auto &dashed_centroid : saved_dashed_centroid)
            prev_saved_dashed_centroid_.push_back(dashed_centroid.first);

        //! ============================================================
        //? ============================================================
        //?         NORMALISASI LINE LENGTH
        //? ============================================================

        float line_length = 35.0f; // Length of the dashed line segments

        if (saved_dashed_centroid.size() > 1)
            last_angle = computeAngle(saved_dashed_centroid[saved_dashed_centroid.size() - 1].first, saved_dashed_centroid[saved_dashed_centroid.size() - 2].first);
        else if (saved_dashed_centroid.size() > 0)
            last_angle = computeAngle(saved_dashed_centroid[saved_dashed_centroid.size() - 1].first, robot_position_);
        last_angle = fabs(last_angle);

        if (last_angle > max_steering_deg_)
            last_angle = max_steering_deg_;

        float norm_angle = (max_steering_deg_ - last_angle) / max_steering_deg_;
        //? =====================================================================

        // if (ada_belokan_) {
        //     if (ada_obs_ && ada_obs_kanan_) {
        //         norm_angle = 1;
        //     } else if (ada_obs_ && !ada_obs_kanan_) {
        //         norm_angle = 0;
        //     } else {
        //         norm_angle = 0;
        //     }
        // } else {
        //     norm_angle = 1;
        // }

        //? ============================================================
        // line_length = line_length_min_ + (norm_angle * (line_length_max_ - line_length_min_));
        line_length = line_length_left_ + (norm_angle * (line_length_right_ - line_length_left_));
        final_used_line_length = line_length;

        // if (final_used_line_length < line_length)
        // {
        //     final_used_line_length += line_length * 0.01;
        //     if (final_used_line_length > line_length)
        //     {
        //         final_used_line_length = line_length;
        //     }
        // }
        // else if (final_used_line_length > line_length)
        // {
        //     final_used_line_length -= line_length * 0.8;
        //     if (final_used_line_length < line_length)
        //     {
        //         final_used_line_length = line_length;
        //     }
        // }

        cv::Point prev_mid_point(robot_position_.x, robot_position_.y);
        cv::Point prev_first_point(robot_position_.x, robot_position_.y);
        cv::Point prev_second_point(robot_position_.x, robot_position_.y);

        if (saved_dashed_centroid.size() < 3)
        {
            used_reference_ = EDGE_REFERENCE;
            // logger.info("No dashed line detected, using edge reference | size: %d <- %d ", saved_dashed_centroid.size(), dashed_contours.size());
        }

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
                second_point = cv::Point(robot_position_.x, robot_position_.y);
                first_point = cv::Point(robot_position_.x, robot_position_.y);
            }
            else
            {
                // second_point = saved_dashed_centroid[i].first;
                // first_point = saved_dashed_centroid[i - 1].first;
                second_point.x = static_cast<int>(saved_dashed_centroid[i].first.x + (line_length_left_)*std::cos(saved_dashed_centroid[i].second * M_PI / 180.0f));
                second_point.y = static_cast<int>(saved_dashed_centroid[i].first.y + (line_length_left_)*std::sin(saved_dashed_centroid[i].second * M_PI / 180.0f));

                first_point.x = static_cast<int>(saved_dashed_centroid[i].first.x - line_length_right_ * std::cos(saved_dashed_centroid[i].second * M_PI / 180.0f));
                first_point.y = static_cast<int>(saved_dashed_centroid[i].first.y - line_length_right_ * std::sin(saved_dashed_centroid[i].second * M_PI / 180.0f));
            }

            if (i == saved_dashed_centroid.size() - 1)
            {
                cv::Point last_point(0, 0);

                last_point.x = static_cast<int>(saved_dashed_centroid[i].first.x + (final_used_line_length + 30) * std::cos((saved_dashed_centroid[i].second + 90) * M_PI / 180.0f));
                last_point.y = static_cast<int>(saved_dashed_centroid[i].first.y + (final_used_line_length + 30) * std::sin((saved_dashed_centroid[i].second + 90) * M_PI / 180.0f));

                last_second_point.x = static_cast<int>(last_point.x + (line_length_left_)*std::cos(saved_dashed_centroid[i].second * M_PI / 180.0f));
                last_second_point.y = static_cast<int>(last_point.y + (line_length_left_)*std::sin(saved_dashed_centroid[i].second * M_PI / 180.0f));

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
    else
    {
        // logger.info("No dashed line detected, using edge reference");
        used_reference_ = EDGE_REFERENCE;
    }

    cv::Mat temp_bev_obs = cv::Mat::zeros(bev_binary.size(), CV_8UC1);

    cv::dilate(bev_obs_binary, temp_bev_obs, cv::Mat(), cv::Point(-1, -1), 15);
    cv::bitwise_not(temp_bev_obs, temp_bev_obs);

    cv::bitwise_and(temp_bev_obs, bev_filtered_binary, bev_filtered_binary);

    // cv::Mat bev_filtered_binary_clean = cv::Mat::zeros(bev_filtered_binary.size(), CV_8UC1);

    //? =====================================================================
    //?         REFERENSI EDGE KE 3
    //? =====================================================================

    //? =====================================================================
    float norm_edge = 0;
    if (ada_belokan_)
    {
        if (ada_obs_ && ada_obs_kanan_)
            norm_edge = 1.5;
        else if (ada_obs_ && !ada_obs_kanan_)
            norm_edge = 0;
        else
            norm_edge = 0;
    }
    else
    {
        norm_edge = 1;
    }

    //? ============================================================
    // float line_length_edge = (line_length_edge_max_ + line_length_edge_min_) - (norm_edge * (line_length_edge_max_ - line_length_edge_min_));
    // final_used_line_length = line_length_edge;

    if (!left_flood_fill_points.empty())
    {
        float prev_deg = 0;
        float total_angle = 0;
        // float line_length_ref_baru_ = final_used_line_length;
        cv::Point prev_second_point = robot_position_;

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

            second_point.x = static_cast<int>(left_flood_fill_points[i].x - (line_length_edge_left_)*std::cos(angle_deg * M_PI / 180.0f));
            second_point.y = static_cast<int>(left_flood_fill_points[i].y - (line_length_edge_left_)*std::sin(angle_deg * M_PI / 180.0f));

            cv::line(bev_color_image, second_point, prev_second_point, cv::Scalar(255, 255, 0), 2);
            cv::line(dashed_line_filtered_edge_left, second_point, prev_second_point, cv::Scalar(255), 5);

            prev_second_point = second_point;
            prev_deg = angle_deg;
        }
    }

    if (!right_flood_fill_points.empty())
    {
        float prev_deg = 0;
        float total_angle = 0;
        // float line_length_ref_baru_ = final_used_line_length;
        cv::Point prev_second_point = robot_position_;

        for (size_t i = 0; i < right_flood_fill_points.size() - 1; i++)
        {
            cv::circle(bev_color_image, right_flood_fill_points[i], 2, cv::Scalar(0, 0, 255), -1);

            float angle_deg = computeAngle(right_flood_fill_points[i], right_flood_fill_points[i + 1]);

            if (i != 0)
            {
                total_angle += fabs(prev_deg - angle_deg);
                // logger.info("angle between: %.2f", (prev_deg - angle_deg));
            }

            cv::Point second_point;

            second_point.x = static_cast<int>(right_flood_fill_points[i].x + (line_length_edge_right_)*std::cos(angle_deg * M_PI / 180.0f));
            second_point.y = static_cast<int>(right_flood_fill_points[i].y + (line_length_edge_right_)*std::sin(angle_deg * M_PI / 180.0f));

            cv::line(bev_color_image, second_point, prev_second_point, cv::Scalar(0, 255, 255), 2);
            cv::line(dashed_line_filtered_edge_right, second_point, prev_second_point, cv::Scalar(255), 5);

            prev_second_point = second_point;
            prev_deg = angle_deg;
        }
    }

    //? ==========================================================
    //? ==========================================================

    //? ============================================================
    //?         NORMALISASI LOOKAHEAD DISTANCE
    //? ============================================================
    // logger.info("Total angle: %.2f", total_angle);
    if (total_angle > max_steering_deg_)
        total_angle = max_steering_deg_;

    float norm_angle = (max_steering_deg_ - total_angle) / max_steering_deg_;
    float delta = lookahead_far_pixel_ - lookahead_near_pixel_;

    if (ada_obs_ || ada_belokan_)
        norm_angle = 0;

    used_lookahead = lookahead_near_pixel_ + (norm_angle * delta);

    if (!perpindahan_lane_)
        used_lookahead += 20;
    // target_velocity_ = speed_curve_ + (norm_angle * (speed_straight_ - speed_curve_));

    //? ============================================================
    //?         CREATE LOOKAHEAD MAT
    //? ============================================================

    used_lookahead_meter = used_lookahead / meter_to_pixel_;

    // Draw look-ahead distance far from robot position
    cv::circle(bev_color_image, cv::Point(robot_position_.x, robot_position_.y), static_cast<int>(lookahead_far_pixel_),
               cv::Scalar(255, 0, 0), 2);

    // Draw look-ahead distance near from robot position
    cv::circle(bev_color_image, cv::Point(robot_position_.x, robot_position_.y), static_cast<int>(lookahead_near_pixel_),
               cv::Scalar(0, 0, 255), 2);

    // Draw look-ahead distance used from robot position
    cv::circle(bev_color_image, cv::Point(robot_position_.x, robot_position_.y), static_cast<int>(used_lookahead),
               cv::Scalar(255, 0, 255), 2);

    // Draw look-ahead for centroid calculation
    cv::Mat look_ahead_used = cv::Mat::zeros(bev_binary.size(), CV_8UC1);
    cv::circle(look_ahead_used, cv::Point(robot_position_.x, robot_position_.y), static_cast<int>(used_lookahead),
               cv::Scalar(255), 5);

    cv::Mat look_ahead_obstacle = cv::Mat::zeros(bev_binary.size(), CV_8UC1);
    cv::circle(look_ahead_obstacle, cv::Point(robot_position_.x, robot_position_.y), static_cast<int>(120),
               cv::Scalar(255), 2);

    // cv::circle(bev_color_image, cv::Point(robot_position_.x, robot_position_.y), static_cast<int>(120),
    //            cv::Scalar(255, 0, 255), 2);

    static int pos_robot = 0;
    float min_dist_robot_left = std::numeric_limits<float>::max();
    float min_dist_robot_right = std::numeric_limits<float>::max();
    static int cntr_ada_di_kanan = 0;

    //! unused
    for (const auto &pt : left_flood_fill_points)
    {
        float dist = cv::norm(pt - robot_position_);
        if (dist < min_dist_robot_left)
            min_dist_robot_left = dist;
    }

    for (const auto &pt : right_flood_fill_points)
    {
        float dist = cv::norm(pt - robot_position_);
        if (dist < min_dist_robot_right)
            min_dist_robot_right = dist;
    }

    // if ((min_dist_robot_left + min_dist_robot_right) > 70) {

    //     if (origin_used_lane_ == RIGHT_LANE) {
    //         if ((min_dist_robot_left < min_dist_robot_right)) {
    //             pos_robot = 0;
    //             cntr_ada_di_kanan = 0;
    //         } else {
    //             cntr_ada_di_kanan++;

    //             cntr_ada_di_kanan = std::min(cntr_ada_di_kanan + 1, 30);
    //             pos_robot = 1;
    //         }
    //     } else {
    //         if ((min_dist_robot_left > min_dist_robot_right)) {
    //             pos_robot = 1;
    //             cntr_ada_di_kanan = 0;
    //         } else {
    //             cntr_ada_di_kanan++;

    //             cntr_ada_di_kanan = std::min(cntr_ada_di_kanan + 1, 30);
    //             pos_robot = 0;
    //         }
    //     }

    //     logger.info("ada obs: %d, pos_robot: %d, cntr jalur tepat: %d", ada_obs_, pos_robot, cntr_ada_di_kanan);

    //     if (cntr_ada_di_kanan > 20 && ada_obs_ == 1) {
    //         ada_obs_ = 0;
    //     }
    // }

    //? =====================================================================
    //?         Compute angle between robot position and closest point
    //? =====================================================================

    static cv::Point closest_point_used(robot_position_.x, robot_position_.y - 20);
    static cv::Point closest_point_used_kiri(robot_position_.x, robot_position_.y - 20);
    static cv::Point closest_point_used_kanan(robot_position_.x, robot_position_.y - 20);
    static cv::Point closest_point_used_edge_kiri(robot_position_.x, robot_position_.y - 20);
    static cv::Point closest_point_used_edge_kanan(robot_position_.x, robot_position_.y - 20);

    centroid_lookahead_used(look_ahead_used, dashed_line_filtered, closest_point_used, angle_used_);
    centroid_lookahead_used(look_ahead_used, dashed_line_filtered_left, closest_point_used_kiri, angle_used_kiri_);
    centroid_lookahead_used(look_ahead_used, dashed_line_filtered_right, closest_point_used_kanan, angle_used_kanan_);
    centroid_lookahead_used(look_ahead_used, dashed_line_filtered_edge_left, closest_point_used_edge_kiri, angle_used_edge_kiri_);
    centroid_lookahead_used(look_ahead_used, dashed_line_filtered_edge_right, closest_point_used_edge_kanan, angle_used_edge_kanan_);

    // cv::circle(bev_color_image, closest_point_used, 5, cv::Scalar(0, 255, 0), -1);
    // cv::circle(bev_color_image, closest_point_used_kiri, 5, cv::Scalar(255, 0, 0), -1);
    // cv::circle(bev_color_image, closest_point_used_kanan, 5, cv::Scalar(0, 0, 255), -1);
    // cv::circle(bev_color_image, closest_point_used_edge_kiri, 5, cv::Scalar(255, 255, 0), -1);
    // cv::circle(bev_color_image, closest_point_used_edge_kanan, 5, cv::Scalar(0, 255, 255), -1);

    // put text in bev_color_image if used reference is EDGE_REFERENCE or DASHED_REFERENCE
    if (used_reference_ == EDGE_REFERENCE)
        cv::putText(bev_color_image, "EDGE REFERENCE", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2.1); // BIRU
    else if (used_reference_ == DASHED_REFERENCE)
        cv::putText(bev_color_image, "DASHED REFERENCE", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2.1); // MERAH

    if (used_lane_ == LEFT_LANE)
        cv::putText(bev_color_image, "USE LEFT LANE", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2.1); // HITAM
    else if (used_lane_ == RIGHT_LANE)
        cv::putText(bev_color_image, "USE RIGHT LANE", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2.1); // PUTIH
    else if (used_lane_ == CENTER_LANE)
        cv::putText(bev_color_image, "USE CENTER LANE", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2.1); // KUNING

    if (ada_belokan_)
        cv::putText(bev_color_image, "ADA BELOKAN", cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2.1); // KUNING
    else
        cv::putText(bev_color_image, "TIDAK ADA BELOKAN", cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 2.1); // MAGENTA

    if (ada_obs_)
        if (ada_obs_kanan_)
            cv::putText(bev_color_image, "ADA DI JALUR", cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2.1); // MERAH
        else
            cv::putText(bev_color_image, "ADA BUKAN DI JALUR", cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2.1); // HIJAU
    else
        cv::putText(bev_color_image, "TIDAK ADA Obs", cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 2.1); // CYAN

    if (pos_robot)
        cv::putText(bev_color_image, "ROBOT DI KANAN", cv::Point(10, 110), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2.1); // MERAH
    else
        cv::putText(bev_color_image, "ROBOT DI KIRI", cv::Point(10, 110), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2.1); // HIJAU

    if (perpindahan_lane_ == 0)
        cv::putText(bev_color_image, "TRANSIENT LANE CHANGE", cv::Point(10, 210), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2.1); // KUNING
    else
        cv::putText(bev_color_image, "LANE CHANGE DONE", cv::Point(10, 210), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 2.1); // MAGENTA

    logger.info("reference: %d, lane: %d, Obs: %d | perpindahan: %d -> %d | %.2f", used_reference_, used_lane_, ada_obs_, perpindahan_lane_, prev_perpindahan_lane_, pos_enc_hindar);
    // used_reference_ = EDGE_REFERENCE;

    static float angle_used_buffer = 0.0f;
    static uint8_t buffer_used_lane = used_lane_;
    static float alpha_used_lane = 0.0f;
    static uint8_t current_lane = used_lane_;
    float counter_transient_small = constant_transient_speed;
    float counter_transient_large = 2 * constant_transient_speed;

    // counter_transient_large = counter_transient_large / speed_motor;
    // counter_transient_small = counter_transient_small / speed_motor;

    if (true)
    {
        if (used_reference_ == DASHED_REFERENCE)
        {
            //? ============================================================
            //?         USE DASHED REFERENCE
            //? ============================================================
            logger.info("used lane: %d, buffer used lane: %d, alpha used lane: %.2f", used_lane_, buffer_used_lane, alpha_used_lane);
            // TRANSIENT LANE CHANGE
            if (used_lane_ != buffer_used_lane)
            {
                perpindahan_lane_ = 0;
                if (used_lane_ == LEFT_LANE)
                    final_angle_used_for_steering_ = angle_used_kiri_ * alpha_used_lane + angle_used_buffer * (1.0f - alpha_used_lane);
                else if (used_lane_ == RIGHT_LANE)
                    final_angle_used_for_steering_ = angle_used_kanan_ * alpha_used_lane + angle_used_buffer * (1.0f - alpha_used_lane);
                if (alpha_used_lane > 0.4f)
                    alpha_used_lane += counter_transient_small;
                else
                    alpha_used_lane += counter_transient_large;
            }
            else
            {
                perpindahan_lane_ = 1;

                if (used_lane_ == LEFT_LANE)
                    final_angle_used_for_steering_ = angle_used_kiri_;
                else if (used_lane_ == RIGHT_LANE)
                    final_angle_used_for_steering_ = angle_used_kanan_;
            }

            if (alpha_used_lane > 0.6f)
            {
                perpindahan_lane_ = 1;
                buffer_used_lane = used_lane_;
                alpha_used_lane = 0.0f;
            }

            angle_used_buffer = final_angle_used_for_steering_;

            // if (used_lane_ == RIGHT_LANE) {
            //     //? ============================================================
            //     //?         USE DASHED REFERENCE RIGHT
            //     //? ============================================================
            //     final_angle_used_for_steering_ = angle_used_kanan_;
            //     cv::line(bev_color_image, robot_position_, closest_point_used_kanan, cv::Scalar(255, 0, 0), 2);
            // } else if (used_lane_ == LEFT_LANE) {
            //     //? ============================================================
            //     //?         USE DASHED REFERENCE LEFT
            //     //? ============================================================
            //     final_angle_used_for_steering_ = angle_used_kiri_;
            //     cv::line(bev_color_image, robot_position_, closest_point_used_kiri, cv::Scalar(255, 0, 0), 2);
            // } else {
            //     //? ============================================================
            //     //?         USE DASHED REFERENCE CENTER
            //     //? ============================================================
            //     final_angle_used_for_steering_ = angle_used_;
            //     cv::line(bev_color_image, robot_position_, closest_point_used, cv::Scalar(255, 0, 0), 2);
            // }
        }
        else if (used_reference_ == EDGE_REFERENCE)
        {
            logger.info("valid left: %d, valid right: %d, used lane: %d", left_valid, right_valid, used_lane_);
            if ((left_valid && right_valid) || (left_valid && used_lane_ == LEFT_LANE) || (right_valid && used_lane_ == RIGHT_LANE))
            {
                //? ============================================================
                //?         USE EDGE REFERENCE
                //? ============================================================
                logger.info("used lane: %d, buffer used lane: %d, alpha used lane: %.2f", used_lane_, buffer_used_lane, alpha_used_lane);
                // TRANSIENT LANE CHANGE
                if (used_lane_ != buffer_used_lane)
                {
                    perpindahan_lane_ = 0;
                    if (used_lane_ == LEFT_LANE)
                        final_angle_used_for_steering_ = angle_used_edge_kiri_ * alpha_used_lane + angle_used_edge_kanan_ * (1.0f - alpha_used_lane);
                    else if (used_lane_ == RIGHT_LANE)
                        final_angle_used_for_steering_ = angle_used_edge_kanan_ * alpha_used_lane + angle_used_buffer * (1.0f - alpha_used_lane);
                    if (alpha_used_lane > 0.4f)
                        alpha_used_lane += counter_transient_small;
                    else
                        alpha_used_lane += counter_transient_large;
                }
                else
                {
                    perpindahan_lane_ = 1;

                    if (used_lane_ == LEFT_LANE)
                        final_angle_used_for_steering_ = angle_used_edge_kiri_;
                    else if (used_lane_ == RIGHT_LANE)
                        final_angle_used_for_steering_ = angle_used_edge_kanan_;
                }

                if (alpha_used_lane > 0.6f)
                {
                    perpindahan_lane_ = 1;
                    buffer_used_lane = used_lane_;
                    alpha_used_lane = 0.0f;
                }

                angle_used_buffer = final_angle_used_for_steering_;
            }
            else if (!left_valid && used_lane_ == LEFT_LANE)
            {
                logger.info("left lane not valid, using max steering deg: %.2f", max_steering_deg_);
                final_angle_used_for_steering_ = -60;
            }
            else if (!right_valid && used_lane_ == RIGHT_LANE)
            {
                logger.info("right lane not valid, using max steering deg: %.2f", max_steering_deg_);
                final_angle_used_for_steering_ = 60;
            }
        }
    }
    else
    {
        final_angle_used_for_steering_ = computeAngle(longest_point, robot_position_);
    }

    if (prev_perpindahan_lane_ == 0 && perpindahan_lane_ == 1 && ada_obs_)
    {
        pos_enc_hindar = jarak_actual_hindar;
        counter_switch_lane = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    }

    prev_used_lane = used_lane_;
    prev_ada_obs_ = ada_obs_;
    prev_perpindahan_lane_ = perpindahan_lane_;

    //! =========================
    //!  draw lane in x axis in midle off robot
    //! =========================

    cv::circle(bev_color_image, cv::Point(robot_position_pixel_camera_.x, robot_position_pixel_camera_.y - (0.5 * meter_to_pixel_)), 2, cv::Scalar(0, 255, 255), -1);
    cv::circle(bev_color_image, cv::Point(robot_position_pixel_camera_.x - (0.1 * meter_to_pixel_), robot_position_pixel_camera_.y - (0.5 * meter_to_pixel_)), 2, cv::Scalar(0, 255, 255), -1);
    cv::circle(bev_color_image, cv::Point(robot_position_pixel_camera_.x + (0.1 * meter_to_pixel_), robot_position_pixel_camera_.y - (0.5 * meter_to_pixel_)), 2, cv::Scalar(0, 255, 255), -1);

    //! ============================================================
    //? ============================================================
    //?                      DRAW FINAL ANGLE
    //? ============================================================
    cv::Point final_angle_point;
    final_angle_point.x = static_cast<int>(robot_position_.x + used_lookahead * std::sin(final_angle_used_for_steering_ * M_PI / 180.0f));
    final_angle_point.y = static_cast<int>(robot_position_.y - used_lookahead * std::cos(final_angle_used_for_steering_ * M_PI / 180.0f));
    cv::circle(bev_color_image, final_angle_point, 5, cv::Scalar(255, 0, 255), -1);

    // final_angle_point.x = (robot_position_pixel_camera_.x + (0.1 * meter_to_pixel_));
    // final_angle_point.y = robot_position_pixel_camera_.y - (0.5 * meter_to_pixel_);
    cv::line(bev_color_image, cv::Point(robot_position_pixel_camera_.x, robot_position_pixel_camera_.y), final_angle_point, cv::Scalar(0, 0, 0), 2);

    //! UNTUK ACKERMAN DARI CAMERA
    // /* Menghitung target velocity */
    float dx = (final_angle_point.x - robot_position_pixel_camera_.x) / meter_to_pixel_;
    float dy = (robot_position_pixel_camera_.y - final_angle_point.y) / meter_to_pixel_;

    // /* Menghitung target steering angle */
    float direction = -atan2(dx, dy);
    // logger.info("dx: %.2f, dy: %.2f dir: %.2f", dx, dy, direction);
    float target_steering_angle = atan2(2 * wheelbase_ * sinf(direction), used_lookahead_meter + wheelbase_);
    while (target_steering_angle > M_PI)
        target_steering_angle -= 2 * M_PI;
    while (target_steering_angle < -M_PI)
        target_steering_angle += 2 * M_PI;

    target_steering_angle = target_steering_angle * 180.0f / M_PI; // Convert to degrees

    //? ============================================================
    //?                      PID CONTROLLER
    //? ============================================================
    static PID pid_steering;
    pid_steering.init(kp_steering_, ki_steering_, kd_steering_, elapsed_time, -max_steering_deg_, max_steering_deg_, -max_steering_deg_, max_steering_deg_);

    static float setpoint_angle = 0.0f;
    // // pseudorandom binary sequence for setpoint angle by time
    // // Generates a square wave (0 or max_steering_deg_) based on time for testing
    // double t = this->now().seconds();
    // int period = 2; // seconds
    // if (static_cast<int>(t / period) % 2 == 0) {
    //     setpoint_angle = max_steering_deg_ / 7;
    // } else {
    //     setpoint_angle = -max_steering_deg_ / 7;
    // }

    // target_steering_angle = 0;
    float error_angle = setpoint_angle - target_steering_angle;
    target_final_angle_ = pid_steering.calculate(error_angle);

    // logger.info("%.2f, %2.f, %.2f", setpoint_angle, error_angle, target_final_angle_);

    // logger.info("Final: %.2f Error: %.2f Used: %.2f", target_final_angle_, error_angle, final_angle_used_for_steering_);

    //!===============================================================================================
    //!         NAVIS
    //!===============================================================================================
    // float first_used_angle = fabs(final_angle_used_for_steering_);

    // if (first_used_angle > max_steering_deg_)
    //     first_used_angle = max_steering_deg_;

    // float norm_angle = (max_steering_deg_ - first_used_angle) / max_steering_deg_;

    // target_velocity_ = speed_curve_ + (norm_angle * (speed_straight_ - speed_curve_));

    //? TUNING KECEPATAN

    // if (ada_obs_) {
    //     if (ada_belokan_) {
    //         target_velocity_ = 0.8;
    //     } else {
    //         target_velocity_ = 0.8;
    //     }
    // } else if (ada_belokan_) {
    //     target_velocity_ = 1.0;
    // } else {
    //     target_velocity_ = 1.2;
    // }

    target_velocity_ = 0;
    for (int8_t i = 0; i < 4; i++)
        target_velocity_ += segment_speed_[i];

    // if (used_reference_ == EDGE_REFERENCE) {
    //     target_velocity_ = 0.8;
    // }

    // target_velocity_ = 1.1;

    // if (ada_obs_slow_down_) {
    //     target_velocity_ = 1;
    // }

    target_velocity_ *= fabs(cosf(target_final_angle_ * scaller_speed * M_PI / 180.0f));

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
        filtered_debug3_cv.encoding = sensor_msgs::image_encodings::MONO8;
        filtered_debug3_cv.image = bev_depth_thres_obs;
        filtered_debug3_msg = *filtered_debug3_cv.toImageMsg();
        pub_debug_binary_3_->publish(filtered_debug3_msg);

        // -- Publish the filtered binary image
        sensor_msgs::msg::Image filtered_binary_msg;
        cv_bridge::CvImage filtered_binary_cv;
        filtered_binary_cv.header.stamp = sync_time_;
        filtered_binary_cv.header.frame_id = "camera_color_optical_frame";
        filtered_binary_cv.encoding = sensor_msgs::image_encodings::MONO8;
        filtered_binary_cv.image = bev_cleaned_binary;
        filtered_binary_msg = *filtered_binary_cv.toImageMsg();
        pub_filtered_binary_->publish(filtered_binary_msg);

        // -- Publish the filtered binary image
        sensor_msgs::msg::Image filtered_road_msg;
        cv_bridge::CvImage filtered_road_cv;
        filtered_road_cv.header.stamp = sync_time_;
        filtered_road_cv.header.frame_id = "camera_color_optical_frame";
        filtered_road_cv.encoding = sensor_msgs::image_encodings::MONO8;
        filtered_road_cv.image = dashed_line_cleaned;
        filtered_road_msg = *filtered_road_cv.toImageMsg();
        pub_road_binary_->publish(filtered_road_msg);

        // -- Publish the filtered binary image
        sensor_msgs::msg::Image filtered_debug_msg;
        cv_bridge::CvImage filtered_debug_cv;
        filtered_debug_cv.header.stamp = sync_time_;
        filtered_debug_cv.header.frame_id = "camera_color_optical_frame";
        filtered_debug_cv.encoding = sensor_msgs::image_encodings::MONO8;
        filtered_debug_cv.image = bev_binary_raw;
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

void VisionCapture2::callback_sub_enc_meter(const std_msgs::msg::Float32::SharedPtr msg)
{
    rclcpp::Time enc_time = this->now();
    static rclcpp::Time last_enc_time = enc_time;
    double dt = (enc_time - last_enc_time).seconds();

    last_enc_time = enc_time;
    enc_meter += msg->data * dt;
    enc_speed = msg->data * dt;
    // logger.warn("Encoder data: %.2f, %.2f dt: %.4f", enc_meter, enc_speed, dt);
}

void VisionCapture2::callback_sub_target_speed(const std_msgs::msg::Float32::SharedPtr msg)
{
    speed_motor = msg->data;
}

void VisionCapture2::callback_sub_lane_used_web(const std_msgs::msg::Int16::SharedPtr msg)
{
    origin_used_lane_ = msg->data;
    used_lane_ = origin_used_lane_;

    // logger.info("Used lane updated to: %d", origin_used_lane_);
}

void VisionCapture2::callback_sub_button(const std_msgs::msg::Int8::SharedPtr msg)
{
    button_1 = (msg->data >> 0) & 0x01;
    button_2 = (msg->data >> 1) & 0x01;
    toogle_button_1 = (msg->data >> 2) & 0x01;
    toogle_button_2 = (msg->data >> 3) & 0x01;
}

void VisionCapture2::callback_sub_initial(const std_msgs::msg::Int8::SharedPtr msg)
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

void VisionCapture2::callback_sub_vision_config(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
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
void VisionCapture2::callback_sub_controlbox(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
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
    std::shared_ptr<VisionCapture2> node_VisionCapture2;

    try
    {
        node_VisionCapture2 = std::make_shared<VisionCapture2>();
        rclcpp::executors::MultiThreadedExecutor executor(
            rclcpp::ExecutorOptions(),
            11);
        executor.add_node(node_VisionCapture2);
        executor.spin();
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vision_capture2"), "Failed to create VisionCapture2 node: %s", e.what());
        rclcpp::shutdown();
    }

    if (node_VisionCapture2)
        node_VisionCapture2.reset();

    rclcpp::shutdown();
    return 0;
}