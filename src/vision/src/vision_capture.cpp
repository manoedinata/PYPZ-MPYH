// C++ Standard Library
#include <algorithm>
#include <chrono>
#include <iostream>
#include <map>
#include <mutex>
#include <thread>

// C Standard Library
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <signal.h>
#include <sstream>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <unistd.h>

// Third-party Libraries
#include "cv_bridge/cv_bridge.h"
#include "pcl_ros/transforms.hpp"
#include <librealsense2/rs.hpp>
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
#include "ros2_interface/msg/apriltag.hpp"
#include "ros2_utils/global_definitions.hpp"
#include "ros2_utils/help_logger.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
typedef point_cloud::Ptr cloud_pointer;

class VisionCapture : public rclcpp::Node
{
  private:
    // -------------------------------------------------
    // Transform
    // -------------------------------------------------
    bool tf_is_initialized = false;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    geometry_msgs::msg::TransformStamped tf_camera_base;
    geometry_msgs::msg::TransformStamped tf_base_camera;
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
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cleaned_pointcloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_yuv_pointcloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered_points_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_sign_points_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_imagecloud_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_laserscan_;
    rclcpp::Publisher<ros2_interface::msg::Apriltag>::SharedPtr pub_apriltags_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_slope_;
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

    int16_t center_cam_x = 0;
    int16_t center_cam_y = 0;

    rclcpp::Time sync_time_;
    // -------------------------------------------------
    // Apriltag Variables
    // -------------------------------------------------
    float param_tag_size = 0.88;
    float param_quad_decimate = 2.0;
    float param_blur = 0.0;
    int param_nthreads = 1;
    bool param_debug = false;
    bool param_refine_edges = true;

    // apriltag_family_t* tf = NULL;
    // apriltag_detector_t* td = NULL;
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
    VisionCapture()
        : Node("vision_capture"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
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
            cfg_.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 60);
            cfg_.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 60);

            pipe_.start(cfg_);
            pipeline_started_ = true;

            logger.info("RealSense pipeline started successfully");
        }
        catch (const rs2::error &e)
        {
            logger.error("Failed to start RealSense pipeline: %s", e.what());
            return;
        }
        // --------------------------------------------------
        //                    APRILTAG
        // --------------------------------------------------
        // tf = tag36h11_create();
        // td = apriltag_detector_create();
        // apriltag_detector_add_family(td, tf);

        // td->quad_decimate = param_quad_decimate;
        // td->quad_sigma = param_blur;
        // td->nthreads = param_nthreads;
        // td->debug = param_debug;
        // td->refine_edges = param_refine_edges;
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
        pub_apriltags_ = this->create_publisher<ros2_interface::msg::Apriltag>(
            "/vision/apriltags", 1);
        pub_slope_ = this->create_publisher<std_msgs::msg::Float32>(
            "/vision/slope", 1);
        // --------------------------------------------------
        // Create timer
        // --------------------------------------------------
        timer_routine_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&VisionCapture::callback_tim_routine, this),
            tim_routine_group_);
        timer_img_routine_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&VisionCapture::callback_tim_img_routine, this),
            tim_routine_group_);
        timer_pointcloud_routine_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&VisionCapture::callback_tim_pointcloud_routine, this),
            tim_routine_group_);
        // --------------------------------------------------
        // Wait for TF to be initialized
        // --------------------------------------------------
        while (!tf_is_initialized)
        {
            rclcpp::sleep_for(std::chrono::seconds(1));
            try
            {
                tf_camera_base = tf_buffer_.lookupTransform("base_link", "camera_color_optical_frame", tf2::TimePointZero);
                tf_base_camera = tf_buffer_.lookupTransform("camera_color_optical_frame", "base_link", tf2::TimePointZero);
                tf_is_initialized = true;
            }
            catch (const tf2::TransformException &ex)
            {
                logger.warn("TF not ready: %s", ex.what());
                rclcpp::sleep_for(std::chrono::milliseconds(100));
            }
        }

        logger.info("VisionCapture node initialized with multithreading");
    }

    ~VisionCapture()
    {
        cleanup_realsense();
    }

  private:
    void callback_tim_img_routine()
    {

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
        // logger.info("Timer img routine elapsed time: %.4f seconds", elapsed_time);
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

        center_cam_x = color_image.cols / 2;
        center_cam_y = color_image.rows - 1;
        //? ==================================================
        //?                 Process Image
        //? ==================================================
        cv::Mat gray_frame;
        cv::Mat otsu_binary;
        cv::Mat filtered_binary = cv::Mat::zeros(color_image.size(), CV_8UC1);
        cv::Mat debug_frame = color_image.clone();

        cv::cvtColor(color_image, gray_frame, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray_frame, gray_frame, cv::Size(31, 31), 0);
        cv::Canny(gray_frame, otsu_binary, 50, 70, 3);
        cv::dilate(otsu_binary, otsu_binary, cv::Mat(), cv::Point(-1, -1), 3);

        // hsv color segmentation
        // cv::Mat hsv_frame;
        // cv::cvtColor(color_image, hsv_frame, cv::COLOR_BGR2HSV);
        // cv::GaussianBlur(hsv_frame, hsv_frame, cv::Size(31, 31), 0);
        // cv::inRange(hsv_frame, cv::Scalar(0, 0, 160), cv::Scalar(180, 255, 255), otsu_binary); //! PERLU JADI PARAMETER

        // cv::morphologyEx(otsu_binary, otsu_binary, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1, -1), 2);

        // cv::threshold(gray_frame, otsu_binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

        // find largest contour area
        std::vector<int> removed_contours_idx;
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(otsu_binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (contours.empty())
        {
            logger.warn("No contours found in the image");
            return;
        }

        std::sort(contours.begin(), contours.end(),
                  [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b)
                  {
                      return cv::contourArea(a) > cv::contourArea(b);
                  });

        int largest_contour_area = cv::contourArea(contours[0]);

        //!=============
        //! Nanti diubah
        //!=============
        for (size_t i = 0; i < contours.size(); i++)
        {
            float height = cv::boundingRect(contours[i]).height;
            float width = cv::boundingRect(contours[i]).width;

            // put text id and area in cnt location
            cv::putText(debug_frame, std::to_string(width) + " " + std::to_string(height),
                        cv::Point(contours[i][0].x, contours[i][0].y - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);

            // Method 1: Filter contours based on area
            // if (cv::contourArea(contours[i]) < largest_contour_area * 0.7) {
            //     removed_contours_idx.push_back(i);
            //     continue;
            // }

            // Method 2: Filter contours based on height
            if (height < 300)
            {
                removed_contours_idx.push_back(i);
                continue;
            }
        }

        cv::drawContours(filtered_binary, contours, -1, cv::Scalar(255), cv::FILLED);
        cv::drawContours(debug_frame, contours, -1, cv::Scalar(255), cv::FILLED);

        //?===================================================
        //?                 COBA
        //?===================================================
        auto start = std::chrono::steady_clock::now();

        cv::Mat yuv_frame;
        cv::Mat combined_road_obs;
        cv::cvtColor(color_image, yuv_frame, cv::COLOR_BGR2YUV);
        const float croped_value = 0.2; // 20% of the height

        cv::resize(yuv_frame, yuv_frame, cv::Size(640, 480), 0, 0, cv::INTER_LINEAR);

        // cv::Rect roi_yuv(0, (yuv_frame.rows * croped_value), yuv_frame.cols, yuv_frame.rows - (yuv_frame.rows * croped_value));
        // yuv_frame = yuv_frame(roi_yuv);
        cv::Mat thres_yuv = cv::Mat::zeros(yuv_frame.size(), CV_8UC1);
        cv::inRange(yuv_frame, cv::Scalar(0, 50, 150), cv::Scalar(255, 130, 255), thres_yuv); //! PERLU JADI PARAMETER
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

        // get area of the largest contour
        int largest_yuv_contour_area = 0;
        if (!yuv_contours.empty())
        {
            largest_yuv_contour_area = static_cast<int>(cv::contourArea(yuv_contours[0]));
            // logger.info("Largest YUV contour area: %d", largest_yuv_contour_area);
        }

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

        // get centroid_yuv from the cleaned_yuv image

        //? ==================================================
        //? ==================================================

        cv::Mat hsv_frame;
        cv::Mat cleaned_binary;
        cv::Mat thres_hsv;
        cv::cvtColor(color_image, hsv_frame, cv::COLOR_BGR2HSV);

        // resize hsv_frame to 640x480
        cv::resize(hsv_frame, hsv_frame, cv::Size(640, 480), 0, 0, cv::INTER_LINEAR);

        // crop 200 pixel from top of frame
        cv::Rect roi(0, (hsv_frame.rows * croped_value), hsv_frame.cols, hsv_frame.rows - (hsv_frame.rows * croped_value));
        hsv_frame = hsv_frame(roi);
        thres_hsv = cv::Mat::zeros(hsv_frame.size(), CV_8UC1);

        cv::inRange(hsv_frame, cv::Scalar(0, 0, 200), cv::Scalar(180, 255, 255), thres_hsv); //! PERLU JADI PARAMETER

        // find countours in the thresholded image
        std::vector<std::vector<cv::Point>> hsv_contours;
        cv::findContours(thres_hsv, hsv_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        // sort contours by height
        std::sort(hsv_contours.begin(), hsv_contours.end(),
                  [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b)
                  {
                      return cv::boundingRect(a).height > cv::boundingRect(b).height;
                  });

        // remove contours that are too small, only get 2 longest countours
        if (hsv_contours.size() > 2)
            hsv_contours.resize(2);

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

            // // get the nearest pixel point to the centroid of the yuv contour
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

        // find the nearest pixel is in left side of centroid_yuv
        bool is_right_side_free = true;
        if (!nearest_pixels.empty())
        {
            // Find the nearest pixel to the centroid_yuv
            auto min_it = std::min_element(distances.begin(), distances.end());
            int min_index = std::distance(distances.begin(), min_it);
            cv::Point nearest_pixel = nearest_pixels[min_index];

            // Check if the nearest pixel is on the left side of the centroid_yuv
            if (nearest_pixel.x > centroid_yuv.x)
                is_right_side_free = false;

            // Draw a circle at the nearest pixel
            // cv::circle(debug_frame, nearest_pixel, 5, cv::Scalar(0, 255, 0), -1);
            // logger.info("Nearest pixel to centroid_yuv: (%d, %d) | yuv contour (%d, %d)", nearest_pixel.x, nearest_pixel.y, centroid_yuv.x, centroid_yuv.y);
        }

        // combine the cleaned binary image with the thresholded YUV image
        combined_road_obs = cv::Mat::zeros(cleaned_binary.size(), CV_8UC1);
        cv::bitwise_or(cleaned_binary, cropped_yuv, combined_road_obs);
        // cv::bitwise_or(cleaned_binary, cleaned_yuv, cleaned_binary);
        // logger.info("Is right side free: %s", is_right_side_free ? "true" : "false");

        // if (is_right_side_free) {
        //     logger.info("obs di kiri");
        // } else {
        //     logger.info("obs di kanan");
        // }
        // logger.info("Processed images in %.4f seconds", this->now().seconds() - start_time);

        // // Ukuran area yang ingin diambil di sekitar centroid
        // int roi_width = 640;
        // int roi_height = 50;

        // cv::Mat cleaned_obstacle_area = cv::Mat::zeros(roi_height, roi_width, CV_8UC1);

        // if (centroid_yuv.x > 0 && centroid_yuv.y > 0)
        // {
        //     // Hitung top-left koordinat ROI, pastikan tidak keluar batas
        //     int x_start = std::max(0, centroid_yuv.x - roi_width / 2);
        //     int y_start = std::max(0, (centroid_yuv.y - 25) - roi_height / 2);

        //     // Pastikan ROI tidak melebihi batas image
        //     int x_end = std::min(x_start + roi_width, cleaned_binary.cols);
        //     int y_end = std::min(y_start + roi_height, cleaned_binary.rows);

        //     // Koreksi ukuran jika ROI terlalu dekat tepi
        //     x_start = x_end - roi_width;
        //     y_start = y_end - roi_height;
        //     x_start = std::max(0, x_start);
        //     y_start = std::max(0, y_start);

        //     // Buat ROI rect dan ambil subregion dari cleaned
        //     cv::Rect obstacle_roi(x_start, y_start, roi_width, roi_height);
        //     cleaned_obstacle_area = cleaned_binary(obstacle_roi);

        //     // Optional: tampilkan atau proses lebih lanjut
        //     // cv::imshow("Obstacle Nearby Region", cleaned_obstacle_area);
        // }

        //? ==================================================
        //?                    HERNANDA
        //? ==================================================
        std::vector<int> height_profile(filtered_binary.cols, 0);

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

        // Apply complete transform to each 3D point using tf2
        auto transform_point = [&](float *point_in, float *point_out)
        {
            geometry_msgs::msg::PointStamped point_base, point_camera;
            point_base.header.frame_id = "base_link";
            point_base.point.x = point_in[0];
            point_base.point.y = point_in[1];
            point_base.point.z = point_in[2];

            try
            {
                tf2::doTransform(point_base, point_camera, tf_base_camera);
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
        };

        float camera_point_left_bottom[3], camera_point_right_bottom[3];
        float camera_point_left_top[3], camera_point_right_top[3];

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
        int bev_width = 400;
        int bev_height = 600;
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
        cv::inRange(bev_hsv_frame, cv::Scalar(0, 0, 200), cv::Scalar(180, 255, 255), bev_binary);

        // Create mask to remove transformation artifacts (black regions)
        cv::Mat valid_region_mask = cv::Mat::zeros(bev_gray_frame.size(), CV_8UC1);

        // Only consider regions that have actual pixel data (not black from transformation)
        cv::threshold(bev_gray_frame, valid_region_mask, 1, 255, cv::THRESH_BINARY);

        // Erode the mask slightly to remove edge artifacts from transformation
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::erode(valid_region_mask, valid_region_mask, kernel, cv::Point(-1, -1), 5);

        // Apply the mask to remove edges detected from black transformation regions
        cv::bitwise_and(bev_binary, valid_region_mask, bev_binary);

        // Connect broken lines using morphological operations
        kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::dilate(bev_binary, bev_binary, kernel, cv::Point(-1, -1), 1);
        cv::erode(bev_binary, bev_binary, kernel, cv::Point(-1, -1), 1);

        // Close gaps in lines using morphological closing
        cv::Mat line_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 1));
        cv::morphologyEx(bev_binary, bev_binary, cv::MORPH_CLOSE, line_kernel);

        cv::Mat half_left = bev_binary.clone();
        cv::Mat half_right = bev_binary.clone();

        // Create masks for left and right halves
        cv::Mat left_mask = cv::Mat::zeros(bev_binary.size(), CV_8UC1);
        cv::Mat right_mask = cv::Mat::zeros(bev_binary.size(), CV_8UC1);

        int mid_x = bev_binary.cols / 2;

        // Fill left half mask
        cv::rectangle(left_mask, cv::Point(0, 0), cv::Point(mid_x, bev_binary.rows), cv::Scalar(255), -1);

        // Fill right half mask
        cv::rectangle(right_mask, cv::Point(mid_x, 0), cv::Point(bev_binary.cols, bev_binary.rows), cv::Scalar(255), -1);

        // Apply masks to get left and right halves
        cv::bitwise_and(half_left, left_mask, half_left);
        cv::bitwise_and(half_right, right_mask, half_right);

        std::vector<std::vector<cv::Point>> left_contours;
        std::vector<std::vector<cv::Point>> right_contours;
        {
            // find countours in the thresholded image
            cv::findContours(half_left, left_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            // sort left_contours by height
            std::sort(left_contours.begin(), left_contours.end(),
                      [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b)
                      {
                          return cv::boundingRect(a).height > cv::boundingRect(b).height;
                      });

            // remove left_contours that are too small, only get 2 longest countours
            // if (left_contours.size() > 1) {
            //     left_contours.resize(1);
            // }
        }
        {
            // find countours in the thresholded image
            cv::findContours(half_right, right_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            // sort right_contours by height
            std::sort(right_contours.begin(), right_contours.end(),
                      [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b)
                      {
                          return cv::boundingRect(a).height > cv::boundingRect(b).height;
                      });

            // remove right_contours that are too small, only get 2 longest countours
            // if (right_contours.size() > 1) {
            //     right_contours.resize(1);
            // }
        }

        // Draw left and right contours on filtered binary image
        cv::Mat filtered_bev_binary = cv::Mat::zeros(bev_binary.size(), CV_8UC1);
        cv::Mat filtered_left_bev_binary = cv::Mat::zeros(bev_binary.size(), CV_8UC1);
        cv::Mat filtered_right_bev_binary = cv::Mat::zeros(bev_binary.size(), CV_8UC1);

        // Draw left contour if it exists
        // if (!left_contours.empty()) {
        //     cv::drawContours(filtered_bev_binary, left_contours, 0, cv::Scalar(255), cv::FILLED);
        //     cv::drawContours(filtered_left_bev_binary, left_contours, 0, cv::Scalar(255), cv::FILLED);
        // }

        // // Draw right contour if it exists
        // if (!right_contours.empty()) {
        //     cv::drawContours(filtered_bev_binary, right_contours, 0, cv::Scalar(255), cv::FILLED);
        //     cv::drawContours(filtered_right_bev_binary, right_contours, 0, cv::Scalar(255), cv::FILLED);
        // }

        // for (int i = 0; i < left_contours.size(); ++i) {
        //     cv::drawContours(filtered_left_bev_binary, left_contours, i, cv::Scalar(255), cv::FILLED);
        // }
        // for (int i = 0; i < right_contours.size(); ++i) {
        //     cv::drawContours(filtered_right_bev_binary, right_contours, i, cv::Scalar(255), cv::FILLED);
        // }

        // Calculate slope from contours using Cartesian coordinates (bottom to top)
        float left_slope = 0.0f;
        float right_slope = 0.0f;
        bool left_slope_calculated = false;
        bool right_slope_calculated = false;

        // Calculate left contour slope
        if (!left_contours.empty())
        {
            const auto &contour = left_contours[0];

            // Find points with maximum and minimum y values (in image coordinates)
            cv::Point bottom_point = contour[0]; // y terbesar (bottom in image)
            cv::Point top_point = contour[0];    // y terkecil (top in image)

            for (const auto &point : contour)
            {
                if (point.y > bottom_point.y)
                    bottom_point = point; // Max y (bottom)
                if (point.y < top_point.y)
                    top_point = point; // Min y (top)
            }

            // Calculate slope in Cartesian coordinates (from bottom to top)
            float dx = top_point.x - bottom_point.x; // Change in x
            float dy = bottom_point.y - top_point.y; // Change in y (inverted for Cartesian)

            if (abs(dy) > 1)
            {                         // Avoid division by zero
                left_slope = dx / dy; // Slope = rise/run in Cartesian coordinates
                left_slope_calculated = true;
            }
        }

        // Calculate right contour slope
        if (!right_contours.empty())
        {
            const auto &contour = right_contours[0];

            // Find points with maximum and minimum y values (in image coordinates)
            cv::Point bottom_point = contour[0]; // y terbesar (bottom in image)
            cv::Point top_point = contour[0];    // y terkecil (top in image)

            for (const auto &point : contour)
            {
                if (point.y > bottom_point.y)
                    bottom_point = point; // Max y (bottom)
                if (point.y < top_point.y)
                    top_point = point; // Min y (top)
            }

            // Calculate slope in Cartesian coordinates (from bottom to top)
            float dx = top_point.x - bottom_point.x; // Change in x
            float dy = bottom_point.y - top_point.y; // Change in y (inverted for Cartesian)

            if (abs(dy) > 1)
            {                          // Avoid division by zero
                right_slope = dx / dy; // Slope = rise/run in Cartesian coordinates
                right_slope_calculated = true;
            }
        }

        // Log the slope values
        if (left_slope_calculated || right_slope_calculated)
            logger.info("Left slope: %.4f (calculated: %s), Right slope: %.4f (calculated: %s)",
                        left_slope, left_slope_calculated ? "true" : "false",
                        right_slope, right_slope_calculated ? "true" : "false");

        float average_slope = 0.0f;
        if (left_slope_calculated && right_slope_calculated)
        {
            average_slope = (left_slope + right_slope) / 2.0;
            logger.info("Average slope: %.4f", average_slope);
        }
        else if (left_slope_calculated)
        {
            average_slope = left_slope;
            logger.info("Average slope (only left): %.4f", average_slope);
        }
        else if (right_slope_calculated)
        {
            average_slope = right_slope;
            logger.info("Average slope (only right): %.4f", average_slope);
        }
        else
        {
            logger.warn("No slopes calculated from contours");
        }

        // Publish
        std_msgs::msg::Float32 slope_msg;
        slope_msg.data = average_slope;
        pub_slope_->publish(slope_msg);

        //! ============================================================

        cv::Mat draw_height_profile = cv::Mat::zeros(480, 640, CV_8UC3);

        // Find maximum height for normalization
        int max_height = *std::max_element(height_profile.begin(), height_profile.end());

        // Draw histogram bars
        for (int x = 0; x < cleaned_binary.cols; ++x)
        {
            int height = height_profile[x];

            // Normalize height to fit in the image (0 to image height)
            int normalized_height = 0;
            if (max_height > 0)
                normalized_height = static_cast<int>((static_cast<float>(height) / max_height) * (draw_height_profile.rows - 1));

            // Draw vertical line from bottom to the normalized height
            if (normalized_height > 0)
                cv::line(draw_height_profile,
                         cv::Point(x, draw_height_profile.rows - 1),
                         cv::Point(x, draw_height_profile.rows - 1 - normalized_height),
                         cv::Scalar(0, 255, 0), 1);
        }

        // Add grid lines for better visualization (optional)
        for (int y = 0; y < draw_height_profile.rows; y += 60)
            cv::line(draw_height_profile, cv::Point(0, y), cv::Point(draw_height_profile.cols - 1, y), cv::Scalar(50, 50, 50), 1);
        for (int x = 0; x < draw_height_profile.cols; x += 80)
            cv::line(draw_height_profile, cv::Point(x, 0), cv::Point(x, draw_height_profile.rows - 1), cv::Scalar(50, 50, 50), 1);

        //? ==================================================
        //?                 Get Pointcloud
        //? ==================================================
        std::vector<cv::Point> point_cloud;

        for (size_t rows = 0; rows < filtered_binary.rows; rows++)
        {
            for (size_t cols = 0; cols < filtered_binary.cols; cols++)
                if (filtered_binary.at<uint8_t>(rows, cols) > 0)
                    point_cloud.emplace_back(cols - center_cam_x, center_cam_y - rows);
        }

        if (point_cloud.empty())
        {
            logger.warn("No pointcloud found in the image");
            return;
        }

        std::vector<cv::Point> cleaned_cloud;

        for (size_t rows = 0; rows < cleaned_binary.rows; rows++)
        {
            for (size_t cols = 0; cols < cleaned_binary.cols; cols++)
                if (cleaned_binary.at<uint8_t>(rows, cols) > 0)
                    cleaned_cloud.emplace_back(cols - center_cam_x, center_cam_y - rows);
        }

        if (cleaned_cloud.empty())
        {
            logger.warn("No cleaned pointcloud found in the image");
            return;
        }

        std::vector<cv::Point> yuv_point_cloud;
        for (size_t rows = 0; rows < cleaned_yuv.rows; rows++)
        {
            for (size_t cols = 0; cols < cleaned_yuv.cols; cols++)
                if (cleaned_yuv.at<uint8_t>(rows, cols) > 0)
                    yuv_point_cloud.emplace_back(cols - center_cam_x, center_cam_y - rows);
        }

        // if (yuv_point_cloud.empty())
        // {
        //     logger.warn("No YUV pointcloud found in the image");
        //     return;
        // }

        //? ==================================================
        //?                Convert to Depth
        //? ==================================================
        pcl::PointCloud<pcl::PointXYZ>::Ptr image_cloud_world_ptr(new pcl::PointCloud<pcl::PointXYZ>());
        {

            int pixel_x = 0;
            int pixel_y = 0;
            float depth_meters = 0.0f;

            for (const auto &point : point_cloud)
            {
                if (!std::isfinite(point.x) || !std::isfinite(point.y))
                    continue;

                pixel_x = static_cast<int>(std::round(point.x + center_cam_x));
                pixel_y = static_cast<int>(std::round(center_cam_y - point.y));

                if (pixel_x < 0 || pixel_x >= depth_image.cols || pixel_y < 0 || pixel_y >= depth_image.rows)
                    continue;

                uint16_t depth_value = depth_image.at<uint16_t>(pixel_y, pixel_x);
                if (depth_value == 0)
                    continue;

                depth_meters = depth_value * 0.001f;

                // Use RealSense built-in function to convert pixel coordinates to 3D point
                float pixel[2] = {static_cast<float>(pixel_x), static_cast<float>(pixel_y)};
                float point_3d[3];
                rs2_deproject_pixel_to_point(point_3d, &intrinsics, pixel, depth_meters);

                // Push back into PCL point cloud
                pcl::PointXYZ img_point;
                img_point.x = point_3d[0];
                img_point.y = point_3d[1];
                img_point.z = point_3d[2];
                image_cloud_world_ptr->push_back(img_point);
            }
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cleaned_cloud_world_ptr(new pcl::PointCloud<pcl::PointXYZ>());
        {

            int pixel_x = 0;
            int pixel_y = 0;
            float depth_meters = 0.0f;

            for (const auto &point : cleaned_cloud)
            {
                if (!std::isfinite(point.x) || !std::isfinite(point.y))
                    continue;

                pixel_x = static_cast<int>(std::round(point.x + center_cam_x));
                pixel_y = static_cast<int>(std::round(center_cam_y - point.y));

                if (pixel_x < 0 || pixel_x >= depth_image.cols || pixel_y < 0 || pixel_y >= depth_image.rows)
                    continue;

                uint16_t depth_value = depth_image.at<uint16_t>(pixel_y, pixel_x);
                if (depth_value == 0)
                    continue;

                depth_meters = depth_value * 0.001f;

                // Use RealSense built-in function to convert pixel coordinates to 3D point
                float pixel[2] = {static_cast<float>(pixel_x), static_cast<float>(pixel_y)};
                float point_3d[3];
                rs2_deproject_pixel_to_point(point_3d, &intrinsics, pixel, depth_meters);

                // Push back into PCL point cloud
                pcl::PointXYZ img_point;
                img_point.x = point_3d[0];
                img_point.y = point_3d[1];
                img_point.z = point_3d[2];
                cleaned_cloud_world_ptr->push_back(img_point);
            }
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr yuv_cloud_world_ptr(new pcl::PointCloud<pcl::PointXYZ>());
        {

            int pixel_x = 0;
            int pixel_y = 0;
            float depth_meters = 0.0f;

            for (const auto &point : yuv_point_cloud)
            {
                if (!std::isfinite(point.x) || !std::isfinite(point.y))
                    continue;

                pixel_x = static_cast<int>(std::round(point.x + center_cam_x));
                pixel_y = static_cast<int>(std::round(center_cam_y - point.y));

                if (pixel_x < 0 || pixel_x >= depth_image.cols || pixel_y < 0 || pixel_y >= depth_image.rows)
                    continue;

                uint16_t depth_value = depth_image.at<uint16_t>(pixel_y, pixel_x);
                if (depth_value == 0)
                    continue;

                depth_meters = depth_value * 0.001f;

                // Use RealSense built-in function to convert pixel coordinates to 3D point
                float pixel[2] = {static_cast<float>(pixel_x), static_cast<float>(pixel_y)};
                float point_3d[3];
                rs2_deproject_pixel_to_point(point_3d, &intrinsics, pixel, depth_meters);

                // Push back into PCL point cloud
                pcl::PointXYZ img_point;
                img_point.x = point_3d[0];
                img_point.y = point_3d[1];
                img_point.z = point_3d[2];
                yuv_cloud_world_ptr->push_back(img_point);
            }
        }

        //? ==================================================
        //?                    Publish PCL
        //? ==================================================
        if (!image_cloud_world_ptr->empty())
        {

            pcl::PointCloud<pcl::PointXYZ>::Ptr image_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());

            // Transform point cloud to base_link frame
            pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
            pcl_ros::transformPointCloud(*image_cloud_world_ptr, transformed_cloud, tf_camera_base);

            // Cropbox filter to remove points outside a certain region
            pcl::CropBox<pcl::PointXYZ> crop_box;
            crop_box.setInputCloud(transformed_cloud.makeShared());
            crop_box.setMin(Eigen::Vector4f(0.0, -0.5, -0.05, 1.0)); // Min bounds in base_link frame
            crop_box.setMax(Eigen::Vector4f(2.5, 0.5, 0.05, 1.0));   // Max bounds in base_link frame
            crop_box.setNegative(false);                             // Keep only points inside the box
            crop_box.filter(*image_cloud_filtered);

            // Offset z-axis 10 cm
            for (auto &point : image_cloud_filtered->points)
                point.z += 0.1f; // Offset z by 10 cm

            // Convert pointcloud to laser scan
            sensor_msgs::msg::LaserScan laser_scan_msg;
            laser_scan_msg.header.stamp = sync_time_;
            laser_scan_msg.header.frame_id = "base_link";
            laser_scan_msg.angle_min = -M_PI;
            laser_scan_msg.angle_max = M_PI;
            laser_scan_msg.angle_increment = 2 * M_PI / 360.0;
            laser_scan_msg.range_min = 0.0;
            laser_scan_msg.range_max = 1.5;

            size_t num_ranges = static_cast<size_t>(std::ceil((laser_scan_msg.angle_max - laser_scan_msg.angle_min) / laser_scan_msg.angle_increment));
            laser_scan_msg.ranges.assign(num_ranges, std::numeric_limits<float>::infinity());

            for (const auto &point : image_cloud_filtered->points)
            {
                float x = point.x;
                float y = point.y;
                float z = point.z;

                // Optional: ignore points that are too high/low
                // if (std::abs(z) > 0.2)
                //     continue;

                float range = std::hypot(x, y);
                float angle = std::atan2(y, x);

                if (range < laser_scan_msg.range_min || range > laser_scan_msg.range_max)
                    continue;

                int index = std::floor((angle - laser_scan_msg.angle_min) / laser_scan_msg.angle_increment);
                if (index < 0 || index >= static_cast<int>(laser_scan_msg.ranges.size()))
                    continue;

                // Keep closest point for each angles
                if (range < laser_scan_msg.ranges[index])
                    laser_scan_msg.ranges[index] = range;
            }

            // Publish the point cloud
            sensor_msgs::msg::PointCloud2 pcl_msg;
            pcl::toROSMsg(*image_cloud_filtered, pcl_msg);
            pcl_msg.header.stamp = sync_time_;
            pcl_msg.header.frame_id = "base_link";
            pub_imagecloud_->publish(pcl_msg);

            // Publish the laser scan
            pub_laserscan_->publish(laser_scan_msg);
        }
        else
        {
            // Publish the empty laser scan and point cloud
            sensor_msgs::msg::PointCloud2 empty_pcl_msg;
            empty_pcl_msg.header.stamp = sync_time_;
            empty_pcl_msg.header.frame_id = "base_link";
            pub_imagecloud_->publish(empty_pcl_msg);
            sensor_msgs::msg::LaserScan empty_laser_scan_msg;
            empty_laser_scan_msg.header.stamp = sync_time_;
            empty_laser_scan_msg.header.frame_id = "base_link";
            empty_laser_scan_msg.angle_min = -M_PI;
            empty_laser_scan_msg.angle_max = M_PI;
            empty_laser_scan_msg.angle_increment = 2 * M_PI / 360.0;
            empty_laser_scan_msg.range_min = 0.0;
            empty_laser_scan_msg.range_max = 1.5;
            size_t num_ranges = static_cast<size_t>(std::ceil((empty_laser_scan_msg.angle_max - empty_laser_scan_msg.angle_min) / empty_laser_scan_msg.angle_increment));
            empty_laser_scan_msg.ranges.assign(num_ranges, std::numeric_limits<float>::infinity());
            pub_laserscan_->publish(empty_laser_scan_msg);
        }

        if (!cleaned_cloud_world_ptr->empty())
        {

            pcl::PointCloud<pcl::PointXYZ>::Ptr cleaned_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());

            // Transform point cloud to base_link frame
            pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
            pcl_ros::transformPointCloud(*cleaned_cloud_world_ptr, transformed_cloud, tf_camera_base);

            // Cropbox filter to remove points outside a certain region
            pcl::CropBox<pcl::PointXYZ> crop_box;
            crop_box.setInputCloud(transformed_cloud.makeShared());
            crop_box.setMin(Eigen::Vector4f(0.0, -0.5, -0.05, 1.0)); // Min bounds in base_link frame
            crop_box.setMax(Eigen::Vector4f(2.5, 0.5, 0.05, 1.0));   // Max bounds in base_link frame
            crop_box.setNegative(false);                             // Keep only points inside the box
            crop_box.filter(*cleaned_cloud_filtered);

            // Publish the point cloud
            sensor_msgs::msg::PointCloud2 pcl_msg;
            pcl::toROSMsg(*cleaned_cloud_filtered, pcl_msg);
            pcl_msg.header.stamp = sync_time_;
            pcl_msg.header.frame_id = "base_link";
            pub_cleaned_pointcloud_->publish(pcl_msg);
        }
        else
        {
            // Publish the empty laser scan and point cloud
            sensor_msgs::msg::PointCloud2 empty_pcl_msg;
            empty_pcl_msg.header.stamp = sync_time_;
            empty_pcl_msg.header.frame_id = "base_link";
            pub_cleaned_pointcloud_->publish(empty_pcl_msg);
        }

        if (!yuv_cloud_world_ptr->empty())
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr yuv_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
            // Transform point cloud to base_link frame
            pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
            pcl_ros::transformPointCloud(*yuv_cloud_world_ptr, transformed_cloud, tf_camera_base);
            // Cropbox filter to remove points outside a certain region
            pcl::CropBox<pcl::PointXYZ> crop_box;
            crop_box.setInputCloud(transformed_cloud.makeShared());
            crop_box.setMin(Eigen::Vector4f(0.4, -1.0, 0.1, 1.0)); // Min bounds in base_link frame
            crop_box.setMax(Eigen::Vector4f(5.0, 1.0, 2.0, 1.0));  // Max bounds in base_link frame
            crop_box.setNegative(false);                           // Keep only points inside the box
            crop_box.filter(*yuv_cloud_filtered);

            // Publish the point cloud
            sensor_msgs::msg::PointCloud2 pcl_msg;
            pcl::toROSMsg(*yuv_cloud_filtered, pcl_msg);
            pcl_msg.header.stamp = sync_time_;
            pcl_msg.header.frame_id = "base_link";
            pub_yuv_pointcloud_->publish(pcl_msg);
        }
        else
        {
            // Publish the empty laser scan and point cloud
            sensor_msgs::msg::PointCloud2 empty_pcl_msg;
            empty_pcl_msg.header.stamp = sync_time_;
            empty_pcl_msg.header.frame_id = "base_link";

            // Fill PointCloud2 empty_pcl_msg with xyz empty data only metadata
            empty_pcl_msg.height = 1;
            empty_pcl_msg.width = 0;
            empty_pcl_msg.is_bigendian = false;
            empty_pcl_msg.is_dense = true;
            empty_pcl_msg.fields.resize(3);
            empty_pcl_msg.fields[0].name = "x";
            empty_pcl_msg.fields[0].offset = 0;
            empty_pcl_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
            empty_pcl_msg.fields[0].count = 1;
            empty_pcl_msg.fields[1].name = "y";
            empty_pcl_msg.fields[1].offset = 4;
            empty_pcl_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
            empty_pcl_msg.fields[1].count = 1;
            empty_pcl_msg.fields[2].name = "z";
            empty_pcl_msg.fields[2].offset = 8;
            empty_pcl_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
            empty_pcl_msg.fields[2].count = 1;
            empty_pcl_msg.point_step = 12; // 3 floats (x, y, z) * 4 bytes each
            empty_pcl_msg.row_step = empty_pcl_msg.point_step * empty_pcl_msg.width;
            empty_pcl_msg.data.resize(empty_pcl_msg.row_step * empty_pcl_msg.height, 0); // Fill with zeros

            pub_yuv_pointcloud_->publish(empty_pcl_msg);
        }

        // //? ==================================================
        // //?                  Aruco Image
        // //? ==================================================
        // if (false) {
        //     logger.info("=============1=============");
        //     cv::Mat aruco_image_gray = cv::Mat::zeros(color_image.size(), CV_8UC1);
        //     cv::cvtColor(color_image, aruco_image_gray, cv::COLOR_BGR2GRAY);
        //     logger.info("=============1.2=============");
        //     image_u8_t im = { aruco_image_gray.cols, aruco_image_gray.rows, aruco_image_gray.cols, aruco_image_gray.data };
        //     logger.info("=============1.3=============");
        //     zarray_t* detections = apriltag_detector_detect(td, &im);

        //     logger.info("=============1.5=============");

        //     if (errno == EAGAIN) {
        //         logger.error("Unable to create the %d threads requested.", td->nthreads);
        //         exit(-1);
        //     }

        //     logger.info("=============2=============");

        //     std::vector<cv::Point3d> tag_points;
        //     float min_dist = FLT_MAX;
        //     float final_yaw_detected = 0.0f;
        //     float final_x, final_y = 0.0f;

        //     apriltag_detection_t* final_detected_tag = NULL;
        //     // Modify the detection loop to include pose estimation
        //     logger.info("Detected %zu apriltags", zarray_size(detections));
        //     for (int i = 0; i < zarray_size(detections); i++) {
        //         apriltag_detection_t* det;
        //         zarray_get(detections, i, &det);

        //         // Draw detection outlines (existing code)
        //         cv::line(debug_frame, cv::Point(det->p[0][0], det->p[0][1]),
        //             cv::Point(det->p[1][0], det->p[1][1]),
        //             cv::Scalar(0, 0xff, 0), 2);
        //         cv::line(debug_frame, cv::Point(det->p[0][0], det->p[0][1]),
        //             cv::Point(det->p[3][0], det->p[3][1]),
        //             cv::Scalar(0, 0, 0xff), 2);
        //         cv::line(debug_frame, cv::Point(det->p[1][0], det->p[1][1]),
        //             cv::Point(det->p[2][0], det->p[2][1]),
        //             cv::Scalar(0xff, 0, 0), 2);
        //         cv::line(debug_frame, cv::Point(det->p[2][0], det->p[2][1]),
        //             cv::Point(det->p[3][0], det->p[3][1]),
        //             cv::Scalar(0xff, 0, 0), 2);

        //         // Pose estimation
        //         apriltag_detection_info_t info;
        //         info.det = det;
        //         info.tagsize = param_tag_size;

        //         info.fx = intrinsics.fx; // Focal length x
        //         info.fy = intrinsics.fy; // Focal length y
        //         info.cx = intrinsics.ppx; // Principal point x
        //         info.cy = intrinsics.ppy; // Principal point y

        //         apriltag_pose_t pose;
        //         double err = estimate_tag_pose(&info, &pose);

        //         // Print pose information
        //         std::string pose_info = "Pose: ";
        //         pose_info += "x: " + std::to_string(pose.t->data[0] * 0.1) + ", ";
        //         pose_info += "y: " + std::to_string(pose.t->data[1] * 0.1) + ", ";
        //         pose_info += "z: " + std::to_string(pose.t->data[2] * 0.1) + ", ";
        //         pose_info += "roll: " + std::to_string(pose.R->data[0]) + ", ";
        //         pose_info += "pitch: " + std::to_string(pose.R->data[1]) + ", ";
        //         pose_info += "yaw: " + std::to_string(pose.R->data[2]) + ", ";
        //         pose_info += "error: " + std::to_string(err);
        //         cv::putText(debug_frame, pose_info, cv::Point(10, 30 + i * 20),
        //             cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

        //         float dist = (pose.t->data[2] * 0.1);
        //         if (dist < min_dist) {
        //             min_dist = dist;
        //             final_detected_tag = det; // Store the tag with the minimum distance
        //             final_yaw_detected = pose.R->data[2]; // Store the yaw angle
        //             final_x = pose.t->data[0] * 0.1; // Store the x position
        //             final_y = pose.t->data[1] * 0.1; // Store the y position
        //         }

        //         // Store tag points for visualization
        //         tag_points.push_back(cv::Point3d(pose.t->data[0] * 0.1, pose.t->data[1] * 0.1, pose.t->data[2] * 0.1));

        //         // Display tag ID (existing code)
        //         std::stringstream ss;
        //         ss << det->id;
        //         std::string text = ss.str();
        //         int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
        //         double fontscale = 1.0;
        //         int baseline;
        //         cv::Size textsize = cv::getTextSize(text, fontface, fontscale, 2, &baseline);
        //         cv::putText(debug_frame, text, cv::Point(det->c[0] - textsize.width / 2, det->c[1] + textsize.height / 2),
        //             fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);
        //     }

        //     logger.info("=============3=============");

        //     apriltag_detections_destroy(detections);

        //     if (final_detected_tag && (final_detected_tag->id >= 0 && final_detected_tag->id <= 5)) {
        //         ros2_interface::msg::Apriltag final_detection_msg;
        //         final_detection_msg.id = (int8_t)final_detected_tag->id;
        //         final_detection_msg.dist = min_dist;
        //         final_detection_msg.yaw = final_yaw_detected;
        //         final_detection_msg.x = final_x;
        //         final_detection_msg.y = final_y;
        //         pub_apriltags_->publish(final_detection_msg);
        //     }

        //     logger.info("=============4=============");
        // }
        //?==================================================================
        // Create convex hull from HSV color image and apply bitwise AND
        cv::Mat hsv_frame_hull;
        cv::Mat thresh_hull;
        cv::cvtColor(color_image, hsv_frame_hull, cv::COLOR_BGR2HSV);

        // Threshold HSV image to detect dark areas (road surface)
        cv::inRange(hsv_frame_hull, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 100), thresh_hull);

        // Find contours in the thresholded image
        std::vector<std::vector<cv::Point>> contours_hull;
        cv::findContours(thresh_hull, contours_hull, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Buat convex hull dari gabungan kontur terbesar dan kedua terbesar
        std::vector<cv::Point> hull_points;

        if (contours_hull.size() >= 2)
        {
            // Urutkan kontur berdasarkan area (dari terbesar ke terkecil)
            std::sort(contours_hull.begin(), contours_hull.end(),
                      [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b)
                      {
                          return cv::contourArea(a) > cv::contourArea(b); // descending order
                      });

            // Gabungkan dua kontur terbesar
            std::vector<cv::Point> merged_contours;
            merged_contours.insert(merged_contours.end(), contours_hull[0].begin(), contours_hull[0].end());
            merged_contours.insert(merged_contours.end(), contours_hull[1].begin(), contours_hull[1].end());

            // Buat convex hull dari gabungan dua kontur
            cv::convexHull(merged_contours, hull_points);
        }
        else if (contours_hull.size() == 1)
        {
            // Jika hanya ada satu kontur, gunakan itu saja
            cv::convexHull(contours_hull[0], hull_points);
        }

        // Buat mask dari convex hull
        cv::Mat hull_mask = cv::Mat::zeros(color_image.size(), CV_8UC1);
        if (!hull_points.empty())
            cv::fillPoly(hull_mask, std::vector<std::vector<cv::Point>>{hull_points}, cv::Scalar(255));

        // Apply bitwise AND between color image and hull mask
        cv::Mat hull_result;
        cv::bitwise_and(color_image, color_image, hull_result, hull_mask);

        //?==================================================================
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
        sensor_msgs::msg::Image filtered_binary_msg;
        cv_bridge::CvImage filtered_binary_cv;
        filtered_binary_cv.header.stamp = sync_time_;
        filtered_binary_cv.header.frame_id = "camera_color_optical_frame";
        filtered_binary_cv.encoding = sensor_msgs::image_encodings::MONO8;
        filtered_binary_cv.image = cleaned_yuv;
        filtered_binary_msg = *filtered_binary_cv.toImageMsg();
        pub_filtered_binary_->publish(filtered_binary_msg);

        // -- Publish the filtered binary image
        sensor_msgs::msg::Image filtered_road_msg;
        cv_bridge::CvImage filtered_road_cv;
        filtered_road_cv.header.stamp = sync_time_;
        filtered_road_cv.header.frame_id = "camera_color_optical_frame";
        filtered_road_cv.encoding = sensor_msgs::image_encodings::MONO8;
        filtered_road_cv.image = bev_binary;
        filtered_road_msg = *filtered_road_cv.toImageMsg();
        pub_road_binary_->publish(filtered_road_msg);

        // -- Publsih color hull
        sensor_msgs::msg::Image color_hull_msg;
        cv_bridge::CvImage color_hull_cv;
        color_hull_cv.header.stamp = sync_time_;
        color_hull_cv.header.frame_id = "camera_color_optical_frame";
        color_hull_cv.encoding = sensor_msgs::image_encodings::BGR8;
        color_hull_cv.image = hull_result;
        color_hull_msg = *color_hull_cv.toImageMsg();
        pub_color_hull_->publish(color_hull_msg);
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
        pcl_ros::transformPointCloud(*point_cloud, points_camera2base, tf_camera_base);

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
        sensor_msgs::msg::PointCloud2 msg_filtered_camera_for_sign;
        pcl::toROSMsg(points_camera2base_filtered_for_sign, msg_filtered_camera_for_sign);
        msg_filtered_camera_for_sign.header.stamp = sync_time_;
        msg_filtered_camera_for_sign.header.frame_id = "base_link"; // transformed frame
        pub_sign_points_->publish(msg_filtered_camera_for_sign);
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

    void setup_signal_handlers()
    {
        std::signal(SIGINT, [](int)
                    {
            RCLCPP_INFO(rclcpp::get_logger("vision_capture"), "Received SIGINT, shutting down gracefully...");
            rclcpp::shutdown(); });

        std::signal(SIGTERM, [](int)
                    {
            RCLCPP_INFO(rclcpp::get_logger("vision_capture"), "Received SIGTERM, shutting down gracefully...");
            rclcpp::shutdown(); });
    }

    void process_obstacle_detection(pcl::PointCloud<pcl::PointXYZ> &points)
    {
        // Check if the point cloud is empty
        if (points.empty())
        {
            logger.warn("No points available for obstacle detection");
            return;
        }
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
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<VisionCapture> node_VisionCapture;

    try
    {
        node_VisionCapture = std::make_shared<VisionCapture>();

        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node_VisionCapture);
        executor.spin();
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vision_capture"), "Failed to create VisionCapture node: %s", e.what());
        rclcpp::shutdown();
    }

    if (node_VisionCapture)
        node_VisionCapture.reset();

    rclcpp::shutdown();
    return 0;
}