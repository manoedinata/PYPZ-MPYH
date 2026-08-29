#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "ros2_utils/global_definitions.hpp"
#include "ros2_utils/help_logger.hpp"
#include <opencv2/core.hpp>
#include <opencv2/core/persistence.hpp>
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/point_stamped.hpp"
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/string.hpp>

#include "pcl_ros/transforms.hpp"
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <boost/thread/mutex.hpp>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <thread>

using namespace std::chrono_literals;

class Detection2Node : public rclcpp::Node
{

  private:
    rclcpp::CallbackGroup::SharedPtr sub_camera_bgr_rs_group_;
    rclcpp::CallbackGroup::SharedPtr sub_camera_depth_rs_group_;
    rclcpp::CallbackGroup::SharedPtr sub_camera_info_rs_group_;
    rclcpp::CallbackGroup::SharedPtr sub_camera_pcl_rs_group_;

    rclcpp::CallbackGroup::SharedPtr tim_routine_group_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_camera_bgr_rs_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_camera_depth_rs_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info_rs_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_camera_pcl_rs_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_point_cloud_3d;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_point_cloud_3d_2;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_road_obs_combined;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_point_cloud_laser_scan;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_frame;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_filtered_binary;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_nearest_obstacle;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered_points;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_flattened_points;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_sign_points;

    rclcpp::TimerBase::SharedPtr timer_routine_;

    // Transform
    // ---------
    bool tf_is_initialized = false;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    geometry_msgs::msg::TransformStamped tf_camera_base;

    HelpLogger logger;

    bool publish_pointcloud2 = true;

    // ------------------------------------------------
    // Mutexes for thread safety avoid race conditions
    // ------------------------------------------------
    std::mutex mutex_image_bgr;
    std::mutex mutex_image_depth;
    std::mutex mutex_camera_info;
    // ------------------------------------------------
    // Data that will be buffered from subscriptions
    // ------------------------------------------------
    cv::Mat image_bgr = cv::Mat();
    cv::Mat image_depth = cv::Mat();
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    // ------------------------------------------------
    // Data time stamps from subscriptions
    // ------------------------------------------------
    rclcpp::Time image_bgr_timestamp = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
    rclcpp::Time image_depth_timestamp = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
    // ------------------------------------------------

    int center_cam_x = 0;
    int center_cam_y = 0;

    /*
     * Camera error flags
     * 0x00000001 - Camera image not received
     * 0x00000010 - Camera depth not received
     * 0x00000100 - Camera info not received
     * 0x00001000 - Camera point cloud not received
     */
    uint8_t camera_error = 0x00000000;

    pcl::PointCloud<pcl::PointXYZ> points_camera;
    pcl::PointCloud<pcl::PointXYZ> points_camera2map;
    pcl::PointCloud<pcl::PointXYZ> points_camera2base;
    pcl::PointCloud<pcl::PointXYZ> points_camera2base_filtered;
    pcl::PointCloud<pcl::PointXYZ> points_camera2base_filtered_for_sign;

  public:
    Detection2Node()
        : Node("detection2_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }

        this->declare_parameter<bool>("publish_pointcloud2", true);
        this->get_parameter("publish_pointcloud2", publish_pointcloud2);

        // Create subscriber callback groups
        sub_camera_bgr_rs_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        sub_camera_depth_rs_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        sub_camera_info_rs_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        sub_camera_pcl_rs_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // Create timer callback group
        tim_routine_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // Create subscription options
        auto sub_img_bgr_options = rclcpp::SubscriptionOptions();
        auto sub_img_depth_rs_options = rclcpp::SubscriptionOptions();
        auto sub_img_info_rs_options = rclcpp::SubscriptionOptions();
        auto sub_pcl_rs_options = rclcpp::SubscriptionOptions();

        sub_img_bgr_options.callback_group = sub_camera_bgr_rs_group_;
        sub_img_depth_rs_options.callback_group = sub_camera_depth_rs_group_;
        sub_img_info_rs_options.callback_group = sub_camera_info_rs_group_;
        sub_pcl_rs_options.callback_group = sub_camera_pcl_rs_group_;

        sub_camera_bgr_rs_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/rs2_cam_main/color/image_raw", 1,
            std::bind(&Detection2Node::callback_sub_camera_bgr_rs, this, std::placeholders::_1),
            sub_img_bgr_options);

        sub_camera_depth_rs_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/rs2_cam_main/aligned_depth_to_color/image_raw", 1,
            std::bind(&Detection2Node::callback_sub_camera_depth_rs, this, std::placeholders::_1),
            sub_img_depth_rs_options);

        sub_camera_info_rs_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/rs2_cam_main/color/camera_info", 1,
            std::bind(&Detection2Node::callback_sub_camera_info_rs, this, std::placeholders::_1),
            sub_img_info_rs_options);

        sub_camera_pcl_rs_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/rs2_cam_main/depth/color/points", 1,
            std::bind(&Detection2Node::callback_sub_camera_pcl_rs, this, std::placeholders::_1),
            sub_pcl_rs_options);

        pub_point_cloud_3d = this->create_publisher<sensor_msgs::msg::PointCloud>(
            "/detection/pointcloud", 1);
        pub_point_cloud_3d_2 = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/detection/pointcloud2", 1);
        pub_point_cloud_laser_scan = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/detection/pointcloud_laser_scan", 1);
        pub_debug_frame = this->create_publisher<sensor_msgs::msg::Image>(
            "/detection/debug_frame", 1);
        pub_filtered_binary = this->create_publisher<sensor_msgs::msg::Image>(
            "/detection/filtered_binary", 1);
        pub_nearest_obstacle = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/detection/nearest_obstacle", 1);
        pub_filtered_points = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/detection/filtered_points", 1);
        pub_flattened_points = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/detection/flattened_points", 1);
        pub_road_obs_combined = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/detection/road_obstacle_combined", 1);
        pub_sign_points = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/detection/sign_points", 1);

        timer_routine_ = this->create_wall_timer(
            10ms,
            std::bind(&Detection2Node::callback_tim_routine, this),
            tim_routine_group_);

        while (!tf_is_initialized)
        {
            rclcpp::sleep_for(1s);
            try
            {
                tf_camera_base = tf_buffer_.lookupTransform("base_link", "camera_depth_optical_frame", tf2::TimePointZero);
                tf_is_initialized = true;
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "TF not ready: %s", ex.what());
                rclcpp::sleep_for(std::chrono::milliseconds(100));
            }
        }

        RCLCPP_INFO(this->get_logger(), "Detection2 node initialized with multithreading");
    }

  private:
    void callback_sub_camera_bgr_rs(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // // ==================================================================
            // //                        DEBUG VISION CAPTURE
            // // ==================================================================
            // double start_time = this->now().seconds();
            // static double last_time = start_time;
            // double elapsed_time = start_time - last_time;
            // last_time = start_time;
            // logger.info("Timer routine elapsed time: %.4f seconds -> %.2f", elapsed_time, 1 / elapsed_time);
            // // ==================================================================
            std::lock_guard<std::mutex> lock(mutex_image_bgr);
            auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            image_bgr = cv_ptr->image.clone();
            camera_error &= 0x11111110;
            image_bgr_timestamp = msg->header.stamp;

            // logger.info("BGR image received: %f", image_bgr_timestamp);
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            camera_error |= 0x00000001;
        }
    }

    void callback_sub_camera_depth_rs(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            std::lock_guard<std::mutex> lock(mutex_image_depth);
            auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            image_depth = cv_ptr->image.clone();
            camera_error &= 0x11111101;
            image_depth_timestamp = msg->header.stamp;

            // logger.info("Depth image received: %f", image_depth_timestamp);
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            camera_error |= 0x00000010;
        }
    }

    void callback_sub_camera_info_rs(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        try
        {
            std::lock_guard<std::mutex> lock(mutex_camera_info);
            camera_matrix = cv::Mat(3, 3, CV_64F, msg->k.data()).clone();
            dist_coeffs = cv::Mat(1, 5, CV_64F, msg->d.data()).clone();
            camera_error &= 0x11111011;

            // logger.info("Camera info received: %s", msg->header.frame_id.c_str());
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert camera info: %s", e.what());
            camera_error |= 0x00000100; // Set error bit for camera info
        }
    }

    void callback_sub_camera_pcl_rs(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        try
        {
            if (!tf_is_initialized)
            {
                RCLCPP_WARN(this->get_logger(), "TF not initialized, skipping point cloud processing");
                return;
            }

            pcl::fromROSMsg(*msg, points_camera);

            if (points_camera.empty())
            {
                RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
                points_camera2base.clear();
                return;
            }

            pcl_ros::transformPointCloud(points_camera, points_camera2base, tf_camera_base);

            // Filter: keep only points above 2 cm from ground
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(points_camera2base.makeShared());
            pass.setFilterFieldName("z");
            pass.setFilterLimits(0.08, 2.0); // Only keep points from 2cm to 2m above ground
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_z(new pcl::PointCloud<pcl::PointXYZ>());
            pass.filter(*filtered_z);

            // Crop region in front of the robot (X = forward, Y = side-to-side, Z already filtered)
            static pcl::CropBox<pcl::PointXYZ> crop_box;
            crop_box.setInputCloud(filtered_z);
            crop_box.setMin(Eigen::Vector4f(0.0, -0.3, 0.05, 1.0)); // In front of robot
            crop_box.setMax(Eigen::Vector4f(1.5, 0.3, 1.0, 1.0));   // 2m ahead, ±1m wide, max 2m tall
            crop_box.setNegative(false);                            // Keep only points inside the box
            crop_box.filter(points_camera2base_filtered);

            // =============== ROAD SIGN DETECTION ===============
            pcl::PassThrough<pcl::PointXYZ> pass_sign;
            pass_sign.setInputCloud(points_camera2base.makeShared());
            pass_sign.setFilterFieldName("z");
            pass_sign.setFilterLimits(0.07, 2.0); // Only keep points from 2cm to 2m above ground
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_z_sign(new pcl::PointCloud<pcl::PointXYZ>());
            pass_sign.filter(*filtered_z_sign);

            // Crop region in front of the robot (X = forward, Y = side-to-side, Z already filtered)
            static pcl::CropBox<pcl::PointXYZ> crop_box_sign;
            crop_box_sign.setInputCloud(filtered_z_sign);
            crop_box_sign.setMin(Eigen::Vector4f(0.02, -1.0, 0.05, 1.0)); // min x, y, z
            crop_box_sign.setMax(Eigen::Vector4f(1.4, -0.01, 1.0, 1.0));  // max x, y, z
            crop_box_sign.setNegative(false);
            crop_box_sign.filter(points_camera2base_filtered_for_sign);

            // Publish the filtered point cloud
            sensor_msgs::msg::PointCloud2 msg_filtered_camera;
            pcl::toROSMsg(points_camera2base_filtered, msg_filtered_camera);
            msg_filtered_camera.header = msg->header;
            msg_filtered_camera.header.frame_id = "base_link"; // transformed frame
            pub_filtered_points->publish(msg_filtered_camera);

            // Publish the point cloud in base frame
            sensor_msgs::msg::PointCloud2 msg_filtered_camera_for_sign;
            pcl::toROSMsg(points_camera2base_filtered_for_sign, msg_filtered_camera_for_sign);
            msg_filtered_camera_for_sign.header = msg->header;
            msg_filtered_camera_for_sign.header.frame_id = "base_link"; // transformed frame
            pub_sign_points->publish(msg_filtered_camera_for_sign);

            camera_error &= 0x11110111; // Clear all camera errors
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error processing point cloud: %s", e.what());
            camera_error |= 0x00001000;
        }
    }

    void callback_tim_routine()
    {
        //  ==================================================
        logger.info("Timer routine callback started");
        double start_time = this->now().seconds();
        double total_start_time = start_time;
        //  ==================================================
        //? ==================================================
        //?                  Timer Routine
        //? ==================================================
        rclcpp::Time current_time_ros = this->now();
        //? ==================================================
        //?           Buffer Data From Subscriptions
        //? ==================================================
        cv::Mat frame_bgr = cv::Mat();
        cv::Mat frame_depth = cv::Mat();
        cv::Mat frame_matrix = cv::Mat();
        cv::Mat frame_coeffs = cv::Mat();
        //? ==================================================
        //?               Get Image from Buffer
        //? ==================================================
        {
            std::lock_guard<std::mutex> lock(mutex_image_bgr);
            frame_bgr = image_bgr.clone();
        }
        {
            std::lock_guard<std::mutex> lock(mutex_image_depth);
            frame_depth = image_depth.clone();
        }
        {
            std::lock_guard<std::mutex> lock(mutex_camera_info);
            frame_matrix = camera_matrix.clone();
            frame_coeffs = dist_coeffs.clone();
        }

        //? ==================================================
        //?                  Safety Check
        //? ==================================================
        if (frame_bgr.empty() || frame_depth.empty())
        {
            logger.warn("Image is empty");
            return;
        }
        if (image_bgr_timestamp.seconds() < current_time_ros.seconds() - 0.5 || image_depth_timestamp.seconds() < current_time_ros.seconds() - 0.5)
        {
            logger.warn("Image timestamp is too old (%.2f) (%.2f) (%.2f)", image_bgr_timestamp.seconds(), image_depth_timestamp.seconds(), current_time_ros.seconds());
            return;
        }
        //  ==================================================
        double elapsed_time = this->now().seconds() - start_time;
        logger.info("Timer routine elapsed time: %.4f seconds for getting image", elapsed_time);
        start_time = this->now().seconds();
        //  ==================================================
        //? ==================================================
        //?                Prepocess Image
        //? ==================================================
        cv::Mat undistorted_frame;
        cv::undistort(frame_bgr, undistorted_frame, frame_matrix, frame_coeffs);

        center_cam_x = undistorted_frame.cols / 2;
        center_cam_y = undistorted_frame.rows - 1;
        //? ==================================================
        //?                 Process Image
        //? ==================================================
        cv::Mat gray_frame;
        cv::Mat otsu_binary;
        cv::Mat filtered_binary = cv::Mat::zeros(undistorted_frame.size(), CV_8UC1);
        cv::Mat debug_frame = undistorted_frame.clone();

        cv::cvtColor(undistorted_frame, gray_frame, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray_frame, gray_frame, cv::Size(31, 31), 0);
        cv::Canny(gray_frame, otsu_binary, 50, 70, 3);
        cv::dilate(otsu_binary, otsu_binary, cv::Mat(), cv::Point(-1, -1), 3);
        // cv::threshold(gray_frame, otsu_binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

        //  ==================================================
        elapsed_time = this->now().seconds() - start_time;
        logger.info("Timer routine elapsed time: %.4f seconds for processing image - Canny", elapsed_time);
        start_time = this->now().seconds();
        //  ==================================================

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

        //  ==================================================
        elapsed_time = this->now().seconds() - start_time;
        logger.info("Timer routine elapsed time: %.4f seconds for processing image - Contour", elapsed_time);
        start_time = this->now().seconds();
        //  ==================================================

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
            if (height < 180)
            {
                removed_contours_idx.push_back(i);
                continue;
            }
        }

        cv::drawContours(filtered_binary, contours, -1, cv::Scalar(255), cv::FILLED);
        cv::drawContours(debug_frame, contours, -1, cv::Scalar(255), cv::FILLED);

        //  ==================================================
        elapsed_time = this->now().seconds() - start_time;
        logger.info("Timer routine elapsed time: %.4f seconds for processing image - Draw Contour", elapsed_time);
        start_time = this->now().seconds();
        //  ==================================================

        //? --------------------------------------------------
        cv::Mat hsv_frame;
        cv::Mat hsv_binary;
        cv::Mat detected_noise;

        cv::cvtColor(undistorted_frame, hsv_frame, cv::COLOR_BGR2HSV);
        cv::inRange(hsv_frame, cv::Scalar(0, 0, 160), cv::Scalar(180, 255, 255), hsv_binary); //! PERLU JADI PARAMETER

        //  ==================================================
        elapsed_time = this->now().seconds() - start_time;
        logger.info("Timer routine elapsed time: %.4f seconds for processing image - Cvt Color ", elapsed_time);
        start_time = this->now().seconds();
        //  ==================================================

        std::vector<std::pair<int, int>> noise_kernel_sizes = {
            {5, 3}, {7, 3}, {11, 3}, {15, 3}};

        multi_region_horizontal_morphology(hsv_binary, detected_noise, noise_kernel_sizes);
        cv::morphologyEx(detected_noise, detected_noise, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9)));
        cv::dilate(detected_noise, detected_noise, cv::Mat(), cv::Point(-1, -1), 7);
        //? --------------------------------------------------
        cv::bitwise_not(detected_noise, detected_noise);
        // cv::bitwise_and(filtered_binary, detected_noise, filtered_binary);
        // cv::morphologyEx(filtered_binary, filtered_binary, cv::MORPH_ERODE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

        cv::cvtColor(detected_noise, detected_noise, cv::COLOR_GRAY2BGR);
        //  ==================================================
        double elapsed_time_process = this->now().seconds() - start_time;
        logger.info("Timer routine elapsed time: %.4f seconds for processing image - Denoise", elapsed_time_process);
        start_time = this->now().seconds();
        //  ==================================================
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
        //? ==================================================
        //?                Convert to Depth
        //? ==================================================
        std::vector<cv::Point3d> point_cloud_world;
        float fx = frame_matrix.at<double>(0, 0);
        float cx = frame_matrix.at<double>(0, 2);
        float fy = frame_matrix.at<double>(1, 1);
        float cy = frame_matrix.at<double>(1, 2);

        int pixel_x = 0.0;
        int pixel_y = 0.0;
        double depth_meters = 0.0;
        cv::Point3d world_point = cv::Point3d(0.0, 0.0, 0.0);

        for (const auto &point : point_cloud)
        {
            if (!std::isfinite(point.x) || !std::isfinite(point.y))
                continue;

            pixel_x = static_cast<int>(std::round(point.x + center_cam_x));
            pixel_y = static_cast<int>(std::round(center_cam_y - point.y));

            if (pixel_x < 0 || pixel_x >= frame_depth.cols || pixel_y < 0 || pixel_y >= frame_depth.rows)
                continue;

            uint16_t depth_value = frame_depth.at<uint16_t>(pixel_y, pixel_x);
            if (depth_value == 0)
                continue;

            depth_meters = depth_value * 0.001;

            world_point.y = (-(pixel_x - cx) * depth_meters) / fx;
            world_point.z = (-(pixel_y - cy) * depth_meters) / fy;
            world_point.x = depth_meters;

            if (world_point.z < 0.1 && world_point.z > -0.2)
                point_cloud_world.push_back(world_point);
        }

        if (point_cloud_world.empty())
        {
            logger.warn("No pointcloud found in the image");
            // return;
        }
        //  ==================================================
        double elapsed_time_convert = this->now().seconds() - start_time;
        logger.info("Timer routine elapsed time: %.4f seconds for converting pointcloud", elapsed_time_convert);
        start_time = this->now().seconds();
        //  ==================================================
        //? ==================================================
        //?                    Publish PCL
        //? ==================================================
        sensor_msgs::msg::PointCloud point_cloud_msg;
        point_cloud_msg.header.stamp = this->now();
        point_cloud_msg.header.frame_id = "camera_link";
        point_cloud_msg.points.resize(point_cloud_world.size());
        for (size_t i = 0; i < point_cloud_world.size(); ++i)
        {
            point_cloud_msg.points[i].x = point_cloud_world[i].x;
            point_cloud_msg.points[i].y = point_cloud_world[i].y;
            point_cloud_msg.points[i].z = point_cloud_world[i].z;
        }
        pub_point_cloud_3d->publish(point_cloud_msg);

        if (false)
        {
            sensor_msgs::msg::PointCloud2 point_cloud2_msg;
            point_cloud2_msg.header.stamp = this->now();
            point_cloud2_msg.header.frame_id = "camera_link";
            point_cloud2_msg.height = 1;
            point_cloud2_msg.width = point_cloud_world.size();
            point_cloud2_msg.is_dense = false;
            point_cloud2_msg.is_bigendian = false;
            point_cloud2_msg.fields.resize(3);
            point_cloud2_msg.fields[0].name = "x";
            point_cloud2_msg.fields[0].offset = 0;
            point_cloud2_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
            point_cloud2_msg.fields[0].count = 1;
            point_cloud2_msg.fields[1].name = "y";
            point_cloud2_msg.fields[1].offset = 4;
            point_cloud2_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
            point_cloud2_msg.fields[1].count = 1;
            point_cloud2_msg.fields[2].name = "z";
            point_cloud2_msg.fields[2].offset = 8;
            point_cloud2_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
            point_cloud2_msg.fields[2].count = 1;
            point_cloud2_msg.point_step = 12;
            point_cloud2_msg.row_step = point_cloud2_msg.point_step * point_cloud2_msg.width;
            point_cloud2_msg.data.resize(point_cloud2_msg.row_step * point_cloud2_msg.height);
            point_cloud2_msg.data.resize(point_cloud2_msg.row_step * point_cloud2_msg.height);
            for (size_t i = 0; i < point_cloud_world.size(); ++i)
            {
                float *ptr = reinterpret_cast<float *>(&point_cloud2_msg.data[i * point_cloud2_msg.point_step]);
                ptr[0] = point_cloud_world[i].x;
                ptr[1] = point_cloud_world[i].y;
                ptr[2] = point_cloud_world[i].z;
            }

            geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_.lookupTransform(
                "base_link",                      // target frame
                point_cloud2_msg.header.frame_id, // source frame
                point_cloud2_msg.header.stamp,
                rclcpp::Duration::from_seconds(0.1)); // timeout

            sensor_msgs::msg::PointCloud2 transformed_cloud;
            tf2::doTransform(point_cloud2_msg, transformed_cloud, transform_stamped);

            pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
            pcl::fromROSMsg(transformed_cloud, pcl_cloud);

            float offset_z = 1.0f; // example: lower the cloud by 5cm

            for (auto &point : pcl_cloud.points)
                point.z = offset_z;

            sensor_msgs::msg::PointCloud2 output_msg;
            pcl::toROSMsg(pcl_cloud, output_msg);
            output_msg.header.frame_id = "base_link";
            output_msg.header.stamp = image_depth_timestamp;
            pub_point_cloud_3d_2->publish(output_msg);

            // concate the point cloud with the filtered points and flat the combined point cloud
            pcl::PointCloud<pcl::PointXYZ> combined_cloud;
            pcl::PointCloud<pcl::PointXYZ> filtered_cloud;

            combined_cloud += pcl_cloud;
            combined_cloud += points_camera2base_filtered;
            // flatten the combined point cloud
            pcl::PointCloud<pcl::PointXYZ> flattened_cloud;
            for (const auto &point : combined_cloud.points)
            {
                pcl::PointXYZ flat_point(point.x, point.y, 0.0); // Set Z to 0 for flattening
                flattened_cloud.points.push_back(flat_point);
            }

            flattened_cloud.width = flattened_cloud.points.size();
            flattened_cloud.height = 1; // Set height to 1 for a flat point cloud
            flattened_cloud.is_dense = true;
            flattened_cloud.header = combined_cloud.header;
            sensor_msgs::msg::PointCloud2 flattened_msg;

            pcl::toROSMsg(flattened_cloud, flattened_msg);
            flattened_msg.header.frame_id = "base_link";
            flattened_msg.header.stamp = image_depth_timestamp;
            pub_road_obs_combined->publish(flattened_msg);

            sensor_msgs::msg::LaserScan scan;
            scan.header.frame_id = "base_link";
            scan.header.stamp = image_depth_timestamp;
            scan.angle_min = -M_PI;
            scan.angle_max = M_PI;
            scan.angle_increment = 2 * M_PI / 360.0;
            scan.range_min = 0.0;
            scan.range_max = 1.5;

            size_t num_ranges = static_cast<size_t>(std::ceil((scan.angle_max - scan.angle_min) / scan.angle_increment));
            scan.ranges.assign(num_ranges, std::numeric_limits<float>::infinity());

            for (const auto &point : pcl_cloud.points)
            {
                float x = point.x;
                float y = point.y;
                float z = point.z;

                // Optional: ignore points that are too high/low
                // if (std::abs(z) > 0.2)
                //     continue;

                float range = std::hypot(x, y);
                float angle = std::atan2(y, x);

                if (range < scan.range_min || range > scan.range_max)
                    continue;

                int index = std::floor((angle - scan.angle_min) / scan.angle_increment);
                if (index < 0 || index >= static_cast<int>(scan.ranges.size()))
                    continue;

                // Keep closest point for each angles
                if (range < scan.ranges[index])
                    scan.ranges[index] = range;
            }

            // logger.info("Publishing LaserScan with %zu ranges", scan.ranges.size());
            pub_point_cloud_laser_scan->publish(scan);
        }
        //  ==================================================
        double elapsed_time_publish = this->now().seconds() - start_time;
        logger.info("Timer routine elapsed time: %.4f seconds for publishing data", elapsed_time_publish);
        start_time = this->now().seconds();
        //  ==================================================
        //? ==================================================
        //?                   Publish Frame
        //? ==================================================
        sensor_msgs::msg::Image debug_frame_msg;
        cv_bridge::CvImage debug_frame_cv;
        debug_frame_cv.header.stamp = this->now();
        debug_frame_cv.header.frame_id = "camera_depth_optical_frame";
        debug_frame_cv.encoding = sensor_msgs::image_encodings::BGR8;
        debug_frame_cv.image = detected_noise;
        debug_frame_msg = *debug_frame_cv.toImageMsg();
        pub_debug_frame->publish(debug_frame_msg);

        cv::Mat filtered_binary_rgb = cv::Mat::zeros(filtered_binary.size(), CV_8UC3);
        cv::cvtColor(filtered_binary, filtered_binary_rgb, cv::COLOR_GRAY2BGR);
        sensor_msgs::msg::Image filtered_binary_msg;
        cv_bridge::CvImage filtered_binary_cv;
        filtered_binary_cv.header.stamp = this->now();
        filtered_binary_cv.header.frame_id = "camera_depth_optical_frame";
        filtered_binary_cv.encoding = sensor_msgs::image_encodings::BGR8;
        filtered_binary_cv.image = filtered_binary_rgb;
        filtered_binary_msg = *filtered_binary_cv.toImageMsg();
        pub_filtered_binary->publish(filtered_binary_msg);
        //  ==================================================
        double elapsed_time_publish_img = this->now().seconds() - start_time;
        logger.info("Timer routine elapsed time: %.4f seconds for publishing image", elapsed_time_publish_img);
        logger.info("Total elapsed time for timer routine: %.4f seconds", this->now().seconds() - total_start_time);
        //  ==================================================
        //? ==================================================
    }

    void multi_region_horizontal_morphology(const cv::Mat &img, cv::Mat &combined, const std::vector<std::pair<int, int>> &kernels, int iterations = 2)
    {
        int h = img.rows;
        int w = img.cols;
        int num_regions = 4;
        int region_height = h / num_regions;

        combined = cv::Mat::zeros(img.size(), img.type());

        for (int i = 0; i < num_regions; ++i)
        {
            int y_start = i * region_height;
            int y_end = (i == num_regions - 1) ? h : (i + 1) * region_height;

            cv::Mat region = img(cv::Rect(0, y_start, w, y_end - y_start));

            cv::Size kernel_size(kernels[i].first, kernels[i].second);
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, kernel_size);

            cv::Mat morphed;
            cv::morphologyEx(region, morphed, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), iterations);
            morphed.copyTo(combined(cv::Rect(0, y_start, w, y_end - y_start)));
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Detection2Node>();

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 0);
    executor.add_node(node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}