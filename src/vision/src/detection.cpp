#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/global_definitions.hpp"
#include "ros2_utils/help_logger.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <boost/thread/mutex.hpp>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <opencv2/core.hpp>
#include <opencv2/core/persistence.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "geometry_msgs/msg/point_stamped.hpp"
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>

#include "pcl_ros/transforms.hpp"
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <limits>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <sstream>
#include <thread>

// #define DEG2RAD *0.017452925
// #define RAD2DEG *57.295780

using namespace std::chrono_literals;

class Detection : public rclcpp::Node
{
public:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_bgr_rs;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_depth_rs;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info_rs;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_camera_depth_points;

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

    // Transform
    // ---------
    bool tf_is_initialized = false;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    geometry_msgs::msg::TransformStamped tf_camera_base;

    HelpLogger logger;

    bool publish_pointcloud2 = false;

    int routine_period_ms = 20;
    std::thread routine_thread_;

    std::mutex mutex_image_bgr;
    std::mutex mutex_image_depth;

    cv::Mat image_bgr;
    cv::Mat image_depth;
    double image_bgr_timestamp;
    double image_depth_timestamp;

    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    int center_cam_x;
    int center_cam_y;

    /*
     * Camera error flags
     * 0x00000001 - Camera image not received
     * 0x00000010 - Camera depth not received
     * 0x00000100 - Camera info not received
     * 0x00001000 - Camera point cloud not received
     */
    uint8_t camera_error = 0x00000000;

    cv::Mat homography_matrix;
    bool homography_loaded = false;

    rclcpp::Time time_frame_capture;

    // Point cloud
    // -----------
    pcl::PointCloud<pcl::PointXYZ> points_camera;
    pcl::PointCloud<pcl::PointXYZ> points_camera2map;
    pcl::PointCloud<pcl::PointXYZ> points_camera2base;
    pcl::PointCloud<pcl::PointXYZ> points_camera2base_filtered;
    pcl::PointCloud<pcl::PointXYZ> points_camera2base_filtered_for_sign;

    Detection()
        : Node("Detection"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }

        this->declare_parameter<bool>("publish_pointcloud2", false);
        this->get_parameter("publish_pointcloud2", publish_pointcloud2);

        sub_image_bgr_rs = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/rs2_cam_main/color/image_raw", 1,
            std::bind(&Detection::callback_sub_image_bgr_rs, this, std::placeholders::_1));

        sub_image_depth_rs = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/rs2_cam_main/aligned_depth_to_color/image_raw", 1,
            std::bind(&Detection::callback_sub_image_depth_rs, this, std::placeholders::_1));

        sub_camera_info_rs = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/rs2_cam_main/color/camera_info", 1,
            std::bind(&Detection::callback_sub_camera_info_rs, this, std::placeholders::_1));

        // sub_camera_depth_points = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        //     "/camera/rs2_cam_main/depth/color/points", 1,
        //     std::bind(&Detection::callback_sub_camera_depth_points, this, std::placeholders::_1));

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

        routine_thread_ = std::thread(std::bind(&Detection::callback_routine, this), this);

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

        logger.info("Detection node initialized");
    }

    ~Detection()
    {
        if (routine_thread_.joinable())
        {
            routine_thread_.join(); // Ensure the thread is joined before exiting
        }
    }

    void callback_sub_camera_info_rs(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        try
        {
            camera_matrix = cv::Mat(3, 3, CV_64F, msg->k.data()).clone();
            dist_coeffs = cv::Mat(1, 5, CV_64F, msg->d.data()).clone();
            camera_error &= 0x11111011;
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert camera info: %s", e.what());
            camera_error |= 0x00000100; // Set error bit for camera info
        }
    }

    void callback_sub_camera_depth_points(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
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

            // Convert from ROS PointCloud2 to PCL

            // Transform the point cloud from camera frame to base frame
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

            // make flatten point cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr points_camera2base_flat(new pcl::PointCloud<pcl::PointXYZ>());
            for (const auto &point : points_camera2base_filtered.points)
            {
                pcl::PointXYZ flat_point(point.x, point.y, 0.0); // Set Z to 0 for flattening
                points_camera2base_flat->points.push_back(flat_point);
            }

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

            //

            points_camera2base_flat->width = points_camera2base_flat->points.size();
            points_camera2base_flat->height = 1; // Set height to 1 for a flat point cloud
            points_camera2base_flat->is_dense = true;
            points_camera2base_flat->header = points_camera2base_filtered.header;

            // Publish the flattened point cloud
            sensor_msgs::msg::PointCloud2 msg_flat;
            pcl::toROSMsg(*points_camera2base_flat, msg_flat);
            msg_flat.header = msg->header;
            msg_flat.header.frame_id = "base_link"; // transformed frame
            pub_flattened_points->publish(msg_flat);

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

    void callback_sub_image_depth_rs(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            std::lock_guard<std::mutex> lock(mutex_image_depth);
            auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            image_depth = cv_ptr->image.clone();
            camera_error &= 0x11111101;
            image_depth_timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
            time_frame_capture = msg->header.stamp;

            // logger.info("Depth image received: %f", image_depth_timestamp);
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            camera_error |= 0x00000010;
        }
    }

    void callback_sub_image_bgr_rs(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            std::lock_guard<std::mutex> lock(mutex_image_bgr);
            auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            image_bgr = cv_ptr->image.clone();
            camera_error &= 0x11111110;
            image_bgr_timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

            // logger.info("BGR image received: %f", image_bgr_timestamp);
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            camera_error |= 0x00000001;
        }
    }

    void callback_routine()
    {
        while (rclcpp::ok())
        {
            double current_time = this->now().seconds();
            cv::Mat frame_bgr;
            cv::Mat frame_depth;

            //? ==================================================
            //?                   Get Image
            //? ==================================================
            {
                std::lock_guard<std::mutex> lock(mutex_image_bgr);
                frame_bgr = image_bgr.clone();
                // logger.info("BGR image size: %d x %d %d", frame_bgr.cols, frame_bgr.rows, camera_error);
            }

            {
                std::lock_guard<std::mutex> lock(mutex_image_depth);
                frame_depth = image_depth.clone();
                // logger.info("Depth image size: %d x %d", frame_depth.cols, frame_depth.rows);
            }

            if (frame_bgr.empty() || frame_depth.empty())
            {
                RCLCPP_WARN(this->get_logger(), "Image is empty");
                std::this_thread::sleep_for(std::chrono::milliseconds(routine_period_ms));
                continue;
            }

            if (image_bgr_timestamp < current_time - 0.5 || image_depth_timestamp < current_time - 0.5)
            {
                logger.warn("Image timestamp is too old (%.2f) (%.2f) (%.2f)", image_bgr_timestamp, image_depth_timestamp, current_time);
                std::this_thread::sleep_for(std::chrono::milliseconds(routine_period_ms));
                continue;
            }
            //? ==================================================
            //?                Prepocess Image
            //? ==================================================
            cv::Mat undistorted_frame;
            cv::undistort(frame_bgr, undistorted_frame, camera_matrix, dist_coeffs);

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

            std::vector<int> removed_contours_idx;
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(otsu_binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            // find largest contour area
            if (contours.empty())
            {
                logger.warn("No contours found in the image");
                std::this_thread::sleep_for(std::chrono::milliseconds(routine_period_ms));
                continue;
            }
            std::sort(contours.begin(), contours.end(),
                      [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b)
                      {
                          return cv::contourArea(a) > cv::contourArea(b);
                      });

            // int largest_contour_area = cv::contourArea(contours[0]);

            for (size_t i = 0; i < contours.size(); i++)
            {
                // float area = cv::contourArea(contours[i]);
                float height = cv::boundingRect(contours[i]).height;
                float width = cv::boundingRect(contours[i]).width;
                // put text id and area in cnt location
                cv::putText(debug_frame, std::to_string(width) + " " + std::to_string(height),
                            cv::Point(contours[i][0].x, contours[i][0].y - 10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);

                // if (cv::contourArea(contours[i]) < largest_contour_area * 0.7) {
                //     removed_contours_idx.push_back(i);
                //     continue;
                // }

                if (height < 180)
                {
                    removed_contours_idx.push_back(i);
                    continue;
                }
            }

            for (size_t i = 0; i < removed_contours_idx.size(); i++)
            {
                contours.erase(contours.begin() + removed_contours_idx[i] - i);
            }

            cv::drawContours(filtered_binary, contours, -1, cv::Scalar(255), cv::FILLED);
            cv::drawContours(debug_frame, contours, -1, cv::Scalar(255), cv::FILLED);

            // cv::Mat edges;
            // cv::Canny(filtered_binary, edges, 50, 70, 3);
            // cv::dilate(edges, edges, cv::Mat(), cv::Point(-1, -1), 5);

            // std::vector<cv::Point> horizontal_points = sliding_windows(edges, edges.cols, 20,
            //     10, debug_frame);
            // std::vector<cv::Point> vertical_points = sliding_windows(edges, 20, edges.rows,
            //     10, debug_frame);

            // std::vector<cv::Point> all_points;
            // all_points.insert(all_points.end(), horizontal_points.begin(), horizontal_points.end());
            // all_points.insert(all_points.end(), vertical_points.begin(), vertical_points.end());
            //? ==================================================
            //?                 Get Pointcloud
            //? ==================================================
            std::vector<cv::Point> point_cloud;
            // find_point_cloud(&point_cloud, filtered_binary, 1);

            for (size_t rows = 0; rows < filtered_binary.rows; rows++)
            {
                for (size_t cols = 0; cols < filtered_binary.cols; cols++)
                {
                    if (filtered_binary.at<uint8_t>(rows, cols) > 0)
                    {
                        point_cloud.emplace_back(cols - center_cam_x, center_cam_y - rows);
                    }
                }
            }

            if (point_cloud.empty())
            {
                logger.warn("No pointcloud found in the image");
                std::this_thread::sleep_for(std::chrono::milliseconds(routine_period_ms));
                continue;
            }
            //? ==================================================
            //?                Convert to Depth
            //? ==================================================
            std::vector<cv::Point3d> point_cloud_world;
            for (const auto &point : point_cloud)
            {
                int pixel_x = point.x + center_cam_x;
                int pixel_y = center_cam_y - point.y;

                if (pixel_x < 0 || pixel_x >= frame_depth.cols || pixel_y < 0 || pixel_y >= frame_depth.rows)
                    continue;

                uint16_t depth_value = frame_depth.at<uint16_t>(pixel_y, pixel_x);
                if (depth_value == 0)
                    continue;

                double depth_meters = depth_value * 0.001; // Convert to meters
                cv::Point3d world_point;
                world_point.y = -(pixel_x - camera_matrix.at<double>(0, 2)) * depth_meters / camera_matrix.at<double>(0, 0);
                world_point.z = -(pixel_y - camera_matrix.at<double>(1, 2)) * depth_meters / camera_matrix.at<double>(1, 1);
                world_point.x = depth_meters;

                if (world_point.z < 0.1 && world_point.z > -0.2 || 1)
                {
                    point_cloud_world.push_back(world_point);
                }
            }

            if (point_cloud_world.empty())
            {
                logger.warn("No pointcloud found in the image");
                std::this_thread::sleep_for(std::chrono::milliseconds(routine_period_ms));
                continue;
            }

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

            if (publish_pointcloud2)
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
                    ptr[1] = point_cloud_world[i].y + 0.1;
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
                {
                    point.z = offset_z;
                }

                sensor_msgs::msg::PointCloud2 output_msg;
                pcl::toROSMsg(pcl_cloud, output_msg);
                output_msg.header.frame_id = "base_link";
                output_msg.header.stamp = time_frame_capture;
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
                flattened_msg.header.stamp = time_frame_capture;
                pub_road_obs_combined->publish(flattened_msg);

                sensor_msgs::msg::LaserScan scan;
                scan.header.frame_id = "base_link";
                scan.header.stamp = time_frame_capture;
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
                    {
                        scan.ranges[index] = range;
                    }
                }

                // logger.info("Publishing LaserScan with %zu ranges", scan.ranges.size());
                pub_point_cloud_laser_scan->publish(scan);
            }
            //? ==================================================
            //?                   Publish Frame
            //? ==================================================
            // sensor_msgs::msg::Image debug_frame_msg;
            // cv_bridge::CvImage debug_frame_cv;
            // debug_frame_cv.header.stamp = this->now();
            // debug_frame_cv.header.frame_id = "camera_depth_optical_frame";
            // debug_frame_cv.encoding = sensor_msgs::image_encodings::BGR8;
            // debug_frame_cv.image = debug_frame;
            // debug_frame_msg = *debug_frame_cv.toImageMsg();
            // pub_debug_frame->publish(debug_frame_msg);

            // // convert filtered_binary to Mat with rgb8 encoding
            // cv::Mat filtered_binary_rgb = cv::Mat::zeros(filtered_binary.size(), CV_8UC3);
            // cv::cvtColor(otsu_binary, filtered_binary_rgb, cv::COLOR_GRAY2BGR);
            // sensor_msgs::msg::Image filtered_binary_msg;
            // cv_bridge::CvImage filtered_binary_cv;
            // filtered_binary_cv.header.stamp = this->now();
            // filtered_binary_cv.header.frame_id = "camera_depth_optical_frame";
            // filtered_binary_cv.encoding = sensor_msgs::image_encodings::BGR8;
            // filtered_binary_cv.image = filtered_binary_rgb;
            // filtered_binary_msg = *filtered_binary_cv.toImageMsg();
            // pub_filtered_binary->publish(filtered_binary_msg);
            //? ==================================================
            std::this_thread::sleep_for(std::chrono::milliseconds(routine_period_ms));
        }
    }

    std::vector<cv::Point> sliding_windows(const cv::Mat &binary_image, int width, int height,
                                           int min_pixels, cv::Mat &output_image)
    {
        std::vector<cv::Point> detected_points;
        cv::Scalar color;

        if (width > height)
        {
            color = cv::Scalar(255, 0, 0);
        }
        else
        {
            color = cv::Scalar(0, 0, 255);
        }

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
                    cv::circle(output_image, cv::Point(cx, cy), 5, color, -1);
                }

                // Draw the window
                // cv::rectangle(output_image, window, color, 1);
            }
        }

        return detected_points;
    }

    void find_point_cloud(std::vector<cv::Point> *line_ptr, const cv::Mat &mask, uint8_t is_rising)
    {
        line_ptr->clear();

        // Precompute cos/sin for all angles to avoid repeated computation
        constexpr float angle_step = 2.5f;
        const int num_angles = static_cast<int>(180.0f / angle_step);
        std::vector<float> cos_vals(num_angles), sin_vals(num_angles);
        for (int i = 0; i < num_angles; ++i)
        {
            float angle_rad = i * angle_step * 0.017452925f;
            cos_vals[i] = std::cos(angle_rad);
            sin_vals[i] = std::sin(angle_rad);
        }

        // For each angle, search along the ray for the first transition
        for (int i = 0; i < num_angles; ++i)
        {
            float cos_a = cos_vals[i];
            float sin_a = sin_vals[i];
            int last_x = -1, last_y = -1;
            for (float magnitude = 0; magnitude < mask.cols * 0.6f; magnitude += 1.0f)
            {
                int pixel_x = static_cast<int>(center_cam_x + magnitude * cos_a + 0.5f);
                int pixel_y = static_cast<int>(center_cam_y - magnitude * sin_a + 0.5f);

                if (pixel_x < 0 || pixel_x >= mask.cols || pixel_y < 0 || pixel_y >= mask.rows)
                    break;

                // Skip duplicate points
                if (pixel_x == last_x && pixel_y == last_y)
                    continue;
                last_x = pixel_x;
                last_y = pixel_y;

                uint8_t pixel_value = mask.at<uint8_t>(pixel_y, pixel_x);

                if (is_rising)
                {
                    if (pixel_value == 255)
                    {
                        line_ptr->emplace_back(pixel_x - center_cam_x, center_cam_y - pixel_y);
                        break;
                    }
                }
                else
                {
                    if (pixel_value == 0)
                    {
                        line_ptr->emplace_back(pixel_x - center_cam_x, center_cam_y - pixel_y);
                        break;
                        break;
                    }
                }
            }
            // }
            // // logger.info("Found %zu points in point cloud and buffer %d", line_ptr->size(), buffer);

            // if (line_ptr->size() < 10 || buffer < 3600) {
            //     line_ptr->clear();

            //     // For each angle, search along the ray for the first transition
            //     for (int i = 0; i < num_angles; ++i) {
            //         float cos_a = cos_vals[i];
            //         float sin_a = sin_vals[i];
            //         int last_x = -1, last_y = -1;
            //         for (int8_t j = -1; j < 2; j += 2) {
            //             for (float magnitude = 0; magnitude < mask.cols * 0.6f; magnitude += 1.0f) {
            //                 int pixel_x = static_cast<int>(center_cam_x + (center_cam_x * 0.25 * j) + magnitude * cos_a + 0.5f);
            //                 int pixel_y = static_cast<int>(center_cam_y - magnitude * sin_a + 0.5f);

            //                 if (pixel_x < 0 || pixel_x >= mask.cols || pixel_y < 0 || pixel_y >= mask.rows)
            //                     break;

            //                 // Skip duplicate points
            //                 if (pixel_x == last_x && pixel_y == last_y)
            //                     continue;
            //                 last_x = pixel_x;
            //                 last_y = pixel_y;

            //                 uint8_t pixel_value = mask.at<uint8_t>(pixel_y, pixel_x);

            //                 if (is_rising) {
            //                     if (pixel_value == 255) {
            //                         line_ptr->emplace_back(pixel_x - center_cam_x, center_cam_y - pixel_y);
            //                         break;
            //                     }
            //                 } else {
            //                     if (pixel_value == 0) {
            //                         line_ptr->emplace_back(pixel_x - center_cam_x, center_cam_y - pixel_y);
            //                         break;
            //                     }
            //                 }
            //             }
            //         }
            //     }
            // }
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_detection = std::make_shared<Detection>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_detection);
    executor.spin();

    return 0;
}