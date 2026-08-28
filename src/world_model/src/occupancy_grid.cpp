#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include "ros2_utils/help_logger.hpp"
#include "ros2_utils/global_definitions.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

struct ObstacleMemory
{
    PointT point;
    rclcpp::Time last_seen;
};

class OccupancyProcessorNode : public rclcpp::Node
{
public:
    float res = 0.02f;            // 2 cm
    int width = 100;              // 2 meter
    int height = 100;             // 2 meter
    float ox = -1.0f, oy = -1.0f; // Origin, agar robot di tengah grid
    float memory_timeout_sec_ = 1.0;
    float blind_spot_radius_ = 0.5;

    HelpLogger logger;

    rclcpp::TimerBase::SharedPtr tim_routine;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_obstacle_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_road_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_grid_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    PointCloudT::Ptr latest_obstacle_{new PointCloudT};
    PointCloudT::Ptr latest_road_{new PointCloudT};

    std::vector<ObstacleMemory> memory_;

    OccupancyProcessorNode() : Node("occupancy_processor_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        this->declare_parameter<float>("res", 0.02f);
        this->get_parameter("res", res);

        this->declare_parameter<int>("width", 100);
        this->get_parameter("width", width);

        this->declare_parameter<int>("height", 100);
        this->get_parameter("height", height);

        this->declare_parameter<float>("ox", -1.0f);
        this->get_parameter("ox", ox);

        this->declare_parameter<float>("oy", -1.0f);
        this->get_parameter("oy", oy);

        this->declare_parameter<float>("memory_timeout_sec", 1.0);
        this->get_parameter("memory_timeout_sec", memory_timeout_sec_);

        this->declare_parameter<float>("blind_spot_radius", 0.5);
        this->get_parameter("blind_spot_radius", blind_spot_radius_);

        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }

        sub_obstacle_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/detection/road_obstacle_combined", 1, std::bind(&OccupancyProcessorNode::obstacleCallback, this, std::placeholders::_1));

        sub_road_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/detection/pointcloud2", 1, std::bind(&OccupancyProcessorNode::roadCallback, this, std::placeholders::_1));

        pub_grid_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("grid_map", 10);

        tim_routine = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&OccupancyProcessorNode::callback_routine, this));

        logger.info("Occupancy Grid node initialized");
    }

    void obstacleCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        sensor_msgs::msg::PointCloud2 transformed_msg;
        try
        {
            tf_buffer_.transform(*msg, transformed_msg, "map");
            pcl::fromROSMsg(transformed_msg, *latest_obstacle_);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
        }
    }

    void roadCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        sensor_msgs::msg::PointCloud2 cloud_msg_map;
        tf_buffer_.transform(*msg, cloud_msg_map, "map");
        pcl::fromROSMsg(cloud_msg_map, *latest_road_);
    }

    void callback_routine()
    {
        if (latest_road_->empty() || latest_obstacle_->empty())
            return;

        nav_msgs::msg::OccupancyGrid grid;

        grid.info.resolution = res;
        grid.info.width = width;
        grid.info.height = height;
        grid.info.origin.position.x = ox;
        grid.info.origin.position.y = oy;
        grid.info.origin.orientation.w = 1.0;
        grid.data.resize(width * height, -1); // Default unknown

        rclcpp::Time now = this->now();

        // TF: ambil posisi robot dalam frame map
        geometry_msgs::msg::TransformStamped tf;
        float robot_x = 0.0f, robot_y = 0.0f, yaw = 0.0f;

        try
        {
            tf = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
            robot_x = tf.transform.translation.x;
            robot_y = tf.transform.translation.y;
            yaw = tf2::getYaw(tf.transform.rotation);
        }
        catch (...)
        {
            RCLCPP_WARN(this->get_logger(), "TF base_link → map unavailable.");
            return;
        }

        // Mark area di depan robot sebagai FREE
        markFrontOfRobot(grid, robot_x, robot_y, yaw, ox, oy, res, width, height);

        // Tambahkan obstacle memory (buffer)
        for (const auto &pt : latest_obstacle_->points)
        {
            memory_.push_back({pt, now});
        }

        // Marking obstacle memory ke grid
        for (const auto &mem : memory_)
        {
            geometry_msgs::msg::PointStamped pt_in, pt_out;
            pt_in.header.frame_id = "map";
            pt_in.point.x = mem.point.x;
            pt_in.point.y = mem.point.y;
            pt_in.point.z = mem.point.z;

            try
            {
                tf_buffer_.transform(pt_in, pt_out, "base_link", tf2::durationFromSec(0.05));
                if ((now - mem.last_seen).seconds() <= memory_timeout_sec_)
                {
                    markCell(grid, mem.point.x, mem.point.y, ox, oy, res, width, height, 100);
                }
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
            }
        }

        // Marking road points sebagai OCCUPIED (100)
        for (const auto &pt : latest_road_->points)
        {
            markCell(grid, pt.x, pt.y, ox, oy, res, width, height, 100);
        }

        // Blind spot masking di sekitar robot dalam frame map
        for (float dx = -blind_spot_radius_; dx <= blind_spot_radius_; dx += 0.05f)
        {
            for (float dy = -blind_spot_radius_; dy <= blind_spot_radius_; dy += 0.05f)
            {
                if (dx * dx + dy * dy <= blind_spot_radius_ * blind_spot_radius_)
                {
                    float mx = robot_x + dx;
                    float my = robot_y + dy;
                    markCell(grid, mx, my, ox, oy, res, width, height, 100);
                }
            }
        }

        // Publish final grid
        grid.header.stamp = now;
        grid.header.frame_id = "map";
        pub_grid_->publish(grid);
    }

    void markFrontOfRobot(nav_msgs::msg::OccupancyGrid &grid,
                          float robot_x, float robot_y, float yaw,
                          float ox, float oy, float res, int width, int height)
    {
        float min_dx = 0.0f, max_dx = 2.0f;
        float min_dy = -0.3f, max_dy = 0.3f;

        for (float dx = min_dx; dx < max_dx; dx += res)
        {
            for (float dy = min_dy; dy < max_dy; dy += res)
            {
                // Rotasi koordinat relatif terhadap arah robot
                float gx = robot_x + dx * std::cos(yaw) - dy * std::sin(yaw);
                float gy = robot_y + dx * std::sin(yaw) + dy * std::cos(yaw);

                markCell(grid, gx, gy, ox, oy, res, width, height, 0); // FREE
            }
        }
    }

    void markCell(nav_msgs::msg::OccupancyGrid &grid, float x, float y,
                  float ox, float oy, float res, int width, int height, int value)
    {
        int cx = static_cast<int>((x - ox) / res);
        int cy = static_cast<int>((y - oy) / res);
        if (cx >= 0 && cx < width && cy >= 0 && cy < height)
        {
            int index = cy * width + cx;
            grid.data[index] = value;
        }
    }

    void markAreaInFrontAsFree(nav_msgs::msg::OccupancyGrid &grid,
                               float ox, float oy, float res,
                               int width, int height)
    {
        // Area yang ingin ditandai:
        // X dari 0m (robot) sampai 1m ke depan
        // Y dari -0.5m sampai +0.5m dari sumbu robot (total 1m lebar)
        float min_x = 0.0f, max_x = 2.0f;
        float min_y = -0.3f, max_y = 0.3f;

        for (float x = min_x; x < max_x; x += res)
        {
            for (float y = min_y; y < max_y; y += res)
            {
                int gx = static_cast<int>((x - ox) / res);
                int gy = static_cast<int>((y - oy) / res);
                if (gx >= 0 && gx < width && gy >= 0 && gy < height)
                {
                    grid.data[gy * width + gx] = 0; // FREE
                }
            }
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OccupancyProcessorNode>());
    rclcpp::shutdown();
    return 0;
}
