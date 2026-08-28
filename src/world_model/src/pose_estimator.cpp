#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/global_definitions.hpp"
#include "ros2_utils/help_logger.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

class PoseEstimator : public rclcpp::Node {
public:
    rclcpp::TimerBase::SharedPtr tim_routine;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_enc_meter;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_encoder;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_gyro;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_pose_offset;

    //----TransformBroadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    HelpLogger logger;

    // Configs (static)
    // =======================================================
    float encoder_to_meter = 1.0;
    int routine_period_ms = 20; // ms

    uint16_t encoder = 0;
    uint16_t prev_encoder = 0;
    int16_t delta_encoder = 0;

    float gyro = 0;
    float prev_gyro = 0;
    bool is_gyro_initialized = false;

    rclcpp::Time last_time_gyro_update;
    rclcpp::Time current_time;

    float final_pose_x = 0.0;
    float final_pose_y = 0.0;
    float final_pose_theta = 0.0;

    float final_vel_x = 0.0;
    float final_vel_y = 0.0;
    float final_vel_theta = 0.0;

    float encoder_now = 0.0;

    PoseEstimator()
        : Node("pose_estimator")
    {
        this->declare_parameter("encoder_to_meter", 1.0);
        this->get_parameter("encoder_to_meter", encoder_to_meter);

        this->declare_parameter<int>("routine_period_ms", 20);
        this->get_parameter("routine_period_ms", routine_period_ms);

        if (!logger.init()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }

        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        pub_enc_meter = this->create_publisher<std_msgs::msg::Float32>("/pose_estimator/encoder_meter", 10);

        sub_encoder = this->create_subscription<std_msgs::msg::Int16>(
            "/hardware/wheel_encoder", 1, std::bind(&PoseEstimator::callback_sub_encoder, this, std::placeholders::_1));
        sub_gyro = this->create_subscription<sensor_msgs::msg::Imu>(
            "/hardware/imu", 1, std::bind(&PoseEstimator::callback_sub_gyro, this, std::placeholders::_1));
        sub_pose_offset = this->create_subscription<nav_msgs::msg::Odometry>(
            "/master/pose_offset", 1, std::bind(&PoseEstimator::callback_sub_pose_offset, this, std::placeholders::_1));

        tim_routine = this->create_wall_timer(std::chrono::milliseconds(routine_period_ms), std::bind(&PoseEstimator::callback_routine, this));

        logger.info("PoseEstimator node initialized");
    }

    // ========================================================

    void callback_sub_pose_offset(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        final_pose_x = msg->pose.pose.position.x;
        final_pose_y = msg->pose.pose.position.y;

        // get_angle from quartenion
        tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        logger.info("Pose offset: %.2f %.2f %.2f", final_pose_x, final_pose_y, yaw);

        final_pose_theta = yaw;
    }

    void callback_sub_encoder(const std_msgs::msg::Int16::SharedPtr msg)
    {
        encoder = -msg->data;
    }

    void callback_sub_gyro(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // get_angle from quartenion
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        gyro = yaw;
        last_time_gyro_update = rclcpp::Clock(RCL_SYSTEM_TIME).now();

        static uint16_t init_gyro_counter = 0;

        if (!is_gyro_initialized) {
            if (init_gyro_counter++ > 50) {
                is_gyro_initialized = true;
            }
            prev_gyro = gyro; // Memastikan 0
        }
    }

    // ========================================================

    ~PoseEstimator()
    {
    }

    void callback_routine()
    {
        current_time = rclcpp::Clock(RCL_SYSTEM_TIME).now();

        float d_gyro = gyro - prev_gyro;

        rclcpp::Duration dt_gyro = current_time - last_time_gyro_update;
        static rclcpp::Duration prev_dt_gyro = dt_gyro;
        if ((prev_dt_gyro.seconds() > 0.12 && dt_gyro.seconds() <= 0.12) || fabs(d_gyro) > 7.28 || !is_gyro_initialized) {
            logger.warn("Gyro restarted");
            d_gyro = 0;
        }
        delta_encoder = encoder - prev_encoder;

        prev_dt_gyro = dt_gyro;
        prev_encoder = encoder;
        prev_gyro = gyro;

        while (d_gyro > M_PI)
            d_gyro -= 2 * M_PI;
        while (d_gyro < -M_PI)
            d_gyro += 2 * M_PI;

        final_vel_x = delta_encoder * cosf(final_pose_theta) * encoder_to_meter;
        final_vel_y = delta_encoder * sinf(final_pose_theta) * encoder_to_meter;
        final_vel_theta = d_gyro;

        final_pose_x += final_vel_x;
        final_pose_y += final_vel_y;
        final_pose_theta += final_vel_theta;

        while (final_pose_theta > M_PI)
            final_pose_theta -= 2 * M_PI;
        while (final_pose_theta < -M_PI)
            final_pose_theta += 2 * M_PI;

        tf2::Quaternion q;
        q.setRPY(0, 0, final_pose_theta);

        // logger.info("Pose: %.2f %.2f %.2f", final_pose_x, final_pose_y, final_pose_theta);

        nav_msgs::msg::Odometry msg_odom;
        msg_odom.header.stamp = current_time;
        msg_odom.header.frame_id = "ho";
        msg_odom.child_frame_id = "base_link";
        msg_odom.pose.pose.position.x = final_pose_x;
        msg_odom.pose.pose.position.y = final_pose_y;
        msg_odom.pose.pose.orientation.x = q.x();
        msg_odom.pose.pose.orientation.y = q.y();
        msg_odom.pose.pose.orientation.z = q.z();
        msg_odom.pose.pose.orientation.w = q.w();
        msg_odom.pose.covariance[0] = 1e-2;
        msg_odom.pose.covariance[7] = 1e-2;
        msg_odom.pose.covariance[14] = 1e6;
        msg_odom.pose.covariance[21] = 1e6;
        msg_odom.pose.covariance[28] = 1e6;
        msg_odom.pose.covariance[35] = 1e-2;
        msg_odom.twist.twist.linear.x = final_vel_x * 1000 / routine_period_ms;
        msg_odom.twist.twist.linear.y = final_vel_y * 1000 / routine_period_ms;
        msg_odom.twist.twist.angular.z = final_vel_theta * 1000 / routine_period_ms;
        msg_odom.twist.covariance[0] = 1e-2;
        msg_odom.twist.covariance[7] = 1e-2;
        msg_odom.twist.covariance[14] = 1e6;
        msg_odom.twist.covariance[21] = 1e6;
        msg_odom.twist.covariance[28] = 1e6;
        msg_odom.twist.covariance[35] = 1e-2;
        pub_odom->publish(msg_odom);

        // Publish the transform
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = current_time;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";
        transform.transform.translation.x = final_pose_x;
        transform.transform.translation.y = final_pose_y;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        tf_broadcaster->sendTransform(transform);

        //* ========================================

        // Publish encoder to meter
        encoder_now += delta_encoder * encoder_to_meter;

        std_msgs::msg::Float32 msg_enc_meter;
        msg_enc_meter.data = encoder_now;
        pub_enc_meter->publish(msg_enc_meter);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node_pose_estimator = std::make_shared<PoseEstimator>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_pose_estimator);
    executor.spin();

    return 0;
}