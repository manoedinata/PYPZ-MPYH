// C++ Standard Library

// C Standard Library
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
// Third-party Libraries

// ROS 2 Libraries
#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/global_definitions.hpp"
#include "ros2_utils/help_logger.hpp"
#include "ros2_utils/pid.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

// ROS 2 Messages
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int8.hpp"

#define DEG2RAD *0.01745329251994
#define RAD2DEG *57.29577951308232

// Helper function to constrain a value between min and max
template <typename T>
T constrain(T value, T min_val, T max_val)
{
    return std::max(min_val, std::min(value, max_val));
}

class MotorMain2 : public rclcpp::Node {
private:
    // -------------------------------------------------
    // Transform
    // -------------------------------------------------

    // -------------------------------------------------
    // Logging
    // -------------------------------------------------
    HelpLogger logger;
    // -------------------------------------------------
    // Subscribers Callback Groups
    // -------------------------------------------------
    rclcpp::CallbackGroup::SharedPtr sub_callback_group_;
    // -------------------------------------------------
    // Timer Callback Group
    // -------------------------------------------------
    rclcpp::CallbackGroup::SharedPtr tim_routine_group_;
    // -------------------------------------------------
    // Subscribers
    // -------------------------------------------------
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_wheel_encoder_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_wheel_encoder_dot_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_target_speed_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_target_steering_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_web_config_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_web_init_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_yaw_feedback_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_tuning_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_button_;
    // -------------------------------------------------
    // Publishers
    // -------------------------------------------------
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_pwm_wheel_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_pwm_enable_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_pwm_steering_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_velocity_feedback_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_request_web_config_;
    // -------------------------------------------------
    // Timer
    // -------------------------------------------------
    rclcpp::TimerBase::SharedPtr timer_routine_;
    // -------------------------------------------------
    // Global Variables
    // -------------------------------------------------
    float steering_dutyCycle2rad = 0.0;
    float steering_rad2dutyCycle = 0.0;
    float offset_sudut_steering_rad = 0.0;

    float yaw_feedback_deg_ = 0;
    float yaw_feedback_deg_dot_ = 0;

    int wheel_encoder_value_ = 0;
    int wheel_encoder_dot_value_ = 0;

    float target_speed_value_ = 0;
    float target_steering_value_ = 0;

    int16_t target_pwm_wheel_ = 0;
    int16_t target_pwm_steering_ = 0;

    float output_steering_ = 0;

    float linear_velocity_mps_ = 0.0;

    float encoder_position_m_ = 0.0; // m
    float encoder_velocity_mps_ = 0.0; // m/s

    int8_t button_1 = 0;
    int8_t button_2 = 0;
    int8_t toogle_button_1 = 0;
    int8_t toogle_button_2 = 0;

    float tuning = 0; // parameter
    float K_model = 0.00066619; // m/s per PWM

    // -------------------------------------------------
    // Parameters
    // -------------------------------------------------
    float k_p_wheel_ = 0.0;
    float k_i_wheel_ = 0.0;
    float k_d_wheel_ = 0.0;
    float k_d_steering_ = 0.0;

    float wheel_radius_ = 0.0; // m
    float encoder_ppr_ = 0.0; // Pulses per revolution
    float cnt_to_meter_ = 0.0; // Conversion factor from encoder counts to meters

    float max_steering_deg_ = 0.0; // Maximum steering angle in degrees
    float min_steering_deg_ = 0.0; // Minimum steering angle in degrees
    float max_steering_pwm_ = 0.0; // Maximum PWM for steering
    float min_steering_pwm_ = 0.0; // Minimum PWM for steering
    float mid_steering_pwm_ = 0.0; // Mid PWM for steering

    float max_wheel_velocity_pwm_ = 0.0;
    float min_wheel_velocity_pwm_ = 0.0;
    float max_wheel_integral_pwm_ = 0.0;
    float min_wheel_integral_pwm_ = 0.0;

    float wheel_base_ = 0.0; // Distance between the wheels (m)

public:
    MotorMain2()
        : Node("motor_main2")
    {
        if (!logger.init()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }
        // --------------------------------------------------
        // Get parameters
        // --------------------------------------------------
        this->declare_parameter<float>("k_p_wheel", 0.0);
        this->get_parameter("k_p_wheel", k_p_wheel_);

        this->declare_parameter<float>("k_i_wheel", 0.0);
        this->get_parameter("k_i_wheel", k_i_wheel_);

        this->declare_parameter<float>("k_p_steering", 0.0);
        this->get_parameter("k_p_steering", k_d_wheel_);

        this->declare_parameter<float>("k_d_steering", 0.0);
        this->get_parameter("k_d_steering", k_d_steering_);

        this->declare_parameter<float>("wheel_radius", 0.0365);
        this->get_parameter("wheel_radius", wheel_radius_);

        this->declare_parameter<float>("encoder_ppr", 0.0);
        this->get_parameter("encoder_ppr", encoder_ppr_);

        this->declare_parameter<float>("cnt_to_meter", 0.0);
        this->get_parameter("cnt_to_meter", cnt_to_meter_);

        this->declare_parameter<float>("max_steering_deg", 25.0);
        this->get_parameter("max_steering_deg", max_steering_deg_);

        this->declare_parameter<float>("min_steering_deg", -25.0);
        this->get_parameter("min_steering_deg", min_steering_deg_);

        this->declare_parameter<float>("max_steering_pwm", 1100.0);
        this->get_parameter("max_steering_pwm", max_steering_pwm_);

        this->declare_parameter<float>("min_steering_pwm", 900.0);
        this->get_parameter("min_steering_pwm", min_steering_pwm_);

        this->declare_parameter<float>("mid_steering_pwm", 1000.0);
        this->get_parameter("mid_steering_pwm", mid_steering_pwm_);

        this->declare_parameter<float>("max_wheel_velocity_pwm", 8000.0);
        this->get_parameter("max_wheel_velocity_pwm", max_wheel_velocity_pwm_);

        this->declare_parameter<float>("min_wheel_velocity_pwm", -8000.0);
        this->get_parameter("min_wheel_velocity_pwm", min_wheel_velocity_pwm_);

        this->declare_parameter<float>("max_wheel_integral_pwm", 2500.0);
        this->get_parameter("max_wheel_integral_pwm", max_wheel_integral_pwm_);

        this->declare_parameter<float>("min_wheel_integral_pwm", -2500.0);
        this->get_parameter("min_wheel_integral_pwm", min_wheel_integral_pwm_);

        this->declare_parameter<float>("wheel_base", 0.22);
        this->get_parameter("wheel_base", wheel_base_);
        // --------------------------------------------------
        // Calculating Servo Center
        // --------------------------------------------------
        steering_dutyCycle2rad = (min_steering_deg_ DEG2RAD - max_steering_deg_ DEG2RAD) / (min_steering_pwm_ - max_steering_pwm_);
        steering_rad2dutyCycle = 1.0 / steering_dutyCycle2rad;
        offset_sudut_steering_rad = mid_steering_pwm_ / (steering_rad2dutyCycle);
        logger.info("steering_dutyCycle2rad: %f %f", steering_dutyCycle2rad, offset_sudut_steering_rad);
        // --------------------------------------------------
        // Create subscribers callback groups
        // --------------------------------------------------
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
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = sub_callback_group_;
        // --------------------------------------------------
        // Create subscribers
        // --------------------------------------------------
        sub_wheel_encoder_ = this->create_subscription<std_msgs::msg::Int16>(
            "/hardware/wheel_encoder", 1,
            std::bind(&MotorMain2::callback_sub_wheel_encoder, this, std::placeholders::_1),
            sub_options);
        sub_button_ = this->create_subscription<std_msgs::msg::Int8>(
            "/hardware/button", 1,
            std::bind(&MotorMain2::callback_sub_button, this, std::placeholders::_1),
            sub_options);
        // sub_wheel_encoder_dot_ = this->create_subscription<std_msgs::msg::Int16>(
        //     "/can/delta_encoder_wheel_counter", 1,
        //     std::bind(&MotorMain2::callback_sub_wheel_encoder_dot, this, std::placeholders::_1),
        //     sub_options);
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/hardware/imu", 1,
            std::bind(&MotorMain2::callback_sub_imu, this, std::placeholders::_1),
            sub_options);
        sub_target_speed_ = this->create_subscription<std_msgs::msg::Float32>(
            "/master/target_speed", 1,
            std::bind(&MotorMain2::callback_sub_target_speed, this, std::placeholders::_1),
            sub_options);
        sub_target_steering_ = this->create_subscription<std_msgs::msg::Float32>(
            "/master/target_steering", 1,
            std::bind(&MotorMain2::callback_sub_target_steering, this, std::placeholders::_1),
            sub_options);
        sub_web_config_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/web/config/configuration", 1,
            std::bind(&MotorMain2::callback_sub_web_config, this, std::placeholders::_1),
            sub_options);
        sub_web_init_ = this->create_subscription<std_msgs::msg::Int16>(
            "/web/config/request_config", 1,
            std::bind(&MotorMain2::callback_sub_web_init, this, std::placeholders::_1),
            sub_options);
        sub_tuning_ = this->create_subscription<std_msgs::msg::Int16>(
            "/web/config/tuning", 1,
            std::bind(&MotorMain2::callback_sub_tuning, this, std::placeholders::_1),
            sub_options);

        // --------------------------------------------------
        // Create publishers
        // --------------------------------------------------
        pub_pwm_wheel_ = this->create_publisher<std_msgs::msg::Int16>(
            "/motor_main/target_pwm_wheel", 1);
        pub_pwm_enable_ = this->create_publisher<std_msgs::msg::Int16>(
            "/motor_main/target_pwm_enable", 1);
        pub_pwm_steering_ = this->create_publisher<std_msgs::msg::Int16>(
            "/motor_main/target_steering", 1);
        pub_velocity_feedback_ = this->create_publisher<std_msgs::msg::Float32>(
            "/motor_main/velocity_feedback", 1);
        pub_request_web_config_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/web/config/configuration_init", 1);

        // --------------------------------------------------
        // LOAD CONFIGURATION FROM YAML
        // --------------------------------------------------
        loadConfig();

        // --------------------------------------------------
        // Create timer
        // --------------------------------------------------
        timer_routine_
            = this->create_wall_timer(
                std::chrono::milliseconds(5),
                std::bind(&MotorMain2::callback_tim_routine, this),
                tim_routine_group_);

        logger.info("MotorMain2 node initialized with multithreading");
    }

    ~MotorMain2() { }

private:
    void callback_tim_routine()
    {

        // ==================================================================
        //                        DEBUG MOTOR MAIN 2
        // ==================================================================
        double start_time = this->now().seconds();
        static double last_time = start_time;
        double elapsed_time = start_time - last_time;
        last_time = start_time;

        if (elapsed_time == 0) {
            return;
        }
        // logger.info("Timer routine elapsed time: %.4f seconds", elapsed_time);
        // ==================================================================
        //?==================================================================
        //?                        PROCESSING ROUTINE
        //?==================================================================
        //? ============================================================
        //?                        VELOCITY CONTROL
        //? ============================================================
        // Model orde‑1: τ·dv/dt + v = K·(PWM – PWM_min)
        // Dari identifikasi:
        const float PWM_min_model = 0.0f; // dead‑zone threshold
        float pwm_ff = target_speed_value_ / K_model + PWM_min_model;
        //? ============================================================
        static PID velocity;
        // logger.info("KP: %.4f, KI: %.4f, KD: %.4f, DT: %.4f", k_p_wheel_, k_i_wheel_, k_d_wheel_, elapsed_time);
        velocity.init(k_p_wheel_, k_i_wheel_, k_d_wheel_, elapsed_time, min_wheel_velocity_pwm_, max_wheel_velocity_pwm_, min_wheel_integral_pwm_, max_wheel_integral_pwm_);

        if (!toogle_button_2) {
            velocity.reset();
        }

        float error_speed = target_speed_value_ - encoder_velocity_mps_;

        // const float mps_to_pwm_factor = 1970.035756;
        float pwm_fb = (int)(velocity.calculate(error_speed, min_wheel_velocity_pwm_, max_wheel_velocity_pwm_));
        float pwm_unsat = pwm_ff + pwm_fb;
        target_pwm_wheel_ = std::max(std::min(pwm_unsat, max_wheel_velocity_pwm_), min_wheel_velocity_pwm_);
        // logger.info("Target Speed: %.2f, Linear Velocity: %f, Error Speed: %.2f, Target PWM Wheel: %d | ff: %.2f fb: %.2f", target_speed_value_, encoder_velocity_mps_, error_speed, target_pwm_wheel_, pwm_ff, pwm_fb);

        if (static_cast<int>(tuning) == 1) {
            tuning_pwm2vel();
        } else if (static_cast<int>(tuning) == 2) {
            tuning_pid_ziegler(K_model);
        } else if (static_cast<int>(tuning) == 3) {
            tuning_pid_prbs(K_model);
        }

        // target_pwm_wheel_ = target_speed_value_;
        // target_pwm_wheel_ = 417.5035596; 0.1 m/s
        // logger.info("%.4f, %d, %.8f", start_time, target_pwm_wheel_, encoder_velocity_mps_);
        // logger.info("%.4f, %.4f, %.8f", start_time, target_speed_value_, encoder_velocity_mps_);

        // float yaw_rate = (float)(target_speed_value_ / (0.23 * 0.5) * tanf(target_steering_value_)) / 200;
        // logger.info("speed: %.2f steering: %.2f yaw: %.2f %.2f", target_speed_value_, target_steering_value_, yaw_rate RAD2DEG, yaw_feedback_deg_dot_);

        //? ============================================================
        //?                        PUBLISHING DATA
        //? ============================================================
        std_msgs::msg::Int16 msg_pwm_wheel;
        msg_pwm_wheel.data = target_pwm_wheel_;
        pub_pwm_wheel_->publish(msg_pwm_wheel);

        std_msgs::msg::Int16 msg_pwm_enable;
        msg_pwm_enable.data = 1;
        pub_pwm_enable_->publish(msg_pwm_enable);

        std_msgs::msg::Float32 msg_velocity_output;
        msg_velocity_output.data = encoder_velocity_mps_;
        pub_velocity_feedback_->publish(msg_velocity_output);
    }

    void callback_sub_wheel_encoder(const std_msgs::msg::Int16::SharedPtr msg)
    {
        //? ============================================================
        //?            DELAY TRANSMISSION -> WAIT FOR CANBUS
        //? ============================================================
        static int8_t counter = 200;
        if (counter > 0) {
            counter--;
            return;
        }
        //? ============================================================
        //?                PROCESSING WHEEL ENCODER DATA
        //? ============================================================
        double start_time = this->now().seconds();
        static double last_time = start_time;
        double dt = start_time - last_time;
        last_time = start_time;

        if (dt == 0) {
            return;
        }

        // Process wheel encoder data
        wheel_encoder_value_ = msg->data;
        // logger.info("Wheel Encoder Value: %d", wheel_encoder_value_);

        static int16_t prev_encoder_value = wheel_encoder_value_;
        int16_t encoder_speed = (wheel_encoder_value_ - prev_encoder_value);
        prev_encoder_value = wheel_encoder_value_;

        // logger.info("Wheel Encoder Value: %d, Speed: %d", wheel_encoder_value_, encoder_speed);

        // safety overflow 16 bit
        if (encoder_speed > 32767) {
            encoder_speed = 32767;
        } else if (encoder_speed < -32768) {
            encoder_speed = -32768;
        }

        encoder_position_m_ += encoder_speed * cnt_to_meter_;
        static float prev_encoder_position_m_ = encoder_position_m_;
        encoder_velocity_mps_ = (encoder_position_m_ - prev_encoder_position_m_) / dt;
        prev_encoder_position_m_ = encoder_position_m_;

        // logger.info("%.4f", encoder_velocity_mps_);

        // logger.info("Encoder Position: %.4f m, Velocity: %.4f m/s dt: %.4f", encoder_position_m_, encoder_velocity_mps_, dt);
    }

    void callback_sub_button(const std_msgs::msg::Int8::SharedPtr msg)
    {
        button_1 = (msg->data >> 0) & 0x01;
        button_2 = (msg->data >> 1) & 0x01;
        toogle_button_1 = (msg->data >> 2) & 0x01;
        toogle_button_2 = (msg->data >> 3) & 0x01;
    }

    void callback_sub_wheel_encoder_dot(const std_msgs::msg::Int16::SharedPtr msg)
    {
        // Process wheel encoder dot data
        wheel_encoder_dot_value_ = msg->data;
        // logger.info("Wheel Encoder Dot Value: %d", wheel_encoder_dot_value_);
        linear_velocity_mps_ = (float)wheel_encoder_dot_value_ * cnt_to_meter_;
    }

    void callback_sub_target_speed(const std_msgs::msg::Float32::SharedPtr msg)
    {
        // Process target speed data
        target_speed_value_ = msg->data;
        // logger.info("Target Speed Value: %.2f", target_speed_value_);
    }

    void callback_sub_target_steering(const std_msgs::msg::Float32::SharedPtr msg)
    {
        // Process target steering data
        target_steering_value_ = msg->data;
        // logger.info("Target Steering Value: %.2f", target_steering_value_);
        target_pwm_steering_ = (int16_t)((offset_sudut_steering_rad - target_steering_value_) * steering_rad2dutyCycle);

        std_msgs::msg::Int16 msg_target_steering;
        msg_target_steering.data = target_pwm_steering_;
        pub_pwm_steering_->publish(msg_target_steering);
    }

    void callback_sub_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        static rclcpp::Time last_time = this->now();
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time).seconds();
        last_time = current_time;

        double roll, pitch, yaw;
        tf2::Quaternion q;
        q.setX(msg->orientation.x);
        q.setY(msg->orientation.y);
        q.setZ(msg->orientation.z);
        q.setW(msg->orientation.w);
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        yaw_feedback_deg_ = (float)(yaw RAD2DEG);
        static float last_yaw = (float)yaw;

        yaw_feedback_deg_dot_ = (yaw - last_yaw) / dt;

        if (yaw_feedback_deg_dot_ > 180)
            yaw_feedback_deg_dot_ -= 360;
        else if (yaw_feedback_deg_dot_ < -180)
            yaw_feedback_deg_dot_ += 360;

        last_yaw = (float)yaw;
    }

    void callback_sub_web_config(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        logger.info("Received web configuration data, processing...");

        // Process web configuration data
        if (msg->data.size() >= 1) {
            k_p_wheel_ = msg->data[0];
            k_i_wheel_ = msg->data[1];
            k_d_wheel_ = msg->data[2];
            k_d_steering_ = msg->data[3];
            wheel_radius_ = msg->data[4];
            encoder_ppr_ = msg->data[5];
            cnt_to_meter_ = msg->data[6];
            max_steering_deg_ = msg->data[7];
            min_steering_deg_ = msg->data[8];
            max_steering_pwm_ = msg->data[9];
            min_steering_pwm_ = msg->data[10];
            mid_steering_pwm_ = msg->data[11];
            max_wheel_velocity_pwm_ = msg->data[12];
            min_wheel_velocity_pwm_ = msg->data[13];
            max_wheel_integral_pwm_ = msg->data[14];
            min_wheel_integral_pwm_ = msg->data[15];
            wheel_base_ = msg->data[16];
            tuning = msg->data[17];
            K_model = msg->data[18];

            // Recalculate parameters based on new configuration
            steering_dutyCycle2rad = (min_steering_deg_ DEG2RAD - max_steering_deg_ DEG2RAD) / (min_steering_pwm_ - max_steering_pwm_);
            steering_rad2dutyCycle = 1.0 / steering_dutyCycle2rad;
            offset_sudut_steering_rad = mid_steering_pwm_ / (steering_rad2dutyCycle);
        }

        saveConfig();
    }

    void callback_sub_web_init(const std_msgs::msg::Int16::SharedPtr msg)
    {
        logger.info("Received web initialization request, publishing current configuration...");

        std_msgs::msg::Float32MultiArray msg_pub;
        msg_pub.data = {
            k_p_wheel_,
            k_i_wheel_, //
            k_d_wheel_,
            k_d_steering_,
            wheel_radius_,
            encoder_ppr_,
            cnt_to_meter_,
            max_steering_deg_,
            min_steering_deg_,
            max_steering_pwm_,
            min_steering_pwm_,
            mid_steering_pwm_,
            max_wheel_velocity_pwm_,
            min_wheel_velocity_pwm_,
            max_wheel_integral_pwm_,
            min_wheel_integral_pwm_,
            wheel_base_,
            tuning,
            K_model
        };
        pub_request_web_config_->publish(msg_pub);
    }

    void callback_sub_tuning(const std_msgs::msg::Int16::SharedPtr msg)
    {
        tuning = msg->data;
    }

    void loadConfig()
    {
        logger.info("Loading configuration from YAML file...");

        char config_file[100];
        sprintf(config_file, "/home/iris/Fira/46-49-52-41/dynamic_conf.yaml");

        YAML::Node config = YAML::LoadFile(config_file);
        k_p_wheel_ = config["Motor"]["k_p_wheel"].as<float>();
        k_i_wheel_ = config["Motor"]["k_i_wheel"].as<float>();
        k_d_wheel_ = config["Motor"]["k_p_steering"].as<float>();
        k_d_steering_ = config["Motor"]["k_d_steering"].as<float>();
        wheel_radius_ = config["Motor"]["wheel_radius"].as<float>();
        encoder_ppr_ = config["Motor"]["encoder_ppr"].as<float>();
        cnt_to_meter_ = config["Motor"]["cnt_to_meter"].as<float>();
        max_steering_deg_ = config["Motor"]["max_steering_deg"].as<float>();
        min_steering_deg_ = config["Motor"]["min_steering_deg"].as<float>();
        max_steering_pwm_ = config["Motor"]["max_steering_pwm"].as<float>();
        min_steering_pwm_ = config["Motor"]["min_steering_pwm"].as<float>();
        mid_steering_pwm_ = config["Motor"]["mid_steering_pwm"].as<float>();
        max_wheel_velocity_pwm_ = config["Motor"]["max_wheel_velocity_pwm"].as<float>();
        min_wheel_velocity_pwm_ = config["Motor"]["min_wheel_velocity_pwm"].as<float>();
        max_wheel_integral_pwm_ = config["Motor"]["max_wheel_integral_pwm"].as<float>();
        min_wheel_integral_pwm_ = config["Motor"]["min_wheel_integral_pwm"].as<float>();
        wheel_base_ = config["Motor"]["wheel_base"].as<float>();
        tuning = config["Motor"]["tuning"].as<float>();
        K_model = config["Motor"]["k_model"].as<float>();

        std_msgs::msg::Float32MultiArray msg;
        msg.data = {
            k_p_wheel_,
            k_i_wheel_,
            k_d_wheel_,
            k_d_steering_,
            wheel_radius_,
            encoder_ppr_,
            cnt_to_meter_,
            max_steering_deg_,
            min_steering_deg_,
            max_steering_pwm_,
            min_steering_pwm_,
            mid_steering_pwm_,
            max_wheel_velocity_pwm_,
            min_wheel_velocity_pwm_,
            max_wheel_integral_pwm_,
            min_wheel_integral_pwm_,
            wheel_base_,
            tuning,
            K_model
        };
        pub_request_web_config_->publish(msg);

        steering_dutyCycle2rad = (min_steering_deg_ DEG2RAD - max_steering_deg_ DEG2RAD) / (min_steering_pwm_ - max_steering_pwm_);
        steering_rad2dutyCycle = 1.0 / steering_dutyCycle2rad;
        offset_sudut_steering_rad = mid_steering_pwm_ / (steering_rad2dutyCycle);
        logger.info("steering_dutyCycle2rad: %f %f", steering_dutyCycle2rad, offset_sudut_steering_rad);
    }

    void saveConfig()
    {
        logger.info("Saving configuration to YAML file...");

        char config_file[100];
        sprintf(config_file, "/home/iris/Fira/46-49-52-41/dynamic_conf.yaml");

        YAML::Node config;
        config["Motor"]["k_p_wheel"] = k_p_wheel_;
        config["Motor"]["k_i_wheel"] = k_i_wheel_;
        config["Motor"]["k_p_steering"] = k_d_wheel_;
        config["Motor"]["k_d_steering"] = k_d_steering_;
        config["Motor"]["wheel_radius"] = wheel_radius_;
        config["Motor"]["encoder_ppr"] = encoder_ppr_;
        config["Motor"]["cnt_to_meter"] = cnt_to_meter_;
        config["Motor"]["max_steering_deg"] = max_steering_deg_;
        config["Motor"]["min_steering_deg"] = min_steering_deg_;
        config["Motor"]["max_steering_pwm"] = max_steering_pwm_;
        config["Motor"]["min_steering_pwm"] = min_steering_pwm_;
        config["Motor"]["mid_steering_pwm"] = mid_steering_pwm_;
        config["Motor"]["max_wheel_velocity_pwm"] = max_wheel_velocity_pwm_;
        config["Motor"]["min_wheel_velocity_pwm"] = min_wheel_velocity_pwm_;
        config["Motor"]["max_wheel_integral_pwm"] = max_wheel_integral_pwm_;
        config["Motor"]["min_wheel_integral_pwm"] = min_wheel_integral_pwm_;
        config["Motor"]["wheel_base"] = wheel_base_;
        config["Motor"]["tuning"] = tuning;
        config["Motor"]["k_model"] = K_model;

        std::ofstream fout(config_file);
        fout << config;
        fout.close();
    }

    int8_t tuning_pwm2vel()
    {
        const uint16_t sequence_ms[3] = { 500, 1000, 500 };
        const uint16_t sequence_pwm[8] = { 1000, 1500, 2000, 2500, 3000, 3500, 4000, 6000 };
        static uint8_t pwm_index = 0;
        static uint8_t sequence_index = 0;

        static std::vector<float> gradient_pwm_mps = { 0 };
        static std::vector<float> current_pwm_mps = { 0 };

        static uint8_t finished = 0;
        static double last_time = this->now().seconds();

        if (this->now().seconds() - last_time > 10.0) {
            pwm_index = 0;
            sequence_index = 0;
            gradient_pwm_mps.clear();
            current_pwm_mps.clear();
            finished = 0;
            logger.info("Resetting PWM to Velocity tuning.");
        }

        if (pwm_index >= 8) {
            if (finished == 0) {
                float average_gradient_total = 0.0;
                for (int i = 0; i < gradient_pwm_mps.size(); i++) {
                    average_gradient_total += gradient_pwm_mps[i];
                }
                average_gradient_total /= gradient_pwm_mps.size();
                average_gradient_total;
                logger.info("Average Gradient Total: %.8f", average_gradient_total);
                logger.info("Tuning PWM to Velocity finished.");
                finished = 1;
            }
            return 0;
        }

        if (sequence_index == 0) {
            current_pwm_mps.clear();
            if (pwm_index == 0) {
                gradient_pwm_mps.clear();
            }
            target_pwm_wheel_ = sequence_pwm[pwm_index];
            if (counter(sequence_ms[sequence_index])) {
                sequence_index++;
            }
        } else if (sequence_index == 1) {
            target_pwm_wheel_ = sequence_pwm[pwm_index];
            current_pwm_mps.push_back((float)encoder_velocity_mps_);
            if (counter(sequence_ms[sequence_index])) {
                sequence_index++;
            }
        } else if (sequence_index == 2) {
            target_pwm_wheel_ = sequence_pwm[pwm_index];

            float average_velocity = 0.0;
            for (int i = 0; i < current_pwm_mps.size(); i++) {
                average_velocity += current_pwm_mps[i];
            }

            average_velocity /= current_pwm_mps.size();
            float gradient = average_velocity / (float)sequence_pwm[pwm_index];
            gradient_pwm_mps.push_back(gradient);
            logger.info("PWM: %d, Average Velocity: %.2f m/s, Gradient: %.8f", target_pwm_wheel_, average_velocity, gradient);

            sequence_index = 0;
            pwm_index++;
        }

        last_time = this->now().seconds();

        return 1;
    }

    int8_t tuning_pid_ziegler(float K_Model)
    {
        const uint16_t sequence_ms[3] = { 500, 2000, 500 };
        const float sequence_speed[1] = { 1.5 };
        static uint8_t speed_index = 0;
        static uint8_t sequence_index = 0;

        static uint8_t finished = 0;
        static double last_time = this->now().seconds();

        if (this->now().seconds() - last_time > 1000.0) {
            speed_index = 0;
            sequence_index = 0;
            finished = 0;
            logger.info("Resetting PID Ziegler tuning.");
        }

        if (speed_index >= 1) {
            if (finished == 0) {
                finished = 1;
                logger.info("PID Ziegler tuning finished.");
            }
            return 0;
        }

        if (sequence_index == 0) {
            target_pwm_wheel_ = 0;
            if (counter(sequence_ms[sequence_index])) {
                sequence_index++;
            }
        } else if (sequence_index == 1) {
            target_pwm_wheel_ = sequence_speed[speed_index] / K_Model;
            if (counter(sequence_ms[sequence_index])) {
                sequence_index++;
            }
        } else if (sequence_index == 2) {
            target_pwm_wheel_ = 0;
            if (counter(sequence_ms[sequence_index])) {
                sequence_index = 0;
                speed_index++;
            }
        }

        logger.info(";%.4f; %.2f; %.2f", this->now().seconds(), sequence_speed[speed_index], encoder_velocity_mps_);

        last_time = this->now().seconds();

        return 1;
    }

    // Pseudorandom Binary Sequence (PRBS) input for system identification
    int8_t tuning_pid_prbs(float K_Model)
    {
        // PRBS parameters
        static const uint16_t step_duration_ms = 500; // Duration of each step
        static const int prbs_levels[2] = { 0, 1 }; // Binary levels: 0 or 1
        static uint8_t prbs_state = 0; // Current PRBS state (0 or 1)
        static uint32_t prbs_seed = 0xACE1u; // LFSR seed for PRBS
        static uint8_t finished = 0;
        static double last_time = this->now().seconds();

        // Reset if too much time has passed
        if (this->now().seconds() - last_time > 1000.0) {
            prbs_state = 0;
            prbs_seed = 0xACE1u;
            finished = 0;
            logger.info("Resetting PRBS tuning.");
        }

        // Generate next PRBS state using a simple 16-bit LFSR
        auto next_prbs_bit = [&]() {
            unsigned lsb = prbs_seed & 1;
            prbs_seed >>= 1;
            if (lsb)
                prbs_seed ^= 0xB400u;
            return lsb;
        };

        // Only run for a fixed number of steps (for example, 100 steps)
        static uint16_t step_count = 0;
        const uint16_t max_steps = 100;
        if (step_count >= max_steps) {
            if (!finished) {
                finished = 1;
                logger.info("PRBS tuning finished.");
            }
            return 0;
        }

        // Step logic
        static int current_level = 0;
        if (counter(step_duration_ms)) {
            prbs_state = next_prbs_bit();
            current_level = prbs_levels[prbs_state];
            step_count++;
        }

        // Map PRBS level to speed (e.g., 0 m/s or 1.5 m/s)
        float prbs_speed = current_level * 1.5f;
        target_pwm_wheel_ = prbs_speed / K_Model;

        logger.info(";%.4f; %d; %.2f; %.2f", this->now().seconds(), current_level, prbs_speed, encoder_velocity_mps_);

        last_time = this->now().seconds();

        return 1;
    }

    int8_t counter(int ms)
    {
        const uint16_t ticks_per_second = 200;
        uint16_t ticks = ms * ticks_per_second / 1000;
        static uint16_t counter = 0;

        if (counter < ticks) {
            counter++;
            return 0;
        } else {
            counter = 0;
            return 1;
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<MotorMain2> node_MotorMain2;

    try {
        node_MotorMain2 = std::make_shared<MotorMain2>();

        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node_MotorMain2);
        executor.spin();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("motor_main2"), "Failed to create MotorMain2 node: %s", e.what());
        rclcpp::shutdown();
    }

    if (node_MotorMain2) {
        node_MotorMain2.reset();
    }

    rclcpp::shutdown();
    return 0;
}
