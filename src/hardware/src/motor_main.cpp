#include "math.h"
#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/global_definitions.hpp"
#include "ros2_utils/help_logger.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#define DEG2RAD *0.01745329251994
#define RAD2DEG *57.29577951308232

class MotorMain : public rclcpp::Node {
public:
    rclcpp::TimerBase::SharedPtr tim_routine;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_pwm_wheel;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_target_steering;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_feedback_steering_rad;

    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_wheel_encoder;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_delta_encoder_wheel_counter;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_feedback_steering;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_target_speed;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_target_steering;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_keyboard;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;

    rclcpp::Clock::SharedPtr clock;

    HelpLogger logger;

    // Configs
    // =======================================================
    float k_p_wheel = 0.0;
    float k_i_wheel = 0.0;
    int routine_period_ms = 20; // ms
    float wheel_radius = 0.0; // m
    float encoder_ppr = 0.0;
    float steering_dutyCycle2rad = 0.0; // Derajat per duty cycle
    float offset_sudut_steering_rad = 0.0;
    float encoder_to_meter = 0.0;

    int16_t target_pwm_wheel = 0;
    int16_t target_steering = 0; // Derajat target = target_steering * 0.01
    int16_t encoder_wheel_counter = 0;
    int16_t delta_encoder_wheel_counter = 0;
    int16_t feedback_steering = 0; // Derajat feedback = feedback_steering * 0.01
    float target_speed = 0.0;
    float target_steering_rad = 0.0; // Derajat target steering
    float steering_rad2dutyCycle = 0.0; // Derajat per duty cycle
    float feedback_steering_rad = 0.0;
    float max_steering_pwm = 900;
    float min_steering_pwm = 800;
    float mid_steering_pwm = 850;
    float max_steering_pwm_rad = 0.57;
    float min_steering_pwm_rad = -0.57;

    double yaw_diff_ = 0.0; // rad/s

    int cntr_enc = 0;

    float output_steering_ = 0;

    // Data kalkulasi
    rclcpp::Time last_encoder_time;

    MotorMain()
        : Node("MotorMain")
    {
        if (!logger.init()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }

        //----Parameters
        this->declare_parameter<float>("k_p_wheel", 0.0);
        this->get_parameter("k_p_wheel", k_p_wheel);

        this->declare_parameter<float>("k_i_wheel", 0.0);
        this->get_parameter("k_i_wheel", k_i_wheel);

        this->declare_parameter<float>("wheel_radius", 0.0);
        this->get_parameter("wheel_radius", wheel_radius);

        this->declare_parameter<float>("encoder_ppr", 0.0);
        this->get_parameter("encoder_ppr", encoder_ppr);

        this->declare_parameter<int>("routine_period_ms", 20);
        this->get_parameter("routine_period_ms", routine_period_ms);

        this->declare_parameter<float>("steering_dutyCycle2rad", __FLT_EPSILON__);
        this->get_parameter("steering_dutyCycle2rad", steering_dutyCycle2rad);

        this->declare_parameter<float>("offset_sudut_steering_rad", 0.0);
        this->get_parameter("offset_sudut_steering_rad", offset_sudut_steering_rad);

        this->declare_parameter<float>("encoder_to_meter", 0.0);
        this->get_parameter("encoder_to_meter", encoder_to_meter);

        this->declare_parameter<float>("max_steering_pwm", 900.0);
        this->get_parameter("max_steering_pwm", max_steering_pwm);

        this->declare_parameter<float>("min_steering_pwm", 800.0);
        this->get_parameter("min_steering_pwm", min_steering_pwm);

        this->declare_parameter<float>("max_steering_pwm_rad", 0.57);
        this->get_parameter("max_steering_pwm_rad", max_steering_pwm_rad);

        this->declare_parameter<float>("min_steering_pwm_rad", -0.57);
        this->get_parameter("min_steering_pwm_rad", min_steering_pwm_rad);

        this->declare_parameter<float>("mid_steering_pwm", 850.0);
        this->get_parameter("mid_steering_pwm", mid_steering_pwm);

        steering_dutyCycle2rad = (min_steering_pwm_rad - max_steering_pwm_rad) / (min_steering_pwm - max_steering_pwm);

        steering_rad2dutyCycle = 1.0 / steering_dutyCycle2rad;
        offset_sudut_steering_rad = mid_steering_pwm / (steering_rad2dutyCycle);
        logger.info("steering_dutyCycle2rad: %f %f", steering_dutyCycle2rad, offset_sudut_steering_rad);

        //----Publishers
        pub_pwm_wheel = this->create_publisher<std_msgs::msg::Int16>("/motor_main/target_pwm_wheel", 1);
        pub_target_steering = this->create_publisher<std_msgs::msg::Int16>("/motor_main/target_steering", 1);
        pub_feedback_steering_rad = this->create_publisher<std_msgs::msg::Float32>("/motor_main/feedback_steering_rad", 1);

        //----Subscriptions
        sub_wheel_encoder = this->create_subscription<std_msgs::msg::Int16>(
            "/hardware/wheel_encoder", 1, std::bind(&MotorMain::callback_sub_wheel_encoder, this, std::placeholders::_1));
        sub_delta_encoder_wheel_counter = this->create_subscription<std_msgs::msg::Int16>(
            "/can/delta_encoder_wheel_counter", 1, std::bind(&MotorMain::callback_sub_delta_encoder_wheel_counter, this, std::placeholders::_1));
        sub_feedback_steering = this->create_subscription<std_msgs::msg::Int16>(
            "/can/feedback_steering", 1, std::bind(&MotorMain::callback_sub_feedback_steering, this, std::placeholders::_1));
        sub_target_speed = this->create_subscription<std_msgs::msg::Float32>(
            "/master/target_speed", 1, std::bind(&MotorMain::callback_sub_target_speed, this, std::placeholders::_1));
        sub_target_steering = this->create_subscription<std_msgs::msg::Float32>(
            "/master/target_steering", 1, std::bind(&MotorMain::callback_sub_target_steering, this, std::placeholders::_1));
        // sub_keyboard = this->create_subscription<std_msgs::msg::Int16>(
        //     "/key_pressed", 1, std::bind(&MotorMain::callback_sub_keyboard, this, std::placeholders::_1));
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/hardware/imu", 1, std::bind(&MotorMain::callback_sub_imu, this, std::placeholders::_1));

        //----Timers
        tim_routine = this->create_wall_timer(std::chrono::milliseconds(routine_period_ms), std::bind(&MotorMain::callback_routine, this));

        // Initialize clock
        clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

        logger.info("MotorMain node initialized");
    }

    void callback_sub_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // from quaternion to euler angles
        double roll, pitch, yaw;
        tf2::Quaternion q;
        q.setX(msg->orientation.x);
        q.setY(msg->orientation.y);
        q.setZ(msg->orientation.z);
        q.setW(msg->orientation.w);
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        static double last_yaw = yaw;

        yaw_diff_ = (yaw - last_yaw); // Assuming callback is called every 20ms
        if (yaw_diff_ > M_PI) {
            yaw_diff_ -= 2 * M_PI;
        } else if (yaw_diff_ < -M_PI) {
            yaw_diff_ += 2 * M_PI;
        }
        last_yaw = yaw;
    }

    void callback_sub_keyboard(const std_msgs::msg::Int16::SharedPtr msg)
    {
        switch (msg->data) {
        case '1':
            target_speed = 0.2;
            break;
        case '2':
            target_speed = 0.4;
            break;
        case '3':
            target_speed = 0.8;
            break;
        case '4':
            target_speed = 1;
            break;
        case '5':
            target_speed = 1.4;
            break;
        case ' ':
            target_speed = 0;
            target_steering_rad = 0;
            cntr_enc = 0;
            break;
        case 'b':
            target_steering_rad += 0.01;
            break;
        case 'n':
            target_steering_rad = 0.0;
            break;
        case 'm':
            target_steering_rad -= 0.01;
            break;

        case '*':
            target_steering -= 1;
            logger.info("target_steering: %d", target_steering);

            break;
        case '(':
            target_steering = 38;
            logger.info("target_steering: %d", target_steering);

            break;
        case ')':
            target_steering += 1;
            logger.info("target_steering: %d", target_steering);
            break;

        // Untuk offset steering
        case '7':
            target_steering_rad = 0;
            break;
        // case '8':
        //     target_steering_rad = feedback_steering_rad + 0.001;
        //     break;
        // case '9':
        //     target_steering_rad = feedback_steering_rad;
        //     break;
        // case '0':
        //     target_steering_rad = feedback_steering_rad - 0.001;
        //     break;
        case '-':
            logger.info("Feedback steering: %f", feedback_steering_rad);
            offset_sudut_steering_rad = feedback_steering_rad;
            break;
        }
    }

    void callback_sub_wheel_encoder(const std_msgs::msg::Int16::SharedPtr msg)
    {
        // Process wheel encoder data
        encoder_wheel_counter = msg->data;
    }

    void callback_sub_delta_encoder_wheel_counter(const std_msgs::msg::Int16::SharedPtr msg)
    {
        // Process delta encoder wheel counter data
        delta_encoder_wheel_counter = msg->data;
    }

    void callback_sub_feedback_steering(const std_msgs::msg::Int16::SharedPtr msg)
    {
        // Process feedback steering data
        feedback_steering = msg->data;

        feedback_steering_rad = feedback_steering * steering_dutyCycle2rad - offset_sudut_steering_rad;
    }

    void callback_sub_target_speed(const std_msgs::msg::Float32::SharedPtr msg)
    {
        // Process target speed data
        target_speed = msg->data;
    }

    void callback_sub_target_steering(const std_msgs::msg::Float32::SharedPtr msg)
    {
        target_steering_rad = msg->data;
    }

    ~MotorMain()
    {
    }

    void callback_routine()
    {

        control_velocity();
        control_steering();
        transmit_all();
    }

    void control_velocity()
    {
        // make ros2 time
        static float integral;
        rclcpp::Time time_now = clock->now();

        float dt = time_now.seconds() - last_encoder_time.seconds();
        float L = 0.22f;
        if (dt < 0.001)
            return;

        // float revolution = (float)delta_encoder_wheel_counter / encoder_ppr;
        // float angular_velocity = (2.0 * M_PI * revolution) / dt;
        // float linear_velocity = angular_velocity * wheel_radius;

        // logger.info("target_speed : %.2f linier: %.2f angular: %.2f", target_speed, linear_velocity, angular_velocity);

        // logger.info("wheel radius: %.2f, encoder_ppr: %.2f, dt: %.2f", wheel_radius, encoder_ppr, dt);

        // float enc_per_sec = target_speed / encoder_to_meter; // encoder per second
        // enc_per_sec = enc_per_sec * dt;                      // encoder per second
        // float linear_velocity = enc_per_sec;

        // float d_enc_per_sec = (float)delta_encoder_wheel_counter * 100;
        // float encoder_to_meter_sec = encoder_to_meter * 50;

        float linear_velocity = (float)delta_encoder_wheel_counter * encoder_to_meter * 0.02;
        float yaw_rate = (float)(linear_velocity / 0.22 * tanf(target_steering_rad));
        float velocity_mps = linear_velocity / 0.02;

        if (abs(velocity_mps) > 0.01) {
            float feedback_servo_rad = atan2(yaw_diff_ * 0.22, velocity_mps);
            if (feedback_servo_rad > M_PI)
                feedback_servo_rad -= 2 * M_PI;
            else if (feedback_servo_rad < -M_PI)
                feedback_servo_rad += 2 * M_PI;

            if (linear_velocity < 0)
                feedback_servo_rad = -feedback_servo_rad;

            float error_steering = target_steering_rad - feedback_servo_rad;
            if (error_steering > M_PI)
                error_steering -= 2 * M_PI;
            else if (error_steering < -M_PI)
                error_steering += 2 * M_PI;

            static const float k_p_steering = 1.0;
            static const float k_i_steering = 0.0;
            static const float k_d_steering = 0.1;

            float proportional_steering = k_p_steering * error_steering;
            static float integral_steering = 0;
            static float prev_error_steering = 0;
            integral_steering += error_steering * dt * k_i_steering;
            float derivative_steering = k_d_steering * (error_steering - prev_error_steering) / dt;

            if (integral_steering > 300 DEG2RAD)
                integral_steering = 300 DEG2RAD;
            else if (integral_steering < -300 DEG2RAD)
                integral_steering = -300 DEG2RAD;

            output_steering_ = proportional_steering + integral_steering + derivative_steering;
            prev_error_steering = error_steering;

            if (output_steering_ > 500 DEG2RAD)
                output_steering_ = 500 DEG2RAD;
            else if (output_steering_ < -500 DEG2RAD)
                output_steering_ = -500 DEG2RAD;

            output_steering_ *= 0.02;

            logger.info("target_steering_rad: %.2f, feedback_servo_rad: %.2f, output_steering_rad: %.2f, yaw_diff: %.2f",
                target_steering_rad RAD2DEG, feedback_servo_rad RAD2DEG, output_steering_ RAD2DEG, yaw_diff_ RAD2DEG);
        } else {
            output_steering_ = target_steering_rad;
        }

        // PID
        float error = target_speed - linear_velocity;
        integral += error * dt;

        if (integral > 9999 * 0.25)
            integral = 9999 * 0.25;
        else if (integral < -9999 * 0.25)
            integral = -9999 * 0.25;

        float output = k_p_wheel * error + k_i_wheel * integral;

        target_pwm_wheel = (int16_t)output;

        if (target_pwm_wheel > 8000)
            target_pwm_wheel = 8000;
        else if (target_pwm_wheel < -8000)
            target_pwm_wheel = -8000;

        last_encoder_time = time_now;
    }

    void control_steering()
    {
        target_steering = (offset_sudut_steering_rad - output_steering_) * steering_rad2dutyCycle;
        // logger.info("target_steering: %d %.2f %f", target_steering, target_steering_rad, feedback_steering_rad);
    }

    void transmit_all()
    {
        std_msgs::msg::Int16 msg_pwm_wheel;
        msg_pwm_wheel.data = target_pwm_wheel; // Replace with actual value
        pub_pwm_wheel->publish(msg_pwm_wheel);

        std_msgs::msg::Int16 msg_target_steering;
        msg_target_steering.data = target_steering; // Replace with actual value
        pub_target_steering->publish(msg_target_steering);

        std_msgs::msg::Float32 msg_feedback_steering_rad;
        msg_feedback_steering_rad.data = feedback_steering_rad; // Replace with actual value
        pub_feedback_steering_rad->publish(msg_feedback_steering_rad);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node_MotorMain = std::make_shared<MotorMain>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_MotorMain);
    executor.spin();

    return 0;
}