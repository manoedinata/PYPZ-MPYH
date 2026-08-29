// C++ Standard Library
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

// C Standard Library
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

// Network Libraries
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>

// Linux CAN Libraries
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>

// Third-party Libraries

// ROS 2 Libraries
#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/global_definitions.hpp"
#include "ros2_utils/help_logger.hpp"
#include "ros2_utils/pid.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

// ROS 2 Messages
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int8.hpp"

#define DEG2RAD *0.01745329251994
#define RAD2DEG *57.29577951308232

#define PC_RECV_PORT 5000
#define STM_RECV_PORT 5001
#define RECV_UDP
#define SEND_UDP

using namespace std;

// Helper function to constrain a value between min and max
template <typename T>
T constrain(T value, T min_val, T max_val)
{
    return std::max(min_val, std::min(value, max_val));
}

class STM_UDP : public rclcpp::Node
{
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
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_pwm_wheel_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_pwm_enable_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_pwm_steering_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_buzzer_urban_;
    // -------------------------------------------------
    // Publishers
    // -------------------------------------------------
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_wheel_encoder_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_button_;
    // -------------------------------------------------
    // Timer
    // -------------------------------------------------
    rclcpp::TimerBase::SharedPtr timer_send_;
    rclcpp::TimerBase::SharedPtr timer_recv_;
    // -------------------------------------------------
    // Global Variables
    // -------------------------------------------------
    //--UDP Connection
    /**
     * @note socket - return a file descriptor for the pc new socket
     * @note bind - give socket FD the local address ADDR (which is LEN bytes long)
     */
    int socket_;
    struct sockaddr_in server_;                  // STM Address (Need to configure to send)
    struct sockaddr_in any_addr_;                // Any Address sending to this PC
    socklen_t any_addr_len_ = sizeof(any_addr_); // Lenght of Any Address
    struct sockaddr_in client_;                  // PC Address (Need to configure to recv)

    //--File Descriptor
    fd_set readfds;

    char recv_buffer[64];
    char send_buffer[64] = "its";

    int8_t buzzer = 0;
    int8_t button_1 = 0;
    int8_t button_2 = 0;

    int8_t buzzer_from_urban = 0;
    int8_t cnt_buzzer_urban = 0;

    uint8_t target_enable = 0;
    int16_t target_pwm_wheel = 0;
    int16_t target_steering = 1000; // Derajat target = target_steering * 0.01
                                    // -------------------------------------------------
                                    // Parameters
                                    // -------------------------------------------------

  public:
    STM_UDP()
        : Node("stm_udp")
    {
        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }
        // --------------------------------------------------
        // Get parameters
        // --------------------------------------------------
        // this->declare_parameter<float>("wheel_base", 0.22);
        // this->get_parameter("wheel_base", wheel_base_);
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
        sub_pwm_wheel_ = this->create_subscription<std_msgs::msg::Int16>(
            "/motor_main/target_pwm_wheel", 1,
            std::bind(&STM_UDP::callback_pwm_wheel, this, std::placeholders::_1),
            sub_options);
        sub_pwm_enable_ = this->create_subscription<std_msgs::msg::Int16>(
            "/motor_main/target_pwm_enable", 1,
            std::bind(&STM_UDP::callback_pwm_enable, this, std::placeholders::_1),
            sub_options);
        sub_pwm_steering_ = this->create_subscription<std_msgs::msg::Int16>(
            "/motor_main/target_steering", 1,
            std::bind(&STM_UDP::callback_target_steering, this, std::placeholders::_1),
            sub_options);
        sub_buzzer_urban_ = this->create_subscription<std_msgs::msg::Int8>(
            "/master/sign_buzzer", 1,
            std::bind(&STM_UDP::callback_buzzer_urban, this, std::placeholders::_1),
            sub_options);
        // --------------------------------------------------
        // Create publishers
        // --------------------------------------------------
        pub_wheel_encoder_ = this->create_publisher<std_msgs::msg::Int16>(
            "/hardware/wheel_encoder", 1);
        pub_button_ = this->create_publisher<std_msgs::msg::Int8>(
            "/hardware/button", 1);
        // --------------------------------------------------
        // Create timer
        // --------------------------------------------------
        timer_send_ = this->create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&STM_UDP::callback_tim_send, this),
            tim_routine_group_);
        timer_recv_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&STM_UDP::callback_tim_recv, this),
            tim_routine_group_);

        init_udp();

        logger.info("STM_UDP node initialized with multithreading");
    }

    ~STM_UDP() {}

  private:
    void callback_tim_send()
    {
        // logger.info("Sending data over UDP");
        send_udp();
    }

    void callback_tim_recv()
    {
        // logger.info("Receiving data over UDP");
        recv_udp();
    }

    void callback_pwm_wheel(const std_msgs::msg::Int16::SharedPtr msg)
    {
        target_pwm_wheel = msg->data;
        // logger.info("Received target PWM wheel: %d", target_pwm_wheel);
    }

    void callback_pwm_enable(const std_msgs::msg::Int16::SharedPtr msg)
    {
        target_enable = (uint8_t)msg->data;
    }

    void callback_target_steering(const std_msgs::msg::Int16::SharedPtr msg)
    {
        target_steering = msg->data;
        // logger.info("Received target steering: %d", target_steering);
    }

    void callback_buzzer_urban(const std_msgs::msg::Int8::SharedPtr msg)
    {
        buzzer_from_urban = 1;
        cnt_buzzer_urban = msg->data;

        logger.info("Buzzer Urban: %d", cnt_buzzer_urban);
    }

    void init_udp()
    {
        socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

#ifdef SEND_UDP
        // Clean data and set STM configuration
        bzero(&server_, sizeof(server_));
        server_.sin_family = AF_INET;
        server_.sin_port = htons(STM_RECV_PORT);
        server_.sin_addr.s_addr = inet_addr("192.168.3.100");
#endif

#ifdef RECV_UDP
        // Clean data and set PC configuration
        bzero(&client_, sizeof(client_));
        client_.sin_family = AF_INET;
        client_.sin_port = htons(PC_RECV_PORT);
        client_.sin_addr.s_addr = INADDR_ANY;

        // set udp timeout
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 2000;
        setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof tv);

        // Bind socket and checking is binding success
        bind(socket_, (struct sockaddr *)&client_, sizeof(client_));
        if (socket_ < 0)
            logger.error("STM UDP ERROR : Binding Failed");

#endif
    }

    void send_udp()
    {
        /*New Data from STM*/
        //=================================================================
        // target_pwm_wheel = -1000;
        static uint8_t toogle_button_1 = 0;
        static uint8_t toogle_button_2 = 0;
        static int8_t init = 1;
        // enable_warning();

        toogle_button_2 = falling_edge(button_2);

        if (toogle_button_2)
        {
            init = 1;
            target_enable = 1;
            // ringtone_1();
        }
        else if (!init)
        {
            target_enable = 1;
        }
        else
        {
            target_enable = 0;
            target_pwm_wheel = 0;
            target_steering = 1000;
        }

        if (buzzer_from_urban)
        {
            ringtone_cnt(cnt_buzzer_urban + 1);
            buzzer_from_urban = 0;
        }
        else
        {
        }

        if (buzzer != 1)
        {
            if (button_1)
                buzzer = 1;
            else
                buzzer = 0;
        }

        static int counter_buzzer = 0;
        if (buzzer)
            counter_buzzer++;
        else
            counter_buzzer = 0;

        if (counter_buzzer > 100)
        {
            buzzer = 0;
            counter_buzzer = 0;
        }

        logger.info("buzzer: %d, cnt_buzzer: %d", buzzer, counter_buzzer);

        // logger.info("enable: %d", target_enable);
        memcpy(send_buffer + 3, &target_enable, sizeof(target_enable));
        memcpy(send_buffer + 4, &target_pwm_wheel, sizeof(target_pwm_wheel));
        memcpy(send_buffer + 6, &target_steering, sizeof(target_steering));
        memcpy(send_buffer + 8, &buzzer, sizeof(buzzer));
        //=================================================================
        // logger.info("STM UDP Sending : Target PWM Wheel: %d, Target Steering: %d, Buzzer: %d", target_pwm_wheel, target_steering, buzzer);
        sendto(socket_, send_buffer, 9, 0, (struct sockaddr *)&server_, sizeof(server_));

        std_msgs::msg::Int8 msg_button;
        int8_t button = 0;
        button = (button & 0b111111110) | (button_1 << 0);
        button = (button & 0b111111101) | (button_2 << 1);
        button = (button & 0b111111011) | (toogle_button_1 << 2);
        button = (button & 0b111110111) | (toogle_button_2 << 3);

        // for (int i = 0; i < 8; i++) {
        //     logger.info("Button %d: %d", i + 1, (button >> i) & 0x01);
        // }

        msg_button.data = button;
        pub_button_->publish(msg_button);
    }

    int8_t rising_edge(int8_t button)
    {
        static int8_t prev_button = 0;
        static int8_t rising_edge = 0;

        if (button == 1 && prev_button == 0)
        {
            rising_edge = !rising_edge;
            if (buzzer)
                buzzer = 0;
        }

        prev_button = button;
        return rising_edge;
    }

    int8_t falling_edge(int8_t button)
    {
        static int8_t prev_button = 0;
        static int8_t falling_edge = 0;

        if (button == 0 && prev_button == 1)
        {
            falling_edge = !falling_edge;
            if (buzzer)
                buzzer = 0;
        }

        prev_button = button;
        return falling_edge;
    }

    void ringtone_1()
    {
        static uint16_t counter = 0;
        static uint8_t state = 0;

        counter++;

        switch (state)
        {
        case 0: // Bip 1
            if (counter == 1)
                buzzer = 1;
            if (counter == 10)
            {
                buzzer = 0;
                state = 1;
                counter = 0;
            }
            break;

        case 1: // Bip 2
            if (counter == 10)
                buzzer = 1;
            if (counter == 20)
            {
                buzzer = 0;
                state = 2;
                counter = 0;
            }
            break;

        case 2: // Delay setelah 2 bip
            if (counter >= 1000)
            {
                state = 0;
                counter = 0;
            }
            break;
        }
    }

    void ringtone_cnt(int8_t cnt)
    {
        static uint16_t counter = 0;
        static uint8_t state = 0;
        static int8_t bip_count = 0;

        counter++;

        switch (state)
        {
        case 0: // Start bip
            if (counter == 1)
                buzzer = 1;
            if (counter == 10)
            {
                buzzer = 0;
                bip_count++;
                counter = 0;
                if (bip_count < cnt)
                    state = 1; // Delay between bips
                else
                    state = 2; // All bips done, long delay
            }
            break;

        case 1: // Delay between bips
            if (counter == 10)
            {
                state = 0;
                counter = 0;
            }
            break;

        case 2: // Delay after all bips
            if (counter >= 100)
            {
                state = 0;
                counter = 0;
                bip_count = 0;
            }
            break;
        }
    }

    void enable_warning()
    {
        static uint16_t counter = 0;
        static uint8_t toogle = 0;
        if (!target_enable)
        {
            counter++;
            if (counter > 400)
            {
                toogle = !toogle;

                if (toogle)
                    counter = 200;
                else
                    counter = 0;

                buzzer = !buzzer;
            }
        }
    }

    void recv_udp()
    {
        int8_t recv_len = recvfrom(socket_, recv_buffer, 64, 0, (struct sockaddr *)&any_addr_, &any_addr_len_);
        if (recv_len == -1)
        {
            if (errno != EAGAIN && errno != EWOULDBLOCK)
                logger.error("errno : %d, error : %s", errno, strerror(errno));
            return;
        }

        int16_t encoder = 0;

        if (recv_len > 0 && recv_buffer[0] == 'i' && recv_buffer[1] == 't' && recv_buffer[2] == 's')
        {
            memcpy(&encoder, recv_buffer + 3, sizeof(encoder));
            memcpy(&button_1, recv_buffer + 5, sizeof(button_1));
            memcpy(&button_2, recv_buffer + 6, sizeof(button_2));

            // logger.info("STM UDP Received : Encoder: %d, Button 1: %d, Button 2: %d", encoder, button_1, button_2);
        }
        else
        {
            logger.info("STM UDP Received : Invalid Data");
        }

        // Publish wheel encoder data
        std_msgs::msg::Int16 msg_wheel_encoder;
        msg_wheel_encoder.data = encoder;
        pub_wheel_encoder_->publish(msg_wheel_encoder);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<STM_UDP> node_STM_UDP;

    try
    {
        node_STM_UDP = std::make_shared<STM_UDP>();

        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node_STM_UDP);
        executor.spin();
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("stm_udp"), "Failed to create STM_UDP node: %s", e.what());
        rclcpp::shutdown();
    }

    if (node_STM_UDP)
        node_STM_UDP.reset();

    rclcpp::shutdown();
    return 0;
}
