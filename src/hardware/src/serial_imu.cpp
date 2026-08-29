#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/global_definitions.hpp"
#include "ros2_utils/help_logger.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"

//--Linux Headers
#include <errno.h>     // Error integer and strerror() function
#include <fcntl.h>     // Contains file controls like O_RDWR
#include <sys/ioctl.h> // ioctl()
#include <termios.h>   // Contains POSIX terminal control definitions
#include <unistd.h>    // write(), read(), close()

#include "boost/asio.hpp"

#define READ_PROTOCOL 0x55
#define WRITE_PROTOCOL 0xFF

#define TYPE_TIME 0x50
#define TYPE_ACC 0x51
#define TYPE_ANGULAR_VELOCITY 0x52
#define TYPE_ANGLE 0x53
#define TYPE_MAGNETIC 0x54
#define TYPE_PORT 0x55
#define TYPE_BAROMETRIC 0x56
#define TYPE_LAT_LONG 0x57
#define TYPE_GROUND_SPEED 0x58
#define TYPE_QUATERNION 0x59
#define TYPE_GPS_ACCURACY 0x5A
#define TYPE_READ 0x5F

#define GRAVITY 9.8

class SerialIMU : public rclcpp::Node
{
  private:
    boost::asio::io_service io_service_;
    boost::asio::serial_port serial_port;
    char recv_buffer[256];
    int16_t recv_len;

  public:
    rclcpp::TimerBase::SharedPtr tim_50hz;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;

    // Configs
    // =======================================================
    std::string port;
    std::string frame_id = "imu";
    bool use_boost = false;
    bool is_riontech = false;
    int baudrate = 115200;

    HelpLogger logger;

    // Vars
    // =======================================================
    int serial_port_fd;

    sensor_msgs::msg::Imu imu_msg;

    SerialIMU()
        : Node("serial_imu"), io_service_(), serial_port(io_service_)
    {
        this->declare_parameter("port", "/dev/ttyUSB0");
        this->get_parameter("port", port);

        this->declare_parameter("use_boost", false);
        this->get_parameter("use_boost", use_boost);

        this->declare_parameter("is_riontech", false);
        this->get_parameter("is_riontech", is_riontech);

        this->declare_parameter("baudrate", 115200);
        this->get_parameter("baudrate", baudrate);

        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }

        if (!use_boost)
        {
            if (init_serial() > 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial");
                rclcpp::shutdown();
            }
        }
        else if (init_serial_boost() > 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial");
            rclcpp::shutdown();
        }

        //----Publisher
        pub_imu = this->create_publisher<sensor_msgs::msg::Imu>("/hardware/imu", 1);

        //----Timer
        tim_50hz = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&SerialIMU::callback_tim_50hz, this));

        logger.info("Serial IMU init on %s (baudrate: %d)", port.c_str(), baudrate);
    }

    void callback_tim_50hz()
    {
        while (rclcpp::ok())
        {
            try
            {
                if (!use_boost)
                    read_serial();
                else
                    read_serial_asio();
            }
            catch (const std::exception &e)
            {
                logger.error("Error: %s", e.what());
            }
            // usleep(10000); // 100ms
        }
        // try
        // {
        //     if (!use_boost)
        //         read_serial();
        //     else
        //         read_serial_asio();
        // }
        // catch (const std::exception &e)
        // {
        //     logger.error("Error: %s", e.what());
        // }
    }

    int8_t init_serial_boost()
    {
        boost::system::error_code ec;
        serial_port.open(port);

        if (ec)
        {
            logger.error("Error %i from opening usb device: %s", ec.value(), ec.message().c_str());
            return 1;
        }

        serial_port.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
        serial_port.set_option(boost::asio::serial_port_base::character_size(8));
        serial_port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial_port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial_port.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

        return 0;
    }

    int8_t init_serial()
    {
        // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
        serial_port_fd = open(port.c_str(), O_RDWR);

        if (serial_port_fd < 0)
        {
            logger.error("Error %i from opening usb device: %s", errno, strerror(errno));
            return 1;
        }

        // Create new termios struct, we call it 'tty' for convention
        struct termios tty;

        // Read in existing settings, and handle any error
        if (tcgetattr(serial_port_fd, &tty) != 0)
        {
            logger.error("Error %i from tcgetattr: %s", errno, strerror(errno));
            return 1;
        }

        tty.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity (most common)
        tty.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication (most common)
        tty.c_cflag &= ~CSIZE;         // Clear all bits that set the data size
        tty.c_cflag |= CS8;            // 8 bits per byte (most common)
        tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control (most common)
        tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;                                                        // Disable echo
        tty.c_lflag &= ~ECHOE;                                                       // Disable erasure
        tty.c_lflag &= ~ECHONL;                                                      // Disable new-line echo
        tty.c_lflag &= ~ISIG;                                                        // Disable interpretation of INTR, QUIT and SUSP
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

        tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
        tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
        // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
        // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

        tty.c_cc[VTIME] = 0; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
        tty.c_cc[VMIN] = 0;

        // Set in/out baud rate to be 9600
        if (baudrate == 9600)
        {
            cfsetispeed(&tty, B9600);
            cfsetospeed(&tty, B9600);
        }
        else if (baudrate == 19200)
        {
            cfsetispeed(&tty, B19200);
            cfsetospeed(&tty, B19200);
        }
        else if (baudrate == 38400)
        {
            cfsetispeed(&tty, B38400);
            cfsetospeed(&tty, B38400);
        }
        else if (baudrate == 115200)
        {
            cfsetispeed(&tty, B115200);
            cfsetospeed(&tty, B115200);
        }

        // status |= TIOCM_DTR; /* turn on DTR */
        // status |= TIOCM_RTS; /* turn on RTS */

        // Save tty settings, also checking for error
        if (tcsetattr(serial_port_fd, TCSANOW, &tty) != 0)
        {
            logger.error("Error %i from tcsetattr: %s", errno, strerror(errno));
            return 1;
        }

        return 0;
    }

    float parse_riontech_value(uint16_t byte_offset)
    {
        int _sign, a, b, c, d, e;
        _sign = (recv_buffer[byte_offset] & 0xFF) / 16;
        if (_sign == 1)
            _sign = 1;
        else
            _sign = -1;
        //  10 10 55 --> -010.55
        a = (recv_buffer[byte_offset] & 0xFF) % 16;
        b = (recv_buffer[byte_offset + 1] & 0xFF) / 16 % 16;
        c = (recv_buffer[byte_offset + 1] & 0xFF) % 16;
        d = (recv_buffer[byte_offset + 2] & 0xFF) / 16 % 16;
        e = (recv_buffer[byte_offset + 2] & 0xFF) % 16;
        return _sign * abs(a * 100.0 + b * 10.0 + c + d * 0.1 + e * 0.01);
    }

    uint8_t rion_checksum(char *data, size_t len)
    {
        uint8_t temp = 0;
        for (size_t i = 0; i < len; i++)
            temp += data[i];
        return temp;
    }

    float rion_parse(uint16_t byte_offset, float factor)
    {
#ifndef BIN2BCD
#define BIN2BCD(data) (((data >> 4) & 0xF) * 10 + (data & 0xF))
#endif

        float temp = 0;
        uint8_t sign = (recv_buffer[byte_offset] & 0x10) != 0;
        temp = ((recv_buffer[byte_offset] & 0x0F) * 10000.00) + (BIN2BCD(recv_buffer[byte_offset + 1]) * 100.00) + (BIN2BCD(recv_buffer[byte_offset + 2]) * 1.00);
        temp *= factor;
        temp *= (sign == 1) ? -1 : 1;
        return temp;
    }

    void parse_riontech_data()
    {
        // for (int16_t i = 0; i < recv_len; i++)
        // {
        //     logger.info("Data: %x", recv_buffer[i]);
        // }

        if (recv_len > 0 && recv_buffer[0] == 0x68 && recv_buffer[1] == 0x1f)
        {
            static const float deg2rad = M_PI / 180;
            float roll = rion_parse(4, 0.01) * deg2rad;
            float pitch = rion_parse(7, 0.01) * deg2rad;
            float yaw = rion_parse(10, 0.01) * deg2rad;
            float acc_x = rion_parse(13, 0.001);
            float acc_y = rion_parse(16, 0.001);
            float acc_z = rion_parse(19, 0.001);
            float gyro_x = rion_parse(22, 0.01) * deg2rad;
            float gyro_y = rion_parse(25, 0.01) * deg2rad;
            float gyro_z = rion_parse(28, 0.01) * deg2rad;

            tf2::Quaternion q_tf2;
            q_tf2.setRPY(roll, pitch, yaw);
            geometry_msgs::msg::Quaternion q_msg;
            q_msg.x = q_tf2.x();
            q_msg.y = q_tf2.y();
            q_msg.z = q_tf2.z();
            q_msg.w = q_tf2.w();
            imu_msg.orientation = q_msg;
            imu_msg.angular_velocity.x = gyro_x;
            imu_msg.angular_velocity.y = gyro_y;
            imu_msg.angular_velocity.z = gyro_z;
            imu_msg.linear_acceleration.x = acc_x;
            imu_msg.linear_acceleration.y = acc_y;
            imu_msg.linear_acceleration.z = acc_z;

            // logger.info("%d %x %x %d %d Yaw: %f", recv_len, recv_buffer[0], recv_buffer[1], recv_buffer[recv_buffer[1] - 1], rion_checksum(&recv_buffer[1], recv_buffer[1] - 1), rion_parse(10, 0.01));
            // logger.info("Yaw Angular Velocity: %f", gyro_z * 180 / M_PI);

            // static float gyro_buffer_z = 0;
            // gyro_buffer_z += gyro_z * 0.02; // Assuming callback is called every 20ms
            // logger.info("YAW: %.2f, %.2f", gyro_buffer_z * 180 / M_PI, yaw * 180 / M_PI);
        }
    }

    void parse_serial_data()
    {
        size_t recv_len = strlen(recv_buffer);

        logger.info("Received===================================: %d", recv_len); //
        for (size_t i = 0; i < recv_len; i++)
        {
            // logger.info("Data: %d", recv_buffer[i]);
            // logger.info("Data: %x", recv_buffer[i]);
            if (recv_buffer[i] == READ_PROTOCOL)
            {
                if (recv_buffer[i + 1] == TYPE_ACC)
                {
                    int16_t acc_x_buffer;
                    int16_t acc_y_buffer;
                    int16_t acc_z_buffer;

                    double acc_x;
                    double acc_y;
                    double acc_z;

                    memcpy(&acc_x_buffer, recv_buffer + i + 2, 2);
                    memcpy(&acc_y_buffer, recv_buffer + i + 4, 2);
                    memcpy(&acc_z_buffer, recv_buffer + i + 6, 2);

                    acc_x = (acc_x_buffer / 32768.0) * 16.0 * GRAVITY;
                    acc_y = (acc_y_buffer / 32768.0) * 16.0 * GRAVITY;
                    acc_z = (acc_z_buffer / 32768.0) * 16.0 * GRAVITY;

                    imu_msg.linear_acceleration.x = acc_x;
                    imu_msg.linear_acceleration.y = acc_y;
                    imu_msg.linear_acceleration.z = acc_z;
                }
                else if (recv_buffer[i + 1] == TYPE_ANGULAR_VELOCITY)
                {
                    int16_t ang_vel_x_buffer;
                    int16_t ang_vel_y_buffer;
                    int16_t ang_vel_z_buffer;

                    double ang_vel_x;
                    double ang_vel_y;
                    double ang_vel_z;

                    memcpy(&ang_vel_x_buffer, recv_buffer + i + 2, 2);
                    memcpy(&ang_vel_y_buffer, recv_buffer + i + 4, 2);
                    memcpy(&ang_vel_z_buffer, recv_buffer + i + 6, 2);

                    ang_vel_x = (ang_vel_x_buffer / 32768.0) * 2000.0 * M_PI / 180; // degree per second
                    ang_vel_y = (ang_vel_y_buffer / 32768.0) * 2000.0 * M_PI / 180; // degree per second
                    ang_vel_z = (ang_vel_z_buffer / 32768.0) * 2000.0 * M_PI / 180; // degree per second

                    imu_msg.angular_velocity.x = ang_vel_x;
                    imu_msg.angular_velocity.y = ang_vel_y;
                    imu_msg.angular_velocity.z = ang_vel_z; //

                    logger.info("ang_vel_z: %f", ang_vel_z * 180 / M_PI);
                }
                else if (recv_buffer[i + 1] == TYPE_ANGLE)
                {
                    int16_t angle_x_buffer;
                    int16_t angle_y_buffer;
                    int16_t angle_z_buffer;

                    double angle_x;
                    double angle_y;
                    double angle_z;

                    memcpy(&angle_x_buffer, recv_buffer + i + 2, 2);
                    memcpy(&angle_y_buffer, recv_buffer + i + 4, 2);
                    memcpy(&angle_z_buffer, recv_buffer + i + 6, 2);

                    angle_x = (angle_x_buffer / 32768.0) * 180.0;
                    angle_y = (angle_y_buffer / 32768.0) * 180.0;
                    angle_z = (angle_z_buffer / 32768.0) * 180.0;

                    double roll = angle_x * M_PI / 180.0;
                    double pitch = angle_y * M_PI / 180.0;
                    double yaw = angle_z * M_PI / 180.0;

                    logger.info("===========================================Yaw: %f", yaw);

                    tf2::Quaternion q_tf2;
                    q_tf2.setRPY(roll, pitch, yaw);
                    geometry_msgs::msg::Quaternion q_msg;
                    q_msg.x = q_tf2.x();
                    q_msg.y = q_tf2.y();
                    q_msg.z = q_tf2.z();
                    q_msg.w = q_tf2.w();
                    imu_msg.orientation = q_msg;
                }
                else if (recv_buffer[i + 1] == TYPE_MAGNETIC)
                {
                    int16_t mag_x;
                    int16_t mag_y;
                    int16_t mag_z;

                    memcpy(&mag_x, recv_buffer + i + 2, 2);
                    memcpy(&mag_y, recv_buffer + i + 4, 2);
                    memcpy(&mag_z, recv_buffer + i + 6, 2);
                }
            }
        }
    }

    int8_t read_serial_asio()
    {
        // logger.info("Reading serial");
        boost::asio::async_read(serial_port, boost::asio::buffer(recv_buffer, 256),
                                [this](const boost::system::error_code &error, std::size_t bytes_transferred)
                                {
                                    // logger.info("Bytes transferred: %d", bytes_transferred);
                                    (void)bytes_transferred;
                                    if (!error)
                                    {
                                        if (is_riontech)
                                            parse_riontech_data();
                                        else
                                            parse_serial_data();
                                        pub_imu->publish(imu_msg);
                                    }
                                });
        return 0;
    }

    int8_t read_serial()
    {
        if (is_riontech)
        {
            uint8_t serialmsg[] = {0x68, 0x04, 0x00, 0x04, 0x08};
            write(serial_port_fd, serialmsg, sizeof(serialmsg));
        }

        uint16_t bytes_available = 0;
        if (ioctl(serial_port_fd, FIONREAD, &bytes_available) == -1)
            return -1;

        // logger.info("Bytes available: %d", bytes_available);

        if (bytes_available == 0)
            return 0;

        if (bytes_available > 256)
            bytes_available = 256;

        recv_len = read(serial_port_fd, &recv_buffer, bytes_available);

        // logger.info("Bytes read: %d", recv_len);

        if (recv_len < 0)
            return -1;

        if (is_riontech)
            parse_riontech_data();
        else
            parse_serial_data();

        pub_imu->publish(imu_msg);

        return 0;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node_imu = std::make_shared<SerialIMU>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_imu);
    executor.spin();

    return 0;
}