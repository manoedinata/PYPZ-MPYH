// asd
#ifndef MASTER_HPP
#define MASTER_HPP

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pcl_ros/transforms.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_interface/msg/point_array.hpp"
#include "ros2_interface/msg/terminal_array.hpp"
#include "ros2_interface/msg/vision_urban.hpp"
#include "ros2_utils/global_definitions.hpp"
#include "ros2_utils/help_logger.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <pcl/common/centroid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// #include "ros2_utils/help_marker.hpp"
#include "boost/filesystem.hpp"
#include "boost/thread/mutex.hpp"
#include "fstream"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "ros2_utils/pid.hpp"
#include "ros2_utils/simple_fsm.hpp"
#include "rtabmap_msgs/msg/info.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "opencv2/opencv.hpp"
#include <boost/algorithm/string.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/persistence.hpp>

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

// include opencv

#define DEG2RAD *0.01745329251994
#define RAD2DEG *57.29577951308232

#define FSM_GLOBAL_INIT 0
#define FSM_GLOBAL_PREOP 1
#define FSM_GLOBAL_SAFEOP 2
#define FSM_GLOBAL_OP_3 3
#define FSM_GLOBAL_OP_4 4
#define FSM_GLOBAL_OP_5 5
#define FSM_GLOBAL_OP_2 6
#define FSM_GLOBAL_RECORD_ROUTE 7
#define FSM_GLOBAL_MAPPING 8
#define FSM_GLOBAL_RECORD_ROUTE_KANAN 9
#define FSM_GLOBAL_RECORD_ROUTE_KIRI 10
#define FSM_GLOBAL_RECORD_ROUTE_TENGAH 11
#define FSM_GLOBAL_RACE_BUTTON 12

#define FSM_GLOBAL_RECORD_DATASET_ROAD 20
#define FSM_GLOBAL_RECORD_DATASET_ROAD_VIDEO 20
#define FSM_GLOBAL_CUSTOM_DEBUG_1 300
#define FSM_GLOBAL_CUSTOM_DEBUG_2 301

#define FSM_LOCAL_PRE_FOLLOW_LANE 0
#define FSM_LOCAL_FOLLOW_LANE 1
#define FSM_LOCAL_MENUNGGU_STATION_1 2
#define FSM_LOCAL_MENUNGGU_STATION_2 3
#define FSM_LOCAL_MENUNGGU_STOP 4

#define IN_TRIM_KECEPATAN 0b10000000000000
#define IN_START_OP3 (0b100000 << 16)
#define IN_STOP_OP3 (0b100 << 16)
#define IN_START_GAS_MANUAL (0b1000 << 16)
#define IN_MANUAL_MUNDUR (0b100 << 16)
#define IN_MANUAL_MAJU (0b10 << 16)
#define IN_NEXT_TERMINAL (0b10000 << 16)
#define IN_SYSTEM_FULL_ENABLE (0b01 << 16)

#define TERMINAL_TYPE_STOP 0x01
#define TERMINAL_TYPE_BELOKAN 0x02
#define TERMINAL_TYPE_STOP1 0x04
#define TERMINAL_TYPE_STOP2 0x08
#define TERMINAL_TYPE_STOP3 0x0C
#define TERMINAL_TYPE_STOP4 0x10
#define TERMINAL_TYPE_LURUS 0x20

#define ARUCO_NULL -1
#define ARUCO_NO_ENTRY 0
#define ARUCO_DEAD_END 1
#define ARUCO_TURN_RIGHT 2
#define ARUCO_TURN_LEFT 3
#define ARUCO_FORWARD 4
#define ARUCO_STOP 5

typedef struct
{
    float x;
    float y;
    float theta;
    float fb_velocity;
    float fb_steering;
    float arah;
} waypoint_t;

typedef struct
{
    float x;
    float y;
} point_t;

typedef struct
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    rclcpp::Time stamp;
} pcl_timed_t;

typedef struct
{
    int8_t id; // ID of the apriltag
    float x;
    float y;
    float z;
    float dist; // Distance to the apriltag in meters
    float roll; // Roll angle of the apriltag in radians
    float pitch; // Pitch angle of the apriltag in radians
    float yaw; // Yaw angle of the apriltag in radians
    double t; // Timestamp of the detection in seconds
} apriltag_t;

typedef struct
{
    int8_t berhenti;
    int16_t pos_target_px_x;
    int16_t pos_target_px_y;
    int16_t pos_robot_px_x;
    int16_t pos_robot_px_y;
    float dist_putih_meter;
    float dist_near_zebracross;
    float target_angle_ungu;
    float target_angle_putih;
    float meter_to_pixel; // Conversion factor from meters to pixels
    float offset_angle; // Offset angle for zebracross detection
    float offset_angle_lane; // Offset angle for zebracross detection
    float dist_near_zebracross_vertical; // Distance to the nearest zebracross in vertical direction
    float dist_near_zebracross_horizontal; // Distance to the nearest zebracross in horizontal direction
    float centroid_sign_x; // X coordinate of the centroid of the detected sign
    float centroid_sign_y; // Y coordinate of the centroid of the detected sign
    float centroid_obs_x; // X coordinate of the centroid of the detected obstacle
    float centroid_obs_y; // Y coordinate of the centroid of the detected obstacle
    float dist_near_zebracross_vertical_kiri; // Distance to the nearest zebracross in vertical direction for left side
    float dist_near_zebracross_vertical_kanan; // Distance to the nearest zebracross in vertical direction for right side
    int8_t jalan_berkelok; // 0: tidak berkelok, 1: berkelok kanan, 2: berkelok kiri
    int8_t mask_jalan_bocor;
    int8_t ada_pertigaan; // 0: tidak bocor,
    float jarak_ke_pertigaan;
} vision_urban_t;

class Master : public rclcpp::Node {
public:
    // Callback group
    rclcpp::CallbackGroup::SharedPtr callback_group_subscribers_;

    rclcpp::TimerBase::SharedPtr tim_routine;
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_ui_test;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_target_velocity;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_target_Steering;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_global_fsm;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_local_fsm;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_posisi_robot;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_posisi_obstacle;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_waypoints;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_waypoints_kanan;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_waypoints_kiri;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_waypoints_tengah;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_point;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_buffered_obs_pointcloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_buffered_road_pointcloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_edge_right_road_pointcloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_edge_left_road_pointcloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_free_path_map;
    rclcpp::Publisher<ros2_interface::msg::TerminalArray>::SharedPtr pub_terminals;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_nearest_obstacle;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_most_left_obstacle;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_most_right_obstacle;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_target_pt;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_state_urban;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_sign_buzzer;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_curr_gyro_deg;

    //--Subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_selected_lane;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_key_pressed;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_key_web_pressed;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_ui_control_velocity_and_steering;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_set_master_fsm;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_pointcloud_laser_scan;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_steering_vision;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_velocity_vision;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_speed_cnn;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_nearest_obstacle;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_filtered_camera;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_road_pointcloud;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_road_obs_pointcloud;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_sign_pointcloud;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_april_tag_status;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_sign_tag_status;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_target_steering;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_detected_apriltag;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_enc_meter;
    rclcpp::Subscription<ros2_interface::msg::VisionUrban>::SharedPtr sub_vision_urban;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_vision_urban_config;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_toggle_debug;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_toggle_debug2;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_button_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_set_record_route_mode;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_set_record_route_mode_kanan;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_set_record_route_mode_kiri;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_set_record_route_mode_tengah;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_set_add_record_route_mode;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_set_terminal; // Aktif -> add terminal, InActive -> save terminal
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_set_terminal_sign; // Aktif -> add terminal, InActive -> save terminal
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_rm_terminal; // Aktif -> add terminal, InActive -> save terminal

    //----Configs
    //===========================================
    float profile_max_acceleration = 40;
    float profile_max_decceleration = 100;
    float profile_max_velocity = 2.5; // m/s (1 m/s == 3.6 km/h)
    float profile_max_accelerate_jerk = 200;
    float profile_max_decelerate_jerk = 3000;
    float profile_max_braking = 3;
    float profile_max_braking_acceleration = 4000;
    float profile_max_braking_jerk = 4000;
    float profile_max_steering_rad = 0.69; // Ini adalah arah roda depannya
    std::string waypoint_file_path;
    std::string waypoint_file_path_kanan;
    std::string waypoint_file_path_kiri;
    std::string waypoint_file_path_tengah;

    std::string terminal_file_path;
    float timeout_terminal_1 = 10;
    float timeout_terminal_2 = 10;
    bool transform_map2odom = false;

    int8_t ada_obs = 0; // 0 = tidak ada, 1 = hindar ke kanan, 2 = hindar ke kiri
    int16_t debug_mode_ = 0;
    int16_t debug_mode2_ = 0;

    float offset_sudut_steering = 0;
    float wheelbase = 0.27;
    float default_lookahead = 0.6;

    int time_cntr_1 = 0; // untuk mengatur waktu
    int time_cntr_2 = 0; // untuk mengatur waktuss
    int time_cntr_3 = 0; // untuk mengatur waktu
    int time_cntr_4 = 0; // untuk mengatur waktu
    int target_vel_obstacle = 0;

    int lama_waktu_menghindar = 4;
    float offset_jarak_hindar = 0.22;
    float kecepatan_default_menghindar = 0.4; // m/s

    int max_counter_belok_kanan = 100; // untuk mengatur berapa lama mobil belok ke kanan
    int max_counter_belok_kiri = 100; // untuk mengatur berapa lama mobil belok ke kiri
    int max_counter_lurus = 30; // untuk mengatur berapa lama mobil mundur
    int max_counter_lurus_awal_kiri = 30; // untuk mengatur berapa lama mobil mundur
    int max_counter_lurus_awal_kanan = 30; // untuk mengatur berapa lama mobil mundur

    int8_t selected_lane = 0; // 1 = kanan, 0 = tengah, -1 = kiri
    int8_t posisi_obs_di_kanan = 0;
    int8_t posisi_robot_di_kanan = 0;

    float enc_meter;

    int16_t key_web_pressed = 0; // untuk menampung tombol yang ditekan dari web

    std::vector<waypoint_t> waypoints;
    std::vector<waypoint_t> waypoints_race_kanan;
    std::vector<waypoint_t> waypoints_race_kiri;
    std::vector<waypoint_t> waypoints_race_tengah;
    std::vector<waypoint_t> waypoints_race_selected;

    ros2_interface::msg::TerminalArray terminals;
    std::vector<point_t> area_special;

    //----Vars
    HelpLogger logger;
    MachineState global_fsm;
    MachineState local_fsm;

    float fb_encoder_meter = 0;
    float fb_final_pose_xyo[3];
    float fb_final_vel_dxdydo[3];
    float fb_steering_angle = 0;
    uint8_t fb_eps_mode = 0;

    int32_t apriltag_status = -1;
    apriltag_t detected_apriltag;

    int32_t sign_detected_status = -1;

    float actuation_ax = 0;
    float actuation_ay = 0;
    float actuation_az = 0;

    float actuation_vx = 0;
    float actuation_vy = 0;
    float actuation_wz = 0; // Ini posisi

    float target_velocity_joy_x = 0;
    float target_velocity_joy_y = 0;
    float target_velocity_joy_wz = 0;

    float target_steering_cnn = 0;
    float target_speed_cnn = 0;

    float target_steering_vision = 0;
    float target_velocity_vision = 0;

    float target_steering = 0;
    float target_steering_obstacle = 0;

    float target_velocity = 0;

    float derajat_steering_kanan_ = -32; // derajat steering untuk belok kanan
    float derajat_steering_kiri_ = 25; // derajat steering untuk belok kiri
    float encoder_belok_kanan_ = 0; // jarak encoder untuk belok kanan (alternatif jika tanpa gyro)
    float encoder_belok_kiri_ = 0; // jarak encoder untuk belok kiri (alternatif jika tanpa gyro)
    float encoder_maju_kanan_ = 0.53; // jarak encoder untuk maju setelah belok kanan (alternatif jika tanpa putih)
    float encoder_maju_kiri_ = 0.53; // jarak encoder untuk maju setelah belok kiri (alternatif jika tanpa putih)
    float encoder_maju_lurus_ = 0.53; // jarak encoder untuk maju setelah lurus (alternatif jika tanpa putih)
    float derajat_gyro_kanan_ = 80; // derajat gyro untuk berhenti setelah belok kanan
    float derajat_gyro_kiri_ = 80; // derajat gyro untuk berhenti setelah belok kiri
    float jarak_ke_zebracros_ = 0.3; // jarak berhenti ke zebracross
    float jarak_ke_putih_ = 0.6; // jarak ke titik putih sebelum berhenti (alternatif jika tanpa zebracross)
    float min_jarak_putih_kanan_ = 0; // jarak minimum ke titik putih sebelum belok kanan
    float min_jarak_putih_kiri_ = 0; // jarak minimum ke titik putih sebelum belok kiri
    float min_jarak_putih_lurus_ = 0; // jarak minimum ke titik putih sebelum lurus
    float encoder_maju_dead_end_ = 0.015; // Default value for dead end maneuver
    float velocity_jalan_otomatis = 0.31241;
    float offset_jarak_sign_pole_ = 0.25; // Offset distance from sign pole
    float last_gyro_angle_ = 78.0f; // Last gyro angle for the robot
    float min_vel_belokan_ = 0.2;
    float jarak_ke_sign_pole_ = 0.25; // Offset distance to sign pole

    uint8_t button_1 = 0;
    uint8_t button_2 = 0;
    uint8_t toogle_button_1 = 0;
    uint8_t toogle_button_2 = 0;

    float dt = 0.02;

    bool tf_is_initialized = false;

    vision_urban_t urban_data;

    tf2::Transform manual_map2odom_tf;

    geometry_msgs::msg::TransformStamped tf_base2map;

    pcl::PointCloud<pcl::PointXYZ> points_obstacle2base;
    pcl::PointCloud<pcl::PointXYZ> points_road2base;

    pcl::PointCloud<pcl::PointXYZ> points_obstacle2map;
    pcl::PointCloud<pcl::PointXYZ> points_road2map;

    pcl::PointCloud<pcl::PointXYZ> points_sign2base;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // queue for storing point clouds of obstacle
    std::vector<pcl_timed_t> obs_pcl_buffer;
    std::vector<pcl_timed_t> road_pcl_buffer;

    pcl::PointCloud<pcl::PointXYZ> buffered_obs;
    pcl::PointCloud<pcl::PointXYZ> buffered_road;
    pcl::PointCloud<pcl::PointXYZ> buffered_road2map;
    pcl::PointCloud<pcl::PointXYZ> buffered_obs2map;

    pcl::PointCloud<pcl::PointXYZ> edge_left, edge_right;
    pcl::PointCloud<pcl::PointXYZ> edge_left_filtered, edge_right_filtered;

    Eigen::Vector3f most_left_obstacle_point;
    Eigen::Vector3f most_right_obstacle_point;

    Eigen::Vector4f obstacle_centroid = Eigen::Vector4f::Zero();
    Eigen::Vector4f obstacle_centroid2base = Eigen::Vector4f::Zero();
    Eigen::Vector4f obstacle_centroid2map = Eigen::Vector4f::Zero();
    Eigen::Vector4f sign_centroid = Eigen::Vector4f::Zero();
    Eigen::Vector4f sign_centroid2map = Eigen::Vector4f::Zero();
    pcl::PointCloud<pcl::PointXYZ> points_road_obs2base;

    rclcpp::Clock clock;

    Master();
    ~Master();

    //----ROS
    void callback_routine();
    void callback_sub_odometry(const nav_msgs::msg::Odometry::SharedPtr msg);
    void callback_sub_key_pressed(const std_msgs::msg::Int16::SharedPtr msg);
    void callback_sub_selected_lane(const std_msgs::msg::Int8::SharedPtr msg);
    void callback_sub_key_web_pressed(const std_msgs::msg::Int16::SharedPtr msg);
    void callback_srv_set_record_route_mode(const std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response);
    void callback_srv_set_record_route_mode_kanan(const std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response);
    void callback_srv_set_record_route_mode_kiri(const std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response);
    void callback_srv_set_record_route_mode_tengah(const std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response);
    void callback_srv_set_add_record_route_mode(const std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response);
    void callback_sub_ui_control_velocity_and_steering(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void callback_sub_set_master_fsm(const std_msgs::msg::Int16::SharedPtr msg);
    void callback_sub_pointcloud_laser_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void callback_sub_steering_vision(const std_msgs::msg::Float32::SharedPtr msg);
    void callback_sub_velocity_vision(const std_msgs::msg::Float32::SharedPtr msg);
    void callback_sub_speed_cnn(const std_msgs::msg::Float32::SharedPtr msg);
    void callback_sub_nearest_obstacle(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void callback_sub_filtered_camera(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void callback_sub_road_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void callback_sub_road_obs_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void callback_sub_sign_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void callback_sub_apriltag(const std_msgs::msg::Int32::SharedPtr msg);
    void callback_sub_sign_detection(const std_msgs::msg::Int32::SharedPtr msg);
    void callback_sub_joy(const sensor_msgs::msg::Joy::SharedPtr msg);
    void callback_sub_target_steering(const std_msgs::msg::Float32::SharedPtr msg);
    void callback_sub_detected_apriltag(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    void callback_sub_enc_meter(const std_msgs::msg::Float32::SharedPtr msg);
    void callback_sub_toggle_debug(const std_msgs::msg::Int16::SharedPtr msg);
    void callback_sub_toggle_debug2(const std_msgs::msg::Int16::SharedPtr msg);
    void callback_srv_set_terminal(const std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response);
    void callback_srv_set_terminal_sign(const std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response);
    void callback_srv_rm_terminal(const std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response);
    void callback_sub_vision_urban(const ros2_interface::msg::VisionUrban::SharedPtr msg);
    void callback_sub_vision_urban_config(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void callback_sub_button(const std_msgs::msg::Int8::SharedPtr msg);

    // Process
    // ===============================================================================================
    void process_transmitter();
    void process_marker();
    void process_local_fsm();
    void process_record_route();
    void process_record_route(std::vector<waypoint_t>& wps);
    void process_add_terminal();
    void process_load_waypoints();
    void process_load_waypoints_race(std::string file_path, std::vector<waypoint_t>& wps);
    void process_save_waypoints();
    void process_save_waypoints_race(std::string file_path, std::vector<waypoint_t>& wps);
    void process_load_terminals();
    void process_save_terminals();
    void process_add_terminal_sign();

    // Motion
    // ===============================================================================================
    void manual_motion(float vx, float vy, float wz);
    void wp2velocity_steering(float lookahead_distance, float* pvelocity, float* psteering, bool is_loop = false);
    void wp2velocity_steering_urban(float lookahead_distance, float* pvelocity, float* psteering, int* counter_diam, point_t arah_belok, int32_t sign_status, bool is_loop = false);
    void wp2velocity_steering_urban_coba(float lookahead_distance, float* pvelocity, float* psteering, bool diam, bool* masuk_terminal_diam, bool is_loop = false);
    void wp2velocity_steering_race(float lookahead_distance, float* pvelocity, float* psteering, bool is_loop);

    int8_t move_forward_distance(float distance_target, float* pvelocity, float start_x, float start_y, float target_distance = 0.2);

    void follow_waypoints_gas_manual(float vx, float vy, float wz, float lookahead_distance, bool is_loop);
    float pythagoras(float x1, float y1, float x2, float y2);
    void follow_waypoints(float vx, float vy, float wz, float lookahead_distance, bool is_loop);
    void cnn_move(float vx, float vy, float wz, float profile_max_velocity, float target_steering_cnn);
    void cnn_move2(float vx, float vy, float wz, float profile_max_velocity, float target_steering_cnn);
    void urban_move(float vx, float vy, float wz);
    void urban_move2(float vx, float vy, float wz, int8_t oto = 0);
    void race_move(float vx, float vy, float wz);
    void combine_road_obstacle_pcl(int8_t* selected_lane_);

    int8_t obstacle_avoidance_move(float vx, float vy, float wz);
    int8_t move_right(float vx, float max_counter, float target_theta);
    int8_t move_left(float vx, float max_counter, float target_theta);
    int8_t stop_move();
    int8_t forward_move(float vx, float max_counter);
    int8_t control_steering(float vx, float target_steering_local);

    // Misc
    // ===============================================================================================
    void set_initialpose(float x, float y, float yaw);
    void set_pose_offset(float x, float y, float yaw);
    void combine_road_obstacle_pcl();

    void centerline_extractor();
    void buffer_obs_road_pcl();
    void free_road_detection();
    void generate_free_path_map();
};

#endif // MASTER_HPP
