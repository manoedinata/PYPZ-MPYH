// asd
#include "master/master.hpp"

Master::Master()
    : Node("master"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
    if (!logger.init())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
        rclcpp::shutdown();
    }

    //----Parameters
    this->declare_parameter<float>("profile_max_acceleration", 0.0);
    this->get_parameter("profile_max_acceleration", profile_max_acceleration);

    this->declare_parameter<float>("profile_max_decceleration", 0.0);
    this->get_parameter("profile_max_decceleration", profile_max_decceleration);

    this->declare_parameter<float>("profile_max_velocity", 0.0);
    this->get_parameter("profile_max_velocity", profile_max_velocity);

    this->declare_parameter<float>("profile_max_accelerate_jerk", 0.0);
    this->get_parameter("profile_max_accelerate_jerk", profile_max_accelerate_jerk);

    this->declare_parameter<float>("profile_max_decelerate_jerk", 0.0);
    this->get_parameter("profile_max_decelerate_jerk", profile_max_decelerate_jerk);

    this->declare_parameter<float>("profile_max_braking", 0.0);
    this->get_parameter("profile_max_braking", profile_max_braking);

    this->declare_parameter<float>("profile_max_braking_acceleration", 0.0);
    this->get_parameter("profile_max_braking_acceleration", profile_max_braking_acceleration);

    this->declare_parameter<float>("profile_max_braking_jerk", 0.0);
    this->get_parameter("profile_max_braking_jerk", profile_max_braking_jerk);

    this->declare_parameter<float>("profile_max_steering_rad", 0.0);
    this->get_parameter("profile_max_steering_rad", profile_max_steering_rad);

    this->declare_parameter("waypoint_file_path", "/home/iris/waypoints.yaml");
    this->get_parameter("waypoint_file_path", waypoint_file_path);

    this->declare_parameter("waypoint_file_path_kanan", "/home/iris/waypoints_kanan.yaml");
    this->get_parameter("waypoint_file_path_kanan", waypoint_file_path_kanan);

    this->declare_parameter("waypoint_file_path_kiri", "/home/iris/waypoints_kiri.yaml");
    this->get_parameter("waypoint_file_path_kiri", waypoint_file_path_kiri);

    this->declare_parameter("waypoint_file_path_tengah", "/home/iris/waypoints_tengah.yaml");
    this->get_parameter("waypoint_file_path_tengah", waypoint_file_path_tengah);

    this->declare_parameter("terminal_file_path", "/home/iris/terminal.yaml");
    this->get_parameter("terminal_file_path", terminal_file_path);

    this->declare_parameter("wheelbase", 0.27);
    this->get_parameter("wheelbase", wheelbase);

    this->declare_parameter("default_lookahead", 0.27);
    this->get_parameter("default_lookahead", default_lookahead);

    this->declare_parameter<int>("time_cntr_1", 0);
    this->get_parameter("time_cntr_1", time_cntr_1);

    this->declare_parameter<int>("time_cntr_2", 0);
    this->get_parameter("time_cntr_2", time_cntr_2);

    this->declare_parameter<int>("time_cntr_3", 0);
    this->get_parameter("time_cntr_3", time_cntr_3);

    this->declare_parameter<int>("time_cntr_4", 0);
    this->get_parameter("time_cntr_4", time_cntr_4);

    this->declare_parameter<int>("target_vel_obstacle", 0);
    this->get_parameter("target_vel_obstacle", target_vel_obstacle);

    this->declare_parameter<int>("lama_waktu_menghindar", 4);
    this->get_parameter("lama_waktu_menghindar", lama_waktu_menghindar);

    this->declare_parameter<float>("kecepatan_default_menghindar", 0.4);
    this->get_parameter("kecepatan_default_menghindar", kecepatan_default_menghindar);

    this->declare_parameter<float>("offset_jarak_hindar", 0.22);
    this->get_parameter("offset_jarak_hindar", offset_jarak_hindar);

    this->declare_parameter<int>("max_counter_belok_kanan", 100);
    this->get_parameter("max_counter_belok_kanan", max_counter_belok_kanan);

    this->declare_parameter<int>("max_counter_belok_kiri", 100);
    this->get_parameter("max_counter_belok_kiri", max_counter_belok_kiri);

    this->declare_parameter<int>("max_counter_lurus", 30);
    this->get_parameter("max_counter_lurus", max_counter_lurus);

    this->declare_parameter<int>("max_counter_lurus_awal_kiri", 30);
    this->get_parameter("max_counter_lurus_awal_kiri", max_counter_lurus_awal_kiri);

    this->declare_parameter<int>("max_counter_lurus_awal_kanan", 30);
    this->get_parameter("max_counter_lurus_awal_kanan", max_counter_lurus_awal_kanan);

    //----Publisher
    pub_target_velocity = this->create_publisher<std_msgs::msg::Float32>("/master/target_speed", 1);
    pub_target_Steering = this->create_publisher<std_msgs::msg::Float32>("/master/target_steering", 1);
    pub_waypoints = this->create_publisher<sensor_msgs::msg::PointCloud>("/master/waypoints", 1);
    pub_waypoints_kanan = this->create_publisher<sensor_msgs::msg::PointCloud>("/master/waypoints_kanan", 1);
    pub_waypoints_kiri = this->create_publisher<sensor_msgs::msg::PointCloud>("/master/waypoints_kiri", 1);
    pub_waypoints_tengah = this->create_publisher<sensor_msgs::msg::PointCloud>("/master/waypoints_tengah", 1);
    pub_global_fsm = this->create_publisher<std_msgs::msg::Int16>("/master/global_fsm", 1);
    pub_local_fsm = this->create_publisher<std_msgs::msg::Int16>("/master/local_fsm", 1);
    pub_posisi_robot = this->create_publisher<std_msgs::msg::Int8>("/master/posisi_robot", 1);
    pub_posisi_obstacle = this->create_publisher<std_msgs::msg::Int8>("/master/posisi_obstacle", 1);
    pub_path_point = this->create_publisher<nav_msgs::msg::Path>("/master/path_point", 1);
    pub_buffered_obs_pointcloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/master/buffered_obs_pointcloud", 1);
    pub_buffered_road_pointcloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/master/buffered_road_pointcloud", 1);
    pub_nearest_obstacle = this->create_publisher<nav_msgs::msg::Odometry>("/master/nearest_obstacle", 1);
    pub_edge_left_road_pointcloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/master/edge_left_road_pointcloud", 1);
    pub_edge_right_road_pointcloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/master/edge_right_road_pointcloud", 1);
    pub_most_left_obstacle = this->create_publisher<geometry_msgs::msg::PointStamped>("/master/most_left_obs", 1);
    pub_most_right_obstacle = this->create_publisher<geometry_msgs::msg::PointStamped>("/master/most_right_obs", 1);
    pub_target_pt = this->create_publisher<geometry_msgs::msg::PointStamped>("/master/target_pt_avoid", 1);
    pub_free_path_map = this->create_publisher<sensor_msgs::msg::PointCloud2>("/master/free_path_map", 1);
    pub_terminals = this->create_publisher<ros2_interface::msg::TerminalArray>("/master/terminals", 1);
    pub_state_urban = this->create_publisher<std_msgs::msg::Int16>("/master/state_urban", 1);
    pub_sign_buzzer = this->create_publisher<std_msgs::msg::Int8>("/master/sign_buzzer", 1);
    pub_curr_gyro_deg = this->create_publisher<std_msgs::msg::Float32>("/master/curr_gyro_deg", 1);

    //----Callback group
    callback_group_subscribers_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    // callback_group_subscribers_ = this->create_callback_group(
    //     rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_subscribers_;

    //----Subscribers
    sub_odometry = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 1, std::bind(&Master::callback_sub_odometry, this, std::placeholders::_1), options);
    sub_key_pressed = this->create_subscription<std_msgs::msg::Int16>(
        "/key_pressed", 1, std::bind(&Master::callback_sub_key_pressed, this, std::placeholders::_1), options);
    sub_selected_lane = this->create_subscription<std_msgs::msg::Int8>(
        "/web/selected_lane", 1, std::bind(&Master::callback_sub_selected_lane, this, std::placeholders::_1), options);
    sub_key_web_pressed = this->create_subscription<std_msgs::msg::Int16>(
        "/key_web_pressed", 1, std::bind(&Master::callback_sub_key_web_pressed, this, std::placeholders::_1), options);
    sub_ui_control_velocity_and_steering = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/master/ui_target_velocity_and_steering", 1, std::bind(&Master::callback_sub_ui_control_velocity_and_steering, this, std::placeholders::_1), options);
    sub_set_master_fsm = this->create_subscription<std_msgs::msg::Int16>(
        "/master/set_master_fsm", 1, std::bind(&Master::callback_sub_set_master_fsm, this, std::placeholders::_1), options);
    // sub_pointcloud_laser_scan = this->create_subscription<sensor_msgs::msg::LaserScan>(
    //     "/vision/laserscan", 1, std::bind(&Master::callback_sub_pointcloud_laser_scan, this, std::placeholders::_1), options);
    sub_steering_vision = this->create_subscription<std_msgs::msg::Float32>(
        "/vision/slope", 1, std::bind(&Master::callback_sub_steering_vision, this, std::placeholders::_1), options);
    sub_velocity_vision = this->create_subscription<std_msgs::msg::Float32>(
        "/vision/velocity", 1, std::bind(&Master::callback_sub_velocity_vision, this, std::placeholders::_1), options);
    // sub_speed_cnn = this->create_subscription<std_msgs::msg::Float32>(
    //     "/onnx_inference/target_speed", 1, std::bind(&Master::callback_sub_speed_cnn, this, std::placeholders::_1), options);
    // sub_nearest_obstacle = this->create_subscription<geometry_msgs::msg::PointStamped>(
    //     "/detection/nearest_obstacle", 1, std::bind(&Master::callback_sub_nearest_obstacle, this, std::placeholders::_1), options);
    // sub_filtered_camera = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    //     "/vision/filtered_points", 1, std::bind(&Master::callback_sub_filtered_camera, this, std::placeholders::_1), options);
    // sub_filtered_camera = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    //     "/vision/yuv_pointcloud", 1, std::bind(&Master::callback_sub_filtered_camera, this, std::placeholders::_1), options);
    // sub_road_pointcloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    //     "/vision/cleaned_pointcloud", 1, std::bind(&Master::callback_sub_road_pointcloud, this, std::placeholders::_1), options);
    // sub_road_obs_pointcloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    //     "/vision/road_obstacle_combined", 1, std::bind(&Master::callback_sub_road_obs_pointcloud, this, std::placeholders::_1), options);
    // sub_sign_pointcloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    //     "/vision/sign_points", 1, std::bind(&Master::callback_sub_sign_pointcloud, this, std::placeholders::_1), options);
    sub_april_tag_status = this->create_subscription<std_msgs::msg::Int32>(
        "/sign/marker/id", 1, std::bind(&Master::callback_sub_apriltag, this, std::placeholders::_1), options);
    sub_sign_tag_status = this->create_subscription<std_msgs::msg::Int32>(
        "/sign/picture/id", 1, std::bind(&Master::callback_sub_sign_detection, this, std::placeholders::_1), options);
    sub_joy = this->create_subscription<sensor_msgs::msg::Joy>(
        "/hardware/joy", 1, std::bind(&Master::callback_sub_joy, this, std::placeholders::_1), options);
    sub_target_steering = this->create_subscription<std_msgs::msg::Float32>(
        "/vision/intersection_point", 1, std::bind(&Master::callback_sub_target_steering, this, std::placeholders::_1), options);
    sub_detected_apriltag = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/apriltag/markers", 1, std::bind(&Master::callback_sub_detected_apriltag, this, std::placeholders::_1), options);
    sub_enc_meter = this->create_subscription<std_msgs::msg::Float32>(
        "/motor_main/velocity_feedback", 1, std::bind(&Master::callback_sub_enc_meter, this, std::placeholders::_1), options);
    sub_vision_urban = this->create_subscription<ros2_interface::msg::VisionUrban>(
        "/vision/urban_data", 1, std::bind(&Master::callback_sub_vision_urban, this, std::placeholders::_1), options);
    sub_vision_urban_config = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/vision/master_config", 1, std::bind(&Master::callback_sub_vision_urban_config, this, std::placeholders::_1), options);
    sub_toggle_debug = this->create_subscription<std_msgs::msg::Int16>(
        "/web/toggle_debug", 1, std::bind(&Master::callback_sub_toggle_debug, this, std::placeholders::_1), options);
    sub_toggle_debug2 = this->create_subscription<std_msgs::msg::Int16>(
        "/web/toggle_debug2", 1, std::bind(&Master::callback_sub_toggle_debug2, this, std::placeholders::_1), options);
    sub_button_ = this->create_subscription<std_msgs::msg::Int8>(
        "/hardware/button", 1, std::bind(&Master::callback_sub_button, this, std::placeholders::_1), options);

    srv_set_record_route_mode = this->create_service<std_srvs::srv::SetBool>(
        "/master/set_record_route_mode", std::bind(&Master::callback_srv_set_record_route_mode, this, std::placeholders::_1, std::placeholders::_2));
    srv_set_record_route_mode_kanan = this->create_service<std_srvs::srv::SetBool>(
        "/master/set_record_route_mode_kanan", std::bind(&Master::callback_srv_set_record_route_mode_kanan, this, std::placeholders::_1, std::placeholders::_2));
    srv_set_record_route_mode_kiri = this->create_service<std_srvs::srv::SetBool>(
        "/master/set_record_route_mode_kiri", std::bind(&Master::callback_srv_set_record_route_mode_kiri, this, std::placeholders::_1, std::placeholders::_2));
    srv_set_record_route_mode_tengah = this->create_service<std_srvs::srv::SetBool>(
        "/master/set_record_route_mode_tengah", std::bind(&Master::callback_srv_set_record_route_mode_tengah, this, std::placeholders::_1, std::placeholders::_2));
    srv_set_add_record_route_mode = this->create_service<std_srvs::srv::SetBool>(
        "/master/set_add_record_route_mode", std::bind(&Master::callback_srv_set_add_record_route_mode, this, std::placeholders::_1, std::placeholders::_2));
    srv_set_terminal = this->create_service<std_srvs::srv::SetBool>(
        "/master/set_terminal", std::bind(&Master::callback_srv_set_terminal, this, std::placeholders::_1, std::placeholders::_2));
    srv_set_terminal_sign = this->create_service<std_srvs::srv::SetBool>(
        "/master/set_terminal_sign", std::bind(&Master::callback_srv_set_terminal_sign, this, std::placeholders::_1, std::placeholders::_2));
    srv_rm_terminal = this->create_service<std_srvs::srv::SetBool>(
        "/master/rm_terminal", std::bind(&Master::callback_srv_rm_terminal, this, std::placeholders::_1, std::placeholders::_2));

    tim_routine = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&Master::callback_routine, this));

    logger.info("Master node initialized");
}

Master::~Master()
{
}

void Master::callback_routine()
{
    // ==================================================================
    //                            DEBUG MASTER
    // ==================================================================
    double start_time = this->now().seconds();
    static double last_time = start_time;
    double elapsed_time = start_time - last_time;
    last_time = start_time;
    // logger.info("Timer routine elapsed time: %.4f seconds -> %.2f hz", elapsed_time, 1.0 / elapsed_time);
    // ==================================================================
    // manual_motion(manual_vx, manual_vy, manual_wz);

    // logger.info("========================== GLOBAL FSM: %d ==========================", global_fsm.value);
    switch (global_fsm.value)
    {
    case FSM_GLOBAL_INIT:

        if (global_fsm.prev_value == FSM_GLOBAL_RECORD_ROUTE)
        {
            process_save_waypoints();
        }
        else if (global_fsm.prev_value == FSM_GLOBAL_RECORD_ROUTE_KANAN)
        {
            process_save_waypoints_race(waypoint_file_path_kanan, waypoints_race_kanan);
        }
        else if (global_fsm.prev_value == FSM_GLOBAL_RECORD_ROUTE_KIRI)
        {
            process_save_waypoints_race(waypoint_file_path_kiri, waypoints_race_kiri);
        }
        else if (global_fsm.prev_value == FSM_GLOBAL_RECORD_ROUTE_TENGAH)
        {
            process_save_waypoints_race(waypoint_file_path_tengah, waypoints_race_tengah);
        }
        else
        {
            process_load_waypoints();
            process_load_waypoints_race(waypoint_file_path_kanan, waypoints_race_kanan);
            process_load_waypoints_race(waypoint_file_path_kiri, waypoints_race_kiri);
            process_load_waypoints_race(waypoint_file_path_tengah, waypoints_race_tengah);
        }
        process_load_terminals();
        global_fsm.value = FSM_GLOBAL_PREOP;
        break;

    case FSM_GLOBAL_PREOP:
        manual_motion(target_velocity_joy_x, 0, target_velocity_joy_wz);
        break;

    case FSM_GLOBAL_OP_3:
        urban_move2(target_velocity_joy_x, 0, target_velocity_joy_wz);

        break;

    case FSM_GLOBAL_OP_4:
        // follow_waypoints(target_velocity_joy_x, 0, target_velocity_joy_wz, default_lookahead, true);
        manual_motion(target_velocity_vision, 0, target_steering_vision);
        // manual_motion(target_velocity_joy_x, 0, target_steering);

        // control_steering(target_velocity_joy_x, -0.2189);

        // cnn_move(target_velocity_joy_x, 0, target_steering_cnn, profile_max_velocity, target_steering_cnn);
        // cnn_move2(target_velocity_joy_x, 0, target_steering_cnn, profile_max_velocity, target_steering_cnn);
        // urban_move(target_velocity_joy_x, 0, target_steering_cnn, profile_max_velocity, apriltag_status, target_steering_cnn);

        // obstacle_avoidance_move(target_velocity_joy_x, 0, target_velocity_joy_wz);
        break;

    case FSM_GLOBAL_RACE_BUTTON:
        manual_motion(target_velocity_vision, 0, target_steering_vision);
        break;
    case FSM_GLOBAL_OP_5:
        follow_waypoints_gas_manual(target_velocity_joy_x, 0, 0, default_lookahead, true);
        break;
    case 100: //? URBAN
        urban_move2(velocity_jalan_otomatis, 0, target_velocity_joy_wz);
        break;
    case 200: //? RACE
        race_move(target_velocity_joy_x, 0, target_velocity_joy_wz);
        break;
    case FSM_GLOBAL_RECORD_DATASET_ROAD:
        manual_motion(target_velocity_joy_x, 0, target_velocity_joy_wz);
        break;
    case 88:

        break;
    case 99:
        break;
    case FSM_GLOBAL_CUSTOM_DEBUG_1:
    {
        // logger.info("gryo: %.2f", fb_final_pose_xyo[2] * 180 / M_PI);
        // if (move_left(0.8, 0.75, 90)) {
        //     manual_motion(-0.01, 0, 0);
        // }
        manual_motion(target_velocity_joy_x, 0, target_steering_vision);
        break;
    }
    case FSM_GLOBAL_CUSTOM_DEBUG_2:
    {

        static float pos_awal = enc_meter;

        static MachineState fsm;
        fsm.reentry(0, 0.5);

        logger.info("FSM : %d || %.2f", fsm.value, enc_meter - pos_awal);
        switch (fsm.value)
        {
        case 0:
            pos_awal = enc_meter;
            fsm.value = 1;
            break;
        case 1:
            if (enc_meter - pos_awal > 1.0)
                fsm.value = 2;
            else
                manual_motion(target_velocity_joy_x, 0, target_velocity_joy_wz);
            break;
        case 2:
            manual_motion(0, 0, target_velocity_joy_wz);
            break;
        }
        break;
    }
    case 32:
        manual_motion(0, 0, 0);
        break;
    case FSM_GLOBAL_RECORD_ROUTE:
        manual_motion(target_velocity_joy_x, 0, target_velocity_joy_wz);
        process_record_route();
        break;
    case FSM_GLOBAL_RECORD_ROUTE_KANAN:
        manual_motion(target_velocity_joy_x, 0, target_velocity_joy_wz);
        process_record_route(waypoints_race_kanan);
        break;
    case FSM_GLOBAL_RECORD_ROUTE_KIRI:
        manual_motion(target_velocity_joy_x, 0, target_velocity_joy_wz);
        process_record_route(waypoints_race_kiri);
        break;
    case FSM_GLOBAL_RECORD_ROUTE_TENGAH:
        manual_motion(target_velocity_joy_x, 0, target_velocity_joy_wz);
        process_record_route(waypoints_race_tengah);
        break;
    }

    process_transmitter();

    global_fsm.prev_value = global_fsm.value;
    local_fsm.prev_value = local_fsm.value;
}

void Master::callback_sub_key_pressed(const std_msgs::msg::Int16::SharedPtr msg)
{
    switch (msg->data)
    {
    case 'j':
        target_velocity_joy_x = 0.5;
        break;
    case 'n':
        target_velocity_joy_x = -0.5;
        break;
    case 'b':
        target_velocity_joy_wz = 0.076928521;
        break;
    case 'm':
        target_velocity_joy_wz = -0.076928521;
        break;
    case ' ':
        target_velocity_joy_x = 0;
        target_velocity_joy_wz = 0;
        break;
    case 'u':
        target_velocity_joy_x = 1.5;
        break;

    case '0':
        global_fsm.value = FSM_GLOBAL_INIT;
        break;
    case '1':
        global_fsm.value = FSM_GLOBAL_PREOP;
        break;
    case '2':
        global_fsm.value = FSM_GLOBAL_SAFEOP;
        break;
    case '3':
        global_fsm.value = FSM_GLOBAL_OP_3;
        break;
    case '4':
        global_fsm.value = FSM_GLOBAL_OP_4;
        break;
    case '5':
        global_fsm.value = FSM_GLOBAL_OP_5;
        break;
    case '6':
        global_fsm.value = FSM_GLOBAL_OP_2;
        break;
    }
}

void Master::callback_sub_key_web_pressed(const std_msgs::msg::Int16::SharedPtr msg)
{
    key_web_pressed = msg->data;
}

void Master::callback_sub_ui_control_velocity_and_steering(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    target_velocity_joy_x = msg->data[0];
    target_velocity_joy_wz = msg->data[1];
}

void Master::callback_sub_selected_lane(const std_msgs::msg::Int8::SharedPtr msg)
{
    // logger.info("Selected lane: %d", msg->data);
    selected_lane = msg->data;
}

void Master::callback_sub_steering_vision(const std_msgs::msg::Float32::SharedPtr msg)
{
    target_steering_vision = msg->data * M_PI / 180.0; // Convert degrees to radians
    // logger.info("Target steering vision: %.2f %f", msg->data, target_steering_vision);
}

void Master::callback_sub_velocity_vision(const std_msgs::msg::Float32::SharedPtr msg)
{
    target_velocity_vision = msg->data; // Convert degrees to radians
    // logger.info("Target velocity vision: %f", target_velocity_vision);
}

void Master::callback_sub_speed_cnn(const std_msgs::msg::Float32::SharedPtr msg)
{
    target_speed_cnn = msg->data;
}

void Master::callback_sub_odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    tf2::Quaternion q;
    double roll, pitch, yaw;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    fb_final_pose_xyo[0] = msg->pose.pose.position.x;
    fb_final_pose_xyo[1] = msg->pose.pose.position.y;
    fb_final_pose_xyo[2] = yaw;

    fb_final_vel_dxdydo[0] = msg->twist.twist.linear.x;
    fb_final_vel_dxdydo[1] = msg->twist.twist.linear.y;
    fb_final_vel_dxdydo[2] = msg->twist.twist.angular.z;
}

void Master::callback_sub_nearest_obstacle(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    // nearest_obstacle = *msg;
}

// GET POINTCLOUD OBSTACLE IN BASE LINK
void Master::callback_sub_filtered_camera(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    auto start = std::chrono::steady_clock::now();

    pcl::fromROSMsg(*msg, points_obstacle2base);

    static const size_t max_obs_buffer_size = 10; // Maximum number of buffered obstacles
    pcl_timed_t obs_buffered_timed;

    buffered_obs.clear();
    ada_obs = 0;
    auto now = this->now();

    if (!points_obstacle2base.points.empty() && points_obstacle2base.points.size() > 100)
    {
        obs_buffered_timed.cloud = points_obstacle2base;
        obs_buffered_timed.stamp = now;

        if (obs_pcl_buffer.size() >= max_obs_buffer_size)
            obs_pcl_buffer.erase(obs_pcl_buffer.begin()); // Remove the oldest entry

        obs_pcl_buffer.push_back(obs_buffered_timed);
    }

    obs_pcl_buffer.erase(
        std::remove_if(obs_pcl_buffer.begin(), obs_pcl_buffer.end(),
                       [now](const pcl_timed_t &obs)
                       { return (now - obs.stamp).seconds() > 2.0; }),
        obs_pcl_buffer.end());

    // // masukkan semua point cloud yang ada di buffer ke dalam suatu point cloud yang besar
    if (!obs_pcl_buffer.empty())
    {
        for (const auto &obs : obs_pcl_buffer)
            buffered_obs += obs.cloud; // Concatenate point clouds

        // Set the header for the combined point cloud
        // buffered_obs.header.stamp = obs_pcl_buffer.back().stamp.nanoseconds() / 1e6; // Convert to milliseconds
        // buffered_obs.header.frame_id = points_obstacle2base.header.frame_id;
    }

    // filter voxel grid
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(buffered_obs.makeShared());
    voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f); // Set leaf size for downsampling
    pcl::PointCloud<pcl::PointXYZ> filtered_obs;
    voxel_filter.filter(filtered_obs);
    buffered_obs = filtered_obs;

    if (!points_obstacle2base.points.empty())
    {
        // pcl::compute3DCentroid(buffered_obs, obstacle_centroid);
        pcl::compute3DCentroid(points_obstacle2base, obstacle_centroid);

        obstacle_centroid2base.x() = obstacle_centroid.x();
        obstacle_centroid2base.y() = obstacle_centroid.y();
        obstacle_centroid2base.z() = 0.0; // Assuming z-coordinate is

        float maggg = sqrtf(obstacle_centroid.x() * obstacle_centroid.x() + obstacle_centroid.y() * obstacle_centroid.y());
        float arah = atan2f(obstacle_centroid.y(), obstacle_centroid.x());
        float x_obs_ = maggg * cosf(fb_final_pose_xyo[2] + arah) + fb_final_pose_xyo[0];
        float y_obs_ = maggg * sinf(fb_final_pose_xyo[2] + arah) + fb_final_pose_xyo[1];

        // obstacle_centroid.x() += fb_final_pose_xyo[0];
        // obstacle_centroid.y() += fb_final_pose_xyo[1];

        obstacle_centroid.x() = x_obs_;
        obstacle_centroid.y() = y_obs_;

        // obstacle_centroid.x() = obstacle_centroid.x() + fb_final_pose_xyo[0] * sin(fb_final_pose_xyo[2]) - fb_final_pose_xyo[1] * cos(fb_final_pose_xyo[2]);
        // obstacle_centroid.y() = obstacle_centroid.y() + fb_final_pose_xyo[0] * cos(fb_final_pose_xyo[2]) + fb_final_pose_xyo[1] * sin(fb_final_pose_xyo[2]);
        obstacle_centroid.z() = 0.0; // Assuming z-coordinate is not relevant for obstacles
    }
    else
    {
        obstacle_centroid.x() = 9999.0;
        obstacle_centroid.y() = 0.0;
        obstacle_centroid.z() = 9999.0;

        obstacle_centroid2base.x() = obstacle_centroid.x();
        obstacle_centroid2base.y() = obstacle_centroid.y();
        obstacle_centroid2base.z() = 9999.0; // Assuming z-coordinate is
    }

    // RCLCPP_INFO(this->get_logger(), "Processing time Of Obs : %ld ms", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count());

    // // ================== TRANSFORM TO MAP FRAME ==================
    // // pcl_ros::transformPointCloud(buffered_obs, buffered_obs2map, tf_base2map);

    // // publish the buffered obstacle point cloud
    // sensor_msgs::msg::PointCloud2 obs_msg;
    // pcl::toROSMsg(buffered_obs, obs_msg);
    // obs_msg.header.stamp = this->now();
    // obs_msg.header.frame_id = points_obstacle2base.header.frame_id;
    // pub_buffered_obs_pointcloud->publish(obs_msg);

    nav_msgs::msg::Odometry msg_odom;
    msg_odom.header.stamp = this->now();
    msg_odom.header.frame_id = "asa";
    msg_odom.child_frame_id = "asddas";
    msg_odom.pose.pose.position.x = obstacle_centroid.x();
    msg_odom.pose.pose.position.y = obstacle_centroid.y();
    pub_nearest_obstacle->publish(msg_odom);

    // logger.info("centroid obstacle: [%.2f, %.2f, %.2f] with %zu points",
    //             obstacle_centroid[0], obstacle_centroid[1], obstacle_centroid[2], buffered_obs.points.size());
}

// GET POINTCLOUD ROAD IN BASE LINK
void Master::callback_sub_road_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    auto start = std::chrono::steady_clock::now();

    pcl::fromROSMsg(*msg, points_road2base);

    static const size_t max_road_buffer_size = 5; // Maximum number of buffered road
    pcl_timed_t road_buffered_timed;

    edge_left_filtered.clear();
    edge_right_filtered.clear();
    buffered_road.clear();

    auto now = this->now();

    if (!points_road2base.points.empty())
    {
        // Buffer the road points
        road_buffered_timed.cloud = points_road2base;
        road_buffered_timed.stamp = now;

        if (road_pcl_buffer.size() >= max_road_buffer_size)
            road_pcl_buffer.erase(road_pcl_buffer.begin()); // Remove the oldest entry

        road_pcl_buffer.push_back(road_buffered_timed);
    }

    road_pcl_buffer.erase(
        std::remove_if(road_pcl_buffer.begin(), road_pcl_buffer.end(),
                       [now](const pcl_timed_t &road)
                       { return (now - road.stamp).seconds() > 3.0; }),
        road_pcl_buffer.end());

    if (!road_pcl_buffer.empty())
    {
        for (const auto &road : road_pcl_buffer)
            buffered_road += road.cloud; // Concatenate point clouds

        // Set the header for the combined point cloud
        // buffered_road.header.stamp = road_pcl_buffer.back().stamp.nanoseconds() / 1e6; // Convert to milliseconds
        // buffered_road.header.stamp = rclcpp::Time(road_pcl_buffer.back().stamp).to_builtin();

        // buffered_road.header.frame_id = points_road2base.header.frame_id;
    }

    // filter voxel grid
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(buffered_road.makeShared());
    voxel_filter.setLeafSize(0.02f, 0.02f, 0.02f); // Set leaf size for downsampling
    pcl::PointCloud<pcl::PointXYZ> filtered_road;
    voxel_filter.filter(filtered_road);
    buffered_road = filtered_road;

    // Pisahkan edge ke kiri dan kanan
    if (!buffered_road.points.empty())
    {
        edge_left.clear();
        edge_right.clear();

        for (const auto &pt : buffered_road.points)
            if (pt.y > 0.0)
                edge_left.push_back(pt);
            else
                edge_right.push_back(pt);
    }

    float left_distance_min = std::numeric_limits<float>::max();
    float right_distance_min = std::numeric_limits<float>::max();

    float left_distance_min_obs = std::numeric_limits<float>::max();
    float right_distance_min_obs = std::numeric_limits<float>::max();

    bool dekat_obs = (pythagoras(obstacle_centroid2base.x(), obstacle_centroid2base.y(), 0, 0) < 2.0);

    if (!edge_left.empty())
    {
        for (const auto &pt : edge_left.points)
        {

            float distance = std::hypot(pt.x, pt.y);
            if (distance < left_distance_min)
                left_distance_min = distance;

            if (dekat_obs)
            {
                float distance_obs = pythagoras(pt.x, pt.y, obstacle_centroid2base.x(), obstacle_centroid2base.y());
                if (distance_obs < left_distance_min_obs)
                    left_distance_min_obs = distance_obs;
            }
        }
    }

    if (!edge_right.empty())
    {
        for (const auto &pt : edge_right.points)
        {

            float distance = std::hypot(pt.x, pt.y);
            if (distance < right_distance_min)
                right_distance_min = distance;

            if (dekat_obs)
            {
                float distance_obs = pythagoras(pt.x, pt.y, obstacle_centroid2base.x(), obstacle_centroid2base.y());
                if (distance_obs < right_distance_min_obs)
                    right_distance_min_obs = distance_obs;
            }
        }
    }

    // if (edge_left_x_min < edge_right_x_max) {
    // logger.info("Edge left x min: %.2f, Edge right x max: %.2f", left_distance_min, right_distance_min);
    // }

    if (left_distance_min < right_distance_min)
    {
        // logger.info("robot di kiri");
        posisi_robot_di_kanan = 0; // Robot di kiri
    }
    else
    {
        // logger.info("robot di kanan");
        posisi_robot_di_kanan = 1; // Robot di kanan
    }
    // logger.info("jarak kiri: %.2f, jarak kanan: %.2f || %.2f", left_distance_min_obs, right_distance_min_obs, right_distance_min_obs + left_distance_min_obs);

    if (dekat_obs)
    {
        if (left_distance_min_obs > 99999 || right_distance_min_obs > 99999)
        {
            if ((left_distance_min_obs > 0.5 && left_distance_min_obs < 99999) || right_distance_min_obs < 0.35)
            {
                // logger.info("obs di kanan 1");
                posisi_obs_di_kanan = 1; // Obstacle di kanan
            }
            else if ((right_distance_min_obs > 0.5 && right_distance_min_obs < 99999) || left_distance_min_obs < 0.35)
            {
                // logger.info("obs di kiri 2");
                posisi_obs_di_kanan = 0; // Obstacle di kiri
            }
            else
            {
                // logger.info("wtff");
                posisi_obs_di_kanan = -1; // Tidak ada obstacle yang jelas
            }
        }
        else if (left_distance_min_obs < right_distance_min_obs)
        {
            // logger.info("kiriiiii dekat obs %.2f, kanan dekat obs %.2f",
            //             left_distance_min_obs, right_distance_min_obs);
            posisi_obs_di_kanan = 0; // Obstacle di kiri
        }
        else
        {
            // logger.info("kanan dekat obs %.2f, kiri dekat obs %.2f",
            //             right_distance_min_obs, left_distance_min_obs);
            posisi_obs_di_kanan = 1; // Obstacle di kanan
        }
    }
    else
    {
        // logger.info("tidak ada obstacle dekat");
    }

    // RCLCPP_INFO(this->get_logger(), "Processing time Of Road: %ld ms", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count());

    // publish posisi robot dan obstacle
    std_msgs::msg::Int8 msg_posisi_robot;
    msg_posisi_robot.data = posisi_robot_di_kanan;
    pub_posisi_robot->publish(msg_posisi_robot);

    std_msgs::msg::Int8 msg_posisi_obs;
    msg_posisi_obs.data = posisi_obs_di_kanan;
    pub_posisi_obstacle->publish(msg_posisi_obs);

    // sensor_msgs::msg::PointCloud2 road_msg;
    // pcl::toROSMsg(buffered_road, road_msg);
    // road_msg.header.stamp = this->now();
    // road_msg.header.frame_id = points_road2base.header.frame_id;
    // pub_buffered_road_pointcloud->publish(road_msg);

    // sensor_msgs::msg::PointCloud2 edge_left_msg;
    // sensor_msgs::msg::PointCloud2 edge_right_msg;
    // pcl::toROSMsg(edge_left, edge_left_msg);
    // pcl::toROSMsg(edge_right, edge_right_msg);
    // edge_left_msg.header.stamp = this->now();
    // edge_left_msg.header.frame_id = points_road2base.header.frame_id;
    // edge_right_msg.header.stamp = this->now();
    // edge_right_msg.header.frame_id = points_road2base.header.frame_id;
    // pub_edge_left_road_pointcloud->publish(edge_left_msg);
    // pub_edge_right_road_pointcloud->publish(edge_right_msg);
}

void Master::callback_sub_apriltag(const std_msgs::msg::Int32::SharedPtr msg)
{
    apriltag_status = msg->data;
}

void Master::callback_sub_sign_detection(const std_msgs::msg::Int32::SharedPtr msg)
{
    sign_detected_status = msg->data;
}

void Master::callback_sub_road_obs_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    pcl::fromROSMsg(*msg, points_road_obs2base);
}

void Master::callback_sub_enc_meter(const std_msgs::msg::Float32::SharedPtr msg)
{
    enc_meter += msg->data * 0.005;
}

void Master::callback_sub_vision_urban(const ros2_interface::msg::VisionUrban::SharedPtr msg)
{
    urban_data.berhenti = msg->berhenti;
    urban_data.pos_target_px_x = msg->pos_target_px_x;
    urban_data.pos_target_px_y = msg->pos_target_px_y;
    urban_data.pos_robot_px_x = msg->pos_robot_px_x;
    urban_data.pos_robot_px_y = msg->pos_robot_px_y;

    urban_data.dist_putih_meter = msg->dist_putih_meter;
    urban_data.dist_near_zebracross = msg->dist_near_zebracross;
    urban_data.target_angle_ungu = msg->target_angle_ungu;
    urban_data.target_angle_putih = msg->target_angle_putih;
    urban_data.meter_to_pixel = msg->meter_to_pixel;
    urban_data.offset_angle = msg->offset_angle_zebracross;
    urban_data.offset_angle_lane = msg->offset_angle_lane;
    urban_data.dist_near_zebracross_vertical = msg->dist_near_zebracross_vertical;
    urban_data.dist_near_zebracross_horizontal = msg->dist_near_zebracross_horizontal;
    urban_data.dist_near_zebracross_vertical_kiri = msg->dist_near_zebracross_vertical_kiri;
    urban_data.dist_near_zebracross_vertical_kanan = msg->dist_near_zebracross_vertical_kanan;

    urban_data.centroid_sign_x = msg->centroid_sign_x;
    urban_data.centroid_sign_y = msg->centroid_sign_y;

    urban_data.centroid_obs_x = msg->centroid_obs_x;
    urban_data.centroid_obs_y = msg->centroid_obs_y;

    urban_data.jalan_berkelok = msg->jalan_berkelok;
    urban_data.mask_jalan_bocor = msg->mask_jalan_bocor;
    urban_data.ada_pertigaan = msg->ada_pertigaan;
    urban_data.jarak_ke_pertigaan = msg->jarak_ke_pertigaan;

    // logger.info("centroid sign: [%.2f, %.2f] | target angle ungu: %.2f | target angle putih: %.2f",
    //             urban_data.centroid_sign_x, urban_data.centroid_sign_y,
    //             urban_data.target_angle_ungu, urban_data.target_angle_putih);

    //
}

void Master::callback_sub_toggle_debug(const std_msgs::msg::Int16::SharedPtr msg)
{
    debug_mode_ = msg->data;
    logger.info("Debug mode set to: %d", debug_mode_);
}

void Master::callback_sub_toggle_debug2(const std_msgs::msg::Int16::SharedPtr msg)
{
    debug_mode2_ = msg->data;
    logger.info("Debug mode set to: %d", debug_mode2_);
}

void Master::callback_sub_button(const std_msgs::msg::Int8::SharedPtr msg)
{
    button_1 = (msg->data >> 0) & 0x01;
    button_2 = (msg->data >> 1) & 0x01;
    toogle_button_1 = (msg->data >> 2) & 0x01;
    toogle_button_2 = (msg->data >> 3) & 0x01;
    static int prev_button_1 = button_1;

    static int8_t counter_toogle_1 = 0;
    static int8_t prev_toogle_1 = 0;

    if (prev_toogle_1 == 0 && toogle_button_1 == 1)
        counter_toogle_1++;

    if (prev_button_1 == 0 && button_1 == 1)
        global_fsm.value = 1;
    if (prev_button_1 == 1 && button_1 == 1)
        global_fsm.value = FSM_GLOBAL_RACE_BUTTON;

    prev_button_1 = button_1;
}

void Master::callback_sub_vision_urban_config(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    for (size_t i = 0; i < msg->data.size(); ++i)
    {
        derajat_steering_kanan_ = msg->data[14];
        derajat_steering_kiri_ = msg->data[15];
        encoder_belok_kanan_ = msg->data[16];
        encoder_belok_kiri_ = msg->data[17];
        encoder_maju_kanan_ = msg->data[18];
        encoder_maju_kiri_ = msg->data[19];
        encoder_maju_lurus_ = msg->data[20];
        jarak_ke_putih_ = msg->data[21];
        derajat_gyro_kanan_ = msg->data[22];
        derajat_gyro_kiri_ = msg->data[23];
        jarak_ke_zebracros_ = msg->data[26];
        min_jarak_putih_kanan_ = msg->data[27];
        min_jarak_putih_kiri_ = msg->data[28];
        min_jarak_putih_lurus_ = msg->data[29];
        encoder_maju_dead_end_ = msg->data[30];  // Default value for dead end maneuver
        velocity_jalan_otomatis = msg->data[31]; // Default velocity for automatic driving
        offset_jarak_sign_pole_ = msg->data[34]; // Offset distance to sign pole
        last_gyro_angle_ = msg->data[35];        // Last gyro angle for sign detection
        min_vel_belokan_ = msg->data[37];        // Minimum velocity for cornering
        jarak_ke_sign_pole_ = msg->data[38];     // Distance to sign pole
    }

    logger.info("INIT %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
                derajat_steering_kanan_, derajat_steering_kiri_,
                encoder_belok_kanan_, encoder_belok_kiri_,
                encoder_maju_kanan_, encoder_maju_kiri_,
                encoder_maju_lurus_, jarak_ke_putih_,
                derajat_gyro_kanan_, derajat_gyro_kiri_);
}

void Master::callback_sub_sign_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    pcl::fromROSMsg(*msg, points_sign2base);

    // check if the point size is greater than 100
    if (points_sign2base.size() > 10)
    {
        pcl::compute3DCentroid(points_sign2base, sign_centroid);
    }
    else
    {
        sign_centroid.x() = 9999.0;
        sign_centroid.y() = 9999.0;
        sign_centroid.z() = 9999.0;
    }
}

void Master::callback_srv_set_record_route_mode(const std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response)
{
    response->success = false;
    if (request->data)
    {
        waypoints.clear();
        global_fsm.value = FSM_GLOBAL_RECORD_ROUTE;
        response->message = "Record route mode enabled";
        response->success = true;
    }
    else
    {
        global_fsm.value = FSM_GLOBAL_INIT;
        response->message = "Record route mode disabled";
        response->success = true;
    }
}

void Master::callback_srv_set_record_route_mode_kanan(const std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response)
{
    response->success = false;
    if (request->data)
    {
        waypoints_race_kanan.clear();
        global_fsm.value = FSM_GLOBAL_RECORD_ROUTE_KANAN;
        response->message = "Record route kanan mode enabled";
        response->success = true;
    }
    else
    {
        global_fsm.value = FSM_GLOBAL_INIT;
        response->message = "Record route mode disabled";
        response->success = true;
    }
}

void Master::callback_srv_set_record_route_mode_kiri(const std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response)
{
    response->success = false;
    if (request->data)
    {
        waypoints_race_kiri.clear();
        global_fsm.value = FSM_GLOBAL_RECORD_ROUTE_KIRI;
        response->message = "Record route kiri mode enabled";
        response->success = true;
    }
    else
    {
        global_fsm.value = FSM_GLOBAL_INIT;
        response->message = "Record route mode disabled";
        response->success = true;
    }
}

void Master::callback_srv_set_record_route_mode_tengah(const std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response)
{
    response->success = false;
    if (request->data)
    {
        waypoints_race_tengah.clear();
        global_fsm.value = FSM_GLOBAL_RECORD_ROUTE_TENGAH;
        response->message = "Record route tengah mode enabled";
        response->success = true;
    }
    else
    {
        global_fsm.value = FSM_GLOBAL_INIT;
        response->message = "Record route mode disabled";
        response->success = true;
    }
}

void Master::callback_srv_set_add_record_route_mode(const std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response)
{
    response->success = false;
    if (request->data)
    {
        global_fsm.value = FSM_GLOBAL_RECORD_ROUTE;
        response->message = "Record route mode enabled";
        response->success = true;
    }
    else
    {
        global_fsm.value = FSM_GLOBAL_INIT;
        response->message = "Record route mode disabled";
        response->success = true;
    }
}

void Master::callback_srv_rm_terminal(const std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response)
{
    response->success = false;
    if (request->data)
    {
        terminals.terminals.clear();
        response->message = "Success pop terminal";
        response->success = true;
    }
    else
    {
        terminals.terminals.pop_back();
        response->message = "Success clear terminals";
        response->success = true;
    }
}
void Master::callback_srv_set_terminal(const std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response)
{
    response->success = false;
    if (request->data)
    {
        process_add_terminal();
        response->message = "Success add terminal";
        response->success = true;
    }
    else
    {
        process_save_terminals();
        response->message = "Success save terminals";
        response->success = true;
    }
}
void Master::callback_srv_set_terminal_sign(const std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response)
{
    response->success = false;
    if (request->data)
    {
        process_add_terminal_sign();
        response->message = "Success add terminal";
        response->success = true;
    }
    else
    {
        process_save_terminals();
        response->message = "Success save terminals";
        response->success = true;
    }
}

void Master::callback_sub_joy(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    if (msg->axes.size() >= 2)
    {
        target_velocity_joy_x = msg->axes[1] * profile_max_velocity * 0.5;

        target_velocity_joy_wz = msg->axes[2] * profile_max_steering_rad;

        if ((target_velocity_joy_wz >= profile_max_steering_rad - 0.001) && (target_velocity_joy_x >= profile_max_velocity * 0.45))
            target_velocity_joy_x = msg->axes[1] * profile_max_velocity * 0.9;
    }

    if (msg->buttons[6] == 1 && global_fsm.value != FSM_GLOBAL_RECORD_DATASET_ROAD)
    {
        global_fsm.value = FSM_GLOBAL_RECORD_DATASET_ROAD;
        logger.info("Switched to FSM_GLOBAL_RECORD_DATASET_ROAD");
    }
    else if (msg->buttons[4] == 1 && global_fsm.value == FSM_GLOBAL_RECORD_DATASET_ROAD)
    {
        global_fsm.value = FSM_GLOBAL_INIT;
        logger.info("Switched to FSM_GLOBAL_INIT");
    }
}

void Master::callback_sub_target_steering(const std_msgs::msg::Float32::SharedPtr msg)
{
    target_steering = msg->data;
    // logger.info("Target steering angle: %f", target_steering);
}

void Master::callback_sub_detected_apriltag(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
    // logger.info("Detected AprilTag markers: %zu", msg->markers.size());

    if (msg->markers.empty())
    {
        // logger.warn("No AprilTag markers detected");
        return;
    }

    const auto &marker = msg->markers[0];
    detected_apriltag.id = marker.id;
    detected_apriltag.x = marker.pose.position.x;
    detected_apriltag.y = marker.pose.position.y;
    detected_apriltag.z = marker.pose.position.z;
    detected_apriltag.dist = sqrt(detected_apriltag.x * detected_apriltag.x + detected_apriltag.y * detected_apriltag.y + detected_apriltag.z * detected_apriltag.z);
    detected_apriltag.roll = marker.pose.orientation.x;
    detected_apriltag.pitch = marker.pose.orientation.y;
    detected_apriltag.yaw = marker.pose.orientation.z;
    detected_apriltag.t = marker.header.stamp.sec + marker.header.stamp.nanosec * 1e-9;

    static float prev_x = detected_apriltag.x;
    static float prev_y = detected_apriltag.y;

    // low pass filter
    detected_apriltag.x = 0.9 * detected_apriltag.x + 0.1 * prev_x;
    detected_apriltag.y = 0.9 * detected_apriltag.y + 0.1 * prev_y;

    prev_x = detected_apriltag.x;
    prev_y = detected_apriltag.y;

    // logger.info("Detected AprilTag ID: %d, Position: (%.2f, %.2f, %.2f), Distance: %.2f, Orientation: (%.2f, %.2f, %.2f), Timestamp: %.2f",
    //     detected_apriltag.id, detected_apriltag.x, detected_apriltag.y, detected_apriltag.z,
    //     detected_apriltag.dist, detected_apriltag.roll, detected_apriltag.pitch, detected_apriltag.yaw,
    //     detected_apriltag.t);
}

void Master::callback_sub_set_master_fsm(const std_msgs::msg::Int16::SharedPtr msg)
{
    global_fsm.value = msg->data;
}

void Master::callback_sub_pointcloud_laser_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_master = std::make_shared<Master>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_master);
    executor.spin();

    return 0;
}
