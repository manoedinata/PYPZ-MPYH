import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch.actions import EmitEvent, RegisterEventHandler, TimerAction, DeclareLaunchArgument


from ament_index_python.packages import get_package_share_directory

path_config_buffer = os.getenv('AMENT_PREFIX_PATH', '')
path_config_buffer_split = path_config_buffer.split(":")
ws_path = path_config_buffer_split[0] + "/../../"
path_config = ws_path + "src/ros2_utils/configs/"

def generate_launch_description():
    
    # SetEnvironmentVariable(name='RMW_IMPLEMENTATION', value='rmw_cyclonedds_cpp'),
    # SetEnvironmentVariable(name='CYCLONEDDS_URI', value='file://' + path_config + 'cyclonedds.xml'),

    # =============================================================================================

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        # fmt: off
        arguments=["-d",os.path.join(path_config,"robot.rviz"),
                   "--ros-args","--log-level","error",]
        # fmt: on
    )

    rosbridge_server = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        respawn=True,
    )

    


    web_video_server = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen',
        respawn=True,
    )

    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi_node',
        output='screen',
        respawn=True,
    )

    rs2_cam_main = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="rs2_cam_main",
        parameters=[
            {
                "camera_name": "camera",
                "camera_namespace": "",
                "enable_accel": False,
                "enable_gyro": False,
                "enable_depth": True,
                "enable_color": True,
                "enable_sync": True,
                "unite_imu_method": 2,
                "align_depth.enable": True,
                "pointcloud.enable": True,

                # "rgb_camera.color_profile": "640x480x60",

                # "rgb_camera.auto_exposure_priority": False,
                # "rgb_camera.auto_exposure_roi": {   
                #     "top": 0,
                #     "bottom": 480,
                #     "left": 0,
                #     "right": 640
                # },
                # "rgb_camera.backlight_compensation": False,
                # "rgb_camera.brightness": 0,
                # "rgb_camera.color_format": "RGB8",
                # "rgb_camera.contrast": 50,
                # "rgb_camera.enable_auto_exposure": False,
                # "rgb_camera.enable_auto_white_balance": False,
                # "rgb_camera.exposure": 156,  # Fixed exposure value (adjust as needed)
                # "rgb_camera.frames_queue_size": 30,
                # "rgb_camera.gain": 64,  # Fixed gain value (adjust as needed)
                # "rgb_camera.gamma": 100,  # Fixed gamma value (adjust as needed)
                # "rgb_camera.global_time_enabled": True,
                # "rgb_camera.hue": 0,  # Fixed hue value
                # "rgb_camera.power_line_frequency": "Disabled",
                # "rgb_camera.saturation": 64,  # Fixed saturation
                # "rgb_camera.sharpness": 50,  # Fixed sharpness
                # "rgb_camera.white_balance": 4600,  # Fixed white balance (adjust as needed)

                # "depth_module.depth_profile": "640x480x60",

                # "depth_module.auto_exposure_limit": 1000,  # Fixed auto exposure limit
                # "depth_module.auto_exposure_limit_toggle": False,
                # "depth_module.auto_exposure_mode": "Manual",
                # "depth_module.auto_exposure_roi": {
                #     "top": 0,
                #     "bottom": 480,
                #     "left": 0,
                #     "right": 640
                # },
                # "depth_module.auto_gain_limit": 100,  # Fixed auto gain limit
                # "depth_module.auto_gain_limit_toggle": False,
                # "depth_module.depth_format": "Z16",
                # "depth_module.emitter_always_on": False,
                # "depth_module.emitter_enabled": True,
                # "depth_module.emitter_frequency": 100,
                # "depth_module.emitter_on_off": True,
                # "depth_module.enable_auto_exposure": False,
                # "depth_module.error_polling_enabled": True,
                # "depth_module.exposure": 156,  # Fixed exposure value (adjust as needed)
                # "depth_module.frames_queue_size": 30,
                # "depth_module.gain": 64,  # Fixed gain value (adjust as needed)
                # "depth_module.global_time_enabled": True,
                # "depth_module.hdr_enabled": False,
                # "depth_module.infra1_format": "Y8",
                # "depth_module.infra2_format": "Y8",
                # "depth_module.infra_format": "Y8",
                # "depth_module.infra_profile": "640x480x90",
                # "depth_module.inter_cam_sync_mode": "None",
                # "depth_module.laser_power": 360,
                # "depth_module.output_trigger_enabled": False,
                # "depth_module.sequence_id": 0,
                # "depth_module.sequence_name": "",
                # "depth_module.sequence_size": 0,
                # "depth_module.thermal_compensation": False,
                # "depth_module.visual_preset": "High Accuracy",
            }
        ],
        arguments=["--ros-args", "--log-level", "error"],
        respawn=True,
        # prefix='nice -n -20 chrt -f 99',
    )

    wit_ros2_imu = Node(
        package='wit_ros2_imu',
        executable='wit_ros2_imu',
        parameters=[{'port': '/dev/imu_usb'},
                    {"baud": 115200}],
        remappings=[('/imu/data_raw', '/hardware/imu')],
        output="screen"

    )

    # =============================================================================================

    rtabmap_slam_rtabmap = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        namespace="slam",
        remappings=[
            ('rgb/image', '/camera/rs2_cam_main/color/image_raw'),
            ('rgb/camera_info', '/camera/rs2_cam_main/color/camera_info'),
            ('depth/image', '/camera/rs2_cam_main/aligned_depth_to_color/image_raw')
        ],
        arguments=["--ros-args", "--log-level", "warn"],
        respawn=True,
    )

    # =============================================================================================

    wifi_control = Node(
        package="communication",
        executable="wifi_control",
        name="wifi_control",
        parameters=[
            {
                "hotspot_ssid": "gh_template",
                "hotspot_password": "gh_template",
            },
        ],
        output="screen",
        respawn=True,
    )

    telemetry = Node(
        package="communication",
        executable="telemetry.py",
        name="telemetry",
        parameters=[{
            "INFLUXDB_URL": "http://172.30.37.21:8086",
            "INFLUXDB_USERNAME": "awm462",
            "INFLUXDB_PASSWORD": "wildan462",
            "INFLUXDB_ORG": "awmawm",
            "INFLUXDB_BUCKET": "ujiCoba",
            "ROBOT_NAME": "gh_template",
        }],
        output="screen",
        respawn=True,
    )

    # =============================================================================================

    ui_server = Node(
        package="web_ui",
        executable="ui_server.py",
        name="ui_server",
        parameters=[
            {
                "ui_root_path": os.path.join(ws_path,"src/web_ui/src")
            },
        ],
        output="screen",
        respawn=True,
    )

    # =============================================================================================

    master = Node(
        package='master',
        executable='master',
        name='master',
        output='screen',
        respawn=True,
        parameters= [
            {
                "profile_max_acceleration" : 2000.0,
                "profile_max_decceleration" : 2000.0,
                "profile_max_velocity" : 1.5, 
                "profile_max_accelerate_jerk" : 30000.0,
                "profile_max_decelerate_jerk" : 30000.0,
                "profile_max_braking" : 3.0,
                "profile_max_braking_acceleration" : 10000.0,
                "profile_max_braking_jerk" : 1000.0,
                "profile_max_steering_rad" : 0.52, 
                "wheelbase" : 0.27,
                "default_lookahead": 0.8,

                "lama_waktu_menghindar" : 40,
                "kecepatan_default_menghindar" : 0.7,
                "offset_jarak_hindar" : 0.4,
                
                "max_counter_lurus" : 5,
                "max_counter_belok_kiri" : 60,
                "max_counter_belok_kanan" : 75,
                "max_counter_lurus_awal_kiri" : 15,
                "max_counter_lurus_awal_kanan" : 35,
                
                "waypoint_file_path" : "/home/iris/waypoints.yaml",
                "waypoint_file_path_kanan" : "/home/iris/waypoints_race_kanan.yaml",
                "waypoint_file_path_kiri" : "/home/iris/waypoints_race_kiri.yaml",
                "waypoint_file_path_tengah" : "/home/iris/waypoints_race_tengah.yaml",

                "terminal_file_path" : "/home/iris/terminal.yaml",
            }
        ],
        # prefix='nice -n -8'
    )

    # =============================================================================================

    CANbus_HAL = Node(
        package='hardware',
        executable='CANbus_HAL',
        name='CANbus_HAL',
        output='screen',
        parameters=[
            {
                "routine_period_ms": 10,
                "max_can_recv_error_counter": 10,
                "can_timeout_us": 50000,
                "bitrate": 1000000,
                "id_driver_wheel" : 4,
                "id_driver_steering" : 8,
            }
        ],
        respawn=True,
        # prefix='nice -n -10',
        # prefix='nice -n -20 chrt -f 98'
    )

    imu_serial = Node(
        package="hardware",
        executable="serial_imu",
        name="serial_imu",
        output="screen",
        parameters=[{
            # "port": "/dev/ttyUSB0",
            # "port": "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
            "use_boost": False,

            "is_riontech": True, 
            "baudrate": 115200,
            "port": "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0"
        }],
        respawn=True,
        # prefix='nice -n -20 chrt -f 90'
    )


    motor_main = Node(
        package='hardware',
        executable='motor_main',
        name='motor_main',
        output='screen',
        respawn=True,
        parameters=[
            {
                "routine_period_ms": 20,
                "k_p_wheel": 1700.0,
                "k_i_wheel": 0.0,
                "wheel_radius": 0.0365,
                "encoder_ppr": 147906.25,
                # "steering_dutyCycle2rad" : 0.011535, 
                # "offset_sudut_steering_rad" : 9.80475,
                # "offset_sudut_steering_rad" : 0.1,
                # "encoder_to_meter" : 0.000001600758,
                "encoder_to_meter" : -0.00000490586,
                "max_steering_pwm" : 1100.0,
                "min_steering_pwm" : 900.0,
                "mid_steering_pwm" : 1000.0,
                "max_steering_pwm_rad" : 0.437,
                "min_steering_pwm_rad" : -0.437,
            }
        ],
        # prefix='nice -n -10',
    )

    keyboard_input = Node(
        package='hardware',
        executable='keyboard_input',
        name='keyboard_input',
        output='screen',
        respawn=True,
        prefix=['xterm -e'],
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='log',
        respawn=True,
        parameters=[
        ],
        remappings=[
            ('/joy', '/hardware/joy')
        ],
        # prefix='nice -n -9'
    )

    # =============================================================================================

    pose_estimator = Node(
        package='world_model',
        executable='pose_estimator',
        name='pose_estimator',
        output='screen',
        parameters=[{
            # "encoder_to_meter" : 0.0000015651508,
            "encoder_to_meter" : -0.00000490586,

        }],
        respawn=True,
        # prefix='nice -n -9'
    )

    occupancy_grid = Node(
        package='world_model',
        executable='occupancy_grid',
        name='occupancy_grid',
        output='screen',
        parameters=[{
          "res" : 0.03,               # meter
          "width" : 200,              # meter
          "height" : 200,             # meter
          "ox" : -1.0, 
          "oy" : -1.0,                # Origin, agar robot di tengah grid
          "memory_timeout_sec" : 5.0,
          "blind_spot_radius" : 0.2,
        }],
        respawn=True,
        # prefix='nice -n -9'
    )

    # =============================================================================================

    vision_capture = Node(
        package='vision',
        executable='vision_capture',
        name='vision_capture',
        output='screen',
        respawn=True,
        parameters=[
            {
                "model_path": "/home/iris/model.onnx",
            }
        ],
        prefix='nice -n -8'
    )

    onnx_inference_node = Node(
        package='vision',
        executable='onnx_inference_node',
        name='onnx_inference_node',
        output='screen',
        respawn=True,
        parameters=[
            {
                "model_path": "/home/iris/model_19_juni.onnx",
                "use_temporal_model" : 0, 
            }
        ],

        prefix='nice -n -8'
        
    )

    detection = Node(
        package='vision',
        executable='detection',
        name='detection',
        output='screen',
        parameters=[
            {
                "publish_pointcloud2": True,
            }
        ],
        respawn=True,
        # prefix='nice -n -8'
    )

    detection2 = Node(
        package='vision',
        executable='detection2',
        name='detection2',
        output='screen',
        parameters=[
            {
            }
        ],
        respawn=True,
        # prefix='nice -n -8'
    )

    lane_detection = Node(
        package='vision',
        executable='lane_detection',
        name='lane_detection',
        output='screen',
        parameters=[
            {
                "publish_pointcloud2": True,
            }
        ],
        respawn=True,
        prefix='nice -n -8'
    )

    apriltag_detection = Node(
        package='apriltag_detection',
        executable='apriltag_detection',
        name='apriltag_detection',
        output='screen',
        respawn=True,
        parameters=[
            {

            }
        ],
    )

    apriltag3 = Node(
        package='vision',
        executable='apriltag3',
        name='apriltag3',
        output='screen',
        respawn=True,
        parameters=[
            {
                "apriltag_size" : 0.88,  
                "apriltag_quad_decimate" : 2.0,  
                "apriltag_blur" : 0.0,  
                "apriltag_nthreads" : 2,  

                # "apriltag_debug" : False,  
                # "apriltag_refine_edges" : True,  
            }
        ],
    )

    yolo_detection = Node(
        package='apriltag_detection',
        executable='yolo_detection',
        name='yolo_detection',
        output='screen',
        respawn=True,
        parameters=[
            {
                "model_path": "/home/iris/best.pt",
                "confidence_threshold": 0.6,
                "iou_threshold": 0.7,
            }
        ],
    )

    onnx_lane_detection = Node(
        package='apriltag_detection',
        executable='onnx_lane_detection',
        name='onnx_lane_detection',
        output='screen',
        respawn=True,
        parameters=[
            {
                "onnx_model_path": "/home/iris/best224.onnx",
                "look_ahead_distance": 300,  # in pixels
                "conf_threshold": 0.5,  # Confidence threshold for detection
                "min_box_area": 1000,  # Minimum area of the bounding box to consider it valid
            }
        ],
    )

    # =============================================================================================

    rtabmap_slam_rtabmap3 = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        namespace="slam",
        parameters=[
            {
                "frame_id": "base_link",
                "map_frame_id": "map",
                "odom_frame_id": "odom",
                "subscribe_depth": True,
                "subscribe_scan": True,
                "subscribe_scan_cloud": False,
                "subscribe_stereo": False,
                "subscribe_rgbd": False,
                "subscribe_rgb": False,
                "subscribe_Odometry": True,
                'scan_max_range': 3.0,  # LiDAR max range (meters)
                'scan_voxel_size': 0.04,  # Downsampling resolution (meters)
                "qos_scan": 1,
                "wait_for_transform": 2.0,

                "odom_tf_linear_variance": 0.0000000001,
                "odom_tf_angular_variance": 0.0000000001,
                "publish_tf": False,
                "publish_map": True,
                "approx_sync": True,
                "use_saved_map": False,
                "sync_queue_size": 30,
                "topic_queue_size": 10,
                "icp_odometry": False, 
                "visual_odometry": False,

                "Rtabmap/DetectionRate": "3.0", # Added by Azzam
                "Rtabmap/CreateIntermediateNodes": "True",
                "Rtabmap/LoopThr": "0.15", # Routine period untuk cek loop closure

                "Mem/STMSize": "30",  # Short-term memory size
                "Mem/IncrementalMemory": "False",  # 
                "Mem/InitWMWithAllNodes": "True",  # 
                "Mem/RehearsalSimilaritys": "0.97",  #
                "Mem/UseOdomFeatures": "True",  #
                'Mem/NotLinkedNodesKept': "True",  # Keep unlinked nodes
                # "Mem/UseOdomFeatures": "False", # (percobaan) untuk disable odometry untuk mencari loop closure

                "Kp/MaxFeatures": "2500", # 2000

                "RGBD/Enabled": "True",
                "RGBD/OptimizeFromGraphEnd": "False", # True agar robot tidak lompat 
                "RGBD/NeighborLinkRefining": "False",  # Added from documentation
                "RGBD/AngularUpdate": "0.1",  # Added from documentation
                "RGBD/LinearUpdate": "0.1",  # Added from documentation
                "RGBD/OptimizeMaxError": "3.0",  # Added from documentation
                "RGBD/InvertedReg": "False",  # Added from documentation
                "RGBD/ProximityPathMaxNeighbors": "10",
                "RGBD/ProximityMaxGraphDepth": "20",
                "RGBD/ProximityByTime": "False",
                "RGBD/ProximityBySpace": "True",  
                # "RGBD/LocalBundleOnLoopClosure": "True",

                "Optimizer/Strategy": "2",  # Added by Azzam
                "Optimizer/Iterations": "10",  # Added by Azzam
                "Optimizer/Epsilon": "0.00001",  # Added by Azzam
                "Optimizer/Robust": "False",  # Added by Azzam
                'Optimizer/GravitySigma':'0.3', 
                "Optimizer/VarianceIgnored": "False",
                "Optimizer/LandmarksIgnored": "False",  # Added by Azzam
                "Optimizer/PriorsIgnored": "True",  # Added by Azzam
                "GTSAM/Incremental": "False",  # Added by Azzam

                "Bayes/PredictionMargin": "0", # Added by Azzam
                "Bayes/FullPredictionUpdate": "False", # Added by Azzam
                "Bayes/PredictionLC": "0.15", # Added by Azzam

                "Odom/Strategy": "0",  # Added by Azzam
                "Odom/ResetCountdown": "0",  # Added by Azzam
                "Odom/Holonomic": "False",  # Added by Azzam
                "Odom/ScanKeyFrameThr": "0.9",  # Added by Azzam, semakin kecil semakin sering lidar update
                "Odom/AlignWithGround": "True",  # Added by Azzam
                "Odom/FilteringStrategy": "1",
                # "OdomF2M/ScanSubtractAngle": "0.0",  # Added by Azzam
                # "OdomF2M/BundleAdjustment": "0",
                # "OdomF2M/ScanMaxSize": "20000",

                "Reg/Strategy": "1",  # Added by Azzam
                "Reg/Force3DoF": "True",  # Added by Azzam
                "Kp/DetectorStrategy": "3",  # 0 = SURF, 1 = SIFT, 2 = ORB, 

                "Icp/Strategy": "1",  # Added by Azzam
                "Icp/MaxTranslation": "1.5", # Added by Azzam
                "Icp/MaxRotation": "0.7", # Added by Azzam
                "Icp/RangeMin": "0.0", # Added by Azzam
                "Icp/RangeMax": "25.0", # Added by Azzam
                "Icp/MaxCorrespondenceDistance": "1.0", # Added by Azzam
                "Icp/Iterations": "40", # Added by Azzam
                "Icp/PointToPlane": "True", # Added by Azzam
                "Icp/VoxelSize": "0.05", # Added by Azzam
                'Icp/PointToPlaneMinComplexity':'0.19', # to be more robust to long corridors with low geometry
                'Icp/PointToPlaneLowComplexityStrategy':'1', # to be more robust to long corridors with low geometry

                "Vis/MaxDepth": "20.0",
                "Vis/MinInliers": "10",

                "Grid/Sensor": "0",  # Added to suppress warning
                "Grid/RangeMin": "0.0",  # Added by Azzam
                "Grid/RangeMax": "150.0",  # Added by Azzam
                'Grid/UpdateRate': "1.0",  # Update map every 1 second (default is often higher)
                'Grid/CellSize': "0.01",  # Increase cell size to reduce map density
                "Grid/FromDepth": "False",  # Added from documentation
                "Grid/IncrementalMapping": "True",  # Added from documentation
                "Grid/Scan2dUnknownSpaceFilled": "False",  # Added by Azzam
                "GridGlobal/UpdateError": "0.1", # Added by Azzam
                "Grid/RayTracing": "False", # Added by Azzam

                "use_sim_time": False,
                "Threads": 12, # Added by Azzam
        
            }
        ],
        remappings=[
            ("odom", "/slam_vo/odom"),
            ('rgb/image', '/vision/color_image'),
            ('depth/image', '/vision/depth_image'),
            ('rgb/camera_info', '/vision/camera_info'),
            ('scan', '/vision/laserscan'),

            # ("odom", "/slam_vo/odom"),
            # ("rgb/image", "/camera/rs2_cam_main/color/image_raw"),
            # ("depth/image", "/camera/rs2_cam_main/aligned_depth_to_color/image_raw"),
            # ("rgb/camera_info", "/camera/rs2_cam_main/color/camera_info"),
            # ('scan', '/detection/pointcloud_laser_scan'),
        ],
        arguments=["--ros-args", "--log-level", "warn"],
        # prefix='nice -n -9 chrt -f 90',
        respawn=True,
    )

    imu_filter_madgwick_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter_madgwick_node",
        parameters=[{"use_mag": False}],
        remappings=[
            ("/imu/data_raw", "/camera/rs2_cam_main/imu"),
            ("/imu/data", "/hardware/imu"),
        ],
        arguments=["--ros-args", "--log-level", "error"],
        respawn=True,
    )

    rgbd_odom_node = Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        name="rgbd_odometry",
        output="screen",
        namespace="slam_vo",
        parameters=[
            {"frame_id": "base_link"},
            {'guess_frame_id':'odom'},
            {'subscribe_depth':True},
            {"subscribe_imu": True},
            {
                "frame_id": "base_link",
                "map_frame_id": "map",
                "odom_frame_id": "odom",
                "subscribe_depth": True,
                "subscribe_scan": True,
                "subscribe_scan_cloud": False,
                "subscribe_stereo": False,
                "subscribe_rgbd": False,
                "subscribe_rgb": False,
                "subscribe_Odometry": True,
                'scan_max_range': 50.0,  # LiDAR max range (meters)
                'scan_voxel_size': 0.05,  # Downsampling resolution (meters)
                "qos_scan": 1,
                "wait_for_transform": 2.0,

                "odom_tf_linear_variance": 0.0000000001,
                "odom_tf_angular_variance": 0.0000000001,
                "publish_tf": False,
                "publish_map": True,
                "approx_sync": False,
                "use_saved_map": False,
                "sync_queue_size": 30,
                "topic_queue_size": 10,
                "icp_odometry": False, 
                "visual_odometry": False,

                "Rtabmap/DetectionRate": "5.0", # Added by Azzam
                "Rtabmap/CreateIntermediateNodes": "True",
                "Rtabmap/LoopThr": "0.11", # Routine period untuk cek loop closure

                "Mem/STMSize": "50",  # Short-term memory size
                "Mem/IncrementalMemory": "False",  # 
                "Mem/InitWMWithAllNodes": "True",  # 
                "Mem/RehearsalSimilaritys": "0.9",  #
                "Mem/UseOdomFeatures": "True",  #
                'Mem/NotLinkedNodesKept': "True",  # Keep unlinked nodes
                # "Mem/UseOdomFeatures": "False", # (percobaan) untuk disable odometry untuk mencari loop closure

                "Kp/MaxFeatures": "2000",

                "RGBD/Enabled": "True",
                "RGBD/OptimizeFromGraphEnd": "False", # True agar robot tidak lompat 
                "RGBD/NeighborLinkRefining": "False",  # Added from documentation
                "RGBD/AngularUpdate": "1.0",  # Added from documentation
                "RGBD/LinearUpdate": "0.1",  # Added from documentation
                "RGBD/OptimizeMaxError": "3.0",  # Added from documentation
                "RGBD/InvertedReg": "False",  # Added from documentation
                "RGBD/ProximityPathMaxNeighbors": "10",
                "RGBD/ProximityMaxGraphDepth": "50",
                "RGBD/ProximityByTime": "False",
                "RGBD/ProximityBySpace": "False",  
                # "RGBD/LocalBundleOnLoopClosure": "True",

                "Optimizer/Strategy": "2",  # Added by Azzam
                "Optimizer/Iterations": "10",  # Added by Azzam
                "Optimizer/Epsilon": "0.00001",  # Added by Azzam
                "Optimizer/Robust": "False",  # Added by Azzam
                'Optimizer/GravitySigma':'0.3', 
                "Optimizer/VarianceIgnored": "False",
                "Optimizer/LandmarksIgnored": "False",  # Added by Azzam
                "Optimizer/PriorsIgnored": "True",  # Added by Azzam
                "GTSAM/Incremental": "False",  # Added by Azzam

                "Bayes/PredictionMargin": "0", # Added by Azzam
                "Bayes/FullPredictionUpdate": "False", # Added by Azzam
                "Bayes/PredictionLC": "0.1", # Added by Azzam

                "Odom/Strategy": "0",  # Added by Azzam
                "Odom/ResetCountdown": "0",  # Added by Azzam
                "Odom/Holonomic": "False",  # Added by Azzam
                "Odom/ScanKeyFrameThr": "0.9",  # Added by Azzam, semakin kecil semakin sering lidar update
                "Odom/AlignWithGround": "True",  # Added by Azzam
                "Odom/FilteringStrategy": "1",
                # "OdomF2M/ScanSubtractAngle": "0.0",  # Added by Azzam
                # "OdomF2M/BundleAdjustment": "0",
                # "OdomF2M/ScanMaxSize": "20000",

                "Reg/Strategy": "1",  # Added by Azzam
                "Reg/Force3DoF": "True",  # Added by Azzam
                "Kp/DetectorStrategy": "3",  # 0 = SURF, 1 = SIFT, 2 = ORB, 

                "Icp/Strategy": "1",  # Added by Azzam
                "Icp/MaxTranslation": "1.5", # Added by Azzam
                "Icp/MaxRotation": "0.7", # Added by Azzam
                "Icp/RangeMin": "0.0", # Added by Azzam
                "Icp/RangeMax": "25.0", # Added by Azzam
                "Icp/MaxCorrespondenceDistance": "1.0", # Added by Azzam
                "Icp/Iterations": "30", # Added by Azzam
                "Icp/PointToPlane": "True", # Added by Azzam
                "Icp/VoxelSize": "0.05", # Added by Azzam
                'Icp/PointToPlaneMinComplexity':'0.19', # to be more robust to long corridors with low geometry
                'Icp/PointToPlaneLowComplexityStrategy':'1', # to be more robust to long corridors with low geometry

                "Vis/MaxDepth": "20.0",
                "Vis/MinInliers": "10",

                "Grid/Sensor": "1",  # Added to suppress warning
                "Grid/RangeMin": "0.0",  # Added by Azzam
                "Grid/RangeMax": "150.0",  # Added by Azzam
                'Grid/UpdateRate': "1.0",  # Update map every 1 second (default is often higher)
                'Grid/CellSize': "0.03",  # Increase cell size to reduce map density
                "Grid/FromDepth": "False",  # Added from documentation
                "Grid/IncrementalMapping": "True",  # Added from documentation
                "Grid/Scan2dUnknownSpaceFilled": "False",  # Added by Azzam
                "GridGlobal/UpdateError": "0.04", # Added by Azzam
                "Grid/RayTracing": "False", # Added by Azzam

                "use_sim_time": False,
                "Threads": 12, # Added by Azzam

            
            } # Added by Azzam
        ],
        remappings=[
            ("rgb/image", "/camera/rs2_cam_main/color/image_raw"),
            ("depth/image", "/camera/rs2_cam_main/aligned_depth_to_color/image_raw"),
            ("rgb/camera_info", "/camera/rs2_cam_main/color/camera_info"),
            ("imu", "/hardware/imu"),
            # ("odom", "vo")
        ],
        arguments=["--ros-args", "--log-level", "warn"],
        respawn=True,
    )


    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_node",
        namespace="slam",
        parameters=[
            {
                # "use_sim_time": True,
                "map_frame": "map",
                "odom_frame": "odom",
                "base_link_frame": "base_link",
                "world_frame": "map",
                "two_d_mode": True,
                "smooth_lagged_data": True,
                "history_length": 1.0,
                "frequency": 25.0,
                "publish_tf": False,

                "odom0": "/odom",
                # fmt: off
                "odom0_config": [
                    True,True,False,
                    False,False,True,
                    False,False,False,
                    False,False,False,
                    False,False,False,
                ],
                # fmt: on
                "odom0_differential": True,
                "odom0_relative": True,
                # "odom0_noise_covariance":    [0.0004, 0.0,    0.0,
                #                             0.0,    0.0004, 0.0,
                #                             0.0,    0.0,    0.05],

                "pose0": "localization_pose", # Ini untuk rtabmap
                # fmt: off
                "pose0_config":[
                    True,True,False,
                    False,False,True,
                    False,False,False,
                    False,False,False,
                    False,False,False,
                ],
                # fmt: on
                "pose0_differential": False,
                "pose0_relative": False,
                
                # "odom1": "/slam_vo/odom",
                # # fmt: off
                # "odom1_config": [
                #     True,True,False,
                #     False,False,True,
                #     False,False,False,
                #     False,False,False,
                #     False,False,False,
                # ],
                # # fmt: on
                # "odom1_differential": True,
                # "odom1_relative": True,
            }
        ],
        arguments=["--ros-args", "--log-level", "warn"],
        # prefix='nice -n -9',
        respawn=True,
    )

    # =============================================================================================

    tf_map_empty = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_map_empty",
        # fmt: off
        arguments=["0.00","0.00","0.00","0.00","0.00","0.00","map","odom",
            "--ros-args","--log-level","error",],
        # fmt: on
        respawn=True,
    )

    tf_base_link_to_body_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_link_to_body_link",
        # fmt: off
        arguments=["0.175","0.00","0.165","0.00","0.00","0.00","base_link","body_link",
            "--ros-args","--log-level","error",],
        # fmt: on
        respawn=True,
    )

    tf_base_link_to_imu_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_link_to_imu_link",
        # fmt: off
        arguments=["0.19","0.00","0.00","0.00","0.00","0.00","base_link","imu_link",
            "--ros-args","--log-level","error",],
        # fmt: on
        respawn=True,
    )

    # tf_base_link_to_camera_link = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="tf_base_link_to_camera_link",
    #     # fmt: off
    #     arguments=["0.25","0.00","0.16225","0.00","0.3","0.00","base_link","camera_link",
    #         "--ros-args","--log-level","error",],
    #     # fmt: on
    #     respawn=True,
    # )

    tf_base_link_to_camera_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_link_to_camera_link",
        # fmt: off
        arguments=["0.0","0.00","0.28","0.0","0.500","0.0","base_link","camera_link",
            "--ros-args","--log-level","error",],
        # fmt: on
        respawn=True,
    )

    tf_base_link_to_camera_iris = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_link_to_camera_iris",
        # fmt: off
        arguments=["0.17","0.00","0.28","0.0","0.5","0.0","base_link","camera_iris",
            "--ros-args","--log-level","error",],
        # fmt: on
        respawn=True,
    )

    # ARGUMENTS TF UNTUK CAMERA COLOR OPTICAL FRAME (HANYA UNTUK TF INI SAJA)
    # POSE X, POSE Y, POSE Z, YAW, ROLL, PITCH, PARENT FRAME, CHILD FRAME
    # DENGAN X SEBAGAI FORWARD, Y SEBAGAI LEFT, Z SEBAGAI UP MAKA YAW = ROTASI Z AXIS, ROLL = ROTASI X AXIS, PITCH = ROTASI Y AXIS

    tf_camera_iris_to_camera_color_optical_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_camera_iris_to_camera_color_optical_frame",
        # fmt: off
        arguments=["0.0","0.00","0.0","-1.5708","0.0","-1.5708","camera_iris","camera_color_optical_frame",
            "--ros-args","--log-level","error",],
        # fmt: on
        respawn=True,
    )

    # =============================================================================================

    # return LaunchDescription(
    #     [
    #         rosapi_node, 
    #         ui_server, 
    #         rosbridge_server, 
    #         web_video_server,

    #         tf_base_link_to_body_link,
    #         tf_base_link_to_imu_link,
    #         tf_base_link_to_camera_link,
    #         tf_base_link_to_camera_iris,
    #         tf_camera_iris_to_camera_color_optical_frame,

    #         vision_capture,
    #         # rs2_cam_main,
    #         # detection2,
    #         # apriltag3,

    #         # detection,
    #         # rtabmap_slam_rtabmap3,
    #         # imu_serial,

    #         # master,
    #         # motor_main,
    #         # CANbus_HAL,

    #         # keyboard_input,
    #     ]
    # )

    return LaunchDescription(
        [
            rosapi_node, 
            ui_server, 
            rosbridge_server, 
            web_video_server,

            # =============================================================================================

            # tf_map_empty, # sementara
            pose_estimator,
            tf_base_link_to_body_link,
            tf_base_link_to_imu_link,
            tf_base_link_to_camera_link,
            tf_base_link_to_camera_iris,
            tf_camera_iris_to_camera_color_optical_frame,


            # =============================================================================================

            # vision_capture,
            # detection,  
            # apriltag3,
            # detection2,  
            # lane_detection,
            # onnx_inference_node,
            # occupancy_grid,
            # apriltag_detection,
            # yolo_detection,
            # onnx_lane_detection,

            # =============================================================================================

            # wit_ros2_imu,

            # =============================================================================================

            # keyboard_input,
            # joy_node,
            master,

            # =============================================================================================

            # rs2_cam_main,
            # imu_filter_madgwick_node,
            imu_serial,

            # ekf_node,

            # =============================================================================================

            CANbus_HAL,
            motor_main, 

            # =============================================================================================

            TimerAction(
                period=0.5,
                actions=[
                    # rgbd_odom_node,
                    # rtabmap_slam_rtabmap3,
                    # ekf_node,
                ],
            ),

            # =============================================================================================
            # rviz2,

        ]
    )
