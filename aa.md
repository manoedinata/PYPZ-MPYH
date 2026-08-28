## Tổng quan

Đây là workspace ROS 2 Humble cho một robot tự hành nhỏ dạng xe Ackermann, có các chức năng chính:

- Điều khiển động cơ bánh và servo lái.
- Giao tiếp phần cứng qua CAN, UDP và serial.
- Đọc encoder, IMU, joystick và nút bấm.
- Xử lý camera Intel RealSense.
- Nhận diện làn đường, vật cản, biển báo và AprilTag.
- Ước lượng odometry và tạo occupancy grid.
- SLAM bằng RTAB-Map.
- Điều khiển robot theo nhiều chế độ FSM.
- Điều khiển và giám sát qua web UI.
- Gửi telemetry lên InfluxDB.

Workspace nằm tại:

`fira`

Kiến trúc logic có thể mô tả như sau:

```mermaid
flowchart LR
    Camera[Intel RealSense] --> Vision[vision]
    Camera --> April[apriltag_detection]
    IMU[IMU serial / Wit IMU] --> Pose[pose_estimator]
    Encoder[Wheel encoder] --> Pose

    Pose --> Odom[/odom]
    Vision --> Perception[Lane / obstacle / sign data]
    April --> Perception

    Odom --> Master[master FSM]
    Perception --> Master
    Joystick[joy / keyboard / web] --> Master

    Master --> Commands[Target speed / steering]
    Commands --> Motor[motor_main]
    Motor --> CAN[CANbus_HAL]
    CAN --> Drivers[Wheel and steering drivers]

    Vision --> Grid[occupancy_grid]
    Odom --> RTAB[RTAB-Map]
    Camera --> RTAB
    Grid --> RTAB

    Web[Browser] <--> Rosbridge[rosbridge + rosapi]
    Web --> UI[web_ui]
    Telemetry[telemetry] --> Influx[InfluxDB]
```

## Cấu trúc package

### `hardware`

Package điều khiển phần cứng cấp thấp:

- `CANbus_HAL.cpp`
- `keyboard_input.cpp`
- `motor_main.cpp`
- `motor_main2.cpp`
- `serial_imu.cpp`
- `stm_udp.cpp`

Vai trò chính:

1. Nhận lệnh tốc độ và góc lái từ tầng điều khiển.
2. Chuyển lệnh ROS 2 thành PWM hoặc frame CAN/UDP.
3. Đọc encoder, phản hồi servo và trạng thái nút.
4. Đọc IMU qua serial.

#### `CANbus_HAL`

`CANbus_HAL.cpp` sử dụng SocketCAN Linux:

```cpp
#include <linux/can.h>
#include <linux/can/raw.h>
```

Các tham số chính:

- Interface mặc định: `can0`
- Bitrate mặc định trong code: `125000`
- Bitrate trong launch: `1000000`
- CAN ID driver bánh: `4`
- CAN ID driver lái: `8`
- Chu kỳ vòng lặp: `10 ms` trong `master.launch.py`, `5 ms` trong `all.launch.py`
- Timeout CAN: `50000 us`

Các topic được khai báo gồm:

- `/hardware/wheel_encoder`
- `/hardware/delta_encoder_wheel_counter`
- `/hardware/feedback_steering`
- Topic nhận PWM bánh
- Topic nhận góc lái
- Topic nhận lệnh keyboard

Điểm quan trọng: giá trị cấu hình trong launch ghi đè giá trị mặc định trong C++. Ví dụ, code mặc định bitrate là `125000`, nhưng launch hiện đặt `1000000`.

#### `motor_main`

`motor_main.cpp` là tầng điều khiển motor servo:

- Nhận encoder bánh.
- Nhận feedback góc lái.
- Nhận `/master/target_speed`.
- Nhận `/master/target_steering`.
- Xuất PWM bánh và PWM servo.
- Xuất feedback góc lái theo radian.

Trong launch, các tham số đáng chú ý là:

```text
routine_period_ms: 20
k_p_wheel: 1700.0
k_i_wheel: 0.0
wheel_radius: 0.0365
encoder_ppr: 147906.25
encoder_to_meter: -0.00000490586
min_steering_pwm: 900
mid_steering_pwm: 1000
max_steering_pwm: 1100
min_steering_pwm_rad: -0.437
max_steering_pwm_rad: 0.437
```

Đây là vòng điều khiển tốc độ khá thấp cấp. `master` quyết định tốc độ/góc mục tiêu; `motor_main` cố gắng bám mục tiêu; `CANbus_HAL` hoặc `stm_udp` truyền lệnh xuống driver.

#### `motor_main2`

Đây có vẻ là một phiên bản điều khiển motor thử nghiệm hoặc phiên bản cũ. Trong `all.launch.py`, node này có thể được bật, nhưng trong `master.launch.py` nó không được sử dụng.

Không nên chạy đồng thời `motor_main` và `motor_main2` nếu cả hai cùng xuất lệnh đến cùng phần cứng.

#### `stm_udp`

`stm_udp.cpp` dùng UDP để giao tiếp với STM hoặc controller bên ngoài.

Nó:

- Nhận lệnh PWM bánh.
- Nhận lệnh bật/tắt PWM.
- Nhận góc lái.
- Nhận lệnh buzzer.
- Gửi encoder và trạng thái button lên ROS 2.
- Có hai timer riêng cho gửi và nhận UDP.
- Dùng thread/executor đa luồng.

Các macro:

```cpp
#define RECV_UDP
#define SEND_UDP
```

cho thấy cả truyền và nhận UDP đang được bật.

Trong `all.launch.py`, `stm_udp` không được khởi động, còn trong phần khai báo node nó có tồn tại. Cần kiểm tra tránh chạy đồng thời cả `stm_udp` và `CANbus_HAL` nếu hai node cùng kiểm soát một actuator.

#### `serial_imu`

Node này đọc IMU qua thiết bị serial. Trong launch:

```text
is_riontech: true
baudrate: 115200
port: /dev/serial/by-id/usb-FTDI_...
```

Hai launch file đang dùng hai serial device khác nhau:

- `all.launch.py`: `A5069RR4`
- `master.launch.py`: `A50285BI`

Điều này có thể phản ánh hai robot hoặc hai bộ IMU khác nhau.

### `communication`

Package giao tiếp bên ngoài robot:

- `wifi_control.cpp`
- `scripts/telemetry.py`

#### `wifi_control`

Node này được cấu hình với:

```text
hotspot_ssid: gh_template
hotspot_password: gh_template
```

Có khả năng quản lý hoặc kiểm tra Wi-Fi hotspot của robot.

#### `telemetry.py`

Node này đọc dữ liệu ROS 2 và ghi vào InfluxDB.

Các topic đầu vào:

- `/distance_travelled`
- `/odom`
- `/can/battery`

Các tham số InfluxDB được truyền trực tiếp trong launch:

```text
INFLUXDB_URL: http://172.30.37.21:8086
INFLUXDB_USERNAME: awm462
INFLUXDB_PASSWORD: wildan462
INFLUXDB_ORG: awmawm
INFLUXDB_BUCKET: ujiCoba
ROBOT_NAME: gh_template
```

Đây là vấn đề bảo mật nghiêm trọng:

- Username/password đang nằm trong file launch.
- `telemetry.py` còn log các giá trị này ra console.
- Password có thể xuất hiện trong log ROS, debug log hoặc repository history.

Nên chuyển các giá trị này sang:

- Environment variables.
- File `.env` ngoài repository.
- ROS 2 parameter file không commit.
- Secret manager nếu chạy thực tế.

Ngoài ra, `publish_period` được mô tả là milliseconds:

```python
self.declare_parameter("publish_period", 10) # in ms
self.timer_routine = self.create_timer(self.publish_period, ...)
```

Nhưng `rclpy.create_timer()` nhận giây, không phải milliseconds. Giá trị `10` sẽ tạo timer 10 giây, không phải 10 ms. Nếu ý định là 10 ms thì phải dùng `0.01`.

### `master`

Đây là node điều phối cấp cao của robot:

- Quản lý FSM.
- Nhận input từ joystick, keyboard, web và vision.
- Đọc odometry, encoder và AprilTag.
- Chọn chế độ hoạt động.
- Tính tốc độ và góc lái mục tiêu.
- Theo dõi vật cản.
- Quản lý waypoint.
- Điều khiển chế độ race, urban, manual, mapping và record route.

File chính:

- `master.cpp`
- `motion.cpp`
- `master_definition.cpp`
- `master.hpp`

#### Các trạng thái FSM chính

Trong `master.hpp`:

```cpp
FSM_GLOBAL_INIT                  0
FSM_GLOBAL_PREOP                 1
FSM_GLOBAL_SAFEOP                2
FSM_GLOBAL_OP_3                  3
FSM_GLOBAL_OP_4                  4
FSM_GLOBAL_OP_5                  5
FSM_GLOBAL_OP_2                  6
FSM_GLOBAL_RECORD_ROUTE          7
FSM_GLOBAL_MAPPING               8
FSM_GLOBAL_RECORD_ROUTE_KANAN    9
FSM_GLOBAL_RECORD_ROUTE_KIRI     10
FSM_GLOBAL_RECORD_ROUTE_TENGAH   11
FSM_GLOBAL_RACE_BUTTON           12
FSM_GLOBAL_RECORD_DATASET_ROAD   20
FSM_GLOBAL_CUSTOM_DEBUG_1        300
FSM_GLOBAL_CUSTOM_DEBUG_2        301
```

Trong `callback_routine()`, node chạy timer mỗi `20 ms`, tương đương 50 Hz.

Một số chế độ:

- `INIT`: load waypoint và terminal.
- `PREOP`: điều khiển thủ công qua joystick.
- `OP_3`: chạy urban.
- `OP_4`: chạy theo dữ liệu vision.
- `RACE_BUTTON`: race bằng tốc độ vision.
- `OP_5`: đi theo waypoint.
- `100`: urban tự động.
- `200`: race.
- `RECORD_DATASET_ROAD`: chạy manual để ghi dataset.

#### Topic output quan trọng

`master` xuất:

- `/master/target_speed`
- `/master/target_steering`
- `/master/global_fsm`
- `/master/local_fsm`
- `/master/waypoints`
- `/master/path_point`
- `/master/nearest_obstacle`
- `/master/free_path_map`
- `/master/terminals`
- `/master/state_urban`
- `/master/sign_buzzer`

Đây là điểm giao tiếp chính giữa tầng lập kế hoạch và tầng hardware.

#### Topic input quan trọng

- `/odom`
- `/key_pressed`
- `/web/selected_lane`
- `/key_web_pressed`
- `/master/ui_target_velocity_and_steering`
- `/master/set_master_fsm`
- `/vision/slope`
- `/vision/velocity`
- `/sign/marker/id`
- `/sign/picture/id`
- `/hardware/joy`
- `/vision/intersection_point`
- `/apriltag/markers`
- `/motor_main/velocity_feedback`
- `/vision/urban_data`
- `/vision/master_config`
- `/hardware/button`

Service của `master` gồm các chức năng:

- Bắt đầu/dừng ghi route.
- Ghi route bên phải/trái/giữa.
- Thêm route.
- Thêm/xóa terminal.
- Điều khiển terminal bằng sign.

#### Tham số waypoint

Launch dùng các file:

```text
/home/iris/waypoints.yaml
/home/iris/waypoints_race_kanan.yaml
/home/iris/waypoints_race_kiri.yaml
/home/iris/waypoints_race_tengah.yaml
/home/iris/terminal.yaml
```

Các đường dẫn này được hard-code và không tồn tại trong workspace hiện tại. Robot phải có đúng các file đó tại `/home/iris`, nếu không các chức năng waypoint sẽ thất bại.

### `vision`

Package xử lý thị giác máy tính bằng C++:

- `vision_capture.cpp`
- `vision_capture2.cpp`
- `vision_capture3.cpp`
- `vision_capture4.cpp`
- `detection.cpp`
- `detection2.cpp`
- `detection_urban.cpp`
- `lane_detection.cpp`
- `ml_detection.cpp`
- `onnx_inference_node.cpp`
- `apriltag3.cpp`
- `apriltag4.cpp`
- `aruco_detection.cpp`

Có thể chia thành bốn nhóm:

1. Thu nhận và tiền xử lý ảnh.
2. Nhận diện làn đường.
3. Nhận diện vật cản/biển báo.
4. Tạo dữ liệu hình học cho occupancy grid và SLAM.

`vision_capture.cpp` xuất nhiều loại dữ liệu:

- Ảnh màu.
- Ảnh depth.
- Camera info.
- Ảnh overlay.
- Ảnh binary.
- Point cloud.
- Filtered points.
- Sign points.
- Cleaned point cloud.
- YUV point cloud.
- LaserScan.
- AprilTag.
- Slope.

Các topic chính mà code thể hiện gồm:

- `/vision/color_image`
- `/vision/depth_image`
- `/vision/camera_info`
- `/vision/laserscan`
- `/vision/slope`
- Các topic point cloud và ảnh debug.

`master` đang sử dụng trực tiếp:

```text
/vision/slope
/vision/velocity
/vision/intersection_point
/vision/urban_data
```

Trong `master.launch.py`, nhiều node vision được bật:

- `vision_capture`
- `detection`
- `detection2`
- `lane_detection`
- `apriltag_detection`
- `apriltag3`
- `yolo_detection`
- `onnx_lane_detection`

Trong `all.launch.py`, nhiều node vision được khai báo nhưng bị comment trong `LaunchDescription`. Vì vậy việc node tồn tại trong file không có nghĩa là nó đang chạy.

### `apriltag_detection`

Đây là package Python:

- `apriltag_detection.py`
- `onnx_lane_detection.py`
- `onnx_sign_detection.py`
- `yolo_detection.py`

Nó chứa các node hoặc wrapper Python cho:

- AprilTag.
- Nhận diện làn đường bằng ONNX.
- Nhận diện sign bằng ONNX.
- YOLO.

Các model được trỏ tới:

```text
/home/iris/model.onnx
/home/iris/model_19_juni.onnx
/home/iris/best.pt
/home/iris/best224.onnx
/home/iris/best_fira3_openvino_model/
```

Các model này cũng không nằm trong workspace. Deployment cần chuẩn bị riêng.

Tham số YOLO:

```text
confidence_threshold: 0.6
iou_threshold: 0.7
```

### `world_model`

Package này có hai node:

- `pose_estimator.cpp`
- `occupancy_grid.cpp`

#### `pose_estimator`

Node này hợp nhất:

- Encoder bánh.
- IMU gyro.
- Pose offset từ `master`.

Input:

```text
/hardware/wheel_encoder
/hardware/imu
/master/pose_offset
```

Output:

```text
/odom
/pose_estimator/encoder_meter
```

Nó giữ trạng thái pose:

```text
final_pose_x
final_pose_y
final_pose_theta
```

và vận tốc:

```text
final_vel_x
final_vel_y
final_vel_theta
```

Pose được tính từ encoder và gyro. Node cũng có `TransformBroadcaster`, nhưng cần kiểm tra phần còn lại của file để xác định chính xác TF nào được phát và có bị trùng với các static transform hay không.

#### `occupancy_grid`

Node này nhận:

- Point cloud vật cản.
- Point cloud mặt đường.

Xuất:

```text
grid_map
```

Tham số trong launch:

```text
res: 0.03
width: 200
height: 200
ox: -1.0
oy: -1.0
memory_timeout_sec: 5.0
blind_spot_radius: 0.2
```

Một grid 200 m x 200 m với độ phân giải 3 cm có khoảng:

```text
6667 x 6667 ~= 44 triệu ô
```

Tùy cách triển khai bộ nhớ, cấu hình này có thể rất nặng. Nếu occupancy grid dùng một byte/ô thì riêng dữ liệu thô đã khoảng 44 MB, chưa tính overhead, copy và message serialization.

### `ros2_interface`

Package định nghĩa custom message/service.

Các message:

- `Apriltag.msg`
- `PointArray.msg`
- `Terminal.msg`
- `TerminalArray.msg`
- `VisionUrban.msg`
- `apriltag.msg`

`VisionUrban.msg` chứa dữ liệu phong phú cho chế độ urban:

- Tọa độ target và robot trên ảnh.
- Khoảng cách đến vạch trắng.
- Góc target.
- Offset làn đường.
- Thông tin zebra crossing.
- Tâm sign.
- Tâm obstacle.
- Trạng thái đường cong.
- Trạng thái giao lộ.
- Khoảng cách đến giao lộ.

Đây là data contract giữa `vision` và `master`.

Service hiện thấy:

```text
Dummy.srv
```

Các service của `master` dùng `std_srvs/SetBool`, không nằm trong custom interface package.

### `ros2_utils`

Package dùng chung:

- `global_definitions.hpp`
- `help_logger.hpp`
- `pid.hpp`
- `rtabmap_params.h`
- `simple_fsm.hpp`
- `system_utils.hpp`
- `help_logger.cpp`
- Các launch file.
- Các config file.

Đây là package hạ tầng chung cho toàn workspace.

#### `all.launch.py`

Đây là launch chính được `run.sh` gọi:

```bash
ros2 launch ros2_utils all.launch.py
```

Nó định nghĩa gần như toàn bộ stack:

- `rosbridge_websocket`
- `web_video_server`
- `rosapi_node`
- RealSense
- IMU
- RTAB-Map
- Wi-Fi
- Telemetry
- Web UI
- Master
- CAN
- Motor
- Pose estimator
- Occupancy grid
- Vision
- AprilTag
- EKF
- Static TF

Tuy nhiên, `LaunchDescription` cuối file chỉ bật một phần node. Các nhóm bị comment gồm nhiều node camera, vision, SLAM, joystick và EKF.

Ở trạng thái hiện tại, `all.launch.py` chủ yếu bật:

- Web UI/rosbridge.
- `pose_estimator`.
- Static TF.
- `master`.
- `imu_serial`.
- `CANbus_HAL`.
- `motor_main`.

Các node sensor và perception quan trọng có thể đang bị tắt trong launch này.

#### `master.launch.py`

File này có cấu hình đầy đủ hơn hoặc là một profile chạy thực tế khác. Nó có thêm:

- `vision_capture2`
- `vision_capture3`
- `vision_capture4`
- `detection_urban`
- Nhiều node perception.
- RTAB-Map với cấu hình lớn.
- RGB-D odometry.
- EKF.
- Các tùy chọn mapping.

Hai file này không chỉ khác tên. Chúng khác:

- Camera configuration.
- IMU serial port.
- Cấu hình align depth.
- Point cloud.
- Tham số CAN.
- Tham số master.
- Số lượng node vision được bật.
- Cấu hình RTAB-Map.
- Một số model path.

Vì vậy cần xem chúng như hai runtime profile khác nhau.

### `web_ui`

Package cung cấp giao diện web tĩnh:

```text
ROS_config.html
ROS_config.js
ROS_if.html
ROS_if.js
ROS_if_imv.html
ROS_if_imv.js
config.html
config.js
index.html
index.js
slam.html
slam.js
```

Các thư viện frontend có sẵn:

- `roslib.min.js`
- `bulma.min.css`
- `konva.min.js`
- `anime.min.js`

`ui_server.py` khởi động:

```bash
python3 -m http.server
```

trong thư mục:

```text
src/web_ui/src
```

Node này được chạy qua ROS 2 nhưng thực tế chỉ dùng ROS 2 để quản lý process HTTP server. Nó không phải HTTP server ROS-native.

`run.sh` dọn các port:

```bash
sudo fuser -k 9090/tcp
sudo fuser -k 8080/tcp
sudo fuser -k 8000/tcp
```

sau đó chạy `all.launch.py`.

Các port có khả năng tương ứng với:

- `9090`: rosbridge websocket.
- `8080`: web UI hoặc web video.
- `8000`: một HTTP/video service khác.

Cần xác nhận bằng runtime vì `python3 -m http.server` mặc định dùng port `8000`.

### `realsense2_camera`

Đây là source package/vendor package cho Intel RealSense ROS 2 driver.

Các launch của workspace cấu hình:

- Depth.
- Color.
- IMU.
- Align depth.
- Point cloud.
- Resolution/fps.

Trong `all.launch.py`:

```text
enable_depth: true
enable_color: true
enable_sync: true
align_depth.enable: false
pointcloud.enable: false
rgb: 640x360x60
depth: 640x360x60
```

Trong `master.launch.py`:

```text
align_depth.enable: true
pointcloud.enable: true
```

RTAB-Map yêu cầu depth đã align với color khi dùng:

```text
/camera/rs2_cam_main/aligned_depth_to_color/image_raw
```

Nếu `align_depth.enable` bị tắt nhưng node vẫn remap tới aligned depth topic, RTAB-Map có thể không nhận được dữ liệu hợp lệ.

### `realsense2_camera_msgs`

Package message/service hỗ trợ driver RealSense:

- Custom messages.
- Custom services.
- API liên quan đến camera profile, device, option và sensor.

Đây là dependency của `realsense2_camera`.

### `wit_ros2_imu`

Package Python cho IMU WitMotion/Wit hoặc thiết bị tương tự.

Trong launch, node được remap:

```text
/imu/data_raw -> /hardware/imu
```

Nhưng trong `all.launch.py`, `wit_ros2_imu` được khai báo và sau đó bị comment trong `LaunchDescription`. Thay vào đó, `imu_serial` của package `hardware` được bật.

Không nên bật đồng thời cả hai node nếu chúng cùng publish `/hardware/imu`.

## Luồng dữ liệu điều khiển

Luồng điều khiển dự kiến:

```text
Joystick / keyboard / web
        |
        v
      master
        |
        +--> /master/target_speed
        |
        +--> /master/target_steering
                    |
                    v
                motor_main
                    |
                    +--> /motor_main/target_pwm_wheel
                    +--> /motor_main/target_steering
                                |
                                v
                         CANbus_HAL hoặc stm_udp
                                |
                                v
                         Motor driver / steering driver
```

Phản hồi đi ngược:

```text
Wheel driver --> CANbus_HAL --> /hardware/wheel_encoder
Steering driver --> CANbus_HAL --> /hardware/feedback_steering
IMU --> serial_imu --> /hardware/imu
Encoder + IMU --> pose_estimator --> /odom
```

`master` dùng `/odom`, vision và input người vận hành để tính lệnh tiếp theo.

## Luồng perception và SLAM

Luồng camera:

```text
RealSense
  |
  +--> color image
  +--> depth image
  +--> camera info
  |
  v
vision_capture / detection / lane_detection
  |
  +--> lane slope
  +--> target speed
  +--> obstacle point cloud
  +--> laser scan
  +--> sign ID
  +--> AprilTag data
  |
  v
master / occupancy_grid / RTAB-Map
```

RTAB-Map được cấu hình theo hai hướng:

1. RGB-D SLAM từ color + aligned depth.
2. Kết hợp với `/slam_vo/odom`, laser scan và IMU.

Trong cấu hình RTAB-Map:

- `Reg/Force3DoF: True`: giới hạn robot thành chuyển động 3 bậc tự do trên mặt phẳng.
- `Icp/Strategy: 1`: dùng ICP.
- `Icp/PointToPlane: True`.
- `Grid/IncrementalMapping: True`.
- `publish_tf: False`: RTAB-Map không tự publish TF.
- `use_saved_map: False`: không load map đã lưu.
- `Threads: 12`: dùng 12 thread.

`robot_localization/ekf_node` cũng được khai báo nhưng trong `all.launch.py` đang bị comment. Cần tránh bật nhiều node cùng publish hoặc hiệu chỉnh pose nếu chưa thống nhất TF tree.

## TF tree dự kiến

Các static transform trong launch:

```text
map -> odom
base_link -> body_link
base_link -> imu_link
base_link -> camera_link
base_link -> camera_iris
camera_iris -> camera_color_optical_frame
```

Có một transform:

```text
map -> odom
```

với pose bằng 0 và được ghi chú là tạm thời.

Một số rủi ro:

- `pose_estimator` có `TransformBroadcaster`, có thể phát TF động.
- RTAB-Map được cấu hình `publish_tf: False`.
- EKF cũng được cấu hình `publish_tf: False`.
- Static `map -> odom` có thể xung đột về mặt thiết kế nếu sau này SLAM/EKF cần kiểm soát transform này.
- `camera_link`, `camera_iris`, `camera_color_optical_frame` cần khớp với frame thực tế mà RealSense driver publish.

## Script vận hành

### Build

`make.sh`:

```bash
export ROS_DISTRO=humble
colcon build --symlink-install --executor parallel --parallel $(nproc)
```

Điểm cần chú ý:

- Không source `/opt/ros/humble/setup.bash` trong script.
- Không kiểm tra lỗi build.
- Dùng `--symlink-install`, phù hợp khi phát triển Python/config.
- Chạy song song theo số CPU.

### Run

`run.sh`:

```bash
sudo fuser -k 9090/tcp
sudo fuser -k 8080/tcp 
sudo fuser -k 8000/tcp

. install/setup.bash 
export ROS_LOCALHOST_ONLY=1
ros2 launch ros2_utils all.launch.py 
```

`ROS_LOCALHOST_ONLY=1` giới hạn ROS 2 discovery trong localhost. Điều này phù hợp khi toàn bộ node nằm trên một máy, nhưng ngăn máy khác truy cập trực tiếp DDS.

Web UI vẫn có thể truy cập qua rosbridge nếu browser kết nối tới đúng máy.

### systemd

- `1_stop.sh`: dừng `run_main.service`.
- `2_start.sh`: khởi động `run_main.service`.

Điều này cho thấy runtime dự kiến có systemd service tên `run_main.service`, nhưng file service không có trong workspace.

### `init_all.sh`

Đây là script nguy hiểm:

```bash
rm -rf src/
mkdir src/
```

Sau đó nó tạo lại một số package bằng `ros2 pkg create`.

Chạy script này sẽ xóa toàn bộ source hiện tại trong `src`, bao gồm:

- Code C++.
- Code Python.
- Custom messages.
- RealSense package.
- Config.
- Launch.
- Tests.

Nó chỉ phù hợp cho việc khởi tạo workspace mới, không phù hợp để “remake” workspace hiện tại. Comment trong `README.md` đang hướng dẫn chạy script này, nên tài liệu hiện tại có nguy cơ gây mất source.

## Các vấn đề kỹ thuật nổi bật

### 1. Hard-code secret

Thông tin InfluxDB nằm trong `all.launch.py` và được log trong `telemetry.py`.

Mức độ: cao.

### 2. `telemetry.py` có thể sai đơn vị timer

`create_timer()` dùng giây, nhưng biến được chú thích milliseconds.

Mức độ: cao nếu telemetry cần cập nhật nhanh.

### 3. Hai launch file không nhất quán

`all.launch.py` và `master.launch.py` có cấu hình khác nhau về:

- Camera.
- IMU.
- CAN.
- Vision.
- SLAM.
- Motor.
- Tham số robot.

Mức độ: cao vì cùng một lệnh build nhưng kết quả runtime phụ thuộc file launch.

### 4. Nhiều node được khai báo nhưng không được bật

Ví dụ trong `all.launch.py`:

- RealSense.
- Vision.
- RTAB-Map.
- EKF.
- Joystick.
- Wit IMU.

Các node này xuất hiện ở phần định nghĩa nhưng bị comment trong `LaunchDescription`.

Mức độ: cao vì dễ tưởng rằng hệ thống đang chạy đầy đủ perception/SLAM trong khi thực tế không chạy.

### 5. Nguy cơ xung đột actuator

Các node sau đều liên quan đến điều khiển phần cứng:

- `motor_main`
- `motor_main2`
- `CANbus_HAL`
- `stm_udp`
- `keyboard_input`
- `joy_node`

Nếu chạy sai profile, nhiều publisher có thể cùng ghi lệnh đến một actuator.

Mức độ: rất cao đối với robot thật.

### 6. Model và waypoint nằm ngoài workspace

Các file `/home/iris/*.onnx`, `/home/iris/*.pt`, `/home/iris/*.yaml` không được version hóa cùng source.

Mức độ: cao đối với reproducibility và deployment.

### 7. Dùng đường dẫn workspace suy ra từ `AMENT_PREFIX_PATH`

Launch file tự tạo:

```python
path_config_buffer = os.getenv('AMENT_PREFIX_PATH', '')
path_config_buffer_split = path_config_buffer.split(":")
ws_path = path_config_buffer_split[0] + "/../../"
```

Cách này phụ thuộc thứ tự `AMENT_PREFIX_PATH` và layout install. Nó dễ sai nếu:

- Có nhiều workspace overlay.
- Source được build trong container khác.
- Workspace được cài ở vị trí khác.
- `AMENT_PREFIX_PATH` rỗng.

Trong khi đó, package share directory đã được lấy bằng `get_package_share_directory('ros2_utils')`, nên config nên được lấy tương đối từ package share directory.

### 8. RTAB-Map và RealSense có thể không khớp topic

Một số node yêu cầu:

```text
/camera/rs2_cam_main/aligned_depth_to_color/image_raw
```

nhưng một profile lại đặt:

```text
align_depth.enable: false
```

Mức độ: cao đối với SLAM.

### 9. Occupancy grid có cấu hình rất lớn

`200 x 200 m`, độ phân giải `0.03 m` tạo khoảng 44 triệu cell.

Mức độ: trung bình đến cao tùy cách cấp phát bộ nhớ.

### 10. Logging quá nhiều thông tin nhạy cảm

`telemetry.py` log URL, username, password, org và bucket.

Mức độ: cao.

### 11. Không có launch argument rõ ràng

Các thông số profile đang nằm cứng trong Python:

- Robot name.
- Model path.
- Serial port.
- CAN bitrate.
- Chế độ camera.
- Waypoint path.
- Tuning motion.

Điều này làm deployment nhiều robot khó và dễ sửa nhầm.

## Cấu hình runtime hiện tại có vẻ như thế nào?

Dựa trên `run.sh`, runtime mặc định là:

```text
run.sh
  -> source install/setup.bash
  -> ROS_LOCALHOST_ONLY=1
  -> ros2 launch ros2_utils all.launch.py
```

Với `all.launch.py`, các thành phần chắc chắn được đưa vào `LaunchDescription` gồm:

- `rosapi_node`
- `ui_server`
- `rosbridge_websocket`
- `web_video_server`
- `pose_estimator`
- Static TF
- `master`
- `imu_serial`
- `CANbus_HAL`
- `motor_main`

Nhiều thành phần perception và SLAM đang bị comment. Vì vậy tên `all.launch.py` hơi gây hiểu nhầm: nó không thực sự bật toàn bộ hệ thống ở trạng thái hiện tại.

## Cách kiểm tra workspace khi chạy thật

Sau khi source ROS 2 và build workspace, các lệnh kiểm tra quan trọng là:

```bash
ros2 node list
ros2 topic list
ros2 topic info /master/target_speed
ros2 topic info /master/target_steering
ros2 topic echo /odom
ros2 topic echo /hardware/imu
ros2 topic hz /hardware/wheel_encoder
ros2 topic hz /vision/color_image
ros2 run tf2_tools view_frames
```

Kiểm tra graph:

```bash
rqt_graph
```

Kiểm tra các node điều khiển phần cứng:

```bash
ros2 node info /master
ros2 node info /motor_main
ros2 node info /CANbus_HAL
ros2 node info /pose_estimator
```

Kiểm tra parameter:

```bash
ros2 param list /master
ros2 param dump /master
ros2 param list /CANbus_HAL
ros2 param dump /motor_main
```

Kiểm tra thiết bị:

```bash
ip link show can0
ls -l /dev/serial/by-id/
ls -l /dev/imu_usb
```

## Kết luận

Workspace này là một stack robot tự hành tương đối đầy đủ, được tổ chức theo các tầng:

```text
hardware
  -> state estimation
  -> perception
  -> world model / SLAM
  -> master FSM
  -> motor control
  -> hardware driver
```

Điểm trung tâm là node `master`. Nó nhận dữ liệu từ vision, odometry, joystick và web; sau đó xuất tốc độ/góc lái cho `motor_main`.

Ba file cần đọc đầu tiên khi tiếp tục phân tích là:

- `all.launch.py`
- `master.launch.py`
- `master.cpp`

Các rủi ro cần xử lý trước khi chạy robot thật:

1. Xác định chỉ một node sở hữu actuator.
2. Xác định launch profile chính.
3. Kiểm tra các node đang thực sự được bật.
4. Kiểm tra toàn bộ TF tree.
5. Di chuyển InfluxDB credentials khỏi source và log.
6. Sửa/kiểm tra đơn vị timer trong `telemetry.py`.
7. Kiểm tra model, waypoint và device path trên máy triển khai.
8. Không chạy `init_all.sh` trên workspace hiện tại vì script xóa toàn bộ `src`.