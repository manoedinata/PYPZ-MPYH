# DETAILS.md — Analisis Workspace ROS 2 `fira`

Workspace ROS 2 Humble untuk robot autonomous skala kecil berpenggerak Ackermann (roda + servo kemudi). Fokus kompetisi/riset: mengikuti jalur (lane following), menghindari obstacle, deteksi sign/AprilTag, waypoint navigation, dan SLAM.

Lokasi workspace: `/home/nata/IRIS/fira`

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

---

## Daftar Isi

1. [Struktur Package](#struktur-package)
2. [Package `hardware`](#package-hardware)
3. [Package `communication`](#package-communication)
4. [Package `master`](#package-master)
5. [Package `vision`](#package-vision)
6. [Package `apriltag_detection`](#package-apriltag_detection)
7. [Package `world_model`](#package-world_model)
8. [Package `ros2_interface`](#package-ros2_interface)
9. [Package `ros2_utils`](#package-ros2_utils)
10. [Package `web_ui`](#package-web_ui)
11. [Package `realsense2_camera` dan `realsense2_camera_msgs`](#package-realsense2_camera-dan-realsense2_camera_msgs)
12. [Package `wit_ros2_imu`](#package-wit_ros2_imu)
13. [Luflow Data Kontrol](#aliran-data-kontrol)
14. [Aliran Perception dan SLAM](#aliran-perception-dan-slam)
15. [TF Tree](#tf-tree)
16. [Script Operasional](#script-operasional)
17. [Masalah Teknis Penting](#masalah-teknis-penting)
18. [Konfigurasi Runtime Saat Ini](#konfigurasi-runtime-saat-ini)
19. [Cara Memeriksa Workspace Saat Dijalankan](#cara-memeriksa-workspace-saat-dijalankan)
20. [Kesimpulan](#kesimpulan)

---

## Struktur Package

```
src/
├── apriltag_detection/       # Deteksi AprilTag, YOLO, lane ONNX (Python)
├── communication/            # WiFi control + telemetry InfluxDB
├── hardware/                 # CAN, UDP STM, motor, IMU serial, keyboard
├── master/                   # FSM utama / pengambil keputusan
├── realsense2_camera/        # Driver Intel RealSense (vendor source)
├── realsense2_camera_msgs/   # Msg/srv untuk driver RealSense
├── ros2_interface/           # Custom msg/srv workspace
├── ros2_utils/               # Launch files, config, helper library
├── vision/                   # Computer vision (C++, OpenCV + ONNX)
├── web_ui/                   # Dashboard web (HTML/JS + HTTP server)
├── wit_ros2_imu/             # Driver IMU WitMotion (Python)
└── world_model/              # Pose estimator + occupancy grid
```

Root workspace juga berisi:

- `run.sh`, `make.sh` — build dan run.
- `1_stop.sh` / `2_start.sh` — kontrol systemd (`run_main.service`).
- `init_all.sh` — scaffolding awal (berbahaya, lihat di bawah).
- `mediamtx.yml` — konfigurasi media server RTSP.
- `dynamic_conf.yaml` — konfigurasi tuning motor.
- `README.md` — hanya template.

---

## Package `hardware`

Source: `src/hardware/src/`

| Node | Fungsi |
|---|---|
| `CANbus_HAL.cpp` | HAL SocketCAN untuk motor roda + servo kemudi |
| `motor_main.cpp` | Kontrol kecepatan roda (PID) + posisi kemudi |
| `motor_main2.cpp` | Varian eksperimen kontrol motor |
| `stm_udp.cpp` | Jembatan UDP ke mikrokontroler STM |
| `serial_imu.cpp` | Driver IMU via serial (Riontech) |
| `keyboard_input.cpp` | Input keyboard untuk teleop |

### `CANbus_HAL.cpp`

Menggunakan SocketCAN Linux:

```cpp
#include <linux/can.h>
#include <linux/can/raw.h>
```

Parameter penting (default di kode vs. nilai di launch):

| Parameter | Default kode | Launch `all.launch.py` | Launch `master.launch.py` |
|---|---|---|---|
| `if_name` | `can0` | (default) | (default) |
| `bitrate` | `125000` | `1000000` | `1000000` |
| `routine_period_ms` | `20` | `5` | `10` |
| `max_can_recv_error_counter` | `10` | `2` | `10` |
| `can_timeout_us` | `100000` | `50000` | `50000` |
| `id_driver_wheel` | `0x04` | `4` | `4` |
| `id_driver_steering` | `0x08` | `8` | `8` |

Publisher:

- `/hardware/wheel_encoder`
- `/hardware/delta_encoder_wheel_counter`
- `/hardware/feedback_steering`

Subscriber:

- Target PWM roda
- Target kemudi
- Keyboard

Catatan penting: nilai di launch **meng-override** default di kode C++.

### `motor_main.cpp`

Topic:

- Publisher: `/motor_main/target_pwm_wheel`, `/motor_main/target_steering`, `/motor_main/feedback_steering_rad`, `/motor_main/velocity_feedback`
- Subscriber: encoder, feedback steering, `/master/target_speed`, `/master/target_steering`, `/hardware/imu`

Parameter kunci di launch:

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

Ini loop kontrol low-level: `master` memberi target kecepatan/kemudi, `motor_main` mengejar target, `CANbus_HAL` / `stm_udp` meneruskan perintah ke driver.

### `motor_main2.cpp`

Varian eksperimental. Diaktifkan di `all.launch.py`, tidak di `master.launch.py`.

Jangan jalankan `motor_main` dan `motor_main2` bersamaan bila keduanya mengirim perintah ke aktuator yang sama.

### `stm_udp.cpp`

Komunikasi UDP dua arah dengan STM:

- Subscribe: PWM roda, PWM enable, target kemudi, buzzer urban.
- Publish: encoder (`Int16`), button (`Int8`).
- Dua timer terpisah untuk kirim dan terima.
- Multithreaded executor.

Macro aktif:

```cpp
#define RECV_UDP
#define SEND_UDP
```

`stm_udp` dideklarasikan di `all.launch.py` namun tidak masuk `LaunchDescription` final. Hindari menjalankan bersama `CANbus_HAL` bila keduanya mengendalikan aktuator sama.

### `serial_imu`

Baca IMU serial. Di launch:

```text
is_riontech: true
baudrate: 115200
port: /dev/serial/by-id/usb-FTDI_...
```

Dua launch memakai device FTDI berbeda:

- `all.launch.py`: `A5069RR4`
- `master.launch.py`: `A50285BI`

Kemungkinan dua robot atau dua unit IMU berbeda.

---

## Package `communication`

### `wifi_control.cpp`

Mengelola/hubungkan hotspot WiFi robot. Parameter launch:

```text
hotspot_ssid: gh_template
hotspot_password: gh_template
```

### `scripts/telemetry.py`

Node Python yang membaca data ROS 2 dan menulis ke InfluxDB.

Subscribe:

- `/distance_travelled` (`Float32`)
- `/odom` (`Odometry`)
- `/can/battery` (`Int16`)

Juga membaca CPU/RAM via `psutil`.

Parameter InfluxDB di-set langsung di launch file:

```text
INFLUXDB_URL: http://172.30.37.21:8086
INFLUXDB_USERNAME: awm462
INFLUXDB_PASSWORD: wildan462
INFLUXDB_ORG: awmawm
INFLUXDB_BUCKET: ujiCoba
ROBOT_NAME: gh_template
```

**Masalah keamanan serius:**

- Username/password hardcoded di launch file.
- `telemetry.py` mencetak semua kredensial ini ke log.
- Kredensial dapat bocor lewat log ROS, debug log, atau riwayat git.

Sebaiknya pindahkan ke environment variable / file `.env` di luar repo / parameter file yang tidak di-commit.

**Bug potensial — satuan timer:**

```python
self.declare_parameter("publish_period", 10) # in ms
self.timer_routine = self.create_timer(self.publish_period, ...)
```

`rclpy.create_timer()` menerima **detik**, bukan milidetik. Nilai `10` berarti timer 10 detik, bukan 10 ms. Bila maksudnya 10 ms, gunakan `0.01`.

---

## Package `master`

Otak robot. File:

- `src/master/src/master.cpp`
- `src/master/src/motion.cpp`
- `src/master/src/master_definition.cpp`
- `src/master/include/master/master.hpp`

Timer utama: `callback_routine()` setiap **20 ms (50 Hz)**.

### FSM Global

Didefinisikan di `master.hpp`:

```cpp
#define FSM_GLOBAL_INIT                  0
#define FSM_GLOBAL_PREOP                 1
#define FSM_GLOBAL_SAFEOP                2
#define FSM_GLOBAL_OP_3                  3
#define FSM_GLOBAL_OP_4                  4
#define FSM_GLOBAL_OP_5                  5
#define FSM_GLOBAL_OP_2                  6
#define FSM_GLOBAL_RECORD_ROUTE          7
#define FSM_GLOBAL_MAPPING               8
#define FSM_GLOBAL_RECORD_ROUTE_KANAN    9
#define FSM_GLOBAL_RECORD_ROUTE_KIRI     10
#define FSM_GLOBAL_RECORD_ROUTE_TENGAH   11
#define FSM_GLOBAL_RACE_BUTTON           12
#define FSM_GLOBAL_RECORD_DATASET_ROAD   20
#define FSM_GLOBAL_RECORD_DATASET_ROAD_VIDEO 20
#define FSM_GLOBAL_CUSTOM_DEBUG_1        300
#define FSM_GLOBAL_CUSTOM_DEBUG_2        301
```

Makna kasar tiap state (dari `master.cpp`):

| State | Perilaku |
|---|---|
| `INIT` | Load/simpan waypoint & terminal |
| `PREOP` | Manual via joystick |
| `OP_3` | Mode urban |
| `OP_4` | Manual motion dari data vision |
| `OP_5` | Follow waypoint, gas manual |
| `RACE_BUTTON` | Race pakai kecepatan vision |
| `100` | Urban otomatis (`urban_move2`) |
| `200` | Race (`race_move`) |
| `RECORD_ROUTE*` | Rekam rute (normal / kanan / kiri / tengah) |
| `RECORD_DATASET_ROAD` | Teleop manual untuk rekam dataset |

### Publisher `master`

- `/master/target_speed`
- `/master/target_steering`
- `/master/global_fsm`
- `/master/local_fsm`
- `/master/waypoints`, `/master/waypoints_kanan`, `/master/waypoints_kiri`, `/master/waypoints_tengah`
- `/master/path_point`
- `/master/nearest_obstacle`
- `/master/buffered_obs_pointcloud`
- `/master/buffered_road_pointcloud`
- `/master/edge_left_road_pointcloud`, `/master/edge_right_road_pointcloud`
- `/master/most_left_obs`, `/master/most_right_obs`
- `/master/target_pt_avoid`
- `/master/free_path_map`
- `/master/terminals`
- `/master/state_urban`
- `/master/sign_buzzer`
- `/master/curr_gyro_deg`
- `/master/posisi_robot`, `/master/posisi_obstacle`

### Subscriber `master`

- `/odom`
- `/key_pressed`
- `/web/selected_lane`, `/key_web_pressed`, `/web/toggle_debug`, `/web/toggle_debug2`
- `/master/ui_target_velocity_and_steering`
- `/master/set_master_fsm`
- `/vision/slope`, `/vision/velocity`, `/vision/intersection_point`
- `/vision/urban_data` (custom msg `VisionUrban`), `/vision/master_config`
- `/sign/marker/id`, `/sign/picture/id`
- `/hardware/joy`, `/hardware/button`
- `/apriltag/markers`
- `/motor_main/velocity_feedback`

### Services `master`

Semua bertipe `std_srvs/SetBool`:

- `/master/set_record_route_mode`
- `/master/set_record_route_mode_kanan`
- `/master/set_record_route_mode_kiri`
- `/master/set_record_route_mode_tengah`
- `/master/set_add_record_route_mode`
- `/master/set_terminal`
- `/master/set_terminal_sign`
- `/master/rm_terminal`

### Parameter `master`

Dari launch:

```text
profile_max_acceleration     : 8.0   (all) / 2000.0 (master)
profile_max_decceleration    : 8.0   / 2000.0
profile_max_velocity         : 3.0   / 1.5
profile_max_accelerate_jerk  : 1600  / 30000
profile_max_decelerate_jerk  : 16000 / 30000
profile_max_braking          : 3.0
profile_max_braking_acceleration : 8.0 / 10000.0
profile_max_braking_jerk     : 1600  / 1000
profile_max_steering_rad     : 0.61  / 0.52
wheelbase                    : 0.27
default_lookahead            : 0.8

lama_waktu_menghindar        : 40
kecepatan_default_menghindar : 0.7
offset_jarak_hindar          : 0.4

max_counter_lurus            : 5
max_counter_belok_kiri       : 60
max_counter_belok_kanan      : 75
max_counter_lurus_awal_kiri  : 15
max_counter_lurus_awal_kanan : 35
```

File eksternal (hard-coded):

```text
/home/iris/waypoints.yaml
/home/iris/waypoints_race_kanan.yaml
/home/iris/waypoints_race_kiri.yaml
/home/iris/waypoints_race_tengah.yaml
/home/iris/terminal.yaml
```

File-file ini **tidak ada di workspace** — harus disiapkan manual di mesin target.

---

## Package `vision`

C++ / OpenCV. Source: `src/vision/src/`

| Node | Fungsi diperkirakan |
|---|---|
| `vision_capture.cpp` | Ambil frame RealSense, proses awal, publish color/depth/pointcloud/laserscan |
| `vision_capture2/3/4.cpp` | Varian lane follower dengan parameter berbeda |
| `detection.cpp` / `detection2.cpp` | Deteksi obstacle / road pointcloud |
| `detection_urban.cpp` | Deteksi mode urban |
| `lane_detection.cpp` | Deteksi lajur |
| `ml_detection.cpp` | Deteksi berbasis ML |
| `onnx_inference_node.cpp` | Inferensi ONNX (CNN) |
| `apriltag3.cpp` / `apriltag4.cpp` | Deteksi AprilTag ( dua varian) |
| `aruco_detection.cpp` | Deteksi ArUco |

`vision_capture` publish banyak topik (lihat `create_publisher` di file):

- `/vision/color_image`, `/vision/depth_image`, `/vision/camera_info`
- `/vision/laserscan`
- `/vision/slope`, `/vision/velocity`
- `/vision/filtered_points`, `/vision/cleaned_pointcloud`, `/vision/yuv_pointcloud`
- `/vision/sign_points`
- `/vision/road_binary`, `/vision/filtered_binary`
- `/vision/pointcloud`, `/vision/imagecloud`
- `ros2_interface/Apriltag`

Topik yang dipakai langsung oleh `master`:

```text
/vision/slope
/vision/velocity
/vision/intersection_point
/vision/urban_data
```

Di `master.launch.py` banyak node vision aktif. Di `all.launch.py` banyak yang **dideklarasikan tapi dikomentari** di `LaunchDescription`.

---

## Package `apriltag_detection`

Package Python (`setup.py`): `src/apriltag_detection/apriltag_detection/`

- `apriltag_detection.py`
- `onnx_lane_detection.py`
- `onnx_sign_detection.py`
- `yolo_detection.py`

Model yang dirujuk (eksternal, di luar workspace):

```text
/home/iris/model.onnx
/home/iris/model_19_juni.onnx
/home/iris/best.pt
/home/iris/best224.onnx
/home/iris/best_fira3_openvino_model/
```

Parameter YOLO di launch:

```text
confidence_threshold: 0.6
iou_threshold: 0.7
camera_string: /dev/v4l/by-id/usb-e-con_systems_See3CAM_CU55_...
```

---

## Package `world_model`

Dua node:

### `pose_estimator.cpp`

Fusi encoder + gyro untuk menghasilkan `/odom`.

- Subscribe: `/hardware/wheel_encoder` (`Int16`), `/hardware/imu` (`Imu`), `/master/pose_offset` (`Odometry`)
- Publish: `/odom`, `/pose_estimator/encoder_meter`
- Publish TF via `TransformBroadcaster`
- Timer default: 20 ms

State internal:

```text
final_pose_x, final_pose_y, final_pose_theta
final_vel_x, final_vel_y, final_vel_theta
```

Parameter: `encoder_to_meter` (default `1.0`; di launch `-0.00000490586`), `routine_period_ms`.

### `occupancy_grid.cpp`

- Subscribe: pointcloud obstacle + road.
- Publish: `grid_map` (`nav_msgs/OccupancyGrid`).

Parameter di launch:

```text
res: 0.03
width: 200
height: 200
ox: -1.0
oy: -1.0
memory_timeout_sec: 5.0
blind_spot_radius: 0.2
```

Grid 200 m x 200 m dengan resolusi 3 cm ≈ 6.667 x 6.667 ≈ **44 juta sel**. Bergantung implementasi, bisa sangat berat di memori.

---

## Package `ros2_interface`

Custom message/service workspace.

Msg:

- `Apriltag.msg`, `apriltag.msg`
- `PointArray.msg`
- `Terminal.msg`, `TerminalArray.msg`
- `VisionUrban.msg`

`VisionUrban.msg` (kontrak data `vision` → `master` untuk mode urban):

```
int8 berhenti
int16 pos_target_px_x
int16 pos_target_px_y
int16 pos_robot_px_x
int16 pos_robot_px_y
float32 dist_putih_meter
float32 dist_near_zebracross
float32 target_angle_ungu
float32 target_angle_putih
float32 meter_to_pixel
float32 offset_angle_zebracross
float32 offset_angle_lane
float32 dist_near_zebracross_vertical
float32 dist_near_zebracross_horizontal
float32 dist_near_zebracross_vertical_kiri
float32 dist_near_zebracross_vertical_kanan
float32 centroid_sign_x
float32 centroid_sign_y
float32 centroid_obs_x
float32 centroid_obs_y
int8 jalan_berkelok
int8 mask_jalan_bocor
int8 ada_pertigaan
float32 jarak_ke_pertigaan
```

Srv: hanya `Dummy.srv`.

---

## Package `ros2_utils`

Helper + infrastruktur:

- `include/ros2_utils/global_definitions.hpp`
- `include/ros2_utils/help_logger.hpp` (dipakai semua node C++)
- `include/ros2_utils/pid.hpp`
- `include/ros2_utils/rtabmap_params.h`
- `include/ros2_utils/simple_fsm.hpp`
- `include/ros2_utils/system_utils.hpp`
- `src/help_logger.cpp`
- `configs/cyclonedds.xml`, `dynamic_conf.yaml`, `robot.rviz`, `static_conf.yaml` (kosong)
- Launch files: `all.launch.py`, `master.launch.py`, `rs_launch.py`, `rs_cam_main.py`, `test_rs_rtabmap.launch.py`

### `all.launch.py` (dipanggil `run.sh`)

Mendeklarasikan hampir seluruh stack:

- `rviz2`
- `rosbridge_websocket`, `web_video_server`, `rosapi`
- RealSense `realsense2_camera_node` (`rs2_cam_main`)
- `wit_ros2_imu`
- `rtabmap_slam/rtabmap` (namespace `slam`)
- `wifi_control`, `telemetry`
- `web_ui` (`ui_server`)
- `master`
- `stm_udp`, `CANbus_HAL`, `motor_main`, `motor_main2`
- `serial_imu`, `keyboard_input`, `joy_node`
- `pose_estimator`, `occupancy_grid`
- `vision_capture`, `vision_capture2/3/4`, `onnx_inference_node`, `detection`, `detection2`, `detection_urban`, `lane_detection`
- `apriltag_detection`, `apriltag3`, `apriltag4`
- `yolo_detection`, `onnx_lane_detection`
- `rtabmap_slam_rtabmap3`, `imu_filter_madgwick_node`, `rgbd_odometry`, `ekf_node`
- Static TF (map→odom, base_link→body_link/imu_link/camera_link/camera_iris, camera_iris→optical)

**Namun `LaunchDescription` final hanya mengaktifkan sebagian.** Node aktif saat ini:

```python
rosapi_node, ui_server, rosbridge_server, web_video_server,
pose_estimator,
tf_base_link_to_body_link, tf_base_link_to_imu_link,
tf_base_link_to_camera_link, tf_base_link_to_camera_iris,
tf_camera_iris_to_camera_color_optical_frame,
master,
imu_serial,
CANbus_HAL,
motor_main,
```

Node kamera, vision, SLAM, EKF, joystick, `wit_ros2_imu`, `occupancy_grid` **dikomentari**.

Cara path configuration ditemukan rentan:

```python
path_config_buffer = os.getenv('AMENT_PREFIX_PATH', '')
ws_path = path_config_buffer.split(":")[0] + "/../../"
path_config = ws_path + "src/ros2_utils/configs/"
```

Bergantung urutan `AMENT_PREFIX_PATH` dan layout install. Sebaiknya gunakan `get_package_share_directory`.

### `master.launch.py`

Profil "penuh" yang berbeda secara signifikan:

- RealSense `align_depth.enable: True` + `pointcloud.enable: True` (di `all.launch.py`: keduanya `False`).
- `vision_capture2/3/4` dan `detection_urban` ada di sini.
- `rtabmap_slam_rtabmap3` dan `rgbd_odometry` dikonfigurasi panjang (ICP, GTSAM, Grid, dll.) — atribusi komentar "Added by Azzam".
- `ekf_node` (`robot_localization`), `imu_filter_madgwick_node`.
- `master` dengan parameter kontrol profil berbeda (mis. `profile_max_velocity` 1.5 vs 3.0 di `all.launch.py`).
- `CANbus_HAL` dengan `routine_period_ms: 10` (vs 5 di `all.launch.py`).
- Port IMU serial berbeda.
- Tidak ada `stm_udp` / `motor_main2`.

**Kesimpulan: dua launch file = dua profil runtime berbeda, bukan duplikat.**

---

## Package `web_ui`

File: `src/web_ui/src/`

- `index.html` + `index.js` + `index.css`
- `ROS_if.html` / `ROS_if.js` / `ROS_if_imv.html` / `ROS_if_imv.js` — interface utama via roslib
- `ROS_config.html` / `ROS_config.js` — halaman konfigurasi
- `config.html` / `config.js`
- `slam.html` / `slam.js` — tampilan SLAM
- `testjs.html`
- Library vendor: `roslib.min.js`, `bulma.min.css`, `konva.min.js`, `anime.min.js`

### `scripts/ui_server.py`

Node Python yang menjalankan `python3 -m http.server` di folder `src/web_ui/src/` sebagai subprocess. Node ini bukan HTTP server ROS-native — hanya wrapper proses HTTP.

Port terkait di `run.sh`:

```bash
sudo fuser -k 9090/tcp   # rosbridge websocket
sudo fuser -k 8080/tcp   # web_video_server
sudo fuser -k 8000/tcp   # http.server UI (default)
```

---

## Package `realsense2_camera` dan `realsense2_camera_msgs`

Vendor source Intel RealSense untuk ROS 2. `realsense2_camera_msgs` berisi msg/srv khusus driver.

Konfigurasi kamera di launch:

| Parameter | `all.launch.py` | `master.launch.py` |
|---|---|---|
| `enable_depth` | True | True |
| `enable_color` | True | True |
| `enable_sync` | True | True |
| `unite_imu_method` | 2 | 2 |
| `align_depth.enable` | **False** | True |
| `pointcloud.enable` | **False** | True |
| `rgb_camera.color_profile` | `640x360x60` | (default) |
| `depth_module.depth_profile` | `640x360x60` | (default) |

**Perhatian:** RTAB-Map di-remap ke `/camera/rs2_cam_main/aligned_depth_to_color/image_raw`. Jika `align_depth.enable: False` (kasus `all.launch.py`), topik aligned depth tidak ada → SLAM tidak dapat depth yang cocok dengan color.

---

## Package `wit_ros2_imu`

Driver IMU WitMotion (Python). Di launch di-remap:

```text
/imu/data_raw → /hardware/imu
```

Deklarasikan di `all.launch.py` tetapi **dikomentari** di `LaunchDescription`. Diaktifkan sebagai gantinya `serial_imu` dari package `hardware`. Jangan aktifkan keduanya bersamaan bila keduanya publish ke `/hardware/imu`.

---

## Aliran Data Kontrol

```
Joystick / keyboard / web
        ↓
     master  (FSM 50 Hz)
        ↓
 /master/target_speed  +  /master/target_steering
        ↓
    motor_main  (PID kecepatan + posisi kemudi, 50 Hz)
        ↓
 /motor_main/target_pwm_wheel + /motor_main/target_steering
        ↓
  CANbus_HAL (atau stm_udp, tergantung konfigurasi)
        ↓
  Driver motor roda + servo kemudi (CAN ID 4 dan 8)
```

Feedback balik:

```
Encoder roda  ← CANbus_HAL ← /hardware/wheel_encoder
Feedback servo ← CANbus_HAL ← /hardware/feedback_steering
IMU          ← serial_imu   ← /hardware/imu
Encoder+IMU  → pose_estimator → /odom
```

---

## Aliran Perception dan SLAM

```
RealSense (rs2_cam_main)
  ├── /camera/.../color/image_raw
  ├── /camera/.../aligned_depth_to_color/image_raw
  └── /camera/.../color/camera_info
        ↓
vision_capture / detection / lane_detection / apriltag3
        ↓
 /vision/slope, /vision/velocity, /vision/intersection_point
 /vision/urban_data (VisionUrban), /vision/laserscan
 /vision/pointcloud_*, /sign/marker/id, /sign/picture/id
        ↓
   master (keputusan)  +  occupancy_grid  +  RTAB-Map
```

RTAB-Map (`rtabmap_slam_rtabmap3`) di-remap ke:

- `odom` ← `/slam_vo/odom` (dari `rgbd_odometry`)
- `rgb/image` ← `/vision/color_image`
- `depth/image` ← `/vision/depth_image`
- `rgb/camera_info` ← `/vision/camera_info`
- `scan` ← `/vision/laserscan`

Parameter penting RTAB-Map:

- `Reg/Force3DoF: True` — hanya gerak planar.
- `Icp/Strategy: 1`, `Icp/PointToPlane: True`.
- `Grid/IncrementalMapping: True`.
- `publish_tf: False` — RTAB-Map tidak publish TF.
- `use_saved_map: False`.
- `Threads: 12`.

`ekf_node` (`robot_localization`) dideklarasikan tapi dikomentari. Jangan aktifkan bersamaan dengan RTAB-Map / pose_estimator bila belum sepakat siapa yang publish TF `map→odom` dan `odom→base_link`.

---

## TF Tree

Static transform yang di-publish:

```text
map → odom                                    (identity, ditandai "sementara")
base_link → body_link                          (0.175, 0, 0.165)
base_link → imu_link                           (0.19, 0, 0)
base_link → camera_link                        (0, 0, 0.28, pitch 0.5)
base_link → camera_iris                        (0.17, 0, 0.28, pitch 0.5)
camera_iris → camera_color_optical_frame       (rotasi -1.5708, 0, -1.5708)
```

Risiko:

- `pose_estimator` memiliki `TransformBroadcaster` (publish TF dinamis).
- RTAB-Map `publish_tf: False`.
- EKF `publish_tf: False`.
- Static `map → odom` bisa konflik desain bila nanti SLAM/EKF yang harus memegang transform ini.
- `camera_link`, `camera_iris`, `camera_color_optical_frame` harus konsisten dengan frame yang dipublish driver RealSense.

---

## Script Operasional

### Build — `make.sh`

```bash
export ROS_DISTRO=humble
colcon build --symlink-install --executor parallel --parallel $(nproc)
```

Catatan:

- Tidak `source /opt/ros/humble/setup.bash`.
- Tidak ada penanganan error build.
- `--symlink-install` cocok untuk dev Python/config.

### Run — `run.sh`

```bash
sudo fuser -k 9090/tcp
sudo fuser -k 8080/tcp
sudo fuser -k 8000/tcp

. install/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 launch ros2_utils all.launch.py
```

`ROS_LOCALHOST_ONLY=1` membatasi discovery DDS ke localhost — aman untuk robot tunggal, tapi node di mesin lain tidak bisa ikut.

### systemd — `1_stop.sh` / `2_start.sh`

```bash
# 1_stop.sh
sudo systemctl stop run_main.service

# 2_start.sh
sudo systemctl start run_main.service
```

Menandakan ada service systemd `run_main.service` di luar workspace.

### `init_all.sh` — **BERBAHAYA**

```bash
rm -rf src/
mkdir src/
```

Kemudian `ros2 pkg create` untuk 8 package. **Menjalankan script ini akan menghapus seluruh source code di `src/`** — termasuk kode C++, Python, custom message, RealSense, config, launch, test.

`README.md` saat ini justru mengarahkan menjalankan script ini untuk "Remake". Jangan jalankan pada workspace yang sudah ada isinya.

### `99_reset_time_cmake.sh`, `6_reset_port.sh`, `4_disable.sh`, `5_enable.sh`, `3_restart.sh`

Script operasional tambahan di root workspace, kemungkinan untuk maintenance. Belum dianalisa detail.

---

## Masalah Teknis Penting

| # | Masalah | Severity | Lokasi |
|---|---|---|---|
| 1 | Hard-coded credentials InfluxDB | **Tinggi** | `all.launch.py`, `telemetry.py` (juga di-log ke console) |
| 2 | `telemetry.py` timer pakai "ms" tapi `create_timer` butuh detik | Tinggi | `communication/scripts/telemetry.py` |
| 3 | `all.launch.py` dan `master.launch.py` tidak konsisten (camera, IMU port, CAN period, master params, vision, SLAM) | Tinggi | `ros2_utils/launch/` |
| 4 | Banyak node dideklarasikan tapi dikomentari di `LaunchDescription` — mudah salah paham stack penuh aktif | Tinggi | `all.launch.py` |
| 5 | Potensi konflik aktuator (`motor_main` vs `motor_main2`, `CANbus_HAL` vs `stm_udp`, `serial_imu` vs `wit_ros2_imu`) | Sangat tinggi | `hardware`, launch |
| 6 | Model ML dan waypoint file di `/home/iris/` — tidak di-version kontrol | Tinggi | referensi di launch |
| 7 | Path config via `AMENT_PREFIX_PATH[0] + "/../../"` — rapuh | Sedang | kedua launch file |
| 8 | RealSense `align_depth.enable: False` tapi RTAB-Map remap ke aligned depth topic | Tinggi | `all.launch.py` + RTAB remap |
| 9 | Occupancy grid 200x200 m @ 0.03 m ≈ 44 juta sel — potensi berat | Sedang–Tinggi | `occupancy_grid` |
| 10 | Logging kredensial sensitif | Tinggi | `telemetry.py` |
| 11 | Tidak ada launch argument yang jelas; semua profil di-hardcode dalam Python | Sedang | kedua launch file |
| 12 | `init_all.sh` menghapus `src/` — README menganjurkan untuk "remake" | Sangat tinggi | root + `README.md` |

---

## Konfigurasi Runtime Saat Ini

Berdasarkan `run.sh`, runtime default:

```
run.sh
  → source install/setup.bash
  → ROS_LOCALHOST_ONLY=1
  → ros2 launch ros2_utils all.launch.py
```

Node aktif dari `all.launch.py` saat ini:

- `rosapi_node`, `ui_server`, `rosbridge_server`, `web_video_server`
- `pose_estimator`
- Static TF (5 publikasi)
- `master`
- `imu_serial`
- `CANbus_HAL`
- `motor_main`

Perception dan SLAM **tidak aktif** di launch ini. Nama `all.launch.py` agak menyesatkan.

---

## Cara Memeriksa Workspace Saat Dijalankan

Setelah source ROS 2 Humble dan build workspace:

```bash
# Lihat node dan topik
ros2 node list
ros2 topic list

# Detail topik penting
ros2 topic info /master/target_speed
ros2 topic info /master/target_steering
ros2 topic echo /odom
ros2 topic echo /hardware/imu
ros2 topic hz /hardware/wheel_encoder
ros2 topic hz /vision/color_image

# TF
ros2 run tf2_tools view_frames

# Graph
rqt_graph

# Info node
ros2 node info /master
ros2 node info /motor_main
ros2 node info /CANbus_HAL
ros2 node info /pose_estimator

# Parameter
ros2 param list /master
ros2 param dump /master
ros2 param list /CANbus_HAL
ros2 param dump /motor_main

# Perangkat
ip link show can0
ls -l /dev/serial/by-id/
ls -l /dev/imu_usb
```

---

## Kesimpulan

Workspace `fira` adalah stack robot autonomous cukup lengkap, terstruktur berlapis:

```
hardware → state estimation → perception → world model/SLAM → master FSM → motor control → hardware driver
```

Pusat koordinasi adalah node `master` yang menerima data vision, odometry, joystick, dan web, lalu mengeluarkan perintah kecepatan/sudut kemudi ke `motor_main`.

Tiga file utama untuk memahami sistem:

1. `src/ros2_utils/launch/all.launch.py`
2. `src/ros2_utils/launch/master.launch.py`
3. `src/master/src/master.cpp`

Sebelum menjalankan di robot fisik, selesaikan:

1. Pastikan hanya satu node yang mengendalikan aktuator.
2. Tentukan launch profile utama (`all` vs `master`).
3. Audit node mana yang benar-benar diaktifkan.
4. Audit TF tree.
5. Pindahkan kredensial InfluxDB keluar dari source dan log.
6. Perbaiki / uji unit timer `telemetry.py`.
7. Pastikan model, waypoint, dan path perangkat tersedia di mesin target.
8. **Jangan jalankan `init_all.sh`** pada workspace yang sudah berisi kode.
