cmake_minimum_required(VERSION 3.8)
project(vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# List semua dependencies yang digunakan
set(var_dependencies
    ament_cmake
    ament_cmake_python  
    rclcpp
    rclpy
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf2_ros
    tf2_sensor_msgs
    tf2_geometry_msgs
    cv_bridge
    OpenCV
    yaml-cpp
    pcl_ros
    pcl_conversions
    ros2_utils
    ros2_interface
    visualization_msgs 
)

find_package(yaml-cpp REQUIRED)

include_directories(include)

# Temukan semua dependencies
foreach(var_dependency ${var_dependencies})
find_package(${var_dependency} REQUIRED)
endforeach()
find_package(tf2_geometry_msgs REQUIRED)
# find_library(ONNXRUNTIME_LIB onnxruntime PATHS /home/iris/onnxruntime-linux-x64-1.22.0/lib NO_DEFAULT_PATH REQUIRED)
find_package(mlpack REQUIRED)
find_package(Armadillo REQUIRED)
find_package(OpenMP REQUIRED)

# Tambah eksekusi untuk node capture dan detection (sesuai proyek sebelumnya)
add_executable(vision_capture src/vision_capture.cpp)
find_package(realsense2 REQUIRED)
ament_target_dependencies(vision_capture ${var_dependencies})
target_link_libraries(vision_capture ${realsense2_LIBRARY})

add_executable(vision_capture2 src/vision_capture2.cpp)
find_package(realsense2 REQUIRED)
ament_target_dependencies(vision_capture2 ${var_dependencies})
target_link_libraries(vision_capture2 ${realsense2_LIBRARY} yaml-cpp mlpack armadillo ${OpenMP_CXX_LIBRARIES})
target_include_directories(vision_capture2 PRIVATE
    ${OpenMP_CXX_INCLUDE_DIRS}
)
target_compile_options(vision_capture2 PRIVATE ${OpenMP_CXX_FLAGS})

add_executable(vision_capture4 src/vision_capture4.cpp)
find_package(realsense2 REQUIRED)
ament_target_dependencies(vision_capture4 ${var_dependencies})
target_link_libraries(vision_capture4 ${realsense2_LIBRARY} yaml-cpp mlpack armadillo ${OpenMP_CXX_LIBRARIES})
target_include_directories(vision_capture4 PRIVATE
    ${OpenMP_CXX_INCLUDE_DIRS}
)
target_compile_options(vision_capture4 PRIVATE ${OpenMP_CXX_FLAGS})

add_executable(vision_capture3 src/vision_capture3.cpp)
find_package(realsense2 REQUIRED)
ament_target_dependencies(vision_capture3 ${var_dependencies})
target_link_libraries(vision_capture3 ${realsense2_LIBRARY} yaml-cpp mlpack armadillo ${OpenMP_CXX_LIBRARIES})
target_include_directories(vision_capture3 PRIVATE
    ${OpenMP_CXX_INCLUDE_DIRS}
)
target_compile_options(vision_capture3 PRIVATE ${OpenMP_CXX_FLAGS})


add_executable(detection src/detection.cpp)
ament_target_dependencies(detection ${var_dependencies})

add_executable(detection2 src/detection2.cpp)
ament_target_dependencies(detection2 ${var_dependencies})

add_executable(detection_urban src/detection_urban.cpp)
ament_target_dependencies(detection_urban ${var_dependencies})
target_link_libraries(detection_urban yaml-cpp)

find_library(APRILTAG_LIBRARY apriltag PATHS /usr/local/lib /usr/lib)
if(NOT APRILTAG_LIBRARY)
message(FATAL_ERROR "apriltag library not found")
endif()

add_executable(apriltag3 src/apriltag3.cpp)
ament_target_dependencies(apriltag3 ${var_dependencies})

target_link_libraries(apriltag3 ${OpenCV_LIBS} ${APRILTAG_LIBRARY})
target_include_directories(apriltag3 PRIVATE /usr/local/include/apriltag)

set_target_properties(apriltag3 PROPERTIES
    INSTALL_RPATH "/usr/local/lib"
    INSTALL_RPATH_USE_LINK_PATH TRUE
)

target_link_libraries(apriltag3 ${OpenCV_LIBS} apriltag)
target_include_directories(apriltag3 PRIVATE /usr/local/include/apriltag)

add_executable(apriltag4 src/apriltag4.cpp)
ament_target_dependencies(apriltag4 ${var_dependencies})

target_link_libraries(apriltag4 ${OpenCV_LIBS} ${APRILTAG_LIBRARY})
target_include_directories(apriltag4 PRIVATE /usr/local/include/apriltag)

set_target_properties(apriltag4 PROPERTIES
    INSTALL_RPATH "/usr/local/lib"
    INSTALL_RPATH_USE_LINK_PATH TRUE
)

target_link_libraries(apriltag4 ${OpenCV_LIBS} apriltag)
target_include_directories(apriltag4 PRIVATE /usr/local/include/apriltag)

add_executable(lane_detection src/lane_detection.cpp)
ament_target_dependencies(lane_detection ${var_dependencies})

# add_executable(onnx_inference_node src/onnx_inference_node.cpp)
# ament_target_dependencies(onnx_inference_node ${var_dependencies})
# target_include_directories(onnx_inference_node PRIVATE ${OpenCV_INCLUDE_DIRS} /home/iris/onnxruntime-linux-x64-1.22.0/include)

# target_link_libraries(onnx_inference_node ${ONNXRUNTIME_LIB})

ament_python_install_package(${PROJECT_NAME})

# Install semua binary
install(TARGETS
  vision_capture
  vision_capture2
  vision_capture3
  vision_capture4
  detection
  detection2
  detection_urban
  lane_detection
  # onnx_inference_node
  apriltag3 
  apriltag4 
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python script
install(PROGRAMS
  scripts/onnx_lane_following_node.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()



