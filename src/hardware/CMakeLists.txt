cmake_minimum_required(VERSION 3.8)
project(hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(var_dependencies
    ament_cmake
    rclcpp
    yaml-cpp
    std_msgs
    nav_msgs
    ros2_utils
    ros2_interface
    tf2_geometry_msgs
    sensor_msgs
    tf2_ros)

find_package(yaml-cpp REQUIRED)
  
foreach(var_dependency ${var_dependencies})
  find_package(${var_dependency} REQUIRED)
endforeach()

find_package(Boost REQUIRED COMPONENTS system)


include_directories(include)

add_executable(keyboard_input src/keyboard_input.cpp)
ament_target_dependencies(keyboard_input rclcpp std_msgs ros2_utils)

add_executable(CANbus_HAL src/CANbus_HAL.cpp)
ament_target_dependencies(CANbus_HAL rclcpp std_msgs ros2_utils)

add_executable(motor_main src/motor_main.cpp)
ament_target_dependencies(motor_main ${var_dependencies})

add_executable(motor_main2 src/motor_main2.cpp)
ament_target_dependencies(motor_main2 ${var_dependencies} yaml-cpp)
target_link_libraries(motor_main2 yaml-cpp)

add_executable(stm_udp src/stm_udp.cpp)
ament_target_dependencies(stm_udp ${var_dependencies} yaml-cpp)
target_link_libraries(stm_udp yaml-cpp)

add_executable(serial_imu src/serial_imu.cpp)
ament_target_dependencies(serial_imu rclcpp std_msgs sensor_msgs ros2_utils tf2_ros)
target_link_libraries(serial_imu ${Boost_LIBRARIES})
set_target_properties(serial_imu PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)


install(TARGETS
  keyboard_input
  CANbus_HAL
  motor_main
  motor_main2
  serial_imu
  stm_udp
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
