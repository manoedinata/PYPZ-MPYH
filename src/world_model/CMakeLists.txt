cmake_minimum_required(VERSION 3.8)
project(world_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find dependencies
set(var_dependencies
    ament_cmake
    rclcpp
    std_msgs
    nav_msgs
    tf2
    ros2_utils
    tf2_geometry_msgs
    pcl_conversions
    pcl_ros
    sensor_msgs
    tf2_eigen
    tf2_sensor_msgs
    tf2_ros)
    
foreach(var_dependency ${var_dependencies})
  find_package(${var_dependency} REQUIRED)
endforeach()
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pose_estimator src/pose_estimator.cpp)
ament_target_dependencies(pose_estimator ${var_dependencies})

add_executable(occupancy_grid src/occupancy_grid.cpp)
ament_target_dependencies(occupancy_grid ${var_dependencies})

install(TARGETS
  pose_estimator
  occupancy_grid
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
