cmake_minimum_required(VERSION 3.8)
project(lidar_publisher)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Executable
add_executable(lidar_publisher
  src/lidar_publisher.cpp
  src/node.cpp
)

# Include directories
target_include_directories(lidar_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# ROS dependencies
ament_target_dependencies(lidar_publisher
  rclcpp
  sensor_msgs
  tf2_ros
  nav_msgs
  geometry_msgs
)

# Link Eigen
target_link_libraries(lidar_publisher
  Eigen3::Eigen
)

# Install target
install(TARGETS
  lidar_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()