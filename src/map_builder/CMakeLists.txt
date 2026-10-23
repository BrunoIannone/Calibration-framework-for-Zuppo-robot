cmake_minimum_required(VERSION 3.8)
project(map_builder)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(map_builder
  src/map_builder.cpp
  src/node.cpp)
# Include directories
target_include_directories(map_builder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(map_builder
  rclcpp
  sensor_msgs
  nav_msgs
  tf2_ros
  )

install(TARGETS
  map_builder
  DESTINATION lib/${PROJECT_NAME})

ament_package()