cmake_minimum_required(VERSION 3.8)
project(ugv_imu C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(ugv_imu_node src/imu_node.cpp src/icm20948.c)
target_include_directories(ugv_imu_node PRIVATE include)
target_link_libraries(ugv_imu_node m)
ament_target_dependencies(ugv_imu_node rclcpp sensor_msgs)

install(TARGETS ugv_imu_node DESTINATION lib/${PROJECT_NAME})

ament_package()
