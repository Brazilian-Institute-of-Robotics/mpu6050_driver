cmake_minimum_required(VERSION 3.8)
project(mpu6050_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
)

# Define library target
add_library(mpu6050_node_lib
  src/mpu6050_node.cpp
  src/mpu6050.cpp
  src/i2c_device.cpp
)
ament_target_dependencies(mpu6050_node_lib rclcpp sensor_msgs)

# Link against ROS 2 libraries
# target_link_libraries(mpu6050_node_lib
#   ${rclcpp_LIBRARIES}
# )

# Define executable targets
add_executable(mpu6050_node src/main.cpp)
ament_target_dependencies(mpu6050_node rclcpp sensor_msgs)
target_link_libraries(mpu6050_node mpu6050_node_lib)

add_executable(mpu6050_calibration_node src/main_calibration.cpp src/mpu6050_calibration_node.cpp)
ament_target_dependencies(mpu6050_calibration_node rclcpp sensor_msgs)
target_link_libraries(mpu6050_calibration_node mpu6050_node_lib)

ament_export_targets(export_${PROJECT_NAME})
# Install targets
install(
  TARGETS mpu6050_node mpu6050_calibration_node mpu6050_node_lib
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Install targets
# install(TARGETS
#   mpu6050_node
#   mpu6050_calibration_node
#   mpu6050_node_lib
#   DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

# Uncomment if you have package specific launch files, config files or other files to install
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(mpu6050_node_lib)
ament_export_dependencies(rclcpp sensor_msgs Eigen3)

ament_package()
