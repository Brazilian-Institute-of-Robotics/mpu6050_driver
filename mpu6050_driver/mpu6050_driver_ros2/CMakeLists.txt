cmake_minimum_required(VERSION 3.5)
project(mpu6050_driver_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(dependencies
  ament_cmake
  rclcpp
  sensor_msgs
  i2c_device_ros2
  Eigen3
  imu_filter_madgwick
)

# find dependencies
foreach(dependency ${dependencies})
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
)

ament_export_include_directories(
  include
)

set(MPU6050_NODE_LIB mpu6050_node_lib)
add_library(${MPU6050_NODE_LIB} SHARED
  src/mpu6050_node_ros2.cpp
  src/mpu6050.cpp
)
ament_target_dependencies(${MPU6050_NODE_LIB} ${dependencies})

install(TARGETS ${MPU6050_NODE_LIB}
  #EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  #INCLUDES DESTINATION include
)

add_executable(mpu6050_driver_ros2_node src/main_node.cpp)
ament_target_dependencies(mpu6050_driver_ros2_node ${dependencies})
target_link_libraries(mpu6050_driver_ros2_node  ${MPU6050_NODE_LIB})

add_executable(mpu6050_driver_calibration_ros2_node src/mpu6050_calibration_node_ros2.cpp)
ament_target_dependencies(mpu6050_driver_calibration_ros2_node ${dependencies})
target_link_libraries(mpu6050_driver_calibration_ros2_node  ${MPU6050_NODE_LIB})

# Install sample action server
install(TARGETS
  mpu6050_driver_ros2_node
  mpu6050_driver_calibration_ros2_node
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(${dependencies})

# Install subdirectories
install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
