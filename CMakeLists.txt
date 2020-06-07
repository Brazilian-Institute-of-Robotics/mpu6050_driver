cmake_minimum_required(VERSION 2.8.3)
project(mpu6050_driver)

add_compile_options(-std=c++11)
# add_definitions(-Wall -Werror)

set(CATKIN_DEPS 
  # diagnostic_msgs
  roscpp
  sensor_msgs
  i2c_device_ros
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPS})
find_package(Eigen3 REQUIRED)
find_package(cmake_modules REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS ${CATKIN_DEPS}
 DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

set(MPU6050_NODE_LIB mpu6050_node_lib)
add_library(${MPU6050_NODE_LIB}
  src/mpu6050_node.cpp
  src/mpu6050.cpp
)

add_executable(mpu6050_node 
  src/main.cpp
)

target_link_libraries(mpu6050_node
  ${catkin_LIBRARIES}
  ${MPU6050_NODE_LIB}
)

add_executable(mpu6050_calibration_node 
  src/main_calibration.cpp
  src/mpu6050_calibration_node.cpp
)

target_link_libraries(mpu6050_calibration_node
  ${catkin_LIBRARIES}
  ${MPU6050_NODE_LIB}
)

# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
