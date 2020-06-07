// Copyright (c) 2020 Mateus Menezes

#include <ros/ros.h>
#include <exception>
#include "mpu6050_driver/mpu6050_node.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "mpu6050_node");
  ros::NodeHandle nh;

  mpu6050_driver::MPU6050Node mpu_node;

  try {
    mpu_node.init();
    mpu_node.run();
  } catch (std::runtime_error error) {
    ROS_FATAL("%s", error.what());
    ros::shutdown();
  }

  return 0;
}
