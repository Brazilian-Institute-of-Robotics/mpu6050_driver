#include <rclcpp/rclcpp.hpp>
#include "mpu6050_driver_ros2/mpu6050_node_ros2.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto mpu_node = std::make_shared<mpu6050_driver_ros2::MPU6050Node>();

  mpu_node->init();

  std::cout << "Node Initiated" << std::endl;

  rclcpp::spin(mpu_node);

  std::cout << "Finished" << std::endl;

  rclcpp::shutdown();

  return 0;
}