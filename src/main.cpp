// Copyright (c) 2020 Mateus Menezes

#include <rclcpp/rclcpp.hpp>
#include <exception>
#include "mpu6050_driver/mpu6050_node.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<mpu6050_driver::MPU6050Node>();

    try {
        node->init();
        rclcpp::spin(node);
    } catch (const std::runtime_error& error) {
        RCLCPP_FATAL(node->get_logger(), "%s", error.what());
        rclcpp::shutdown();
    }

    rclcpp::shutdown();
    return 0;
}
