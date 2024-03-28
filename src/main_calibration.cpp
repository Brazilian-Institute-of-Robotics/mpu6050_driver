/* ============================================
MIT License

//  Copyright (c) 2020 Mateus Meneses
//  Copyright (c) 2024 Mohamed Abdelkader

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
===============================================
*/

#include <rclcpp/rclcpp.hpp>
#include <exception>
#include "mpu6050_driver/mpu6050_calibration_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto mpu_calibration_node = std::make_shared<mpu6050_driver::MPU6050CalibrationNode>();

  try {
    mpu_calibration_node->init();
    rclcpp::spin(mpu_calibration_node);
  } catch (const std::runtime_error& error) {
    RCLCPP_FATAL(mpu_calibration_node->get_logger(), "%s", error.what());
    rclcpp::shutdown();
  }

  rclcpp::shutdown();
  return 0;
}

