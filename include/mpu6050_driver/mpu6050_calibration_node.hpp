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

#ifndef MPU6050_DRIVER_MPU6050_CALIBRATION_NODE_HPP_
#define MPU6050_DRIVER_MPU6050_CALIBRATION_NODE_HPP_

#include <eigen3/Eigen/Dense>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "mpu6050_driver/mpu6050.hpp"
#include "mpu6050_driver/mpu6050_node.hpp" // Make sure this is adapted for ROS2
#include "sensor_msgs/msg/imu.hpp"

namespace mpu6050_driver {

/**
 * @brief A ROS2 node for calibrating the MPU6050 sensor.
 *
 * This class extends the MPU6050Node to implement a calibration process for the MPU6050 sensor,
 * using a PID algorithm to compute offsets for all axes. Calibration steps are executed periodically
 * based on a timer.
 */
class MPU6050CalibrationNode : public MPU6050Node {
public:
  /**
   * @brief Construct a new MPU6050CalibrationNode object
   */
  MPU6050CalibrationNode();

  /**
   * @brief Initialize calibration node, setting up parameters, publisher, and timer.
   */
  void init();

private:
  /**
   * @brief Declare and load parameters needed for calibration.
   */
  void declare_parameters();
  void loadParameters();

  /**
   * @brief Compute offsets using a PID control algorithm.
   *
   * This method computes the offsets needed to calibrate the MPU6050 sensor.
   * The offsets are determined by comparing actual sensor readings against expected
   * values (e.g., zero for acceleration and gyroscope when the sensor is stationary,
   * except for the Z acceleration which should be equal to gravitational acceleration).
   */
  void computeOffsets();

  /**
   * @brief Apply computed offsets to the MPU6050 sensor.
   */
  void adjustOffsets();

  /**
   * @brief Publish the computed offsets.
   *
   * This method publishes the current offsets to a ROS2 topic.
   */
  void publishOffsets();

  /**
   * @brief Check if the calibration process is complete.
   *
   * @return true if the calibration errors are within the acceptable tolerance, false otherwise.
   */
  bool isCalibrationFinished();

  /**
   * @brief Perform a single step of the calibration process.
   *
   * This method is called periodically by a timer. It computes offsets, applies them,
   * publishes the offsets, and checks if the calibration process is complete.
   */
  void performCalibrationStep();

  /**
   * @brief Print the final offsets to the console.
   *
   * This method is called once the calibration process is deemed complete, logging
   * the final offsets for manual inspection or configuration.
   */
  void printOffsets();

  // ROS2 Publisher for publishing the calibration offsets
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_offsets_pub_;
  // ROS2 Timer for triggering periodic calibration steps
  rclcpp::TimerBase::SharedPtr calibration_timer_;

  // PID control parameters
  float kp_, ki_, delta_;

  // Matrices for PID calculations
  Eigen::Matrix<float, 3, 2> p_term_matrix_, i_term_matrix_, error_matrix_, offset_matrix_;
};

}  // namespace mpu6050_driver

#endif  // MPU6050_DRIVER_MPU6050_CALIBRATION_NODE_HPP_
