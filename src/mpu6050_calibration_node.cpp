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

#include "mpu6050_driver/mpu6050_calibration_node.hpp"
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace mpu6050_driver {

MPU6050CalibrationNode::MPU6050CalibrationNode()
: MPU6050Node(), // Ensure base class is adapted for ROS2
  i_term_matrix_(Eigen::Matrix<float, 3, 2>::Zero()),
  p_term_matrix_(Eigen::Matrix<float, 3, 2>::Zero()),
  offset_matrix_(Eigen::Matrix<float, 3, 2>::Zero()),
  error_matrix_(Eigen::Matrix<float, 3, 2>::Zero()) {
    declare_parameters();
}

void MPU6050CalibrationNode::declare_parameters() {
    this->declare_parameter<float>("kp", 0.1f);
    this->declare_parameter<float>("ki", 0.1f);
    this->declare_parameter<float>("delta", 0.5f);
    loadParameters();
}

void MPU6050CalibrationNode::loadParameters() {
    kp_ = this->get_parameter("kp").as_double();
    ki_ = this->get_parameter("ki").as_double();
    delta_ = this->get_parameter("delta").as_double();
}

void MPU6050CalibrationNode::init() {
    //MPU6050Node::init(); // Assuming MPU6050Node::init() is correctly adapted for ROS2
    imu_offsets_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_offsets", 10);

    auto timer_callback = [this]() -> void {
        this->performCalibrationStep();
    };

    auto timer_period = std::chrono::milliseconds(static_cast<int>(1000 / pub_rate_));
    calibration_timer_ = this->create_wall_timer(timer_period, timer_callback);

    RCLCPP_INFO(this->get_logger(), "MPU6050 Calibration Node has started.");
}

void MPU6050CalibrationNode::performCalibrationStep() {
    computeOffsets();
    adjustOffsets();
    publishOffsets();

    if (isCalibrationFinished()) {
        printOffsets();
        calibration_timer_->cancel(); // Optionally stop the timer
    }
}

void MPU6050CalibrationNode::computeOffsets() {
    float dt = 1.0 / pub_rate_; // Time delta based on publication rate, for integral calculation

    auto imu_raw_data = mpu6050_.getRawMotion6(); // Fetch raw sensor data
    // Adjust the Z-axis acceleration to account for gravity, assuming the sensor's output needs to be scaled
    imu_raw_data.accel.z -= 16384; // Assuming raw value representing 1g, adjust based on your sensor's datasheet

    // Calculate the error as the difference between expected (setpoint) and actual values
    // Assuming the setpoints are 0 for all axes except Z acceleration due to gravity
    error_matrix_ << -(imu_raw_data.accel.x / 8), -(imu_raw_data.gyro.x / 4),
                     -(imu_raw_data.accel.y / 8), -(imu_raw_data.gyro.y / 4),
                     -(imu_raw_data.accel.z / 8), -(imu_raw_data.gyro.z / 4);

    // Update PID terms: Proportional and Integral. Derivative term can be added for improved control
    p_term_matrix_ = kp_ * error_matrix_; // Proportional term
    i_term_matrix_ += ki_ * error_matrix_ * dt; // Integral term accumulates over time

    // Calculate offsets by combining P and I terms. D term can be included for a full PID controller
    offset_matrix_ = p_term_matrix_ + i_term_matrix_;

    // Optional: Implement rate limiting or clamping on offset_matrix_ to ensure stability and prevent overshooting
}


void MPU6050CalibrationNode::adjustOffsets() {
  mpu6050_.setXAccelOffset(static_cast<int16_t>(offset_matrix_(0, 0)));
  mpu6050_.setYAccelOffset(static_cast<int16_t>(offset_matrix_(1, 0)));
  mpu6050_.setZAccelOffset(static_cast<int16_t>(offset_matrix_(2, 0)));
  mpu6050_.setXGyroOffset(static_cast<int16_t>(offset_matrix_(0, 1)));
  mpu6050_.setYGyroOffset(static_cast<int16_t>(offset_matrix_(1, 1)));
  mpu6050_.setZGyroOffset(static_cast<int16_t>(offset_matrix_(2, 1)));
}

void MPU6050CalibrationNode::publishOffsets() {
    sensor_msgs::msg::Imu imu_offsets_msg;

    // Assuming offset_matrix_ stores the calculated offsets in a manner such that:
    // - The first column represents acceleration offsets for x, y, z axes respectively.
    // - The second column represents angular velocity (gyro) offsets for x, y, z axes respectively.
    imu_offsets_msg.linear_acceleration.x = offset_matrix_(0, 0);
    imu_offsets_msg.linear_acceleration.y = offset_matrix_(1, 0);
    imu_offsets_msg.linear_acceleration.z = offset_matrix_(2, 0);

    imu_offsets_msg.angular_velocity.x = offset_matrix_(0, 1);
    imu_offsets_msg.angular_velocity.y = offset_matrix_(1, 1);
    imu_offsets_msg.angular_velocity.z = offset_matrix_(2, 1);

    // Set the frame ID and timestamp for the message
    imu_offsets_msg.header.frame_id = imu_frame_id_; // Ensure imu_frame_id_ is correctly initialized
    imu_offsets_msg.header.stamp = this->get_clock()->now();

    // Publish the offsets
    imu_offsets_pub_->publish(imu_offsets_msg);
}

bool MPU6050CalibrationNode::isCalibrationFinished() {
    // Determine if calibration is complete based on error_matrix_ and delta_
    return error_matrix_.isApprox(Eigen::Matrix<float, 3, 2>::Zero(), delta_);
}

void MPU6050CalibrationNode::printOffsets() {
    RCLCPP_INFO(this->get_logger(), "Final offset of Accel X axis = %d", static_cast<int16_t>(offset_matrix_(0, 0)));
    RCLCPP_INFO(this->get_logger(), "Final offset of Accel Y axis = %d", static_cast<int16_t>(offset_matrix_(1, 0)));
    RCLCPP_INFO(this->get_logger(), "Final offset of Accel Z axis = %d", static_cast<int16_t>(offset_matrix_(2, 0)));
    RCLCPP_INFO(this->get_logger(), "Final offset of Gyro  X axis = %d", static_cast<int16_t>(offset_matrix_(0, 1)));
    RCLCPP_INFO(this->get_logger(), "Final offset of Gyro  Y axis = %d", static_cast<int16_t>(offset_matrix_(1, 1)));
    RCLCPP_INFO(this->get_logger(), "Final offset of Gyro  Z axis = %d", static_cast<int16_t>(offset_matrix_(2, 1)));
    RCLCPP_INFO(this->get_logger(), "Insert these values above in the config file");
}


}  // namespace mpu6050_driver
