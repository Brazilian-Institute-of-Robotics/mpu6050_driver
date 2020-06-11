/* ============================================
MIT License

//  Copyright (c) 2020 Mateus Meneses

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

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/Imu.h"

#include "mpu6050_driver/mpu6050_calibration_node.hpp"

namespace mpu6050_driver {

MPU6050CalibrationNode::MPU6050CalibrationNode()
  : MPU6050Node()
  , i_term_matrix_(3, 2)
  , p_term_matrix_(3, 2)
  , offset_matrix_(3, 2)
  , error_matrix_(3, 2) {}

void MPU6050CalibrationNode::loadParameters() {
  ros::NodeHandle ph("~");
  this->getParameterHelper<float>(ph, "kp", &kp_, 0.1);
  this->getParameterHelper<float>(ph, "ki", &ki_, 0.1);
  this->getParameterHelper<float>(ph, "delta", &delta_, 0.5);
}

void MPU6050CalibrationNode::init() {
  MPU6050Node::init();
  this->loadParameters();

  i_term_matrix_ = Eigen::Matrix<float, 3, 2>::Zero();

  imu_offsets_pub_ = nh_.advertise<sensor_msgs::Imu>("imu_offsets", 1);

  ROS_INFO("MPU6050 Calibration Node has started");
}

void MPU6050CalibrationNode::computeOffsets() {
  float dt = 1.0 / pub_rate_;  // How it isn't a dynamic system, sample time doesn't must exactly computed

  IMUData<int16_t> imu_raw_data = mpu6050_.getRawMotion6();
  imu_raw_data.accel.z -= 16384;  // Remove gravity contribution

  /* The divisions here is beacause the offsets need to be set when the MPU is 
  in the less sensitive mode (accel in 16g mode and gyro in 2000 degrees/sec mode).
  For more details, see https://forum.arduino.cc/index.php?topic=535717.0
  Another thing, the minus sign is because the error is calculated as
  setpoint - plant_value, though all set point is always 0, then error = -plant_value */
  error_matrix_ << -(imu_raw_data.accel.x / 8), -(imu_raw_data.gyro.x / 4),
                   -(imu_raw_data.accel.y / 8), -(imu_raw_data.gyro.y / 4),
                   -(imu_raw_data.accel.z / 8), -(imu_raw_data.gyro.z / 4);

  p_term_matrix_ = kp_ * error_matrix_;
  i_term_matrix_ += ki_ * error_matrix_ * dt;

  offset_matrix_ = p_term_matrix_ + i_term_matrix_;
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
  sensor_msgs::Imu imu_offsets_msg;

  imu_offsets_msg.linear_acceleration.x = offset_matrix_(0, 0);
  imu_offsets_msg.linear_acceleration.y = offset_matrix_(1, 0);
  imu_offsets_msg.linear_acceleration.z = offset_matrix_(2, 0);

  imu_offsets_msg.angular_velocity.x = offset_matrix_(0, 1);
  imu_offsets_msg.angular_velocity.y = offset_matrix_(1, 1);
  imu_offsets_msg.angular_velocity.z = offset_matrix_(2, 1);

  imu_offsets_msg.header.frame_id = imu_frame_id_;
  imu_offsets_msg.header.stamp = ros::Time::now();

  imu_offsets_pub_.publish(imu_offsets_msg);
}

bool MPU6050CalibrationNode::isCalibrationFinished() {
  return error_matrix_.isApprox(Eigen::Matrix<float, 3, 2>::Zero(), delta_) ? true : false;
}

void MPU6050CalibrationNode::printOffsets() {
  ROS_INFO("Final offset of Accel X axis = %d", static_cast<int16_t>(offset_matrix_(0, 0)));
  ROS_INFO("Final offset of Accel Y axis = %d", static_cast<int16_t>(offset_matrix_(1, 0)));
  ROS_INFO("Final offset of Accel Z axis = %d", static_cast<int16_t>(offset_matrix_(2, 0)));
  ROS_INFO("Final offset of Gyro  X axis = %d", static_cast<int16_t>(offset_matrix_(0, 1)));
  ROS_INFO("Final offset of Gyro  Y axis = %d", static_cast<int16_t>(offset_matrix_(1, 1)));
  ROS_INFO("Final offset of Gyro  Z axis = %d", static_cast<int16_t>(offset_matrix_(2, 1)));
  ROS_INFO("Insert these value above in the config file");
}

void MPU6050CalibrationNode::run() {
  ros::Rate loop_rate(pub_rate_);

  while (ros::ok()) {
    this->computeOffsets();
    this->adjustOffsets();
    this->publishMPUData();
    this->publishOffsets();

    if (this->isCalibrationFinished()) {
      this->printOffsets();
      ros::shutdown();
    }

    loop_rate.sleep();
  }
}

}  // namespace mpu6050_driver
