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

#include <cmath>
#include "mpu6050_driver/mpu6050_node.hpp"
#include "sensor_msgs/Imu.h"

namespace mpu6050_driver {

static const float gravity_value = 9.81;
static const float deg_to_rad_factor = M_PI / 180.0;

MPU6050Node::MPU6050Node()
  : axes_offsets_(6) {}

void MPU6050Node::init() {
  mpu_data_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data_raw", 1);

  this->loadParameters();

  mpu6050_.setAddress(static_cast<uint8_t>(mpu6050_addr_));
  mpu6050_.initialize(i2c_bus_uri_);
  mpu6050_.setDLPFMode(static_cast<uint8_t>(4));
  mpu6050_.setIntDataReadyEnabled(true);
  this->setMPUOffsets();

  ROS_INFO("MPU6050 Node has started");
}

void MPU6050Node::run() {
  ros::Rate loop_rate(pub_rate_);

  while (ros::ok()) {
    ros::spinOnce();
    if (mpu6050_.getIntDataReadyStatus()) this->publishMPUData();
    loop_rate.sleep();
  }
}

void MPU6050Node::publishMPUData() {
  sensor_msgs::Imu imu_data;
  IMUData<float> mpu_data;

  mpu_data = mpu6050_.getMotion6();

  imu_data.linear_acceleration.x = mpu_data.accel.x * gravity_value;
  imu_data.linear_acceleration.y = mpu_data.accel.y * gravity_value;
  imu_data.linear_acceleration.z = mpu_data.accel.z * gravity_value;

  imu_data.angular_velocity.x =  mpu_data.gyro.x * deg_to_rad_factor;
  imu_data.angular_velocity.y =  mpu_data.gyro.y * deg_to_rad_factor;
  imu_data.angular_velocity.z =  mpu_data.gyro.z * deg_to_rad_factor;

  imu_data.header.frame_id = imu_frame_id_;
  imu_data.header.stamp = ros::Time::now();

  mpu_data_pub_.publish(imu_data);
}

void MPU6050Node::loadParameters() {
  ros::NodeHandle ph("~");
  getParameterHelper<std::string>(ph, "bus_uri", &i2c_bus_uri_, "/dev/i2c-1");
  getParameterHelper<int>(ph, "mpu_address", &mpu6050_addr_, 0x68);
  getParameterHelper<float>(ph, "pub_rate", &pub_rate_, 30);
  getParameterHelper<std::string>(ph, "frame_id", &imu_frame_id_, "imu");
  ph.param<std::vector<int>>("axes_offsets", axes_offsets_, std::vector<int>(6, 0));
}

void MPU6050Node::setMPUOffsets() {
  mpu6050_.setXAccelOffset(axes_offsets_[0]);
  mpu6050_.setYAccelOffset(axes_offsets_[1]);
  mpu6050_.setZAccelOffset(axes_offsets_[2]);
  mpu6050_.setXGyroOffset(axes_offsets_[3]);
  mpu6050_.setYGyroOffset(axes_offsets_[4]);
  mpu6050_.setZGyroOffset(axes_offsets_[5]);
}

}  // namespace mpu6050_driver
