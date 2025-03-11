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

#include <chrono>
#include <cmath>
//#include <rclcpp/rclcpp.hpp>
#include "mpu6050_driver_ros2/mpu6050_node_ros2.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace mpu6050_driver_ros2 {

static const float gravity_value = 9.81;
static const float deg_to_rad_factor = M_PI / 180.0;

MPU6050Node::MPU6050Node()
    : Node("mpu_driver_node"), axes_offsets_(6) {}

void MPU6050Node::init() {
    mpu_data_pub_ =
        this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 1);
    timer_ = this->create_wall_timer(std::chrono::nanoseconds(pub_dt_ns_),
                                     std::bind(&MPU6050Node::run, this));

    this->loadParameters();

    mpu6050_.setAddress(static_cast<uint8_t>(mpu6050_addr_));
    mpu6050_.initialize(i2c_bus_uri_);
    mpu6050_.setDLPFMode(static_cast<uint8_t>(4));
    mpu6050_.setIntDataReadyEnabled(true);
    this->setMPUOffsets();

    RCLCPP_INFO(get_logger(), "MPU6050 Node has started");
}

void MPU6050Node::run() {
    if (mpu6050_.getIntDataReadyStatus()) this->publishMPUData();
}

void MPU6050Node::publishMPUData() {
    sensor_msgs::msg::Imu imu_data;
    IMUData<float> mpu_data;

    mpu_data = mpu6050_.getMotion6();

    imu_data.linear_acceleration.x = mpu_data.accel.x * gravity_value;
    imu_data.linear_acceleration.y = mpu_data.accel.y * gravity_value;
    imu_data.linear_acceleration.z = mpu_data.accel.z * gravity_value;

    imu_data.angular_velocity.x = mpu_data.gyro.x * deg_to_rad_factor;
    imu_data.angular_velocity.y = mpu_data.gyro.y * deg_to_rad_factor;
    imu_data.angular_velocity.z = mpu_data.gyro.z * deg_to_rad_factor;

    imu_data.header.frame_id = imu_frame_id_;
    imu_data.header.stamp = rclcpp::Clock().now();

    mpu_data_pub_->publish(imu_data);
}

void MPU6050Node::loadParameters() {
    declare_parameter("bus_uri", "/dev/i2c-1");
    declare_parameter("mpu_address", 0x68);
    declare_parameter("pub_dt_ns", 10000000);
    declare_parameter("frame_id", "imu");

    const std::vector<int64_t> axes_offsets = {0, 0, 0, 0, 0, 0};
    declare_parameter("axes_offsets", axes_offsets);

    get_parameter("bus_uri", i2c_bus_uri_);
    get_parameter("mpu_address", mpu6050_addr_);
    get_parameter("pub_dt_ns", pub_dt_ns_);
    get_parameter("frame_id", imu_frame_id_);
    axes_offsets_ = get_parameter("axes_offsets").as_integer_array();
}

void MPU6050Node::setMPUOffsets() {
    mpu6050_.setXAccelOffset(axes_offsets_[0]);
    mpu6050_.setYAccelOffset(axes_offsets_[1]);
    mpu6050_.setZAccelOffset(axes_offsets_[2]);
    mpu6050_.setXGyroOffset(axes_offsets_[3]);
    mpu6050_.setYGyroOffset(axes_offsets_[4]);
    mpu6050_.setZGyroOffset(axes_offsets_[5]);
}

}  // namespace mpu6050_driver_ros2
