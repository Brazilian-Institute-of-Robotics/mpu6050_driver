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
#include "mpu6050_driver/mpu6050_node.hpp"

namespace mpu6050_driver {

static const float gravity_value = 9.81f;
static const float deg_to_rad_factor = M_PI / 180.0f;

MPU6050Node::MPU6050Node()
: Node("mpu6050_node") {
    init();
}

void MPU6050Node::init() {
    loadParameters();

    mpu6050_.setAddress(static_cast<uint8_t>(mpu6050_addr_));
    mpu6050_.initialize(i2c_bus_uri_);
    mpu6050_.setDLPFMode(static_cast<uint8_t>(4));
    mpu6050_.setIntDataReadyEnabled(true);
    setMPUOffsets();

    mpu_data_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);

    auto timer_callback = [this]() -> void {
        this->publishMPUData();
    };

    data_publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000 / pub_rate_)), timer_callback);

    RCLCPP_INFO(this->get_logger(), "MPU6050 Node has started.");
}

void MPU6050Node::loadParameters() {
    this->declare_parameter("bus_uri", "/dev/i2c-1");
    this->declare_parameter("mpu_address", 0x68);
    this->declare_parameter("pub_rate", 30.0f);
    this->declare_parameter("frame_id", "imu_link");
    this->declare_parameter("axes_offsets", std::vector<int>{0, 0, 0, 0, 0, 0});

    this->get_parameter("bus_uri", i2c_bus_uri_);
    this->get_parameter("mpu_address", mpu6050_addr_);
    this->get_parameter("pub_rate", pub_rate_);
    this->get_parameter("frame_id", imu_frame_id_);
    this->get_parameter("axes_offsets", axes_offsets_);
}

void MPU6050Node::setMPUOffsets() {
    mpu6050_.setXAccelOffset(axes_offsets_[0]);
    mpu6050_.setYAccelOffset(axes_offsets_[1]);
    mpu6050_.setZAccelOffset(axes_offsets_[2]);
    mpu6050_.setXGyroOffset(axes_offsets_[3]);
    mpu6050_.setYGyroOffset(axes_offsets_[4]);
    mpu6050_.setZGyroOffset(axes_offsets_[5]);
}

void MPU6050Node::publishMPUData() {
    if (!mpu6050_.getIntDataReadyStatus()) {
        return; // Skip this cycle if data is not ready
    }

    auto imu_data = sensor_msgs::msg::Imu();
    auto mpu_data = mpu6050_.getMotion6();

    imu_data.linear_acceleration.x = mpu_data.accel.x * gravity_value;
    imu_data.linear_acceleration.y = mpu_data.accel.y * gravity_value;
    imu_data.linear_acceleration.z = mpu_data.accel.z * gravity_value - gravity_value; // Assuming Z axis is aligned with gravity
    imu_data.angular_velocity.x = mpu_data.gyro.x * deg_to_rad_factor;
    imu_data.angular_velocity.y = mpu_data.gyro.y * deg_to_rad_factor;
    imu_data.angular_velocity.z = mpu_data.gyro.z * deg_to_rad_factor;

    imu_data.header.frame_id = imu_frame_id_;
    imu_data.header.stamp = this->get_clock()->now();

    mpu_data_pub_->publish(imu_data);
}

} // namespace mpu6050_driver
