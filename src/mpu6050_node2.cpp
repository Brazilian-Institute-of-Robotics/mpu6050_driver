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
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Dense>
#include "mpu6050_driver/mpu6050.hpp"

namespace mpu6050_driver {

static const float gravity_value = 9.81;
static const float deg_to_rad_factor = M_PI / 180.0;

class MPU6050CorrectedNode {
public:
    MPU6050CorrectedNode() : nh_("~"), mpu6050_(), A_(Eigen::Matrix3d::Identity()), B_(Eigen::Vector3d::Zero()), gyro_offsets_(Eigen::Vector3d::Zero()) {
        loadParameters();
        if (!computeInverseCalibrationMatrix()) {
            ROS_ERROR("Calibration matrix is not invertible. Please recalibrate the sensor.");
            ros::shutdown();
        }
        mpu6050_.setAddress(static_cast<uint8_t>(mpu6050_addr_));
        mpu6050_.initialize(i2c_bus_uri_);
        mpu6050_.setDLPFMode(static_cast<uint8_t>(3));
        mpu6050_.setIntDataReadyEnabled(true);
    }

    void init() {
        mpu_data_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu/data_raw", 1);
        ROS_INFO("MPU6050 Corrected Node has started");
    }

    void run() {
        ros::Rate loop_rate(pub_rate_);

        while (ros::ok()) {
            ros::spinOnce();
            if (mpu6050_.getIntDataReadyStatus()) this->publishCorrectedMPUData();
            loop_rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    std::string i2c_bus_uri_;
    int mpu6050_addr_;
    float pub_rate_;
    std::string imu_frame_id_;
    mpu6050_driver::MPU6050 mpu6050_;
    ros::Publisher mpu_data_pub_;

    Eigen::Matrix3d A_;  // Calibration matrix
    Eigen::Vector3d B_;  // Offset vector
    Eigen::Vector3d gyro_offsets_;  // Gyroscope offsets

    void loadParameters() {
        nh_.param<std::string>("bus_uri", i2c_bus_uri_, "/dev/i2c-1");
        nh_.param<int>("mpu_address", mpu6050_addr_, 0x68);
        nh_.param<float>("pub_rate", pub_rate_, 30);
        nh_.param<std::string>("frame_id", imu_frame_id_, "imu");

        nh_.param("Xgain", A_(0, 0), 1.0);
        nh_.param("YtoX", A_(0, 1), 0.0);
        nh_.param("ZtoX", A_(0, 2), 0.0);
        nh_.param("XtoY", A_(1, 0), 0.0);
        nh_.param("Ygain", A_(1, 1), 1.0);
        nh_.param("ZtoY", A_(1, 2), 0.0);
        nh_.param("XtoZ", A_(2, 0), 0.0);
        nh_.param("YtoZ", A_(2, 1), 0.0);
        nh_.param("Zgain", A_(2, 2), 1.0);

        nh_.param("Xofs", B_(0), 0.0);
        nh_.param("Yofs", B_(1), 0.0);
        nh_.param("Zofs", B_(2), 0.0);

        nh_.param("gyro_x_offset", gyro_offsets_(0), 0.0);
        nh_.param("gyro_y_offset", gyro_offsets_(1), 0.0);
        nh_.param("gyro_z_offset", gyro_offsets_(2), 0.0);
    }

    bool computeInverseCalibrationMatrix() {
        try {
            A_ = A_.inverse();
            return true;
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to invert calibration matrix: %s", e.what());
            return false;
        }
    }

    void publishCorrectedMPUData() {
        sensor_msgs::Imu imu_data;
        IMUData<float> mpu_data;

        mpu_data = mpu6050_.getMotion6();

        Eigen::Vector3d raw_acc(mpu_data.accel.x, mpu_data.accel.y, mpu_data.accel.z);
        Eigen::Vector3d corrected_acc = A_ * (raw_acc - B_);

        imu_data.linear_acceleration.x = corrected_acc.x() * gravity_value;
        imu_data.linear_acceleration.y = corrected_acc.y() * gravity_value;
        imu_data.linear_acceleration.z = corrected_acc.z() * gravity_value;

        Eigen::Vector3d raw_gyro(mpu_data.gyro.x, mpu_data.gyro.y, mpu_data.gyro.z);
        Eigen::Vector3d corrected_gyro = raw_gyro - gyro_offsets_;

        imu_data.angular_velocity.x = corrected_gyro.x() * deg_to_rad_factor;
        imu_data.angular_velocity.y = corrected_gyro.y() * deg_to_rad_factor;
        imu_data.angular_velocity.z = corrected_gyro.z() * deg_to_rad_factor;

        imu_data.header.frame_id = imu_frame_id_;
        imu_data.header.stamp = ros::Time::now();

        mpu_data_pub_.publish(imu_data);
    }
};

}  // namespace mpu6050_driver

int main(int argc, char** argv) {
    ros::init(argc, argv, "mpu6050_node");
    mpu6050_driver::MPU6050CorrectedNode node;
    node.init();
    node.run();
    return 0;
}
