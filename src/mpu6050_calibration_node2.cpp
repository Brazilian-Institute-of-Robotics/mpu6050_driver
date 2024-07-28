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
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include "mpu6050_driver/mpu6050.hpp"

class SensorCalibration {
public:
    SensorCalibration() {}

    void addMeasurement(const Eigen::Vector3d& measured_acc, const Eigen::Vector3d& true_acc) {
        measured_accs_.push_back(measured_acc);
        true_accs_.push_back(true_acc);
    }

    void addGyroMeasurement(const Eigen::Vector3d& measured_gyro) {
        measured_gyros_.push_back(measured_gyro);
    }

    void calibrate() {
        int N = measured_accs_.size();
        Eigen::MatrixXd Acc(3, N);
        Eigen::MatrixXd TrueAcc(4, N);

        for (int i = 0; i < N; ++i) {
            Acc.col(i) = measured_accs_[i];
            TrueAcc.col(i) << true_accs_[i], 1.0;
        }

        Eigen::MatrixXd TrueAccT = TrueAcc.transpose();
        Eigen::MatrixXd product = Acc * TrueAccT;
        Eigen::MatrixXd inv_product = (TrueAcc * TrueAccT).inverse();

        Eigen::MatrixXd Unknowns = product * inv_product;

        // Extract the offsets, gains, and cross-gains
        Xgain_ = Unknowns(0, 0);
        YtoX_ = Unknowns(0, 1);
        ZtoX_ = Unknowns(0, 2);
        Xofs_ = Unknowns(0, 3);

        XtoY_ = Unknowns(1, 0);
        Ygain_ = Unknowns(1, 1);
        ZtoY_ = Unknowns(1, 2);
        Yofs_ = Unknowns(1, 3);

        XtoZ_ = Unknowns(2, 0);
        YtoZ_ = Unknowns(2, 1);
        Zgain_ = Unknowns(2, 2);
        Zofs_ = Unknowns(2, 3);

        A_ << Xgain_, YtoX_, ZtoX_,
              XtoY_, Ygain_, ZtoY_,
              XtoZ_, YtoZ_, Zgain_;

        B_ << Xofs_, Yofs_, Zofs_;

        // Compute gyro offsets
        gyro_offsets_ = Eigen::Vector3d::Zero();
        for (const auto& gyro : measured_gyros_) {
            gyro_offsets_ += gyro;
        }
        gyro_offsets_ /= measured_gyros_.size();
    }

    void writeCalibrationToYAML(const std::string& filename) const {
        YAML::Emitter out;
        out << YAML::BeginMap;
        out << YAML::Key << "bus_uri" << YAML::Value << "/dev/i2c-1";
        out << YAML::Key << "mpu_address" << YAML::Value << "0x68";
        out << YAML::Key << "pub_rate" << YAML::Value << 200;
        out << YAML::Key << "frame_id" << YAML::Value << "imu";
        out << YAML::Key << "axes_offsets" << YAML::Value << YAML::Flow << std::vector<int>{0, 0, 0, 0, 0, 0};
        out << YAML::Key << "ki" << YAML::Value << 0.2;
        out << YAML::Key << "kp" << YAML::Value << 0.1;
        out << YAML::Key << "delta" << YAML::Value << 0.2;

        out << YAML::Key << "Xgain" << YAML::Value << Xgain_;
        out << YAML::Key << "YtoX" << YAML::Value << YtoX_;
        out << YAML::Key << "ZtoX" << YAML::Value << ZtoX_;
        out << YAML::Key << "Xofs" << YAML::Value << Xofs_;
        out << YAML::Key << "XtoY" << YAML::Value << XtoY_;
        out << YAML::Key << "Ygain" << YAML::Value << Ygain_;
        out << YAML::Key << "ZtoY" << YAML::Value << ZtoY_;
        out << YAML::Key << "Yofs" << YAML::Value << Yofs_;
        out << YAML::Key << "XtoZ" << YAML::Value << XtoZ_;
        out << YAML::Key << "YtoZ" << YAML::Value << YtoZ_;
        out << YAML::Key << "Zgain" << YAML::Value << Zgain_;
        out << YAML::Key << "Zofs" << YAML::Value << Zofs_;

        out << YAML::Key << "gyro_x_offset" << YAML::Value << gyro_offsets_.x();
        out << YAML::Key << "gyro_y_offset" << YAML::Value << gyro_offsets_.y();
        out << YAML::Key << "gyro_z_offset" << YAML::Value << gyro_offsets_.z();
        out << YAML::EndMap;

        std::ofstream fout(filename);
        fout << out.c_str();
    }

    void printCalibration() {
        std::cout << "Xgain: " << Xgain_ << ", YtoX: " << YtoX_ << ", ZtoX: " << ZtoX_ << ", Xofs: " << Xofs_ << std::endl;
        std::cout << "XtoY: " << XtoY_ << ", Ygain: " << Ygain_ << ", ZtoY: " << ZtoY_ << ", Yofs: " << Yofs_ << std::endl;
        std::cout << "XtoZ: " << XtoZ_ << ", YtoZ: " << YtoZ_ << ", Zgain: " << Zgain_ << ", Zofs: " << Zofs_ << std::endl;
        std::cout << "Gyro offsets: " << gyro_offsets_.transpose() << std::endl;
    }

private:
    std::vector<Eigen::Vector3d> measured_accs_;
    std::vector<Eigen::Vector3d> true_accs_;
    std::vector<Eigen::Vector3d> measured_gyros_;

    double Xgain_, Ygain_, Zgain_;
    double XtoY_, XtoZ_, YtoX_, YtoZ_, ZtoX_, ZtoY_;
    double Xofs_, Yofs_, Zofs_;

    Eigen::Vector3d gyro_offsets_;
    Eigen::Matrix3d A_;  // Matrix of gains and cross-axis gains
    Eigen::Vector3d B_;  // Vector of offsets
};

class MPU6050CalibrationNode {
public:
    MPU6050CalibrationNode() : nh_("~"), mpu6050_() {
        loadParameters();
        mpu6050_.setAddress(static_cast<uint8_t>(mpu6050_addr_));
        mpu6050_.initialize(i2c_bus_uri_);
        mpu6050_.setDLPFMode(static_cast<uint8_t>(3));
        mpu6050_.setIntDataReadyEnabled(true);
    }

    void run() {
        std::vector<std::string> side_names{
            "+X up", "-X up", "+Y up", "-Y up", "+Z up", "-Z up"
        };

        std::vector<Eigen::Vector3d> true_accs{
            {1.0, 0.0, 0.0}, {-1.0, 0.0, 0.0},
            {0.0, 1.0, 0.0}, {0.0, -1.0, 0.0},
            {0.0, 0.0, 1.0}, {0.0, 0.0, -1.0}
        };

        std::set<int> used_sides;

        while (ros::ok() && used_sides.size() < 6) {
            std::cout << "Place the sensor in a new position and press Enter." << std::endl;
            std::cin.ignore();

            Eigen::Vector3d avg_measurement = collectMeasurements();

            int side = detectSide(avg_measurement);
            if (used_sides.find(side) != used_sides.end()) {
                std::cout << "This side has already been used. Please use another side." << std::endl;
                continue;
            }

            std::cout << "Collecting measurements for side " << side_names[side] << "." << std::endl;
            std::cout << "Hold the current position for 2 seconds..." << std::endl;
            ros::Duration(2.0).sleep();  // Wait for 2 seconds to ensure the IMU is stable

            collectMeasurementsForSide(true_accs[side]);
            used_sides.insert(side);
            std::cout << "Measurements for side " << side_names[side] << " collected." << std::endl;
        }

        std::cout << "Now, place the sensor with +Z pointing up and keep it level. Press Enter to start gyroscope calibration." << std::endl;
        std::cin.ignore();

        collectGyroMeasurements();
        
        if (ros::ok()) {
            calibration_.calibrate();
            calibration_.printCalibration();
            calibration_.writeCalibrationToYAML(yaml_output_path_);
        }

        ros::shutdown();
    }

private:
    ros::NodeHandle nh_;
    std::string i2c_bus_uri_;
    int mpu6050_addr_;
    int num_measurements_;
    std::string yaml_output_path_;
    mpu6050_driver::MPU6050 mpu6050_;
    SensorCalibration calibration_;

    void loadParameters() {
        nh_.param<std::string>("bus_uri", i2c_bus_uri_, "/dev/i2c-1");
        nh_.param<int>("mpu_address", mpu6050_addr_, 0x68);
        nh_.param<int>("num_measurements", num_measurements_, 100);
        nh_.param<std::string>("yaml_output_path", yaml_output_path_, "");
    }

    Eigen::Vector3d collectMeasurements() {
        std::vector<Eigen::Vector3d> measurements;
        for (int i = 0; i < num_measurements_; ++i) {
            if (mpu6050_.getIntDataReadyStatus()) {
                mpu6050_driver::IMUData<float> mpu_data = mpu6050_.getMotion6();
                Eigen::Vector3d measured_acc(mpu_data.accel.x, mpu_data.accel.y, mpu_data.accel.z);
                measurements.push_back(measured_acc);
                ros::Duration(0.02).sleep(); // Adjust the delay as necessary
            }
        }
        Eigen::Vector3d avg_measurement = Eigen::Vector3d::Zero();
        for (const auto& measurement : measurements) {
            avg_measurement += measurement;
        }
        avg_measurement /= measurements.size();
        return avg_measurement;
    }

    void collectMeasurementsForSide(const Eigen::Vector3d& true_acc) {
        for (int i = 0; i < num_measurements_; ++i) {
            if (mpu6050_.getIntDataReadyStatus()) {
                mpu6050_driver::IMUData<float> mpu_data = mpu6050_.getMotion6();
                Eigen::Vector3d measured_acc(mpu_data.accel.x, mpu_data.accel.y, mpu_data.accel.z);
                calibration_.addMeasurement(measured_acc, true_acc);
                ros::Duration(0.02).sleep(); // Adjust the delay as necessary
            }
        }
    }

    void collectGyroMeasurements() {
        for (int i = 0; i < num_measurements_; ++i) {
            if (mpu6050_.getIntDataReadyStatus()) {
                mpu6050_driver::IMUData<float> mpu_data = mpu6050_.getMotion6();
                Eigen::Vector3d measured_gyro(mpu_data.gyro.x, mpu_data.gyro.y, mpu_data.gyro.z);
                calibration_.addGyroMeasurement(measured_gyro);
                ros::Duration(0.02).sleep(); // Adjust the delay as necessary
            }
        }
    }

    int detectSide(const Eigen::Vector3d& measurement) {
        int max_index;
        measurement.cwiseAbs().maxCoeff(&max_index);
        double sign = measurement[max_index] > 0 ? 1.0 : -1.0;
        return max_index * 2 + (sign < 0 ? 1 : 0);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mpu6050_calibration_node");
    MPU6050CalibrationNode node;
    node.run();
    return 0;
}
