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

#ifndef MPU6050_DRIVER_MPU6050_CALIBRATION_HPP_
#define MPU6050_DRIVER_MPU6050_CALIBRATION_HPP_

#include <eigen3/Eigen/Dense>

#include <cstdint>
#include <string>

#include "ros/ros.h"

#include "mpu6050_driver/mpu6050.hpp"
#include "mpu6050_driver/mpu6050_node.hpp"

namespace mpu6050_driver {

class MPU6050CalibrationNode : public MPU6050Node {
 public:
  MPU6050CalibrationNode();
  void init();
  void computeOffsets();
  void publishOffsets();
  void run();

 private:
  void loadParameters();

  ros::NodeHandle nh_;
  ros::Publisher imu_offsets_pub_;

  float kp_;
  float ki_;

  Eigen::Matrix<float, 3, 2> i_term_matrix_;
  Eigen::Matrix<float, 3, 2> p_term_matrix_;
  Eigen::Matrix<float, 3, 2> offset_matrix_;
  Eigen::Matrix<float, 3, 2> error_matrix_;
};

}  // namespace mpu6050_driver

#endif  // MPU6050_DRIVER_MPU6050_CALIBRATION_HPP_
