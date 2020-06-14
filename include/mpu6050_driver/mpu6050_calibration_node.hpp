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

#ifndef MPU6050_DRIVER_MPU6050_CALIBRATION_NODE_HPP_
#define MPU6050_DRIVER_MPU6050_CALIBRATION_NODE_HPP_

#include <eigen3/Eigen/Dense>

#include <cstdint>
#include <string>

#include "ros/ros.h"

#include "mpu6050_driver/mpu6050.hpp"
#include "mpu6050_driver/mpu6050_node.hpp"

namespace mpu6050_driver {

/**
 * @brief Implement MPU6050 calibration process through a ROS node
 * 
 * This class implements a node to compute the offsets that must be applied in
 * the MPU6050 to fix misaligment mistakes. For this purpose, is being used the
 * PID algorithm to compute the offsets of all MPU6050 axes.
 * 
 */
class MPU6050CalibrationNode : public MPU6050Node {
 public:
  /**
   * @brief Construct a new MPU6050CalibrationNode object
   * 
   */
  MPU6050CalibrationNode();

  /**
   * @brief Run all neccessary setup for calibration process.
   * 
   * This method invokes the methods responsible to setup the MPU and the ROS node
   * such as MPU6050 initialization, ros parameters loading and node advertising.
   * 
   */
  void init();

  /**
   * @brief Print, using ROS_INFO, the current calibration offsets applied
   * in MPU6050.
   * 
   * In each loop of calibration process, new offsets are computed and applied
   * in MPU6050 offsets registers. Then, this method will show the current
   * offsets that are beign applied
   * 
   */
  void printOffsets();

  /**
   * @brief Run the calibration node.
   * 
   * The calibration node is characterized as an infinit loop which each loop
   * realize the computation of calibration offsets, the offsets adjustments and
   * the publication of offset data and current MPU6050 data. The loop frequency
   * is defined in the configuration file. The loop ends when the error between
   * the setpoint and current offsets values is close zero with the tolerance
   * defined by the field delta in the configuration file
   * 
   * @see computeOffsets
   * @see adjustOffsets
   * @see publishOffsets
   * @see isCalibrationFinished 
   * 
   */
  void run();

 private:
  /**
   * @brief Load the parameters from ROS parameter server
   * 
   */
  void loadParameters();

  /**
   * @brief Compute the offsets that must be applied to calibrate the MPU
   * 
   * The computation of the calibration offsets is perfomed using the PID
   * algorithm. For that, take on that all the values, except for aceleration in
   * the Z axis that must be equal gravity aceleration, must be zero or close
   * zero when the imu sensor is not moving and is on a flat surface
   * 
   * @see kp_
   * @see ki_
   * @see i_term_matrix_
   * @see p_term_matrix_
   * @see offset_matrix_
   * @see error_matrix_
   */
  void computeOffsets();

  /**
   * @brief Apply the calibration offsets computed in the MPU6050
   * 
   * @see computeOffsets
   */
  void adjustOffsets();

  /**
   * @brief Publish the calibration offsets computed in a topic
   * 
   */
  void publishOffsets();

  /**
   * @brief Chech whether calibration process is finished
   * 
   * The calibration process will ends when the error of PID algorithm for all
   * imu axes is close zero with the tolerance delta_, which is defined in the
   * configuration file
   * 
   * @return true If the error is in the tolerance boundary
   * @return false If the error isn't in the tolerance boundary
   * 
   * @see delta_
   * @see error_matrix_
   */
  bool isCalibrationFinished();

  /** Nodehandle for calibration node**/
  ros::NodeHandle nh_;
  /** Publisher object for calibration offsets data **/
  ros::Publisher imu_offsets_pub_;

  /** Store proportional gain for PID algorithm **/
  float kp_;
  /** Store integral gain for PID algorithm **/
  float ki_;
  /** Store the error tolerance acceptable **/
  float delta_;

  /** Store the proportional terms of PID algorithm of all MPU axes **/
  Eigen::Matrix<float, 3, 2> p_term_matrix_;
  /** Store the integral terms of PID algorithm of all MPU axes **/
  Eigen::Matrix<float, 3, 2> i_term_matrix_;
  /** Store the error of PID algorithm of all MPU axes **/
  Eigen::Matrix<float, 3, 2> error_matrix_;
  /** Store the calibration offsets of all MPU axes **/
  Eigen::Matrix<float, 3, 2> offset_matrix_;
};

}  // namespace mpu6050_driver

#endif  // MPU6050_DRIVER_MPU6050_CALIBRATION_NODE_HPP_
