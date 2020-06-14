#ifndef MPU6050_DRIVER_MPU6050_NODE_HPP_
#define MPU6050_DRIVER_MPU6050_NODE_HPP_

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

#include <cstdint>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "mpu6050_driver/mpu6050.hpp"

namespace mpu6050_driver {

/**
 * @brief Provide MPU6050 data to ROS nodes
 * 
 * This class intend to act as middle layer between the MPU6050 library and ROS
 * applications. Thus, this class basically apply the basics settings to
 * comunicate with the MPU6050 sensor and then get data from de sensor and make it
 * available from ROS interfaces 
 * 1
 */
class MPU6050Node {
 public:
  /**
   * @brief Construct a new MPU6050Node object
   * 
   */
  MPU6050Node();

  /**
   * @brief Initialize the node and set up the MPU6050 
   * 
   * This method invokes the methods responsible to setup the MPU and the ROS node
   * such as MPU6050 initialization, ros parameters loading and node advertising.
   * 
   */
  void init();

  /**
   * @brief Run the MPU6050 node
   * 
   * This method is a loop which periodically get data from the MPU6050 sensor
   * and publish in a ROS topic. The frequency which the data will be published
   * is defined in the node configuration file
   * 
   * @see pub_rate_
   * @see publishMPUData
   */
  void run();

  /**
   * @brief Publish the MPU6050 data
   * 
   * Get the linear aceleration and the angular velocity of all MPU axes, fills
   * the sensor_msgs/Imu message and then publish in a topic called "imu"
   * 
   * @see mpu_data_pub_
   */
  void publishMPUData();

  /**
   * @brief Helper method to get parameters from parameter server.
   * 
   * This method will checks if a param with @p param_name exists in parameter
   * server. If it isn't exists, the @p default_value will be assigned to @p 
   * param. But diferent of NodeHandle class param method, this method print
   * messages reporting whether or not the parameter was found.
   * 
   * @tparam T Data type of the parameter.
   * @param nh ROS Nodehandle.
   * @param param_name Name of the parameter.
   * @param param Pointer to variable which will store the parameter value.
   * @param default_value Value assigned to @p param if the parameter was not found.
   */
  template <typename T>
  static void getParameterHelper(const ros::NodeHandle &nh, std::string param_name, T *param, T default_value) {
    if (!nh.param<T>(param_name, *param, default_value)) {
      ROS_WARN_STREAM_NAMED("mpu6050_ros_driver", "No " << param_name <<
      " parameter found in the parameter server. Using default parameter value: " << default_value);
    } else {
      ROS_INFO_STREAM("Parameter " << param_name << " found and it value is " << *param);
    }
  }

 protected:
  /**
   * @brief Load the parameters from ROS parameter server
   * 
   */
  void loadParameters();

  /**
   * @brief Set the calibration offsets to fix misaligment mistake
   * 
   * This method get the values from the configuration file and apply in the
   * calibration registers of MPU6050. The offsets values is computed before
   * using the calibration node.
   * 
   * @see MPU6050CalibrationNode
   * 
   */
  void setMPUOffsets();

  /** Nodehandle for main node **/
  ros::NodeHandle nh_;
  /** Publisher object used to publish MPU6050 data **/
  ros::Publisher mpu_data_pub_;

  /** Object used to set/get data to/from MPU6050 **/
  MPU6050 mpu6050_;
  /** Store the I2C address of MPU6050 sensor **/
  int mpu6050_addr_;
  /** Store the Bus URI used to open I2C comunication **/
  std::string i2c_bus_uri_;

  /** Store the frequency which the MPU6050 data will be publised **/
  float pub_rate_;
  /** Store the name of the frame used in the sensos_msgs/Imu message **/
  std::string imu_frame_id_;
  /** Store the offsets to be applied in the MPU6050 initialization **/
  std::vector<int> axes_offsets_;
};

}  // namespace mpu6050_driver

#endif  // MPU6050_DRIVER_MPU6050_NODE_HPP_
