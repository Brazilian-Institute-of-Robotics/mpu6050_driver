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

class MPU6050Node {
 public:
  MPU6050Node();
  void init();
  void run();
  void publishMPUData();

  /**
   * @brief Helper method to get parameters from parameter server.
   * 
   * This method will checks if a param with @p param_name exists in parameter
   * server. If it isn't exists, the @p default_value will be assigned to @p param.
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
  void loadParameters();
  void setMPUOffsets();
  ros::NodeHandle nh_;
  ros::Publisher mpu_data_pub_;

  MPU6050 mpu6050_;

  int mpu6050_addr_;
  std::string i2c_bus_uri_;

  float pub_rate_;
  std::string imu_frame_id_;
  std::vector<int> axes_offsets_;
};

}  // namespace mpu6050_driver

#endif  // MPU6050_DRIVER_MPU6050_NODE_HPP_
