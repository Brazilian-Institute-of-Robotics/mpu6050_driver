

/* ============================================
MIT License

//  Copyright (c) 2020 Mateus Meneses
//  Copyright (c) 2024 Mohamed Abdelkader

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

#ifndef MPU6050_DRIVER_MPU6050_NODE_HPP_
#define MPU6050_DRIVER_MPU6050_NODE_HPP_

#include <cstdint>
#include <string>
#include <vector>
#include <memory> // For std::shared_ptr

#include "rclcpp/rclcpp.hpp" // ROS2 header
#include "mpu6050_driver/mpu6050.hpp" // Adjust this include path according to your project structure
#include "sensor_msgs/msg/imu.hpp" // ROS2 IMU message type

namespace mpu6050_driver {

/**
 * @brief Provides MPU6050 data to ROS2 nodes.
 * 
 * This class acts as a middle layer between the MPU6050 library and ROS2
 * applications. It initializes communication with the MPU6050 sensor and publishes its data
 * to a ROS2 topic at a configured rate.
 */
class MPU6050Node : public rclcpp::Node {
public:
  /**
   * @brief Construct a new MPU6050Node object
   * 
   * The constructor initializes the ROS2 node handle and sets up the MPU6050 sensor
   * interface.
   */
  MPU6050Node();

  /**
   * @brief Initialize the node and set up the MPU6050 sensor.
   * 
   * This method loads parameters from the ROS2 parameter server, sets up the MPU6050 sensor
   * with any specified calibration offsets, and initializes the publisher for publishing
   * sensor data to a ROS2 topic.
   */
  void init();

  MPU6050 mpu6050_; // Interface to the MPU6050 sensor
  float pub_rate_; // Publication rate for MPU6050 data
  std::string imu_frame_id_; // Frame ID for the published Imu messages

private:
  

  /**
   * @brief Publish MPU6050 sensor data.
   * 
   * This method is called periodically according to the configured publication rate. It
   * retrieves data from the MPU6050 sensor and publishes it on the "imu/data_raw" topic as
   * a sensor_msgs/msg/Imu message.
   */
  void publishMPUData();

  /**
   * @brief Load node parameters from the ROS2 parameter server.
   * 
   * Parameters include the I2C bus URI, MPU6050 address, publication rate, frame ID for
   * the published messages, and any initial calibration offsets for the sensor.
   */
  void loadParameters();

  /**
   * @brief Apply calibration offsets to correct for sensor misalignment.
   * 
   * This method applies configured offset values to the MPU6050's calibration registers
   * to correct for any known sensor misalignments.
   */
  void setMPUOffsets();

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr mpu_data_pub_; // Publisher for MPU6050 data
  rclcpp::TimerBase::SharedPtr data_publish_timer_; // Timer for periodic data publishing


  int mpu6050_addr_; // I2C address of the MPU6050 sensor
  std::string i2c_bus_uri_; // I2C bus URI

  std::vector<int> axes_offsets_; // Calibration offsets for sensor initialization
};

}  // namespace mpu6050_driver

#endif  // MPU6050_DRIVER_MPU6050_NODE_HPP_
