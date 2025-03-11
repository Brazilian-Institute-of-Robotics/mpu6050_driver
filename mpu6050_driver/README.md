# mpu6050_driver_ros2

Tested under ROS2 Foxy, Ubuntu 20.04 LTS

1. Clone the repo
2. Build
3. Connect the mpu6050 to the RaspberryPi
4. Run calibration node `ros2 run mpu6050_driver_ros2 mpu6050_driver_calibration_ros2_node`
5. Add the results to `mpu_settings.yaml`
6. Launch the node (recommended the one with filter)
7. PLAY!