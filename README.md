# MPU6050 Driver

# Overview
This is a ROS package to use the MPU6050 IMU sensor on platforms with Raspberry
boards. The package was developed to be used in the [Doogie Mouse robot]. The
sensor libraries were adapted from the [Arduino Library] of MPU6050 sensor.

**Keywords:** MPU6050, ROS, Raspberry

## License
The source code is released under a [MIT license](LICENSE).

**Author:** Mateus Meneses  
**Maintainer:** Mateus Meneses, mateusmenezes95@gmail.com

The MPU6050 Driver package has been tested under [ROS] Kinetic and [Raspbian Jessie].
This is research code, expect that it changes often and any fitness for a
particular purpose is disclaimed.

# Installation

## Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (Middleware for robotics),
- [i2c_device_ros] (C++ library to read/write from/to I2C devices)

## Building

The first step to build this package is install its dependency. To that, clone
the [i2c_device_ros] package into your workspace

```sh
$ cd <YOUR_WS>/src
$ git clone https://github.com/Brazilian-Institute-of-Robotics/i2c_device_ros.git
```

Now, clone the latest version from this repository into your catkin workspace

```sh
$ cd <YOUR_WS>/src
$ git clone https://github.com/Brazilian-Institute-of-Robotics/mpu6050_driver.git
$ cd ../
```

Then, to build the package you could use two options:

- pure catkin
```sh
$ catkin_make -DCATKIN_WHITELIST_PACKAGES="mpu6050_driver;i2c_device_ros;
```
- catkin tool
```sh
$ catkin build mpu6050_driver
```
# Usage

## Calibration Process

The first thing to do is run the calibration process. Because the misalignment
from sensor assembly mistakes, it's necessary to calibrate the MPU sensor. For
more detailed explanation about calibration process and MPU6050 sensor, see this
great article [The MPU6050 Explained]. To run the calibration process, put the
sensor on its final place on your robot (this is strongly important!) and then
run the calibration node:

```sh
$ roslaunch mpu6050_driver mpu6050_calibration.launch
```

The process will take a few minutes to finish. While that, you can see the IMU
data in the topic called "imu" and the offsets applied to MPU6050 calibration
registers in the topic "imu_offsets". In a new terminal, you can see the imu
messages runnig

```sh
$ rostopic echo /imu -c
```

and the offsets running on a new terminal

```sh
$ rostopic echo /imu_offsets -c
```

You'll see the angular velocity and linear acceleration values converging to
zero, except for the accelerometer Z axis which the value will converge to gravity
aceleration value (9,8 m/s²).

When the calibration process finish, you'll see the final offsets. For not run
the calibration process all the time that you turn on your robot, pick these
values and put in the offsets field in [mpu_settings](config/mpu_settings.yaml)
config file.

## Running main node

After to calibrate the MPU sensor, you can run the main node with

```sh
$ roslaunch mpu6050_driver mpu6050_driver.launch
```

# Config files

* **[mpu_settings.yaml]:** All parameters of MPU6050 used in the calibration node
and main node

# Launch files

* **[mpu6050_calibration.launch](launch/mpu6050_calibration.launch):** Launch the
calibration node

* **[mpu6050_driver.launch](launch/mpu6050_driver.launch):** Launch the main node

# Nodes

## imu_mpu6050_node

Publish the MPU6050 data

### Published Topics
* **`/imu`** ([sensor_msgs/IMU])
Imu data with the values from MPU6050 accelerometer and gyroscope

### Parameters

See the [mpu_settings.yaml]

# More info

For a overview of the APIs, generate the class documentation by running the
following commands:
```sh
$ cd doc/
$ doxygen Doxyfile
```
and the documentation files will be automatically generated.
For more detailed overview of the APIs please refer to the source code (it's fairly simple).

# Bugs & Feature Requests

Please report bugs and request features using the
[Issue Tracker](https://github.com/mateusmenezes95/mpu6050_driver/issues).

[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[Eigen]: http://eigen.tuxfamily.org
[std_srvs/Trigger]: http://docs.ros.org/api/std_srvs/html/srv/Trigger.html
[sensor_msgs/Imu]: http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
[Doogie Mouse robot]: https://github.com/Brazilian-Institute-of-Robotics/doogie
[Arduino Library]: https://github.com/ElectronicCats/mpu6050
[Raspbian Jessie]: https://www.raspberrypi.org/downloads/raspbian/
[i2c_device_ros]: https://github.com/Brazilian-Institute-of-Robotics/i2c_device_ros
[The MPU6050 Explained]: https://mjwhite8119.github.io/Robots/mpu6050
[mpu_settings.yaml]: config/mpu_settings.yaml
