# MPU6050 Driver

## Overview

This is a ROS package to use the MPU6050 IMU sensor on platforms with Raspberry boards. The package was developed to be used in the [Doogie Mouse robot]. The sensor libraries were adapted from the [Arduino Library] of MPU6050 sensor.

**Keywords:** MPU6050, ROS, Raspberry

### License

The source code is released under a [MIT license](LICENSE).

**Author:** Mateus Meneses
**Maintainer:** Mateus Meneses, mateusmenezes95@gmail.com

The MPU6050 Driver package has been tested under [ROS] Kinetic and [Raspbian Jessie]. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (Middleware for robotics),
- [wiringPi] (GPIO Interface library for the Raspberry Pi)

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

```sh
$ cd catkin_ws/src
$ git clone https://github.com/mateusmenezes95/mpu6050_driver.git
$ cd ../
$ rosdep install --from-paths . --ignore-src
$ catkin_make
```

## Usage

Describe the quickest way to run this software, for example:

Run the main node with

```sh
$ roslaunch mpu6050_driver mpu6050_driver.launch
```

## Config files

Config file folder/set 1

* **config_file_1.yaml** Shortly explain the content of this config file

## Launch files

* **launch_file_1.launch:** shortly explain what is launched (e.g standard simulation, simulation with gdb,...)

     Argument set 1

     - **`argument_1`** Short description (e.g. as commented in launch file). Default: `default_value`.

    Argument set 2

    - **`...`**

## Nodes

### imu_mpu6050_node

Reads temperature measurements and computed the average.

#### Subscribed Topics

* **`/temperature`** ([sensor_msgs/Temperature])

	The temperature measurements from which the average is computed.


#### Published Topics

...


#### Services

* **`get_average`** ([std_srvs/Trigger])

	Returns information about the current average. For example, you can trigger the computation from the console with

		rosservice call /ros_package_template/get_average


#### Parameters

* **`subscriber_topic`** (string, default: "/temperature")

	The name of the input topic.

* **`cache_size`** (int, default: 200, min: 0, max: 1000)

	The size of the cache.

## 4. More info

For a overview of the APIs, generate the class documentation by running the following commands:
```sh
$ cd doc/
$ doxygen Doxyfile
```
and the documentation files will be automatically generated.
For more detailed overview of the APIs please refer to the source code (it's fairly simple).

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/mateusmenezes95/mpu6050_driver/issues).

[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[Eigen]: http://eigen.tuxfamily.org
[std_srvs/Trigger]: http://docs.ros.org/api/std_srvs/html/srv/Trigger.html
[sensor_msgs/Temperature]: http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html
[Doogie Mouse robot]: https://github.com/Brazilian-Institute-of-Robotics/doogie
[Arduino Library]: https://github.com/ElectronicCats/mpu6050
[Raspbian Jessie]: https://www.raspberrypi.org/downloads/raspbian/
[wiringPi]: http://wiringpi.com/
