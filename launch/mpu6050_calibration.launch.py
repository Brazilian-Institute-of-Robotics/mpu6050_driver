from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the share directory of the 'mpu6050_driver' package
    config_path = os.path.join(get_package_share_directory('mpu6050_driver'), 'config', 'mpu_settings_calib.yaml')

    return LaunchDescription([
        # Declare node to be launched
        Node(
            package='mpu6050_driver',
            executable='mpu6050_calibration_node',
            name='mpu_calibration_node',
            output='screen',
            parameters=[config_path]
        )
    ])
