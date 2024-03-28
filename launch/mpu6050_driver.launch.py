from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Define the path to the configuration file
    config_file_path = os.path.join(get_package_share_directory('mpu6050_driver'), 'config', 'mpu_settings.yaml')

    return LaunchDescription([
        Node(
            package='mpu6050_driver',
            executable='mpu6050_node',
            name='mpu6050_node',
            output='screen',
            parameters=[config_file_path]  # Load parameters from the YAML file
        )
    ])
