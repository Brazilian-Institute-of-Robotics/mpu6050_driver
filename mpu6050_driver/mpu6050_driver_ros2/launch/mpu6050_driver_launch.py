import os
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    config_dir = os.path.join(get_package_share_directory('mpu6050_driver_ros2'), 'config')

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package='mpu6050_driver_ros2',
                executable='mpu6050_driver_ros2_node',
                name='mpu_driver_node',
                output='screen',
                parameters=[os.path.join(config_dir, 'mpu_settings.yaml')],
            )
        ]
    )
