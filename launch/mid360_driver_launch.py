from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('mid360_driver'),
        'config',
        'MID360_config.json'
    )


    return LaunchDescription([
        Node(
            package="mid360_driver",
            executable="mid360_driver_node",
            name="mid360_driver_node",
            parameters=[{
                "config_path": config_path
            }]
        )
    ])
