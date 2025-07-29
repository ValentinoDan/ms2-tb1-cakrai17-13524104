from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('pkg_13524104'),
        'config',
        'connector_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='pkg_13524104',
            executable='connector_node.py',
            name='connector_node',
            output='screen',
            parameters=[config]
        )
    ])
