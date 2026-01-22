from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('server_rviz')
    rviz_cfg = os.path.join(pkg_path, 'rviz', 'server.rviz')

    return LaunchDescription([
        Node(
            package='server_rviz',
            executable='global_map_publisher',
            name='global_map_pub',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='server_rviz',
            arguments=['-d', rviz_cfg],
            output='screen'
        )
    ])
