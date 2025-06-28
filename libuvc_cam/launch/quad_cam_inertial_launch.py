from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('libuvc_cam'),
        'config',
        'quad_cam_inertial.yaml'
    )

    return LaunchDescription([
        Node(
            package='libuvc_cam',
            executable='libuvc_cam_node',
            name='xiaoyu_quad_cam_inertial',
            parameters=[config_file]
        )
    ])
