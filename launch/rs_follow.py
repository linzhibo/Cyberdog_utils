
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cyberdog_utils',
            node_executable='rs_follower',
            node_name='rs_follower',
            remappings=[
                ('/cmd_vel', '/mi123456/cmd_out'),
                ('/depth', '/mi123456/camera/depth'),
            ]
        )
    ])