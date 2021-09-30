
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cyberdog_utils',
            node_executable='rs_follower', #foxy change node_executable to executable, same for node_name
            node_name='rs_follower',
            output="screen",
            emulate_tty=True,
            parameters=[
                {"depth_topic": "/camera/depth/image_rect_raw",
                 "cmd_topic":"/mi123456/cmd_vel"
                }
            ]
        )
    ])