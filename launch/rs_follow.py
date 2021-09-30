
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
                {
                    "depth_topic": "/camera/depth/image_rect_raw",
                    "cmd_topic":"/mi123456/cmd_vel",
                    "min_y": 0.1,
                    "max_y": 0.5,
                    "min_x": -0.3,
                    "max_x": 0.3,
                    "max_z": 1.5,
                    "goal_z": 0.6,
                    "z_scale": 1.0,
                    "x_scale": 5.0,
                    "enabled": True,
                }
            ]
        )
    ])