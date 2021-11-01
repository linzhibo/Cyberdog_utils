
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cyberdog_utils',
            executable='rs_follower', #foxy change node_executable to executable, same for node_name
            name='rs_follower',
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "depth_topic_cam_info": "/mi1046017/camera/depth/camera_info",
                    "depth_topic": "/mi1046017/camera/depth/image_rect_raw",
                    "cmd_topic":"/mi1046017/body_cmd",
                    "min_y": -0.5,
                    "max_y": -0.1,
                    "min_x": -0.3,
                    "max_x": 0.3,
                    "max_z": 2.0,
                    "goal_z": 0.6,
                    "z_scale": 2.0,
                    "x_scale": 2.0,
                    "enabled": True,
                }
            ]
        )
    ])