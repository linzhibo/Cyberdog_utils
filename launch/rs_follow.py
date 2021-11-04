import os
from launch import LaunchDescription
from launch_ros.actions import Node
from athena_pycommon import Get_Namespace

def generate_launch_description():

    get_namespace = launch.actions.DeclareLaunchArgument(
        'namespace',
        default_value=Get_Namespace(),
        description='Namespace of cyberdog')
    namespace = launch.substitutions.LaunchConfiguration('namespace')

    depth_topic_cam_info = os.path.join(namespace , "camera/depth/camera_info")
    depth_topic = os.path.join(namespace , "camera/depth/image_rect_raw")
    cmd_topic = os.path.join(namespace , "body_cmd")

    follower_node = Node(
            package='cyberdog_utils',
            executable='rs_follower', #foxy change node_executable to executable, same for node_name
            name='rs_follower',
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "depth_topic_cam_info": depth_topic_cam_info,
                    "depth_topic": depth_topic,
                    "cmd_topic": cmd_topic,
                    "min_y": -0.5,
                    "max_y": -0.1,
                    "min_x": -0.3,
                    "max_x": 0.3,
                    "max_z": 2.0,
                    "goal_z": 0.6,
                    "z_scale": 2.0,
                    "x_scale": 2.0,
                    "enabled": True,
                    "namespace": namespace,
                }
            ]
        )

    return LaunchDescription([
        get_namespace,
        follower_node,
    ])