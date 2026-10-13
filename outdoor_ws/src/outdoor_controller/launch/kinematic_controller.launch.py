import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    outdoor_controller_dir = get_package_share_directory("outdoor_controller")
    config_file = os.path.join(outdoor_controller_dir, "config", "outdoor_controller.yaml")

    return LaunchDescription([
        Node(
            package="twist_mux",
            executable="twist_mux",
            name="twist_mux",
            output="screen",
            parameters=[config_file],
        ),

        Node(
            package='outdoor_controller',
            executable='cmd_vel_filter_node',
            name='cmd_vel_filter_node',
            output='screen',
            parameters=[config_file],
        ),

        Node(
            package='outdoor_controller',
            executable='kinematic_controller_node',
            name='kinematic_controller_node',
            output='screen',
            parameters=[config_file],
        )


    ])