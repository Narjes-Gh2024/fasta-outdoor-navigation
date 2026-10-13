import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    outdoor_odometry_dir = get_package_share_directory("outdoor_odometry")
    # fast_lio_dir = get_package_share_directory("fast_lio")
    # config_file_lidar_odometry = os.path.join(fast_lio_dir, "config", "rslidar.yaml")
    config_file_odometry_backup = os.path.join(outdoor_odometry_dir, "config", "backup_odometry.yaml")
    config_file_outdoor_odometry = os.path.join(outdoor_odometry_dir, "config", "outdoor_odometry.yaml")
        
    return LaunchDescription([
        # IncludeLaunchDescription(
        #         PythonLaunchDescriptionSource(
        #             os.path.join(fast_lio_dir, "launch", "mapping.launch.py")
        #         ),
        #     ), 

        # Node(
        #     package='outdoor_odometry',
        #     executable='frame_converter_node',
        #     name='frame_converter_node',
        #     output='screen',
        #     parameters=[config_file_lidar_odometry],
        # ),

        Node(
            package='outdoor_odometry',
            executable='encoder_odometry_node',
            name='encoder_odometry_node',
            output='screen',
            parameters=[config_file_outdoor_odometry],
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='odometry_backup_ekf_node',
            output='screen',
            parameters=[config_file_odometry_backup],
        ),

        Node(
            package='outdoor_odometry',
            executable='odometry_state_machine_node',
            name='odometry_state_machine_node',
            output='screen',
            parameters=[config_file_outdoor_odometry],
        ),
    ])
