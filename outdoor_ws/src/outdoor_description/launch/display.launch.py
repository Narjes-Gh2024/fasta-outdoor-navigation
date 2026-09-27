from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='outdoor_description').find('outdoor_description')
    default_model_path = os.path.join(pkg_share, 'src', 'description', 'off_road_urdf.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'rviz.rviz')
    
    # robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}],
        # remappings=[('/joint_states', '/joint_command')]  # <-- REMAP HERE
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition=IfCondition(LaunchConfiguration('rviz_launch'))
    )
    
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.5', '0', '0', '0', 'base_link', 'sim_lidar'], 
        condition=IfCondition(LaunchConfiguration('isaac_sim'))
    )
    
    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'sim_imu'], 
        condition=IfCondition(LaunchConfiguration('isaac_sim'))
    )
    
    static_tf_front_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.65', '0', '0.2', '0', '0', '0', 'base_link', 'front_camera'], 
        condition=IfCondition(LaunchConfiguration('isaac_sim'))
    )
    
    static_tf_back_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-0.65', '0', '0.2', '3.14159', '0', '0', 'base_link', 'back_camera'], 
        condition=IfCondition(LaunchConfiguration('isaac_sim'))
    )
    
    static_tf_left_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.43', '0.2', '0', '0', '1.5708', 'base_link', 'left_camera'], 
        condition=IfCondition(LaunchConfiguration('isaac_sim'))
    )
    
    static_tf_right_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '-0.43', '0.2', '0', '0', '-1.5708', 'base_link', 'right_camera'], 
        condition=IfCondition(LaunchConfiguration('isaac_sim'))
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(name='rviz_launch', default_value='True', description='Flag to enable rviz'),
        DeclareLaunchArgument(name='gui', default_value='True', description='Flag to enable joint_state_publisher_gui'),
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot model file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        DeclareLaunchArgument(name='isaac_sim', default_value='True', description='Flag to enable Isaac Sim'),

        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node, 
        static_tf_lidar,
        static_tf_imu,
        static_tf_front_camera,
        static_tf_back_camera,
        static_tf_left_camera,
        static_tf_right_camera
    ])
