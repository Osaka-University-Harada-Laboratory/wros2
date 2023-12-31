import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    rviz_config = PathJoinSubstitution(
        [FindPackageShare('wros2_tutorials'),
         'config',
         'grasps.rviz'])
    config = DeclareLaunchArgument(
        'config',
        default_value='planning_params_robotiqhe_example.yaml')
    rosparam_config = PathJoinSubstitution(
        [FindPackageShare('wros2_tutorials'),
        'config',
        LaunchConfiguration('config')])

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link']),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'object']),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config]),
        Node(
            package='wros2_tutorials',
            executable='grasp_planning_service',
            parameters=[rosparam_config],
            name='grasp_planning_server',
            output='screen')])
