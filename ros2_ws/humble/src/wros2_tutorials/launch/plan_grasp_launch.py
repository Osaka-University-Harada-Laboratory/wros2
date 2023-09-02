from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    packages_name = "wros2_tutorials"
    rviz_file_name = "grasps.rviz"

    object_mesh_path = LaunchConfiguration(
        'object_mesh_path',
        default="/ros2_ws/src/wros2_tutorials/wrs/0000_examples/objects/tubebig.stl")
    gripper_name = LaunchConfiguration(
        'gripper_name',
        default="robotiqhe")

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(packages_name), "config", rviz_file_name])

    return LaunchDescription([
        DeclareLaunchArgument(
            'object_mesh_path',
            default_value=object_mesh_path,
            description='Path to the target object mesh file'),
        DeclareLaunchArgument(
            'gripper_name',
            default_value=gripper_name,
            description='Gripper name to be used for planning'),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"]),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "world", "object"]),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file]),
        Node(
            package='wros2_tutorials',
            executable='grasp_planning_service',
            parameters=[
                {'object_mesh_path': object_mesh_path},
                {'gripper_name': gripper_name}],
            name='grasp_planning_server',
            output='screen')])
