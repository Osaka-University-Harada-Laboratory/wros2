from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    packages_name = "wros2_tutorials"
    rviz_file_name = "grasps.rviz"

    tf2_node_baselink = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"])
    tf2_node_object = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "object"])
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(packages_name), "config", rviz_file_name])
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file])
    grasp_plan_service_node = Node(
        package='wros2_tutorials',
        executable='grasp_planning_service',
        name='grasp_planning_service',
        output='screen')

    return LaunchDescription([
        tf2_node_baselink,
        tf2_node_object,
        rviz2_node,
        grasp_plan_service_node])
