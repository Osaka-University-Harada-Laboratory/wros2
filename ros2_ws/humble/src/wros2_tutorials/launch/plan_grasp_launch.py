from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    packages_name = "wros2_tutorials"
    rviz_file_name = "grasps.rviz"

    object_stl_path = LaunchConfiguration(
        'object_stl_path',
        default='/ros2_ws/src/wrs/0000_examples/objects/tubebig.stl')
    gripper_name = LaunchConfiguration(
        'gripper_name',
        default='robotiqhe')

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
        parameters=[
            {'object_stl_path': object_stl_path},
            {'gripper_name': gripper_name}],
        name='grasp_planning_server',
        output='screen')

    return LaunchDescription([
        tf2_node_baselink,
        tf2_node_object,
        rviz2_node,
        grasp_plan_service_node])
