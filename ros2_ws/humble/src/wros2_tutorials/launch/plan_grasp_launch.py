import os
import launch
import launch_ros


def generate_launch_description():

    rviz_config = launch.substitutions.PathJoinSubstitution(
        [launch_ros.substitutions.FindPackageShare('wros2_tutorials'),
         'config',
         'grasps.rviz'])
    config = launch.actions.DeclareLaunchArgument(
        'config',
        default_value='planning_params_robotiqhe_example.yaml')
    rosparam_config = launch.substitutions.PathJoinSubstitution(
        [launch_ros.substitutions.FindPackageShare('wros2_tutorials'),
         'config',
         launch.substitutions.LaunchConfiguration('config')])

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link']),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'object']),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config]),
        launch_ros.actions.Node(
            package='wros2_tutorials',
            executable='grasp_planning_service',
            parameters=[
                rosparam_config,
                {'config_filename': launch.substitutions.LaunchConfiguration('config')}],
            name='grasp_planning_server',
            output='screen')])
