"""Start LIO-SAM without xacro; RViz is optional and disabled by default."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share_dir = get_package_share_directory('lio_sam')
    parameters = [
        LaunchConfiguration('params_file'),
        {'use_sim_time': ParameterValue(
            LaunchConfiguration('use_sim_time'), value_type=bool)},
    ]
    actions = [
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(share_dir, 'config', 'params.yaml'),
            description='ROS 2 parameter YAML file.'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        # This is the non-xacro TF published by upstream run.launch.py.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'map', '--child-frame-id', 'odom'],
            parameters=parameters, output='screen'),
    ]
    for suffix in ('imuPreintegration', 'imageProjection',
                   'featureExtraction', 'mapOptimization'):
        executable = 'lio_sam_' + suffix
        actions.append(Node(
            package='lio_sam', executable=executable, name=executable,
            parameters=parameters, output='screen'))
    actions.append(Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(share_dir, 'config', 'rviz2.rviz')],
        parameters=[parameters[1]],
        condition=IfCondition(LaunchConfiguration('rviz')), output='screen'))
    return LaunchDescription(actions)
