#!/usr/bin/env python3
"""
ROS2 launch file for mission_2.

Usage:
  ros2 launch mission_2 mission_2.launch.py
  ros2 launch mission_2 mission_2.launch.py executable:=control_cav_mission_2_ver2 enable_roi:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _create_nodes(context, *args, **kwargs):
    exec_name = LaunchConfiguration('executable').perform(context)
    enable_roi = LaunchConfiguration('enable_roi').perform(context).lower() in ('1', 'true', 'yes')

    nodes = []

    nodes.append(Node(
        package='mission_2',
        executable=exec_name,
        name='control_cav_mission_2',
        output='screen',
        emulate_tty=True,
    ))

    if enable_roi:
        nodes.append(Node(
            package='mission_2',
            executable='roi_visualizer',
            name='roi_visualizer',
            output='screen',
            emulate_tty=True,
        ))

    return nodes


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('executable', default_value='control_cav_mission_2', description='Control executable'))
    ld.add_action(DeclareLaunchArgument('enable_roi', default_value='false', description='Start roi_visualizer'))
    ld.add_action(OpaqueFunction(function=_create_nodes))

    return ld
