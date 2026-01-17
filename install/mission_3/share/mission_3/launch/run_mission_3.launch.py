#!/usr/bin/env python3
"""Launch both CAV control and Control Tower for mission_3."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Optional arg to enable ROI visualizer in mission_3 package
    ld.add_action(DeclareLaunchArgument('enable_roi', default_value='false', description='Start roi_visualizer'))

    cav = Node(
        package='mission_3',
        executable='control_cav_mission_3',
        name='control_cav_mission_3',
        output='screen',
        emulate_tty=True,
    )

    tower = Node(
        package='mission_3',
        executable='control_tower_mission_3',
        name='control_tower_mission_3',
        output='screen',
        emulate_tty=True,
    )

    ld.add_action(cav)
    ld.add_action(tower)

    # Optional ROI visualizer (runs from same package if requested)
    enable_roi = LaunchConfiguration('enable_roi')
    from launch.actions import OpaqueFunction

    def _maybe_roi(context, *args, **kwargs):
        if enable_roi.perform(context).lower() in ('1','true','yes'):
            return [Node(package='mission_3', executable='roi_visualizer', name='roi_visualizer', output='screen', emulate_tty=True)]
        return []

    ld.add_action(OpaqueFunction(function=_maybe_roi))

    return ld
