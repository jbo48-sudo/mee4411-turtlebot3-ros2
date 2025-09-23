#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    collision_detector = Node(
        package='mee4411_simulation',
        executable='collision_detector',
        output='screen',
        parameters=[
            {'tb3_model': EnvironmentVariable('TURTLEBOT3_MODEL')},
            
        ]
    )

    return LaunchDescription([
        collision_detector,
        RegisterEventHandler(
            OnProcessExit(
                target_action=collision_detector,
                on_exit=[
                    EmitEvent(event=Shutdown()),
                ]
            )
        ),
    ])
