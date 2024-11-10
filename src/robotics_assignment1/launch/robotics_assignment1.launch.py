#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim_node',
            output='screen'
        ),
        Node(
            package='robotics_assignment1',
            executable='SpawnTurtlesServer.py',
            name='spawn_turtles_server',
            output='screen'
        ),
        Node(
            package='robotics_assignment1',
            executable='MoveTurtle1ActionServer.py',
            name='move_turtle1_action_server',
            output='screen'
        )
    ])