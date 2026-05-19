#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
point_recorder.launch.py

Lanza el nodo point_recorder_node para grabar posiciones articulares.
No lanza MoveIt2 ni RViz2; asume que ya están corriendo en otra terminal.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kuka_pick_place_demo',
            executable='point_recorder_node',
            name='point_recorder_node',
            output='screen',
            parameters=[{
                'points_file': 'config/pick_place_points.yaml',
                'joint_names': [
                    'joint_a1', 'joint_a2', 'joint_a3',
                    'joint_a4', 'joint_a5', 'joint_a6',
                ],
                'autosave_backup': True,
            }],
        ),
    ])
