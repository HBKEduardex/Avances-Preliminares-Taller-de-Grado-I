#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pick_place_sequence.launch.py

Lanza el nodo pick_place_sequence_node para ejecutar o validar
una secuencia de pick and place. No lanza MoveIt2; asume que ya
está corriendo en otra terminal.

Argumentos aceptados:
  - execute (default: false)
  - points_file
  - planning_group
  - velocity_scaling
  - acceleration_scaling
  - return_home
  - wait_between_points
  - stop_on_failure
  - reference_frame
  - move_group_action_name
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Declarar argumentos ─────────────────────────────────────
    execute_arg = DeclareLaunchArgument(
        'execute', default_value='false',
        description='Si es true, planifica y ejecuta. '
                    'Si es false, solo planifica (validación).'
    )
    points_file_arg = DeclareLaunchArgument(
        'points_file',
        default_value='config/pick_place_points.yaml',
        description='Ruta al archivo YAML con los puntos.'
    )
    planning_group_arg = DeclareLaunchArgument(
        'planning_group', default_value='manipulator',
        description='Grupo de planificación de MoveIt2.'
    )
    velocity_scaling_arg = DeclareLaunchArgument(
        'velocity_scaling', default_value='0.10',
        description='Factor de escalamiento de velocidad (0.0-1.0).'
    )
    acceleration_scaling_arg = DeclareLaunchArgument(
        'acceleration_scaling', default_value='0.10',
        description='Factor de escalamiento de aceleración (0.0-1.0).'
    )
    return_home_arg = DeclareLaunchArgument(
        'return_home', default_value='true',
        description='Regresar a home al final de la secuencia.'
    )
    wait_between_points_arg = DeclareLaunchArgument(
        'wait_between_points', default_value='1.0',
        description='Segundos de espera entre puntos.'
    )
    stop_on_failure_arg = DeclareLaunchArgument(
        'stop_on_failure', default_value='true',
        description='Detener secuencia si un punto falla.'
    )
    reference_frame_arg = DeclareLaunchArgument(
        'reference_frame', default_value='base_link',
        description='Frame de referencia para planificación.'
    )
    move_group_action_name_arg = DeclareLaunchArgument(
        'move_group_action_name', default_value='/move_action',
        description='Nombre del action server de MoveIt2 MoveGroup.'
    )
    use_fake_controller_arg = DeclareLaunchArgument(
        'use_fake_controller', default_value='false',
        description='Si es true, lanza un action server falso para la ejecución de trayectorias.'
    )

    # ── Nodo de secuencia ───────────────────────────────────────
    sequence_node = Node(
        package='kuka_pick_place_demo',
        executable='pick_place_sequence_node',
        name='pick_place_sequence_node',
        output='screen',
        parameters=[{
            'planning_group': LaunchConfiguration('planning_group'),
            'execute': LaunchConfiguration('execute'),
            'velocity_scaling': LaunchConfiguration(
                'velocity_scaling'),
            'acceleration_scaling': LaunchConfiguration(
                'acceleration_scaling'),
            'points_file': LaunchConfiguration('points_file'),
            'return_home': LaunchConfiguration('return_home'),
            'wait_between_points': LaunchConfiguration(
                'wait_between_points'),
            'stop_on_failure': LaunchConfiguration(
                'stop_on_failure'),
            'reference_frame': LaunchConfiguration(
                'reference_frame'),
            'move_group_action_name': LaunchConfiguration(
                'move_group_action_name'),
        }],
    )

    # ── Nodo controlador falso (opcional) ───────────────────────
    fake_controller_node = Node(
        package='kuka_pick_place_demo',
        executable='fake_trajectory_controller_node',
        name='fake_trajectory_controller_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_fake_controller')),
    )

    return LaunchDescription([
        execute_arg,
        points_file_arg,
        planning_group_arg,
        velocity_scaling_arg,
        acceleration_scaling_arg,
        return_home_arg,
        wait_between_points_arg,
        stop_on_failure_arg,
        reference_frame_arg,
        move_group_action_name_arg,
        use_fake_controller_arg,
        sequence_node,
        fake_controller_node,
    ])
