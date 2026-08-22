#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
trajectory_planner.launch.py

Lanza UNICAMENTE los dos nodos de este paquete:

    kuka_trajectory_generator_node  (generacion, solo plan)
    kuka_trajectory_preview_node    (previsualizacion en RViz2)

Este launch es ADICIONAL y OPCIONAL. No levanta MoveIt2, ni RViz2, ni el
robot_state_publisher, ni el bridge, ni ningun controlador: esos componentes
ya los levantan los launches existentes y no se tocan.

Requisito previo (en otra terminal, sin cambios respecto a hoy):

    ros2 launch kuka_kr6_moveit_config demo.launch.py \
        use_gui:=false use_rviz:=true
  o
    ros2 launch kuka_gui_moveit_bridge kuka_bridge_system.launch.py

Uso:
    ros2 launch kuka_moveit_trajectory_planner trajectory_planner.launch.py

Solo el generador o solo la preview:
    ros2 launch kuka_moveit_trajectory_planner trajectory_planner.launch.py \
        use_preview:=false
    ros2 launch kuka_moveit_trajectory_planner trajectory_planner.launch.py \
        use_generator:=false

Reproducir mas despacio:
    ros2 launch kuka_moveit_trajectory_planner trajectory_planner.launch.py \
        playback_rate:=0.5
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('kuka_moveit_trajectory_planner'),
        'config', 'kuka_moveit_trajectory_planner.yaml')

    use_generator_arg = DeclareLaunchArgument(
        'use_generator',
        default_value='true',
        description='Lanzar el nodo de generacion de trayectorias')

    use_preview_arg = DeclareLaunchArgument(
        'use_preview',
        default_value='true',
        description='Lanzar el nodo de previsualizacion en RViz2')

    playback_rate_arg = DeclareLaunchArgument(
        'playback_rate',
        default_value='1.0',
        description='Escala de tiempo de la previsualizacion (1.0 = real)')

    log_points_arg = DeclareLaunchArgument(
        'log_points',
        default_value='false',
        description='Imprimir en terminal todos los puntos generados')

    generator_node = Node(
        package='kuka_moveit_trajectory_planner',
        executable='kuka_trajectory_generator_node',
        name='kuka_trajectory_generator_node',
        output='both',
        emulate_tty=True,
        parameters=[
            config_file,
            {
                'log_points': ParameterValue(
                    LaunchConfiguration('log_points'), value_type=bool),
            },
        ],
        condition=IfCondition(LaunchConfiguration('use_generator')),
    )

    preview_node = Node(
        package='kuka_moveit_trajectory_planner',
        executable='kuka_trajectory_preview_node',
        name='kuka_trajectory_preview_node',
        output='both',
        emulate_tty=True,
        parameters=[
            config_file,
            {
                'playback_rate': ParameterValue(
                    LaunchConfiguration('playback_rate'), value_type=float),
            },
        ],
        condition=IfCondition(LaunchConfiguration('use_preview')),
    )

    return LaunchDescription([
        use_generator_arg,
        use_preview_arg,
        playback_rate_arg,
        log_points_arg,
        generator_node,
        preview_node,
    ])
