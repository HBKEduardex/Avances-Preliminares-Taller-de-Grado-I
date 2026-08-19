#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
trajectory_logger.launch.py

Lanza UNICAMENTE el nodo registrador de trayectorias.

Este launch NO levanta MoveIt2, RViz, el bridge ni el controlador: esos
componentes ya los levanta kuka_gui_moveit_bridge/kuka_bridge_system.launch.py.
El logger es un observador pasivo que se conecta a un sistema ya en marcha,
por lo que se ejecuta en una terminal aparte y puede iniciarse y detenerse
sin afectar a nada.

Uso tipico:
    ros2 launch kuka_trajectory_logger trajectory_logger.launch.py

Con argumentos:
    ros2 launch kuka_trajectory_logger trajectory_logger.launch.py \
        output_directory:=/root/taller1/ros2_ws/trajectory_logs \
        print_points:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ── Configuracion por defecto del paquete ───────────────────────────────
    logger_config_file = os.path.join(
        get_package_share_directory('kuka_trajectory_logger'),
        'config', 'kuka_trajectory_logger.yaml')

    # ── Argumentos del launch (sobrescriben el YAML) ────────────────────────
    trajectory_topic_arg = DeclareLaunchArgument(
        'trajectory_topic',
        default_value='/display_planned_path',
        description='Topico moveit_msgs/msg/DisplayTrajectory a observar')

    output_directory_arg = DeclareLaunchArgument(
        'output_directory',
        default_value='',
        description='Carpeta de los CSV (vacio = automatico)')

    print_points_arg = DeclareLaunchArgument(
        'print_points',
        default_value='true',
        description='Imprimir todos los puntos de cada trayectoria')

    save_velocities_arg = DeclareLaunchArgument(
        'save_velocities',
        default_value='true',
        description='Guardar velocidades entregadas por MoveIt2')

    save_accelerations_arg = DeclareLaunchArgument(
        'save_accelerations',
        default_value='true',
        description='Guardar aceleraciones entregadas por MoveIt2')

    save_effort_arg = DeclareLaunchArgument(
        'save_effort',
        default_value='false',
        description='Guardar effort (solo si MoveIt2 realmente lo entrega)')

    # ── Nodo ────────────────────────────────────────────────────────────────
    logger_node = Node(
        package='kuka_trajectory_logger',
        executable='kuka_trajectory_logger_node',
        name='kuka_trajectory_logger_node',
        output='both',
        emulate_tty=True,
        parameters=[
            logger_config_file,
            {
                'trajectory_topic': LaunchConfiguration('trajectory_topic'),
                'output_directory': LaunchConfiguration('output_directory'),
                'print_points': ParameterValue(
                    LaunchConfiguration('print_points'), value_type=bool),
                'save_velocities': ParameterValue(
                    LaunchConfiguration('save_velocities'), value_type=bool),
                'save_accelerations': ParameterValue(
                    LaunchConfiguration('save_accelerations'), value_type=bool),
                'save_effort': ParameterValue(
                    LaunchConfiguration('save_effort'), value_type=bool),
            },
        ],
    )

    return LaunchDescription([
        trajectory_topic_arg,
        output_directory_arg,
        print_points_arg,
        save_velocities_arg,
        save_accelerations_arg,
        save_effort_arg,
        logger_node,
    ])
