#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
baseline.launch.py — LANZAMIENTO UNICO de la condicion MOVEIT2 BASE.

Un solo comando levanta todo:

    ros2 launch kuka_kr6_moveit_baseline baseline.launch.py

Arranca:
  1. robot_state_publisher   — modelo del KR6 R900 y de la celda (copia local)
  2. joint_state_publisher   — publica /joint_states en el HOME ORIGINAL (A5=0)
  3. move_group              — MoveIt2 con la configuracion BASE de este paquete
  4. rviz2                   — con la configuracion de este paquete
  5. GUI de pruebas          — fork que habla directamente con /move_action
  6. Analizador de continuidad — observador pasivo de /display_planned_path

NO arranca ningun controlador. La condicion base es SOLO PLANIFICACION, por
el limite 4 de la seccion 1.4.1 de la propuesta ("la validacion preliminar se
limitara a la visualizacion y evaluacion cinematica de movimientos mediante
ROS2, RViz2 y MoveIt2") y porque ros2_control no esta instalado en este
contenedor. Ver README seccion 9.

Este launch NO sustituye a ninguno existente: es adicional y vive solo en
este paquete.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    AndSubstitution,
    Command,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

import yaml

PACKAGE = 'kuka_kr6_moveit_baseline'


def load_yaml(relative_path):
    """Carga un YAML del share de ESTE paquete."""
    absolute = os.path.join(get_package_share_directory(PACKAGE), relative_path)
    with open(absolute, 'r') as handle:
        return yaml.safe_load(handle)


def generate_launch_description():
    # ── Argumentos (todos con defecto listo para la demostracion) ───────
    args = [
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Abrir RViz2 con la configuracion del baseline'),
        DeclareLaunchArgument(
            'use_gui', default_value='true',
            description='Abrir la GUI de pruebas del baseline'),
        DeclareLaunchArgument(
            'use_analyzer', default_value='true',
            description='Arrancar el analizador de continuidad articular'),
        DeclareLaunchArgument(
            'write_csv', default_value='true',
            description='El analizador escribe CSV en analysis_output/'),
        DeclareLaunchArgument(
            'output_directory', default_value='',
            description='Carpeta de los CSV (vacio = analysis_output/ '
                        'dentro del paquete)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Usar tiempo de simulacion'),

        # ── J.3 — MODO ARCHIVO y REPLANIFICACION ────────────────────────
        # Los tres defectos DESACTIVAN estas rutas: el comportamiento por
        # defecto del launch es EXACTAMENTE el de antes de anadirlas.
        DeclareLaunchArgument(
            'analyze_json', default_value='false',
            description='MODO ARCHIVO: analizar una secuencia grabada en JSON '
                        'en vez de escuchar /display_planned_path. '
                        'ATENCION: el JSON es del sistema AFINADO, no del '
                        'baseline; la salida se etiqueta AFINADA/ORIGEN '
                        'EXTERNO. Por defecto false.'),
        DeclareLaunchArgument(
            'replan_json', default_value='false',
            description='REPLANIFICAR con el baseline la tarea del JSON '
                        '(variantes A articular y B cartesiana) y generar la '
                        'tabla comparativa de tres condiciones. '
                        'Por defecto false.'),
        DeclareLaunchArgument(
            'input_json', default_value='',
            description='Ruta ABSOLUTA del JSON de secuencia. Se abre en SOLO '
                        'LECTURA. Necesario para analyze_json o replan_json.'),
        DeclareLaunchArgument(
            'replan_variant_joint', default_value='true',
            description='Ejecutar la variante A (metas articulares)'),
        DeclareLaunchArgument(
            'replan_variant_cartesian', default_value='true',
            description='Ejecutar la variante B (metas cartesianas por FK)'),
        DeclareLaunchArgument(
            'preview_json', default_value='false',
            description='REPRODUCIR en RViz la secuencia grabada del JSON, '
                        'para VER las configuraciones articulares. Publica en '
                        '/display_planned_path; NO mueve el robot. '
                        'Por defecto false.'),
        DeclareLaunchArgument(
            'preview_time_scale', default_value='1.0',
            description='Factor de reproduccion de la animacion. 1.0 = tiempos '
                        'del archivo; 3.0 = tres veces mas lento. Solo afecta '
                        'a la visualizacion, nunca a las posiciones ni al CSV.'),
        DeclareLaunchArgument(
            'preview_loop', default_value='true',
            description='Repetir la animacion en bucle'),
        DeclareLaunchArgument(
            'preview_segment', default_value='-1',
            description='Reproducir SOLO un segmento (1..14). -1 = la '
                        'secuencia completa. Util para aislar T1, que es la '
                        'maniobra de escape de la singularidad.'),
    ]
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── Modelo: xacro COPIADO en este paquete ──────────────────────────
    xacro_file = PathJoinSubstitution(
        [FindPackageShare(PACKAGE), 'urdf', 'kr6r900sixx.xacro'])
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file]), value_type=str)}

    srdf_file = PathJoinSubstitution(
        [FindPackageShare(PACKAGE), 'config', 'kuka_kr6_baseline.srdf'])
    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(
            Command(['cat ', srdf_file]), value_type=str)}

    robot_description_kinematics = {
        'robot_description_kinematics': load_yaml('config/kinematics.yaml')}

    # Limites derivados MECANICAMENTE del URDF. A diferencia del sistema
    # afinado, aqui SI se cargan como robot_description_planning.
    robot_description_planning = {
        'robot_description_planning': load_yaml('config/joint_limits.yaml')}

    ompl = load_yaml('config/ompl_planning.yaml')
    planning_pipeline = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl,
    }

    planning_scene_monitor = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # ── HOME ORIGINAL DEL ROBOT: A5 = 0 ────────────────────────────────
    # Configuracion SINGULAR de muñeca (ejes A4 y A6 alineados, rank(J)=5).
    # Se conserva a proposito: es la condicion que el estudio evidencia.
    home_joint_positions = {
        'joint_a1': 0.0,
        'joint_a2': -1.5707963,
        'joint_a3': 1.5707963,
        'joint_a4': 0.0,
        'joint_a5': 0.0,
        'joint_a6': 0.0,
    }

    nodes = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen',
             parameters=[robot_description, {'use_sim_time': use_sim_time}]),

        Node(package='joint_state_publisher', executable='joint_state_publisher',
             name='joint_state_publisher', output='screen',
             parameters=[{'zeros': home_joint_positions,
                          'use_sim_time': use_sim_time}]),

        Node(package='moveit_ros_move_group', executable='move_group',
             output='screen',
             parameters=[robot_description,
                         robot_description_semantic,
                         robot_description_kinematics,
                         robot_description_planning,
                         planning_pipeline,
                         planning_scene_monitor,
                         {'use_sim_time': use_sim_time}]),

        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', PathJoinSubstitution(
                 [FindPackageShare(PACKAGE), 'rviz', 'baseline.rviz'])],
             parameters=[robot_description,
                         robot_description_semantic,
                         robot_description_kinematics,
                         planning_pipeline,
                         {'use_sim_time': use_sim_time}],
             condition=IfCondition(LaunchConfiguration('use_rviz'))),

        Node(package=PACKAGE, executable='kr6_baseline_test_gui',
             name='kr6_baseline_test_gui', output='both', emulate_tty=True,
             parameters=[{
                 'use_sim_time': use_sim_time,
                 # Rellena el campo JSON del panel REPRODUCIR de la GUI.
                 'preview_json': LaunchConfiguration('input_json'),
             }],
             condition=IfCondition(LaunchConfiguration('use_gui'))),

        # Analizador EN VIVO (modo topico). Es el comportamiento historico y
        # sigue siendo el de por defecto: analyze_json arranca en 'false'.
        Node(package=PACKAGE, executable='kr6_baseline_continuity_analyzer',
             name='kr6_baseline_continuity_analyzer',
             output='both', emulate_tty=True,
             parameters=[{
                 'write_csv': ParameterValue(
                     LaunchConfiguration('write_csv'), value_type=bool),
                 'output_directory': LaunchConfiguration('output_directory'),
                 'use_sim_time': use_sim_time,
             }],
             condition=IfCondition(
                 AndSubstitution(
                     LaunchConfiguration('use_analyzer'),
                     NotSubstitution(LaunchConfiguration('analyze_json'))))),

        # ── J.3 — MODO ARCHIVO: analiza el JSON grabado y termina ────────
        Node(package=PACKAGE, executable='kr6_baseline_continuity_analyzer',
             name='kr6_baseline_json_analyzer',
             output='both', emulate_tty=True,
             parameters=[{
                 'input_json': LaunchConfiguration('input_json'),
                 'write_csv': ParameterValue(
                     LaunchConfiguration('write_csv'), value_type=bool),
                 'output_directory': LaunchConfiguration('output_directory'),
                 'use_sim_time': use_sim_time,
             }],
             condition=IfCondition(LaunchConfiguration('analyze_json'))),

        # ── J.3 — REPLANIFICACION con el baseline (variantes A y B) ──────
        Node(package=PACKAGE, executable='kr6_baseline_replan',
             name='kr6_baseline_replan',
             output='both', emulate_tty=True,
             parameters=[{
                 'input_json': LaunchConfiguration('input_json'),
                 'write_csv': ParameterValue(
                     LaunchConfiguration('write_csv'), value_type=bool),
                 'output_directory': LaunchConfiguration('output_directory'),
                 'run_variant_joint': ParameterValue(
                     LaunchConfiguration('replan_variant_joint'),
                     value_type=bool),
                 'run_variant_cartesian': ParameterValue(
                     LaunchConfiguration('replan_variant_cartesian'),
                     value_type=bool),
                 'use_sim_time': use_sim_time,
             }],
             condition=IfCondition(LaunchConfiguration('replan_json'))),

        # ── Reproduccion en RViz de una secuencia grabada ────────────────
        # Publica en /display_planned_path para que RViz la anime. NO mueve
        # el robot: no hay controlador ni cliente de accion.
        Node(package=PACKAGE, executable='kr6_baseline_json_preview',
             name='kr6_baseline_json_preview',
             output='both', emulate_tty=True,
             parameters=[{
                 'input_json': LaunchConfiguration('input_json'),
                 'time_scale': ParameterValue(
                     LaunchConfiguration('preview_time_scale'),
                     value_type=float),
                 'loop': ParameterValue(
                     LaunchConfiguration('preview_loop'), value_type=bool),
                 'segment': ParameterValue(
                     LaunchConfiguration('preview_segment'), value_type=int),
                 'use_sim_time': use_sim_time,
             }],
             condition=IfCondition(LaunchConfiguration('preview_json'))),
    ]

    return LaunchDescription(args + nodes)
