#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pick_place_sequence_node.py

Nodo ROS2 que lee una secuencia de puntos articulares desde un archivo
YAML y envía cada uno como objetivo al action server de MoveIt2
(move_action) para planificar y opcionalmente ejecutar la trayectoria.

Modo de operación:
  - execute=false (por defecto): solo planifica y reporta si la
    planificación fue exitosa para cada punto. No ejecuta.
  - execute=true: planifica y ejecuta cada punto de la secuencia.
    Requiere un controlador de trayectoria activo (simulado o real).

Uso:
  ros2 launch kuka_pick_place_demo pick_place_sequence.launch.py execute:=false
  ros2 launch kuka_pick_place_demo pick_place_sequence.launch.py execute:=true
"""

import os
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
import threading

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    PlanningOptions,
    RobotState,
    WorkspaceParameters,
)
from sensor_msgs.msg import JointState

import yaml


class PickPlaceSequenceNode(Node):
    """Nodo que ejecuta una secuencia de pick and place con MoveIt2."""

    def __init__(self):
        super().__init__('pick_place_sequence_node')

        # ── Parámetros ──────────────────────────────────────────
        self.declare_parameter('planning_group', 'manipulator')
        self.declare_parameter('execute', False)
        self.declare_parameter('velocity_scaling', 0.10)
        self.declare_parameter('acceleration_scaling', 0.10)
        self.declare_parameter(
            'points_file', 'config/pick_place_points.yaml')
        self.declare_parameter('return_home', True)
        self.declare_parameter('wait_between_points', 1.0)
        self.declare_parameter('stop_on_failure', True)
        self.declare_parameter('reference_frame', 'base_link')
        self.declare_parameter('move_group_action_name', '/move_action')

        self._planning_group = self.get_parameter('planning_group') \
            .get_parameter_value().string_value
        self._execute = self.get_parameter('execute') \
            .get_parameter_value().bool_value
        self._velocity_scaling = self.get_parameter('velocity_scaling') \
            .get_parameter_value().double_value
        self._acceleration_scaling = self.get_parameter(
            'acceleration_scaling').get_parameter_value().double_value
        self._points_file = self.get_parameter('points_file') \
            .get_parameter_value().string_value
        self._return_home = self.get_parameter('return_home') \
            .get_parameter_value().bool_value
        self._wait_between = self.get_parameter(
            'wait_between_points').get_parameter_value().double_value
        self._stop_on_failure = self.get_parameter('stop_on_failure') \
            .get_parameter_value().bool_value
        self._reference_frame = self.get_parameter('reference_frame') \
            .get_parameter_value().string_value
        self._action_name = self.get_parameter(
            'move_group_action_name').get_parameter_value().string_value

        # Resolver ruta del archivo de puntos
        if not os.path.isabs(self._points_file):
            try:
                from ament_index_python.packages import \
                    get_package_share_directory
                pkg_share = get_package_share_directory(
                    'kuka_pick_place_demo')
                self._points_file = os.path.join(
                    pkg_share, self._points_file)
            except Exception:
                self._points_file = os.path.abspath(self._points_file)

        # ── Modo de operación ───────────────────────────────────
        mode_str = 'PLANIFICAR + EJECUTAR' if self._execute \
            else 'SOLO PLANIFICAR (validación)'
        self.get_logger().info(f'Modo: {mode_str}')
        self.get_logger().info(f'Archivo de puntos: {self._points_file}')
        self.get_logger().info(
            f'Planning group: {self._planning_group}')
        self.get_logger().info(
            f'Velocity scaling: {self._velocity_scaling}')
        self.get_logger().info(
            f'Acceleration scaling: {self._acceleration_scaling}')

        # ── Action client para MoveGroup ────────────────────────
        self.get_logger().info(
            f'Action server configurado: {self._action_name}')
        self._action_client = ActionClient(
            self, MoveGroup, self._action_name)

    def run_sequence(self):
        """Iniciar la secuencia de puntos de forma síncrona."""

        # Cargar puntos
        points = self._load_points()
        if not points:
            self.get_logger().error(
                'No se encontraron puntos en el archivo YAML. '
                'Usa point_recorder_node para grabar puntos primero.')
            rclpy.shutdown()
            return

        self.get_logger().info(
            f'Secuencia cargada: {len(points)} puntos')
        for i, pt in enumerate(points):
            self.get_logger().info(
                f'  [{i+1}] {pt["name"]}')

        # Esperar al action server
        self.get_logger().info(
            f'Esperando al action server {self._action_name} ...')
        if not self._action_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error(
                f'No se pudo conectar al action server '
                f'{self._action_name}. '
                f'Asegúrate de que MoveIt2 está corriendo.')
            rclpy.shutdown()
            return
        self.get_logger().info(
            f'Conectado a {self._action_name}.')

        # Ejecutar secuencia
        time.sleep(1.0)
        success_count = 0
        fail_count = 0

        for i, point in enumerate(points):
            name = point['name']
            joints = point['joints']
            self.get_logger().info(
                f'\n{"="*50}\n'
                f'[{i+1}/{len(points)}] Procesando punto: {name}\n'
                f'{"="*50}')

            success = self._send_goal(name, joints)

            if success:
                success_count += 1
                self.get_logger().info(
                    f'✓ Punto "{name}" completado exitosamente.')
            else:
                fail_count += 1
                self.get_logger().error(
                    f'✗ Punto "{name}" falló.')
                if self._stop_on_failure:
                    self.get_logger().error(
                        'stop_on_failure=true, deteniendo secuencia.')
                    break

            # Esperar entre puntos
            if i < len(points) - 1 and self._wait_between > 0:
                self.get_logger().info(
                    f'Esperando {self._wait_between}s antes del '
                    f'siguiente punto...')
                time.sleep(self._wait_between)

        # Resumen
        self.get_logger().info(
            f'\n{"="*50}\n'
            f'RESUMEN DE SECUENCIA\n'
            f'  Total puntos: {len(points)}\n'
            f'  Exitosos:     {success_count}\n'
            f'  Fallidos:     {fail_count}\n'
            f'  Modo:         '
            f'{"EJECUTAR" if self._execute else "SOLO PLAN"}\n'
            f'{"="*50}')

        self.get_logger().info('Secuencia finalizada.')
        rclpy.shutdown()

    def _send_goal(self, name, joints):
        """Enviar un objetivo articular a MoveGroup."""
        # Construir constraints
        joint_constraints = []
        for jname, jvalue in joints.items():
            jc = JointConstraint()
            jc.joint_name = jname
            jc.position = float(jvalue)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            joint_constraints.append(jc)

        constraints = Constraints()
        constraints.name = name
        constraints.joint_constraints = joint_constraints

        # Construir MotionPlanRequest
        request = MotionPlanRequest()
        request.group_name = self._planning_group
        request.num_planning_attempts = 10
        request.allowed_planning_time = 10.0
        request.max_velocity_scaling_factor = self._velocity_scaling
        request.max_acceleration_scaling_factor = \
            self._acceleration_scaling
        request.goal_constraints = [constraints]

        # Workspace
        ws = WorkspaceParameters()
        ws.header.frame_id = self._reference_frame
        ws.min_corner.x = -2.0
        ws.min_corner.y = -2.0
        ws.min_corner.z = -2.0
        ws.max_corner.x = 2.0
        ws.max_corner.y = 2.0
        ws.max_corner.z = 2.0
        request.workspace_parameters = ws

        # Construir MoveGroup.Goal
        goal = MoveGroup.Goal()
        goal.request = request

        # Planning options
        planning_options = PlanningOptions()
        planning_options.plan_only = not self._execute
        planning_options.replan = False
        planning_options.look_around = False
        goal.planning_options = planning_options

        self.get_logger().info(
            f'Enviando objetivo "{name}" a {self._action_name} '
            f'(plan_only={planning_options.plan_only})...')

        # Enviar goal de forma asíncrona pero esperar activamente
        send_goal_future = self._action_client.send_goal_async(goal)
        while rclpy.ok() and not send_goal_future.done():
            time.sleep(0.01)

        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(
                f'Goal "{name}" rechazado por el action server.')
            return False

        self.get_logger().info(f'Goal "{name}" aceptado, esperando resultado...')

        # Esperar resultado
        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            time.sleep(0.01)

        result = result_future.result()
        if result is None:
            self.get_logger().error(
                f'No se recibió resultado para "{name}".')
            return False

        move_result = result.result
        error_code = move_result.error_code.val

        # MoveItErrorCodes: SUCCESS = 1
        if error_code == 1:
            if self._execute:
                self.get_logger().info(
                    f'Plan + Ejecución exitosa para "{name}".')
            else:
                traj = move_result.planned_trajectory
                if traj.joint_trajectory.points:
                    n_points = len(traj.joint_trajectory.points)
                    duration = traj.joint_trajectory.points[-1] \
                        .time_from_start
                    self.get_logger().info(
                        f'Plan exitoso para "{name}": '
                        f'{n_points} puntos de trayectoria, '
                        f'duración estimada: {duration.sec}.'
                        f'{duration.nanosec // 1000000:03d}s')
                else:
                    self.get_logger().info(
                        f'Plan exitoso para "{name}" '
                        f'(sin detalles de trayectoria).')
            return True
        else:
            self.get_logger().error(
                f'MoveIt error_code={error_code} para "{name}".')
            return False

    def _load_points(self):
        """Cargar puntos desde el archivo YAML."""
        if not os.path.isfile(self._points_file):
            self.get_logger().error(
                f'Archivo no encontrado: {self._points_file}')
            return []

        try:
            with open(self._points_file, 'r') as f:
                data = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f'Error leyendo YAML: {e}')
            return []

        if data is None or 'sequence' not in data:
            return []

        sequence = data['sequence']
        if not isinstance(sequence, list):
            return []

        # Validar cada punto
        valid_points = []
        for pt in sequence:
            if not isinstance(pt, dict):
                continue
            if 'name' not in pt or 'joints' not in pt:
                self.get_logger().warn(
                    f'Punto inválido (falta name/joints): {pt}')
                continue
            if not isinstance(pt['joints'], dict):
                self.get_logger().warn(
                    f'Punto "{pt["name"]}" tiene joints inválidos.')
                continue
            valid_points.append(pt)

        return valid_points


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceSequenceNode()

    # Ejecutar el spin de ROS en un hilo separado para no bloquear la secuencia
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run_sequence()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Excepción en la secuencia: {e}')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        spin_thread.join()


if __name__ == '__main__':
    main()
