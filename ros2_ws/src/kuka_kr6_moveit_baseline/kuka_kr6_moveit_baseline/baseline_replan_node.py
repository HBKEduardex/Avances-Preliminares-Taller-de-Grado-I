#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
baseline_replan_node.py

USO 2 del requisito J: REPLANIFICAR con la condicion base la MISMA tarea que
el sistema afinado grabo en un JSON, y comparar las tres condiciones.

═══════════════════════════════════════════════════════════════════════════
  QUE HACE, EXACTAMENTE
═══════════════════════════════════════════════════════════════════════════
  1. Lee el JSON externo (SOLO LECTURA) con trajectory_json_reader.
  2. Analiza la trayectoria GRABADA           -> condicion AFINADA/EXTERNA.
  3. Replanifica los 14 segmentos por separado con METAS ARTICULARES
                                              -> condicion BASELINE-A.
  4. Replanifica los 14 segmentos por separado con METAS CARTESIANAS
     derivadas de los mismos joints por FK    -> condicion BASELINE-B.
  5. Escribe un CSV por condicion y una tabla comparativa de tres filas por
     segmento mas un resumen agregado.

═══════════════════════════════════════════════════════════════════════════
  DECISIONES METODOLOGICAS DEL OPERADOR (J-D1 .. J-D5)
═══════════════════════════════════════════════════════════════════════════
  J-D1  POR SEGMENTO, nunca global. Con P15 ~ P1 un replan global degeneraria
        a "no moverse" y la tarea dejaria de existir. Ademas los eventos de
        gripper en P4/P7/P10/P13 hacen obligatorios los puntos intermedios.

  J-D2  Las METAS son los source_points P1..PN del JSON, NO los extremos
        realmente alcanzados por el sistema afinado. Esos extremos se desvian
        hasta 0.199726 deg de los P, y esa desviacion ES la tolerancia de
        0.2 deg del sistema afinado (joint_goal_tolerance_rad = 0.0034906585).
        Heredarla contaminaria el baseline, que alcanza con 1e-4 rad
        = 0.0057 deg.

  J-D3  ESTADO INICIAL ENCADENADO: T1 arranca en P1; T2..TN arrancan en el
        final REAL del plan del baseline del segmento anterior. Replica el
        segment_chaining = previous_trajectory_end del sistema afinado.

  J-D4  DOS VARIANTES, A y B, porque separan la causa:
          - si A ya muestra recorrido excesivo, el problema es el PLANIFICADOR
          - si solo B lo muestra, es la SELECCION DE CONFIGURACION (IK)

  J-D5  Metadatos de procedencia (md5 del JSON y tip declarado) en el CSV.

═══════════════════════════════════════════════════════════════════════════
  PROHIBICION DURA (#10.4): cartesian_diagnostic NUNCA
═══════════════════════════════════════════════════════════════════════════
  La pose objetivo de la variante B se deriva SIEMPRE de los joints de Pk con
  continuity_metrics.forward_kinematics_tip(q, DECLARED_TIP). JAMAS del campo
  cartesian_diagnostic del JSON, que arrastra un TCP de TG2 de ~149.77 mm
  inexistente en el URDF de taller1. Derivarla por FK elimina ese offset por
  construccion.

═══════════════════════════════════════════════════════════════════════════
  SEGURIDAD
═══════════════════════════════════════════════════════════════════════════
  plan_only = True SIEMPRE, sin parametro que lo desactive. Este nodo NO
  ejecuta, NO controla el gripper y NO tiene ninguna via hacia el robot
  fisico. El unico cliente de accion es /move_action (planificacion).
"""

import csv
import math
import os
import threading
import time
from datetime import datetime
from typing import List, Optional, Sequence, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
    RobotState,
    WorkspaceParameters,
)
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

from .continuity_metrics import (
    DECLARED_TIP,
    JOINT_NAMES,
    Waypoint,
    csv_header,
    csv_rows,
    forward_kinematics_tip,
    reorder_to_canonical,
)
from .sequence_analysis import (
    CONDITION_BASELINE_CART,
    CONDITION_BASELINE_JOINT,
    CONDITION_RECORDED,
    analyze_sequence,
    comparison_table,
)
from .trajectory_json_reader import JsonTrajectoryError
from .trajectory_json_reader import load as load_json_sequence


def _matrix_to_quaternion(R) -> Tuple[float, float, float, float]:
    """Matriz de rotacion 3x3 -> cuaternion (x, y, z, w). Metodo de Shepperd."""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return float(x), float(y), float(z), float(w)


def default_output_directory() -> str:
    """analysis_output/ dentro de este paquete. Nunca se escribe fuera."""
    here = os.path.dirname(os.path.realpath(__file__))
    return os.path.join(os.path.dirname(here), 'analysis_output')


class BaselineReplanNode(Node):
    """Replanifica una tarea grabada con la condicion base. plan_only fijo."""

    def __init__(self):
        super().__init__('kr6_baseline_replan')

        self.declare_parameter('input_json', '')
        self.declare_parameter('planning_group', 'manipulator')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tip_frame', DECLARED_TIP)
        self.declare_parameter('move_action_name', '/move_action')
        self.declare_parameter('output_directory', '')
        self.declare_parameter('write_csv', True)
        # Variantes a ejecutar. Ambas por defecto (J-D4).
        self.declare_parameter('run_variant_joint', True)
        self.declare_parameter('run_variant_cartesian', True)
        # ── DEFECTOS de MoveIt2. Todos NO VERIFICADO (README seccion 2).
        self.declare_parameter('allowed_planning_time', 5.0)
        self.declare_parameter('num_planning_attempts', 1)
        self.declare_parameter('max_velocity_scaling_factor', 1.0)
        self.declare_parameter('max_acceleration_scaling_factor', 1.0)
        self.declare_parameter('goal_joint_tolerance', 1.0e-4)
        self.declare_parameter('goal_position_tolerance', 1.0e-4)
        self.declare_parameter('goal_orientation_tolerance', 1.0e-3)
        self.declare_parameter('plan_timeout_sec', 30.0)

        gp = self.get_parameter
        self._input_json = str(gp('input_json').value or '').strip()
        self.group = gp('planning_group').value
        self.base_frame = gp('base_frame').value
        self.tip_frame = gp('tip_frame').value
        self._action_name = gp('move_action_name').value
        self._write_csv = bool(gp('write_csv').value)
        self._run_joint = bool(gp('run_variant_joint').value)
        self._run_cart = bool(gp('run_variant_cartesian').value)
        self._planning_time = float(gp('allowed_planning_time').value)
        self._attempts = int(gp('num_planning_attempts').value)
        self._vel_scale = float(gp('max_velocity_scaling_factor').value)
        self._acc_scale = float(gp('max_acceleration_scaling_factor').value)
        self._joint_tol = float(gp('goal_joint_tolerance').value)
        self._pos_tol = float(gp('goal_position_tolerance').value)
        self._ori_tol = float(gp('goal_orientation_tolerance').value)
        self._plan_timeout = float(gp('plan_timeout_sec').value)

        self._output_dir = gp('output_directory').value or default_output_directory()
        self._stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._action_client = ActionClient(self, MoveGroup, self._action_name)
        self._done = threading.Event()
        self._failed = False

        if not self._input_json:
            self.get_logger().error(
                'Falta el parametro input_json. Este nodo solo tiene sentido '
                'con un archivo de secuencia. No hace nada.')
            self._failed = True
            self._done.set()
            return

        threading.Thread(target=self._run, daemon=True).start()

    # ── construccion de peticiones ───────────────────────────────────────

    def _base_request(self) -> MotionPlanRequest:
        ws = WorkspaceParameters()
        ws.header.frame_id = self.base_frame
        ws.min_corner.x = ws.min_corner.y = ws.min_corner.z = -2.0
        ws.max_corner.x = ws.max_corner.y = ws.max_corner.z = 2.0

        req = MotionPlanRequest()
        req.group_name = self.group
        req.workspace_parameters = ws
        req.num_planning_attempts = self._attempts
        req.allowed_planning_time = self._planning_time
        req.max_velocity_scaling_factor = self._vel_scale
        req.max_acceleration_scaling_factor = self._acc_scale
        # pipeline_id y planner_id VACIOS: pipeline y planificador por defecto.
        return req

    def _start_state(self, q_rad: Sequence[float]) -> RobotState:
        """J-D3: estado inicial EXPLICITO, no el estado actual del robot."""
        state = RobotState()
        js = JointState()
        js.name = list(JOINT_NAMES)
        js.position = [float(v) for v in q_rad]
        state.joint_state = js
        state.is_diff = False
        return state

    def _joint_goal(self, q_rad: Sequence[float]) -> Constraints:
        constraints = Constraints()
        constraints.name = 'baseline_replan_joint_goal'
        for name, value in zip(JOINT_NAMES, q_rad):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(value)
            jc.tolerance_above = self._joint_tol
            jc.tolerance_below = self._joint_tol
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        return constraints

    def _cartesian_goal(self, q_rad: Sequence[float]) -> Constraints:
        """
        Meta cartesiana DERIVADA DE LOS JOINTS por la FK de este paquete.

        PROHIBICION #10.4: no se lee ninguna pose del JSON. La pose sale de
        forward_kinematics_tip(q, tip_frame), asi que el offset de TCP de TG2
        (~149.77 mm) queda eliminado por construccion.
        """
        T = forward_kinematics_tip(q_rad, self.tip_frame)
        qx, qy, qz, qw = _matrix_to_quaternion(T[:3, :3])

        pose = Pose()
        pose.position.x = float(T[0, 3])
        pose.position.y = float(T[1, 3])
        pose.position.z = float(T[2, 3])
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self._pos_tol]
        volume = BoundingVolume()
        volume.primitives.append(sphere)
        volume.primitive_poses.append(pose)

        pc = PositionConstraint()
        pc.header.frame_id = self.base_frame
        pc.link_name = self.tip_frame
        pc.constraint_region = volume
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = self.base_frame
        oc.link_name = self.tip_frame
        oc.orientation = pose.orientation
        oc.absolute_x_axis_tolerance = self._ori_tol
        oc.absolute_y_axis_tolerance = self._ori_tol
        oc.absolute_z_axis_tolerance = self._ori_tol
        oc.weight = 1.0

        constraints = Constraints()
        constraints.name = 'baseline_replan_cartesian_goal'
        constraints.position_constraints.append(pc)
        constraints.orientation_constraints.append(oc)
        return constraints

    # ── planificacion de un segmento ─────────────────────────────────────

    def _plan_segment(self, start_rad, goal_rad, cartesian: bool
                      ) -> Optional[List[Waypoint]]:
        """Planifica UN segmento. Devuelve los waypoints o None si falla."""
        req = self._base_request()
        req.start_state = self._start_state(start_rad)
        req.goal_constraints = [
            self._cartesian_goal(goal_rad) if cartesian
            else self._joint_goal(goal_rad)]

        options = PlanningOptions()
        options.plan_only = True      # INVARIANTE: el baseline NO ejecuta
        options.replan = False
        options.look_around = False

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = options

        send_future = self._action_client.send_goal_async(goal)
        deadline = time.monotonic() + self._plan_timeout
        while rclpy.ok() and not send_future.done():
            if time.monotonic() > deadline:
                return None
            time.sleep(0.02)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return None

        result_future = handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            if time.monotonic() > deadline:
                return None
            time.sleep(0.05)
        wrapped = result_future.result()
        if wrapped is None:
            return None
        result = wrapped.result
        if result.error_code.val != 1:      # 1 = SUCCESS
            self.get_logger().warn(
                f'  planificacion fallida, error_code = {result.error_code.val}')
            return None

        jt = result.planned_trajectory.joint_trajectory
        if not jt.points:
            return None

        waypoints: List[Waypoint] = []
        for i, point in enumerate(jt.points):
            positions = reorder_to_canonical(jt.joint_names, point.positions)
            if positions is None:
                self.get_logger().warn(
                    'El plan no contiene los seis joints del grupo.')
                return None
            stamp = point.time_from_start
            seconds = float(stamp.sec) + float(stamp.nanosec) * 1e-9
            waypoints.append(Waypoint(
                index=i,
                time_from_start_s=seconds,
                positions_rad=positions,
                # NO se recalculan ni se rellenan con ceros: si MoveIt no las
                # entrega, quedan vacias y salen como nan en el CSV.
                velocities_rad_s=(reorder_to_canonical(
                    jt.joint_names, point.velocities) or []),
                accelerations_rad_s2=(reorder_to_canonical(
                    jt.joint_names, point.accelerations) or []),
            ))
        return waypoints

    def _replan_all(self, sequence, cartesian: bool):
        """
        J-D1 por segmento + J-D3 encadenado.

        El estado inicial de T1 es P1. El de Tk (k>1) es el final REAL del
        plan del baseline de T(k-1), no el source_point: eso replica el
        segment_chaining = previous_trajectory_end del sistema afinado y deja
        que la deriva sea la propia de cada condicion.
        """
        variant = 'B (cartesiana)' if cartesian else 'A (articular)'
        self.get_logger().info(
            f'\n── REPLANIFICANDO variante {variant} — '
            f'{len(sequence.segments)} segmentos ──')
        per_segment: List[List[Waypoint]] = []
        current = list(sequence.source_points_rad[0])      # T1 arranca en P1
        for seg in sequence.segments:
            goal = sequence.source_points_rad[seg.to_point_index]   # J-D2
            waypoints = self._plan_segment(current, goal, cartesian)
            if waypoints is None:
                self.get_logger().error(
                    f'  {seg.id}: NO RESUELTO. Se continua con el siguiente; '
                    'el estado inicial pasa a ser la meta teorica.')
                per_segment.append([])
                current = list(goal)
                continue
            reached = waypoints[-1].positions_rad
            drift = max(abs(math.degrees(a - b))
                        for a, b in zip(reached, goal))
            self.get_logger().info(
                f'  {seg.id}: {len(waypoints):4d} waypoints, '
                f'{waypoints[-1].time_from_start_s:7.3f} s, '
                f'desviacion respecto a la meta {drift:.6f} deg')
            per_segment.append(waypoints)
            current = list(reached)        # J-D3: encadenado real
        return per_segment

    # ── orquestacion ─────────────────────────────────────────────────────

    def _run(self):
        try:
            self._run_inner()
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().error(f'Replanificacion abortada: {exc}')
            self._failed = True
        finally:
            self._done.set()

    def _run_inner(self):
        try:
            sequence = load_json_sequence(self._input_json)
        except (JsonTrajectoryError, OSError, ValueError) as exc:
            self.get_logger().error(
                f'\nMODO REPLANIFICACION ABORTADO\n{exc}\n'
                'No se produce ningun CSV.')
            self._failed = True
            return

        self.get_logger().info(
            '\n'
            '╔══════════════════════════════════════════════════════════════╗\n'
            '║  REPLANIFICACION BASELINE DE UNA TAREA GRABADA               ║\n'
            '╠══════════════════════════════════════════════════════════════╣\n'
            '║  El JSON de entrada es del SISTEMA AFINADO (TG2).            ║\n'
            '║  Solo se toma de el la TAREA (P1..PN). Las trayectorias      ║\n'
            '║  BASELINE se generan aqui desde cero.                        ║\n'
            '╠══════════════════════════════════════════════════════════════╣\n'
            f'║  Archivo : {os.path.basename(sequence.path)[:49]:<49}║\n'
            f'║  md5     : {sequence.md5:<49}║\n'
            f'║  Puntos  : {len(sequence.source_points_rad):<49}║\n'
            f'║  Segment.: {len(sequence.segments):<49}║\n'
            f'║  Tip     : {self.tip_frame:<49}║\n'
            '╚══════════════════════════════════════════════════════════════╝')

        if sequence.limit_violations:
            self.get_logger().error(
                f'{len(sequence.limit_violations)} puntos violan los limites '
                'del URDF baseline. La replanificacion fallara en ellos.')

        metadata = {
            'source_file': sequence.path,
            'source_md5': sequence.md5,
            'declared_tip': self.tip_frame,
        }
        segment_ids = [s.id for s in sequence.segments]

        runs = [analyze_sequence(
            [s.waypoints for s in sequence.segments], segment_ids,
            condition=CONDITION_RECORDED, metadata=metadata)]

        if not self._action_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error(
                f'{self._action_name} no disponible. Se analiza SOLO la '
                'trayectoria grabada; no hay filas de baseline.')
        else:
            if self._run_joint:
                runs.append(analyze_sequence(
                    self._replan_all(sequence, cartesian=False), segment_ids,
                    condition=CONDITION_BASELINE_JOINT, metadata=metadata))
            if self._run_cart:
                runs.append(analyze_sequence(
                    self._replan_all(sequence, cartesian=True), segment_ids,
                    condition=CONDITION_BASELINE_CART, metadata=metadata))

        paths = []
        if self._write_csv:
            os.makedirs(self._output_dir, exist_ok=True)
            for run in runs:
                paths.append(self._write_run(run, sequence))

        # Q.3 — recuento explicito por consola antes de la tabla.
        for run in runs[1:]:
            if run.failed_segment_ids or run.chain_rebuilt_segment_ids:
                self.get_logger().warn(
                    f'\n{run.condition}: '
                    f'{len(run.failed_segment_ids)} segmento(s) NO RESUELTO(S) '
                    f'{run.failed_segment_ids} y '
                    f'{len(run.chain_rebuilt_segment_ids)} con CADENA '
                    f'RECONSTRUIDA {run.chain_rebuilt_segment_ids}. '
                    'A partir del primer fallo el encadenado ya NO es '
                    'equivalente al de la condicion afinada (J-D3).')
            else:
                self.get_logger().info(
                    f'{run.condition}: cadena INTACTA, '
                    f'{len(run.segments)} segmentos.')

        table = comparison_table(runs, segment_ids)
        self.get_logger().info(f'\n{table}')
        if self._write_csv:
            report = os.path.join(
                self._output_dir, f'comparacion_{self._stamp}.txt')
            try:
                with open(report, 'w', encoding='utf-8') as handle:
                    handle.write(table + '\n')
                paths.append(report)
            except OSError as exc:
                self.get_logger().error(f'No se pudo escribir el informe: {exc}')
        for path in paths:
            if path:
                self.get_logger().info(f'  escrito: {path}')

    _SLUG = {
        CONDITION_RECORDED: 'afinada_externa',
        CONDITION_BASELINE_JOINT: 'baseline_A_articular',
        CONDITION_BASELINE_CART: 'baseline_B_cartesiana',
    }

    def _write_run(self, run, sequence) -> str:
        stem = os.path.splitext(os.path.basename(sequence.path))[0]
        slug = self._SLUG.get(run.condition, 'condicion')
        name = f'{slug}_continuity_{self._stamp}_{stem}.csv'
        path = os.path.join(self._output_dir, name)
        try:
            with open(path, 'w', newline='') as handle:
                writer = csv.writer(handle)
                writer.writerow(csv_header())
                for item in run.segments:
                    writer.writerows(csv_rows(item))
        except OSError as exc:
            self.get_logger().error(f'No se pudo escribir el CSV: {exc}')
            return ''
        return path

    def wait(self) -> bool:
        while rclpy.ok() and not self._done.wait(timeout=0.2):
            pass
        return not self._failed


def main(args=None):
    rclpy.init(args=args)
    node = BaselineReplanNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    ok = True
    try:
        ok = node.wait()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    main()
