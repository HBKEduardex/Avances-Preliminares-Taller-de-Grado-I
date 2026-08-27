#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
replan_core.py

Nucleo de replanificacion compartido: construye las peticiones a /move_action
y encadena los segmentos. Lo usan el nodo `kr6_baseline_replan` y el boton
PLANIFICAR Y REPRODUCIR de la GUI, para que no diverjan.

═══════════════════════════════════════════════════════════════════════════
  QUE SE TOMA DEL JSON Y QUE NO
═══════════════════════════════════════════════════════════════════════════
  SE TOMA   : unicamente los source_points P1..PN, es decir las METAS de la
              tarea. Son los puntos que el operador enseño.

  NO SE TOMA: los trajectory_points intermedios. Esos los genero el SISTEMA
              AFINADO y reproducirlos no demuestra nada sobre la condicion
              base. El baseline debe planificar el camino DESDE CERO.

  NO SE TOMA: cartesian_diagnostic (prohibicion #10.4). Cuando hace falta una
              pose se DERIVA de los joints con forward_kinematics_tip().
═══════════════════════════════════════════════════════════════════════════

SEGURIDAD: plan_only = True siempre. Sin cliente de ejecucion, sin gripper.
"""

import math
import time
from typing import List, Optional, Sequence, Tuple

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

import rclpy

from .continuity_metrics import (
    JOINT_NAMES,
    Waypoint,
    forward_kinematics_tip,
    reorder_to_canonical,
)


class PlanParams:
    """Defectos de MoveIt2. Todos NO VERIFICADO, ver README seccion 2."""

    def __init__(self, group='manipulator', base_frame='base_link',
                 tip_frame='tool0', planning_time=5.0, attempts=1,
                 vel_scale=1.0, acc_scale=1.0, joint_tol=1.0e-4,
                 pos_tol=1.0e-4, ori_tol=1.0e-3, timeout=30.0):
        self.group = group
        self.base_frame = base_frame
        self.tip_frame = tip_frame
        self.planning_time = planning_time
        self.attempts = attempts
        self.vel_scale = vel_scale
        self.acc_scale = acc_scale
        self.joint_tol = joint_tol
        self.pos_tol = pos_tol
        self.ori_tol = ori_tol
        self.timeout = timeout


def matrix_to_quaternion(R) -> Tuple[float, float, float, float]:
    """Matriz de rotacion 3x3 -> cuaternion (x, y, z, w). Metodo de Shepperd."""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w, x = 0.25 * s, (R[2, 1] - R[1, 2]) / s
        y, z = (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w, x = (R[2, 1] - R[1, 2]) / s, 0.25 * s
        y, z = (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w, x = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s
        y, z = 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w, x = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s
        y, z = (R[1, 2] + R[2, 1]) / s, 0.25 * s
    return float(x), float(y), float(z), float(w)


def base_request(p: PlanParams) -> MotionPlanRequest:
    ws = WorkspaceParameters()
    ws.header.frame_id = p.base_frame
    ws.min_corner.x = ws.min_corner.y = ws.min_corner.z = -2.0
    ws.max_corner.x = ws.max_corner.y = ws.max_corner.z = 2.0

    req = MotionPlanRequest()
    req.group_name = p.group
    req.workspace_parameters = ws
    req.num_planning_attempts = p.attempts
    req.allowed_planning_time = p.planning_time
    req.max_velocity_scaling_factor = p.vel_scale
    req.max_acceleration_scaling_factor = p.acc_scale
    # pipeline_id y planner_id VACIOS: pipeline y planificador por defecto.
    return req


def start_state(q_rad: Sequence[float]) -> RobotState:
    """J-D3: estado inicial EXPLICITO, no el estado actual del robot."""
    state = RobotState()
    js = JointState()
    js.name = list(JOINT_NAMES)
    js.position = [float(v) for v in q_rad]
    state.joint_state = js
    state.is_diff = False
    return state


def joint_goal(q_rad: Sequence[float], p: PlanParams) -> Constraints:
    c = Constraints()
    c.name = 'baseline_joint_goal'
    for name, value in zip(JOINT_NAMES, q_rad):
        jc = JointConstraint()
        jc.joint_name = name
        jc.position = float(value)
        jc.tolerance_above = p.joint_tol
        jc.tolerance_below = p.joint_tol
        jc.weight = 1.0
        c.joint_constraints.append(jc)
    return c


def cartesian_goal(q_rad: Sequence[float], p: PlanParams) -> Constraints:
    """
    Meta cartesiana DERIVADA DE LOS JOINTS por la FK de este paquete.

    PROHIBICION #10.4: nunca se lee una pose del JSON.
    """
    T = forward_kinematics_tip(q_rad, p.tip_frame)
    qx, qy, qz, qw = matrix_to_quaternion(T[:3, :3])

    pose = Pose()
    pose.position.x = float(T[0, 3])
    pose.position.y = float(T[1, 3])
    pose.position.z = float(T[2, 3])
    pose.orientation.x, pose.orientation.y = qx, qy
    pose.orientation.z, pose.orientation.w = qz, qw

    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [p.pos_tol]
    volume = BoundingVolume()
    volume.primitives.append(sphere)
    volume.primitive_poses.append(pose)

    pc = PositionConstraint()
    pc.header.frame_id = p.base_frame
    pc.link_name = p.tip_frame
    pc.constraint_region = volume
    pc.weight = 1.0

    oc = OrientationConstraint()
    oc.header.frame_id = p.base_frame
    oc.link_name = p.tip_frame
    oc.orientation = pose.orientation
    oc.absolute_x_axis_tolerance = p.ori_tol
    oc.absolute_y_axis_tolerance = p.ori_tol
    oc.absolute_z_axis_tolerance = p.ori_tol
    oc.weight = 1.0

    c = Constraints()
    c.name = 'baseline_cartesian_goal'
    c.position_constraints.append(pc)
    c.orientation_constraints.append(oc)
    return c


def plan_segment(action_client, start_rad: Sequence[float],
                 goal_rad: Sequence[float], cartesian: bool,
                 p: PlanParams, logger=None) -> Optional[List[Waypoint]]:
    """
    Planifica UN segmento contra /move_action. Devuelve waypoints o None.

    plan_only = True SIEMPRE. Este camino no puede ejecutar nada.
    """
    req = base_request(p)
    req.start_state = start_state(start_rad)
    req.goal_constraints = [
        cartesian_goal(goal_rad, p) if cartesian else joint_goal(goal_rad, p)]

    options = PlanningOptions()
    options.plan_only = True      # INVARIANTE: el baseline NO ejecuta
    options.replan = False
    options.look_around = False

    goal = MoveGroup.Goal()
    goal.request = req
    goal.planning_options = options

    send_future = action_client.send_goal_async(goal)
    deadline = time.monotonic() + p.timeout
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
        if logger is not None:
            logger(f'error_code = {result.error_code.val}')
        return None

    jt = result.planned_trajectory.joint_trajectory
    if not jt.points:
        return None

    waypoints: List[Waypoint] = []
    for i, point in enumerate(jt.points):
        positions = reorder_to_canonical(jt.joint_names, point.positions)
        if positions is None:
            return None
        stamp = point.time_from_start
        waypoints.append(Waypoint(
            index=i,
            time_from_start_s=float(stamp.sec) + float(stamp.nanosec) * 1e-9,
            positions_rad=positions,
            # NO se recalculan ni se rellenan con ceros.
            velocities_rad_s=(reorder_to_canonical(
                jt.joint_names, point.velocities) or []),
            accelerations_rad_s2=(reorder_to_canonical(
                jt.joint_names, point.accelerations) or []),
        ))
    return waypoints


def plan_sequence(action_client, source_points_rad, segment_ids,
                  cartesian: bool, p: PlanParams,
                  progress=None, should_stop=None):
    """
    Replanifica la tarea COMPLETA con el baseline, segmento a segmento.

    METAS  : los source_points P1..PN del JSON (J-D2). NUNCA los waypoints
             intermedios del sistema afinado.
    CADENA : T1 arranca en P1; Tk arranca en el final REAL del plan de T(k-1)
             (J-D3), replicando previous_trajectory_end.

    Devuelve (grupos_de_waypoints, fallidos, reconstruidos).
    `grupos[k]` es [] si el segmento k no se resolvio.
    """
    groups: List[List[Waypoint]] = []
    failed: List[str] = []
    rebuilt: List[str] = []
    broken = False
    current = list(source_points_rad[0])          # T1 arranca en P1

    for k, sid in enumerate(segment_ids):
        if should_stop is not None and should_stop():
            break
        goal = list(source_points_rad[k + 1])     # J-D2
        if progress is not None:
            progress(k, len(segment_ids), sid, None)
        wps = plan_segment(action_client, current, goal, cartesian, p)
        if wps is None:
            failed.append(sid)
            groups.append([])
            broken = True
            # La cadena queda RECONSTRUIDA: el siguiente arranca en la meta
            # teorica, un estado en el que el baseline nunca estuvo.
            current = list(goal)
            if progress is not None:
                progress(k, len(segment_ids), sid, 'NO RESUELTO')
            continue
        if broken:
            rebuilt.append(sid)
        groups.append(wps)
        drift = max(abs(math.degrees(a - b))
                    for a, b in zip(wps[-1].positions_rad, goal))
        if progress is not None:
            progress(k, len(segment_ids), sid,
                     f'{len(wps)} wp, {wps[-1].time_from_start_s:.2f} s, '
                     f'desviacion {drift:.5f} deg')
        current = list(wps[-1].positions_rad)     # J-D3
    return groups, failed, rebuilt
