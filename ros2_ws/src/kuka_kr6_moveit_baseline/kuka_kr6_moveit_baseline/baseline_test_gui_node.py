#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
baseline_test_gui_node.py

GUI de pruebas de la condicion MOVEIT2 BASE.

FORK de kuka_gui_moveit_bridge/kuka_bridge_test_gui_node.py (commit 0c2c022).
El original NO se ha tocado.

QUE SE CONSERVA
  - envio de objetivos articulares A1..A6 en GRADOS
  - envio de objetivos cartesianos X,Y,Z + A,B,C (convencion KUKA) en mm/grados
  - visualizacion del estado articular y cartesiano
  - log con marca de tiempo

QUE SE ELIMINA Y POR QUE
  - Toda la ruta por kuka_moveit_bridge_node y sus topicos /kuka_bridge/*.
    Motivo: ese nodo NO es generico. Lleva parametros de ingenieria del
    operador (velocity_scaling 0.1, acceleration_scaling 0.1, planning_time
    10.0, planning_attempts 10, tolerancias ALREADY_AT_TARGET, joint_tolerance
    0.2 deg) y ademas su parametro plan_only viene en false, es decir EJECUTA.
    Usarlo importaria la condicion afinada dentro del baseline y romperia las
    propiedades (b) NO AFINADA y (c) CONGELADA del encargo.
    Esta GUI habla DIRECTAMENTE con /move_action.
  - El panel APLICAR PLAN_ONLY y el servicio SetParameters contra el bridge.
    Aqui plan_only es SIEMPRE True y no es configurable.
  - El boton READY (estado definido por el operador, no del Setup Assistant).
  - La resolucion previa de IK con semilla del estado actual mediante
    /compute_ik. Una semilla es un ajuste explicitamente listado en la
    seccion 5 del encargo. El objetivo cartesiano se envia como restricciones
    de posicion y orientacion en el MotionPlanRequest, que es la via por
    defecto de MoveIt2.

TIP DECLARADO
  - tool0, coherente con el <chain tip_link="tool0"> del SRDF del baseline.
  - link_name de PositionConstraint y OrientationConstraint = self.tip_frame.
  - tool0 comparte ORIGEN exacto con link_6 y flange (ambas transformadas
    fijas intermedias tienen traslacion nula), asi que la POSICION X/Y/Z es la
    misma en los tres frames. Lo que cambia es la ORIENTACION de referencia:
    tool0 = link_6 girado +90 deg en Y. En el HOME baseline eso significa
    A/B/C = [0, 90, 0] en vez de [0, 0, 0].

SEGURIDAD
  - plan_only = True SIEMPRE. Esta GUI no puede ejecutar nada.
  - No existe ningun socket, ni cliente EKI, ni topico del bridge TCP/IP.
  - El unico cliente de accion es /move_action (planificacion).
"""

import math
import os
import queue
import threading
import tkinter as tk
from datetime import datetime
from tkinter import scrolledtext

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

import tf2_ros
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
    WorkspaceParameters,
)
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

from moveit_msgs.msg import DisplayTrajectory

from .continuity_metrics import AXES, JOINT_NAMES, RAD_TO_DEG
from .json_preview_node import (
    build_display_from_groups,
    build_display_trajectory,
    preview_qos,
)
from .replan_core import PlanParams, plan_sequence
from .trajectory_json_reader import JsonTrajectoryError
from .trajectory_json_reader import load as load_json_sequence
from .trajectory_json_writer import (
    JsonTrajectoryWriteError,
    build_baseline_document,
    write_baseline_sequence,
)
from .baseline_preflight import format_report, preflight_document
from .kuka_pipeline_limits import (
    EXPERIMENTAL_PTP_VELOCITY_PCT,
    contract_velocity_scaling,
)

# ─────────────────────────────────────────────────────────────────────────────
# Constantes
# ─────────────────────────────────────────────────────────────────────────────

DEG_TO_RAD = math.pi / 180.0

#: Limites del URDF copiado (urdf/kr6r900sixx_macro.xacro). NO son un ajuste:
#: son exactamente los del modelo. Solo se usan para avisar en la GUI antes de
#: enviar; MoveIt2 aplica los suyos igualmente.
JOINT_LIMITS_DEG = {
    'joint_a1': (-170.0, 170.0),
    'joint_a2': (-190.0, 45.0),
    'joint_a3': (-120.0, 156.0),
    'joint_a4': (-185.0, 185.0),
    'joint_a5': (-120.0, 120.0),
    'joint_a6': (-350.0, 350.0),
}

#: HOME ORIGINAL del robot (A5 = 0). Es una configuracion SINGULAR de muñeca:
#: los ejes de A4 y A6 quedan alineados. Se conserva a proposito.
HOME_JOINT_DEG = [0.0, -90.0, 90.0, 0.0, 0.0, 0.0]

MOVEIT_ERRORS = {
    1: 'SUCCESS', 99999: 'FAILURE', -1: 'PLANNING_FAILED',
    -2: 'INVALID_MOTION_PLAN', -3: 'MOTION_PLAN_INVALIDATED_BY_ENV_CHANGE',
    -4: 'CONTROL_FAILED', -6: 'TIMED_OUT', -7: 'PREEMPTED',
    -10: 'START_STATE_IN_COLLISION', -11: 'START_STATE_VIOLATES_PATH_CONSTRAINTS',
    -12: 'GOAL_IN_COLLISION', -13: 'GOAL_VIOLATES_PATH_CONSTRAINTS',
    -14: 'GOAL_CONSTRAINTS_VIOLATED', -15: 'INVALID_GROUP_NAME',
    -16: 'INVALID_GOAL_CONSTRAINTS', -17: 'INVALID_ROBOT_STATE',
    -18: 'INVALID_LINK_NAME', -21: 'FRAME_TRANSFORM_FAILURE',
    -31: 'NO_IK_SOLUTION',
}


def kuka_abc_deg_to_quaternion(a_deg, b_deg, c_deg):
    """
    KUKA ABC en grados -> cuaternion (x, y, z, w).

    Convencion KUKA: A = yaw (Z), B = pitch (Y), C = roll (X),
    R = Rz(A) * Ry(B) * Rx(C). Misma convencion que usa el resto del
    proyecto, para que los objetivos sean comparables entre condiciones.
    """
    a, b, c = (a_deg * DEG_TO_RAD, b_deg * DEG_TO_RAD, c_deg * DEG_TO_RAD)
    cy, sy = math.cos(a / 2.0), math.sin(a / 2.0)
    cp, sp = math.cos(b / 2.0), math.sin(b / 2.0)
    cr, sr = math.cos(c / 2.0), math.sin(c / 2.0)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    return (x / n, y / n, z / n, w / n)


def quaternion_to_kuka_abc_deg(qx, qy, qz, qw):
    """Cuaternion -> KUKA ABC en grados."""
    sinr = 2.0 * (qw * qx + qy * qz)
    cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
    c = math.atan2(sinr, cosr)
    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))
    b = math.asin(sinp)
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    a = math.atan2(siny, cosy)
    return (a * RAD_TO_DEG, b * RAD_TO_DEG, c * RAD_TO_DEG)


# ─────────────────────────────────────────────────────────────────────────────
# Nodo ROS 2
# ─────────────────────────────────────────────────────────────────────────────

class BaselineTestGuiNode(Node):
    """Nodo ROS de la GUI. Solo planifica contra /move_action."""

    def __init__(self):
        super().__init__('kr6_baseline_test_gui')

        self.declare_parameter('planning_group', 'manipulator')
        self.declare_parameter('base_frame', 'base_link')
        # TIP = tool0. Debe coincidir con el tip_link del <chain> del SRDF
        # (config/kuka_kr6_baseline.srdf). tool0 es el tip que produce un
        # Setup Assistant real con este URDF (REP-199). Comparte origen con
        # link_6 y flange; solo cambia la ORIENTACION de referencia: +90 deg
        # en Y. Ver README seccion 10.0.
        self.declare_parameter('tip_frame', 'tool0')
        self.declare_parameter('move_action_name', '/move_action')
        # ── Reproduccion de una secuencia grabada (boton REPRODUCIR).
        #    Publicar en un topico de VISUALIZACION no rompe la invariante:
        #    /display_planned_path no ejecuta nada. Sigue sin haber cliente de
        #    accion que no sea /move_action, ni ningun topico de ejecucion.
        self.declare_parameter('preview_json', '')
        self.declare_parameter('preview_topic', '/display_planned_path')
        # ── Guardado del plan del BASELINE (boton GUARDAR ULTIMO PLAN).
        #    Vacio = se deriva del JSON de entrada:
        #    <padre-del-json>/trajectories_baseline. Nunca se escribe
        #    junto al JSON de origen (prohibicion J.0).
        self.declare_parameter('baseline_output_dir', '')
        # ── CONDICIONES DE LA DEMOSTRACION ──────────────────────────
        # Escalado de velocidad del panel DEMOSTRACION. NO cambia la
        # geometria del camino: TOTG lo aplica DESPUES de que OMPL haya
        # decidido el camino, asi que solo altera la parametrizacion temporal
        # y, con ella, la densidad de muestreo. El defecto es el escalado MAS
        # ALTO que sigue cumpliendo el contrato de 10 deg: el ajuste MINIMO
        # necesario. Apretarlo mas acercaria el baseline a la densidad del
        # sistema afinado (0.1) sin que el contrato lo exija.
        # Ver kuka_pipeline_limits.DELTA_LAW.
        self.declare_parameter(
            'demo_velocity_scaling', round(contract_velocity_scaling(), 4))
        # Informar a MoveIt de los soft limits REALES durante la planificacion.
        self.declare_parameter('enforce_pipeline_limits', True)
        # Guardado automatico al terminar de planificar (variante cartesiana).
        self.declare_parameter('auto_save_plan', True)
        # Condicion experimental de velocidad fisica PTP.
        self.declare_parameter(
            'kuka_ptp_velocity_pct', EXPERIMENTAL_PTP_VELOCITY_PCT)
        # ── Parametros de planificacion: DEFECTOS de MoveIt2 ────────────
        # Todos NO VERIFICADO (no hay plantilla local). Ver README seccion 2.
        self.declare_parameter('allowed_planning_time', 5.0)
        self.declare_parameter('num_planning_attempts', 1)
        self.declare_parameter('max_velocity_scaling_factor', 1.0)
        self.declare_parameter('max_acceleration_scaling_factor', 1.0)
        self.declare_parameter('goal_joint_tolerance', 1.0e-4)
        self.declare_parameter('goal_position_tolerance', 1.0e-4)
        self.declare_parameter('goal_orientation_tolerance', 1.0e-3)

        gp = self.get_parameter
        self.group = gp('planning_group').value
        self.base_frame = gp('base_frame').value
        self.tip_frame = gp('tip_frame').value
        self.preview_json = str(gp('preview_json').value or '').strip()
        self._preview_topic = str(gp('preview_topic').value)
        self.baseline_output_dir = str(
            gp('baseline_output_dir').value or '').strip()
        self._demo_vel_scaling = float(gp('demo_velocity_scaling').value)
        self._enforce_limits = bool(gp('enforce_pipeline_limits').value)
        self.auto_save_plan = bool(gp('auto_save_plan').value)
        self.ptp_velocity_pct = float(gp('kuka_ptp_velocity_pct').value)
        self._action_name = gp('move_action_name').value
        self._planning_time = float(gp('allowed_planning_time').value)
        self._attempts = int(gp('num_planning_attempts').value)
        self._vel_scale = float(gp('max_velocity_scaling_factor').value)
        self._acc_scale = float(gp('max_acceleration_scaling_factor').value)
        self._joint_tol = float(gp('goal_joint_tolerance').value)
        self._pos_tol = float(gp('goal_position_tolerance').value)
        self._ori_tol = float(gp('goal_orientation_tolerance').value)

        self.joint_queue = queue.Queue()
        self.cart_queue = queue.Queue()
        self.status_queue = queue.Queue()

        self._busy = False
        self._busy_lock = threading.Lock()

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 10)
        self.create_timer(0.2, self._publish_cartesian_state)
        self._preview_publisher = self.create_publisher(
            DisplayTrajectory, self._preview_topic, preview_qos())
        self._preview_cache = {}
        # Ultimo plan del baseline, EN MEMORIA, esperando a guardarse.
        # No se escribe nada en disco hasta que el operador lo pide.
        self._last_plan = None

        # UNICO cliente de accion del nodo: planificacion.
        self._action_client = ActionClient(self, MoveGroup, self._action_name)

        self.status_queue.put('ESPERANDO /joint_states')

    # ── Entrada ──────────────────────────────────────────────────────

    def _on_joint_state(self, msg: JointState):
        by_name = dict(zip(msg.name, msg.position))
        if not all(n in by_name for n in JOINT_NAMES):
            return
        self.joint_queue.put([by_name[n] * RAD_TO_DEG for n in JOINT_NAMES])

    def _publish_cartesian_state(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                self.base_frame, self.tip_frame, rclpy.time.Time())
        except tf2_ros.TransformException:
            return
        t, r = tf.transform.translation, tf.transform.rotation
        a, b, c = quaternion_to_kuka_abc_deg(r.x, r.y, r.z, r.w)
        self.cart_queue.put([t.x * 1000.0, t.y * 1000.0, t.z * 1000.0,
                             a, b, c])

    # ── Construccion del objetivo ────────────────────────────────────

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
        # pipeline_id y planner_id se dejan VACIOS a proposito: asi
        # move_group aplica el pipeline y el planificador por defecto.
        return req

    def send_joint_goal(self, values_deg):
        """Objetivo articular. Devuelve (aceptado, mensaje)."""
        constraints = Constraints()
        constraints.name = 'baseline_joint_goal'
        for name, value in zip(JOINT_NAMES, values_deg):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(value) * DEG_TO_RAD
            jc.tolerance_above = self._joint_tol
            jc.tolerance_below = self._joint_tol
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        req = self._base_request()
        req.goal_constraints = [constraints]
        return self._send(req, 'articular')

    def send_cartesian_goal(self, x_mm, y_mm, z_mm, a_deg, b_deg, c_deg):
        """
        Objetivo cartesiano como restricciones de posicion y orientacion.

        Es la via por defecto de MoveIt2: NO se resuelve IK previamente ni se
        usa ninguna semilla (una semilla seria un ajuste, ver seccion 5).
        """
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = float(x_mm) / 1000.0
        pose.pose.position.y = float(y_mm) / 1000.0
        pose.pose.position.z = float(z_mm) / 1000.0
        qx, qy, qz, qw = kuka_abc_deg_to_quaternion(a_deg, b_deg, c_deg)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self._pos_tol]
        volume = BoundingVolume()
        volume.primitives.append(sphere)
        volume.primitive_poses.append(pose.pose)

        pc = PositionConstraint()
        pc.header.frame_id = self.base_frame
        pc.link_name = self.tip_frame
        pc.constraint_region = volume
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = self.base_frame
        oc.link_name = self.tip_frame
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = self._ori_tol
        oc.absolute_y_axis_tolerance = self._ori_tol
        oc.absolute_z_axis_tolerance = self._ori_tol
        oc.weight = 1.0

        constraints = Constraints()
        constraints.name = 'baseline_cartesian_goal'
        constraints.position_constraints.append(pc)
        constraints.orientation_constraints.append(oc)

        req = self._base_request()
        req.goal_constraints = [constraints]
        return self._send(req, 'cartesiano')

    def publish_preview(self, path: str, segment: int, time_scale: float):
        """
        Publica una secuencia grabada para que RViz la anime.

        SOLO VISUALIZACION: no mueve el robot, no planifica y no escribe nada.
        El JSON se abre en SOLO LECTURA. Devuelve (ok, mensaje).
        """
        try:
            sequence = self._preview_cache.get(path)
            if sequence is None:
                sequence = load_json_sequence(path)
                self._preview_cache[path] = sequence
        except (JsonTrajectoryError, OSError, ValueError) as exc:
            return False, f'No se pudo leer el JSON: {exc}'

        try:
            display, n_points, total = build_display_trajectory(
                sequence, time_scale, segment)
        except (IndexError, ValueError) as exc:
            return False, f'No se pudo construir la trayectoria: {exc}'

        self._preview_publisher.publish(display)
        alcance = ('toda la secuencia' if segment < 0
                   else f'solo el segmento T{segment}')
        return True, (
            f'REPRODUCIENDO en RViz: {alcance}, {n_points} waypoints, '
            f'{total:.2f} s a {time_scale:.1f}x. '
            f'origen={sequence.source} md5={sequence.md5[:8]}...')

    def replan_and_preview(self, path: str, cartesian: bool,
                           time_scale: float, progress):
        """
        Replanifica la tarea del JSON CON EL BASELINE y publica la animacion.

        Del JSON se toman SOLO los source_points P1..PN (las metas). Los
        waypoints intermedios del sistema afinado NO se usan: el camino lo
        genera el baseline desde cero. Eso es lo que hay que ver.

        plan_only = True: no mueve el robot.
        """
        try:
            sequence = self._preview_cache.get(path)
            if sequence is None:
                sequence = load_json_sequence(path)
                self._preview_cache[path] = sequence
        except (JsonTrajectoryError, OSError, ValueError) as exc:
            return False, f'No se pudo leer el JSON: {exc}'

        if not self._action_client.wait_for_server(timeout_sec=10.0):
            return False, f'{self._action_name} no disponible. ¿Esta move_group?'

        # El escalado de velocidad de la DEMOSTRACION sustituye al 1.0 por
        # defecto SOLO para fijar la densidad de muestreo. La aceleracion, el
        # planificador, los adaptadores y las tolerancias siguen siendo los de
        # la condicion base.
        params = PlanParams(
            group=self.group, base_frame=self.base_frame,
            tip_frame=self.tip_frame, planning_time=self._planning_time,
            attempts=self._attempts, vel_scale=self._demo_vel_scaling,
            acc_scale=self._acc_scale, joint_tol=self._joint_tol,
            pos_tol=self._pos_tol, ori_tol=self._ori_tol,
            enforce_pipeline_limits=self._enforce_limits)

        ids = [seg.id for seg in sequence.segments]
        groups, failed, rebuilt = plan_sequence(
            self._action_client, sequence.source_points_rad, ids,
            cartesian, params, progress=progress)

        # El plan queda retenido ANTES de animarlo: si la animacion falla,
        # el plan sigue siendo el dato del estudio y no debe perderse.
        self._last_plan = {
            'sequence': sequence,
            'groups': groups,
            'segment_ids': list(ids),
            'failed': list(failed),
            'rebuilt': list(rebuilt),
            'params': params,
            'cartesian': cartesian,
            'planned_at': datetime.now(),
        }

        try:
            display, n_points, total, used = build_display_from_groups(
                groups, time_scale)
        except ValueError as exc:
            return False, str(exc)

        self._preview_publisher.publish(display)
        variante = 'B cartesiana' if cartesian else 'A articular'
        detalle = ''
        if failed:
            detalle = (f'  |  NO RESUELTOS: {failed}  |  '
                       f'CADENA RECONSTRUIDA en: {rebuilt}')
        message = (
            f'PLAN DEL BASELINE (variante {variante}) publicado: '
            f'{used}/{len(ids)} segmentos, {n_points} waypoints, '
            f'{total:.2f} s a {time_scale:.1f}x.{detalle}')

        # ── GUARDADO AUTOMATICO ─────────────────────────────────────
        # Serializa el plan que se ACABA de generar. No replanifica, no
        # recalcula IK y no regenera puntos: usa el mismo _last_plan.
        if self.auto_save_plan:
            saved_ok, saved_message = self.save_last_plan()
            message = f'{message}\n{saved_message}'
            if not saved_ok:
                return False, message
        return True, message

    def clear_preview(self):
        """Publica una trayectoria vacia para limpiar la animacion."""
        self._preview_publisher.publish(DisplayTrajectory())

    def default_output_dir(self) -> str:
        """
        Destino por defecto del plan guardado.

        Se deriva del JSON de entrada: hermano de su carpeta, nunca la carpeta
        misma. Con /root/taller1/trajectories/x.json el destino es
        /root/taller1/trajectories_baseline (prohibicion J.0).
        """
        if self.baseline_output_dir:
            return self.baseline_output_dir
        reference = self.preview_json
        if self._last_plan is not None:
            reference = self._last_plan['sequence'].path
        if not reference:
            return os.path.abspath('trajectories_baseline')
        source_dir = os.path.dirname(os.path.abspath(reference))
        return os.path.join(
            os.path.dirname(source_dir), 'trajectories_baseline')

    def save_last_plan(self, out_dir: str = ''):
        """
        Serializa el ULTIMO plan del baseline y lo valida offline.

        FLUJO (seccion 28 del encargo):

            plan en memoria
                -> documento JSON (misma RobotTrajectory, sin replanificar)
                -> preflight offline contra el contrato KUKA
                     |- OK      -> baseline_cartesiano_vel5_<fecha>.json
                     |- FALLA   -> baseline_cartesiano_raw_<fecha>.json
                                   marcado RAW_NO_EJECUTABLE, con la causa

        NO replanifica. NO recalcula IK. NO regenera puntos. NO hace clamp de
        ninguna articulacion: un punto fuera de limites hace que el archivo se
        marque como no ejecutable, no que se le recorte el valor.
        """
        plan = self._last_plan
        if plan is None:
            return False, ('No hay ningun plan del baseline en memoria. '
                           'Pulsa antes PLANIFICAR CON BASELINE.')

        target = (out_dir or '').strip() or self.default_output_dir()
        try:
            document = build_baseline_document(
                plan['sequence'], plan['groups'], plan['segment_ids'],
                plan['failed'], plan['rebuilt'], plan['params'],
                plan['cartesian'], planned_at=plan['planned_at'],
                ptp_velocity_pct=self.ptp_velocity_pct)
        except (JsonTrajectoryWriteError, OSError, ValueError) as exc:
            return False, f'No se pudo construir el JSON: {exc}'

        result = preflight_document(document, self.ptp_velocity_pct)

        try:
            path = write_baseline_sequence(
                target, plan['sequence'], plan['groups'],
                plan['segment_ids'], plan['failed'], plan['rebuilt'],
                plan['params'], plan['cartesian'],
                planned_at=plan['planned_at'],
                ptp_velocity_pct=self.ptp_velocity_pct,
                executable=result.ok, document=document)
        except (JsonTrajectoryWriteError, OSError, ValueError) as exc:
            return False, f'No se pudo guardar el plan: {exc}'

        report = format_report(result, path)
        for line in report.splitlines():
            self.status_queue.put(('log', line))

        total = sum(len(g) for g in plan['groups'])
        variante = 'cartesiano' if plan['cartesian'] else 'articular'
        if result.ok:
            return True, (
                f'JSON BASELINE EJECUTABLE ({variante}): {path}  |  '
                f'{len(plan["segment_ids"])} segmentos, {total} waypoints, '
                f'PTP {self.ptp_velocity_pct:g}% en todos los segmentos.')
        return False, (
            f'JSON GUARDADO COMO RAW, NO EJECUTABLE: {path}  |  '
            f'{len(result.errors)} incumplimientos del contrato KUKA. '
            f'Causa: {result.errors[0]}')

    def _send(self, request: MotionPlanRequest, kind: str):
        with self._busy_lock:
            if self._busy:
                return False, 'Ya hay una planificacion en curso.'
            self._busy = True

        options = PlanningOptions()
        options.plan_only = True     # INVARIANTE: el baseline NO ejecuta
        options.replan = False
        options.look_around = False

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = options

        threading.Thread(target=self._await, args=(goal, kind),
                         daemon=True).start()
        return True, f'Objetivo {kind} enviado a {self._action_name}.'

    def _await(self, goal, kind):
        import time
        try:
            self.status_queue.put('PLANIFICANDO')
            if not self._action_client.wait_for_server(timeout_sec=10.0):
                self.status_queue.put(
                    f'ERROR: {self._action_name} no disponible')
                return
            send_future = self._action_client.send_goal_async(goal)
            while rclpy.ok() and not send_future.done():
                time.sleep(0.02)
            handle = send_future.result()
            if handle is None or not handle.accepted:
                self.status_queue.put('ERROR: objetivo rechazado')
                return
            result_future = handle.get_result_async()
            while rclpy.ok() and not result_future.done():
                time.sleep(0.05)
            result = result_future.result()
            if result is None:
                self.status_queue.put('ERROR: resultado nulo')
                return
            code = int(result.result.error_code.val)
            if code == 1:
                traj = result.result.planned_trajectory.joint_trajectory
                self.status_queue.put(
                    f'PLAN OK ({kind}): {len(traj.points)} waypoints, '
                    f'{result.result.planning_time:.3f} s de planificacion')
            else:
                self.status_queue.put(
                    f'ERROR MoveIt {code}: '
                    f'{MOVEIT_ERRORS.get(code, "DESCONOCIDO")}')
        except Exception as exc:                          # noqa: BLE001
            self.status_queue.put(f'ERROR: excepcion {exc}')
        finally:
            with self._busy_lock:
                self._busy = False


# ─────────────────────────────────────────────────────────────────────────────
# Interfaz Tkinter
# ─────────────────────────────────────────────────────────────────────────────

BG_DARK = '#1a252f'
BG_PANEL = '#22303c'
FG_TEXT = '#ecf0f1'


class BaselineTestGui:
    """Ventana de pruebas. Corre en el hilo principal (Tkinter mainloop)."""

    def __init__(self, root: tk.Tk, node: BaselineTestGuiNode):
        self.root = root
        self.node = node
        self._last_joint_deg = None
        self._last_cart = None
        self._copied_joints = False
        self._copied_cart = False

        root.title('KUKA KR6 R900 — GUI de pruebas · CONDICION MOVEIT2 BASE')
        root.minsize(920, 760)
        root.configure(bg=BG_DARK)
        root.protocol('WM_DELETE_WINDOW', self._on_close)

        self._replanning = False
        self._build_ui()
        self.root.after(100, self._poll)

    # ── Construccion ─────────────────────────────────────────────────

    def _build_ui(self):
        banner = tk.Label(
            self.root,
            text=('CONDICION MOVEIT2 BASE — SOLO PLANIFICACION.  '
                  'No ejecuta movimiento y no se conecta al KUKA real.'),
            bg='#8e44ad', fg='white', font=('DejaVu Sans', 10, 'bold'), pady=6)
        banner.pack(fill='x')

        self._status = tk.Label(
            self.root, text='Iniciando...', bg='#2c3e50', fg='white',
            font=('DejaVu Sans Mono', 10), pady=6)
        self._status.pack(fill='x')

        body = tk.Frame(self.root, bg=BG_DARK)
        body.pack(fill='both', expand=True, padx=8, pady=8)

        self._joint_state_labels = self._state_panel(
            body, 'ESTADO ARTICULAR (deg)', AXES, 0)
        self._cart_state_labels = self._state_panel(
            body, f'ESTADO CARTESIANO de {self.node.tip_frame} '
                  f'en {self.node.base_frame} (mm / deg)',
            ['X', 'Y', 'Z', 'A', 'B', 'C'], 1)

        self._joint_entries = self._target_panel(
            body, 'OBJETIVO ARTICULAR (deg)', AXES, 2,
            self._send_joints, 'ENVIAR OBJETIVO ARTICULAR',
            self._copy_joints, extra=('CARGAR HOME', self._load_home))
        self._cart_entries = self._target_panel(
            body, f'OBJETIVO CARTESIANO de {self.node.tip_frame} '
                  f'en {self.node.base_frame} (mm / deg)',
            ['X', 'Y', 'Z', 'A', 'B', 'C'], 3,
            self._send_cart, 'ENVIAR OBJETIVO CARTESIANO',
            self._copy_cart)

        self._build_preview_panel(body)

        log_frame = tk.LabelFrame(
            body, text=' REGISTRO ', bg=BG_PANEL, fg=FG_TEXT,
            font=('DejaVu Sans', 9, 'bold'))
        log_frame.pack(fill='both', expand=True, pady=4)
        self._log_widget = scrolledtext.ScrolledText(
            log_frame, height=12, bg='#101820', fg='#d0d0d0',
            font=('DejaVu Sans Mono', 9), wrap='word')
        self._log_widget.pack(fill='both', expand=True, padx=4, pady=4)
        self._log_widget.configure(state='disabled')

        self._log('GUI baseline iniciada. plan_only = True (fijo).')
        self._log(f'Grupo: {self.node.group} | tip: {self.node.tip_frame} | '
                  f'accion: {self.node._action_name}')
        self._log('HOME original A5=0: configuracion SINGULAR de muñeca '
                  '(ejes A4 y A6 alineados).')
        self._log(f'AVISO: los campos A/B/C se refieren a {self.node.tip_frame}, '
                  'girado +90 deg en Y respecto a link_6. En HOME baseline el '
                  'tip muestra A/B/C = [0, 90, 0], NO [0, 0, 0].')

    def _build_preview_panel(self, parent):
        """
        Panel de la DEMOSTRACION.

        Del JSON se toman SOLO los puntos enseñados P1..PN. El camino lo
        planifica el BASELINE desde cero: eso es lo que hay que ver.
        """
        frame = tk.LabelFrame(
            parent,
            text=(' DEMOSTRACION — el BASELINE planifica la tarea del JSON '
                  'desde cero  (solo visual) '),
            bg=BG_PANEL, fg='#f39c12', font=('DejaVu Sans', 9, 'bold'))
        frame.pack(fill='x', pady=4)

        tk.Label(
            frame,
            text=('Del JSON se usan SOLO los puntos P1..PN (las metas). '
                  'Los waypoints del sistema afinado NO se reutilizan.'),
            bg=BG_PANEL, fg='#bdc3c7',
            font=('DejaVu Sans', 8)).pack(anchor='w', padx=8, pady=(4, 0))

        row1 = tk.Frame(frame, bg=BG_PANEL)
        row1.pack(fill='x', padx=6, pady=(4, 2))
        tk.Label(row1, text='JSON', bg=BG_PANEL, fg='#7fb3d5',
                 font=('DejaVu Sans Mono', 9, 'bold')).pack(side='left')
        self._preview_path = tk.Entry(
            row1, font=('DejaVu Sans Mono', 9), bg='#101820', fg=FG_TEXT,
            insertbackground='white')
        self._preview_path.pack(side='left', fill='x', expand=True, padx=6)
        self._preview_path.insert(0, self.node.preview_json)

        row2 = tk.Frame(frame, bg=BG_PANEL)
        row2.pack(padx=6, pady=(0, 4))
        tk.Label(row2, text='velocidad de la animacion', bg=BG_PANEL,
                 fg='#7fb3d5', font=('DejaVu Sans Mono', 9)).grid(
                     row=0, column=0, padx=(0, 6))
        self._preview_scale = tk.Entry(
            row2, width=6, justify='center', font=('DejaVu Sans Mono', 10),
            bg='#101820', fg=FG_TEXT, insertbackground='white')
        self._preview_scale.insert(0, '2.0')
        self._preview_scale.grid(row=0, column=1, padx=(0, 18))
        tk.Label(row2, text='segmento (0 = toda la tarea)', bg=BG_PANEL,
                 fg='#7fb3d5', font=('DejaVu Sans Mono', 9)).grid(
                     row=0, column=2, padx=(0, 6))
        self._preview_segment = tk.Entry(
            row2, width=6, justify='center', font=('DejaVu Sans Mono', 10),
            bg='#101820', fg=FG_TEXT, insertbackground='white')
        self._preview_segment.insert(0, '0')
        self._preview_segment.grid(row=0, column=3)

        buttons = tk.Frame(frame, bg=BG_PANEL)
        buttons.pack(pady=(0, 4))
        self._btn_cart = self._button(
            buttons, '\u25b6  PLANIFICAR CON BASELINE  (cartesiano)',
            self._play_baseline_cart, '#c0392b')
        self._btn_joint = self._button(
            buttons, '\u25b6  ... (articular)',
            self._play_baseline_joint, '#d35400')

        buttons2 = tk.Frame(frame, bg=BG_PANEL)
        buttons2.pack(pady=(0, 6))
        self._button(buttons2, 'ver el JSON afinado (referencia)',
                     self._play_recorded, '#7f8c8d')
        self._button(buttons2, 'LIMPIAR', self._clear_preview, '#7f8c8d')

        # ── Guardado del plan recien generado ────────────────────────
        row3 = tk.Frame(frame, bg=BG_PANEL)
        row3.pack(fill='x', padx=6, pady=(0, 2))
        tk.Label(row3, text='guardar en', bg=BG_PANEL, fg='#7fb3d5',
                 font=('DejaVu Sans Mono', 9, 'bold')).pack(side='left')
        self._save_dir = tk.Entry(
            row3, font=('DejaVu Sans Mono', 9), bg='#101820', fg=FG_TEXT,
            insertbackground='white')
        self._save_dir.pack(side='left', fill='x', expand=True, padx=6)
        self._save_dir.insert(0, self.node.default_output_dir())

        buttons3 = tk.Frame(frame, bg=BG_PANEL)
        buttons3.pack(pady=(0, 6))
        self._btn_save = self._button(
            buttons3, '\U0001f4be  GUARDAR ULTIMO PLAN (.json)',
            self._save_last_plan, '#2980b9')
        tk.Label(
            frame,
            text=('El archivo guardado usa el MISMO contrato que el JSON '
                  'afinado, marcado como BASELINE_SIN_AFINAR, y es la entrada '
                  'de tools/compare_planned_trajectories.py.'),
            bg=BG_PANEL, fg='#bdc3c7', wraplength=560, justify='left',
            font=('DejaVu Sans', 8)).pack(anchor='w', padx=8, pady=(0, 4))

    # ── acciones del panel ───────────────────────────────────────────

    def _preview_inputs(self):
        path = self._preview_path.get().strip()
        if not path:
            self._log('Indica la ruta del JSON.', 'warn')
            return None
        try:
            scale = float(self._preview_scale.get().strip() or '1.0')
        except ValueError:
            self._log('La velocidad debe ser un numero.', 'warn')
            return None
        try:
            raw = int(float(self._preview_segment.get().strip() or '0'))
        except ValueError:
            self._log('El segmento debe ser un numero entero.', 'warn')
            return None
        return path, scale, (-1 if raw <= 0 else raw)

    def _play_baseline_cart(self):
        self._start_replan(cartesian=True)

    def _play_baseline_joint(self):
        self._start_replan(cartesian=False)

    def _start_replan(self, cartesian: bool):
        data = self._preview_inputs()
        if data is None:
            return
        path, scale, _segment = data
        if self._replanning:
            self._log('Ya hay una replanificacion en curso.', 'warn')
            return
        self._replanning = True
        self._btn_cart.config(state='disabled')
        self._btn_joint.config(state='disabled')
        self.node.status_queue.put('PLANIFICANDO con el baseline...')
        variante = 'B cartesiana' if cartesian else 'A articular'
        self._log(f'PLANIFICANDO la tarea completa con el BASELINE '
                  f'(variante {variante}). Puede tardar; mira el progreso.')
        self._log('Del JSON se usan SOLO los puntos P1..PN. El camino lo '
                  'genera MoveIt2 por defecto, desde cero.')

        def progress(k, total, sid, detail):
            suffix = f': {detail}' if detail else ': planificando...'
            text = f'  [{k + 1}/{total}] {sid}{suffix}'
            self.node.status_queue.put(('log', text))

        def worker():
            try:
                ok, message = self.node.replan_and_preview(
                    path, cartesian, scale, progress)
            except Exception as exc:                  # noqa: BLE001
                ok, message = False, f'Replanificacion abortada: {exc}'
            self.node.status_queue.put(('log', message))
            self.node.status_queue.put(('replan_done', ok))

        threading.Thread(target=worker, daemon=True).start()

    def _play_recorded(self):
        """Reproduce el JSON grabado TAL CUAL. Es la referencia afinada."""
        data = self._preview_inputs()
        if data is None:
            return
        path, scale, segment = data
        ok, message = self.node.publish_preview(path, segment, scale)
        self._log(message, 'info' if ok else 'error')
        if ok:
            self._log('OJO: esto es la trayectoria del sistema AFINADO, no del '
                      'baseline. Es la referencia, no la condicion a demostrar.')

    def _save_last_plan(self):
        """Escribe el ultimo plan del baseline. No replanifica nada."""
        if self._replanning:
            self._log('Espera a que termine la planificacion.', 'warn')
            return
        ok, message = self.node.save_last_plan(self._save_dir.get())
        self._log(message, 'info' if ok else 'error')
        if ok:
            self._log('Este archivo es la CONDICION BASE REGENERADA con los '
                      'mismos puntos, no la trayectoria preliminar historica.')

    def _clear_preview(self):
        self.node.clear_preview()
        self._log('Animacion limpiada.')

    def _state_panel(self, parent, title, labels, row):
        frame = tk.LabelFrame(parent, text=f' {title} ', bg=BG_PANEL,
                              fg=FG_TEXT, font=('DejaVu Sans', 9, 'bold'))
        frame.pack(fill='x', pady=4)
        out = []
        inner = tk.Frame(frame, bg=BG_PANEL)
        inner.pack(padx=6, pady=6)
        for i, name in enumerate(labels):
            tk.Label(inner, text=name, bg=BG_PANEL, fg='#7fb3d5',
                     font=('DejaVu Sans Mono', 10, 'bold'),
                     width=4).grid(row=0, column=i, padx=6)
            value = tk.Label(inner, text='—', bg='#101820', fg='#2ecc71',
                             font=('DejaVu Sans Mono', 11), width=11)
            value.grid(row=1, column=i, padx=6, pady=2)
            out.append(value)
        return out

    def _target_panel(self, parent, title, labels, row, send_fn, send_text,
                      copy_fn, extra=None):
        frame = tk.LabelFrame(parent, text=f' {title} ', bg=BG_PANEL,
                              fg=FG_TEXT, font=('DejaVu Sans', 9, 'bold'))
        frame.pack(fill='x', pady=4)
        inner = tk.Frame(frame, bg=BG_PANEL)
        inner.pack(padx=6, pady=6)
        entries = []
        for i, name in enumerate(labels):
            tk.Label(inner, text=name, bg=BG_PANEL, fg='#7fb3d5',
                     font=('DejaVu Sans Mono', 10, 'bold'),
                     width=4).grid(row=0, column=i, padx=6)
            entry = tk.Entry(inner, width=11, justify='center',
                             font=('DejaVu Sans Mono', 11),
                             bg='#101820', fg=FG_TEXT, insertbackground='white')
            entry.insert(0, '0.0')
            entry.grid(row=1, column=i, padx=6, pady=2)
            entries.append(entry)
        buttons = tk.Frame(frame, bg=BG_PANEL)
        buttons.pack(pady=(0, 6))
        self._button(buttons, send_text, send_fn, '#c0392b')
        self._button(buttons, 'COPIAR ESTADO ACTUAL', copy_fn, '#2980b9')
        if extra:
            self._button(buttons, extra[0], extra[1], '#16a085')
        return entries

    @staticmethod
    def _button(parent, text, command, colour):
        b = tk.Button(parent, text=text, command=command, bg=colour,
                      fg='white', font=('DejaVu Sans', 9, 'bold'),
                      relief='flat', padx=12, pady=5, cursor='hand2')
        b.pack(side='left', padx=4)
        return b

    # ── Acciones ─────────────────────────────────────────────────────

    def _read(self, entries, count=6):
        values = []
        for i, entry in enumerate(entries):
            try:
                values.append(float(entry.get().strip()))
            except ValueError:
                self._log(f'Valor invalido en el campo {i + 1}.', 'error')
                return None
        return values if len(values) == count else None

    def _send_joints(self):
        values = self._read(self._joint_entries)
        if values is None:
            return
        for name, value in zip(JOINT_NAMES, values):
            low, high = JOINT_LIMITS_DEG[name]
            if not (low <= value <= high):
                self._log(f'{name} = {value:.2f} deg fuera del limite del '
                          f'URDF [{low}, {high}]. No se envia.', 'error')
                return
        ok, message = self.node.send_joint_goal(values)
        if ok:
            detail = ', '.join(f'{a}={v:.2f}' for a, v in zip(AXES, values))
            self._log(f'OBJETIVO ARTICULAR -> {detail}')
        else:
            self._log(f'RECHAZADO: {message}', 'error')

    def _send_cart(self):
        values = self._read(self._cart_entries)
        if values is None:
            return
        ok, message = self.node.send_cartesian_goal(*values)
        if ok:
            self._log('OBJETIVO CARTESIANO -> '
                      f'X={values[0]:.1f} Y={values[1]:.1f} Z={values[2]:.1f} '
                      f'mm | A={values[3]:.1f} B={values[4]:.1f} '
                      f'C={values[5]:.1f} deg')
        else:
            self._log(f'RECHAZADO: {message}', 'error')

    def _copy_joints(self):
        if self._last_joint_deg is None:
            self._log('Sin estado articular todavia.', 'warn')
            return
        for entry, value in zip(self._joint_entries, self._last_joint_deg):
            entry.delete(0, 'end')
            entry.insert(0, f'{value:.3f}')
        self._log('Estado articular copiado al objetivo.')

    def _copy_cart(self):
        if self._last_cart is None:
            self._log('Sin estado cartesiano todavia.', 'warn')
            return
        for entry, value in zip(self._cart_entries, self._last_cart):
            entry.delete(0, 'end')
            entry.insert(0, f'{value:.3f}')
        self._log('Estado cartesiano copiado al objetivo.')

    def _load_home(self):
        for entry, value in zip(self._joint_entries, HOME_JOINT_DEG):
            entry.delete(0, 'end')
            entry.insert(0, f'{value:.3f}')
        self._log('HOME original cargado en el objetivo (A5=0, SINGULAR). '
                  'No enviado todavia.')

    # ── Refresco ─────────────────────────────────────────────────────

    def _poll(self):
        try:
            while True:
                data = self.node.joint_queue.get_nowait()
                self._last_joint_deg = data
                for label, value in zip(self._joint_state_labels, data):
                    label.config(text=f'{value:8.3f}')
                if not self._copied_joints:
                    self._copied_joints = True
                    self._copy_joints()
                    self._status.config(text='LISTO', bg='#27ae60')
        except queue.Empty:
            pass
        try:
            while True:
                data = self.node.cart_queue.get_nowait()
                self._last_cart = data
                for label, value in zip(self._cart_state_labels, data):
                    label.config(text=f'{value:8.2f}')
                if not self._copied_cart:
                    self._copied_cart = True
                    self._copy_cart()
        except queue.Empty:
            pass
        try:
            while True:
                item = self.node.status_queue.get_nowait()
                # La cola admite un string (estado) o una tupla (kind, valor).
                if isinstance(item, tuple):
                    kind, value = item
                    if kind == 'log':
                        self._log(value)
                    elif kind == 'replan_done':
                        self._replanning = False
                        self._btn_cart.config(state='normal')
                        self._btn_joint.config(state='normal')
                        done = 'PLAN DEL BASELINE LISTO' if value else 'ERROR'
                        self._status.config(
                            text=done,
                            bg='#27ae60' if value else '#c0392b')
                    continue
                text = item
                self._status.config(text=text)
                colour = '#27ae60'
                if text.startswith('ERROR'):
                    colour = '#c0392b'
                elif 'PLANIFICANDO' in text or 'ESPERANDO' in text:
                    colour = '#e67e22'
                self._status.config(bg=colour)
                self._log(text)
        except queue.Empty:
            pass
        self.root.after(100, self._poll)

    def _log(self, text, level='info'):
        stamp = datetime.now().strftime('%H:%M:%S')
        self._log_widget.configure(state='normal')
        self._log_widget.insert('end', f'[{stamp}] {text}\n')
        self._log_widget.see('end')
        self._log_widget.configure(state='disabled')

    def _on_close(self):
        self.root.quit()
        self.root.destroy()


def main(args=None):
    rclpy.init(args=args)
    node = BaselineTestGuiNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    root = tk.Tk()
    BaselineTestGui(root, node)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
