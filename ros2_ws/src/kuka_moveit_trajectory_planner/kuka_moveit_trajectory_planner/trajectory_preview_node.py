#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
trajectory_preview_node.py

Reproduce en RViz2 una secuencia de trayectorias ya generada (el mismo JSON
que produce trajectory_generator_node) SIN MOVER EL ROBOT.

Contrato ROS 2 (std_msgs/msg/String con JSON):
    entrada : /kuka_moveit/trajectory_preview/request_json
    salida  : /kuka_moveit/trajectory_preview/status_json

Mecanismos de visualizacion (los dos son de solo dibujo):

  1. moveit_msgs/msg/DisplayTrajectory  ->  /display_planned_path
     Es el mecanismo nativo de MoveIt2/RViz2: el display "Planned Path" del
     panel MotionPlanning ya esta suscrito a ese topico en la configuracion
     actual de RViz, por lo que no hay que tocar nada en RViz.
     Se publica UNA SOLA VEZ una unica RobotTrajectory continua con los N
     segmentos encadenados: republicar un DisplayTrajectory por segmento hace
     que RViz reinicie su animacion en cada frontera (parpadeo).

  2. moveit_msgs/msg/DisplayRobotState -> /kuka_moveit/trajectory_preview/robot_state
     Publicacion punto a punto respetando time_from_start sobre un unico
     reloj global que nunca vuelve a cero entre segmentos. Es el mecanismo
     que reproduce la temporizacion EXACTA de la secuencia. Requiere anadir
     en RViz un display "RobotState" apuntando a ese topico (opcional).

La secuencia visual continua se construye SOLO EN MEMORIA con
build_preview_sequence(): omite la pose duplicada de cada frontera
(Ti[-1] == T(i+1)[0]) y traslada los tiempos a un reloj global. El JSON
recibido, sus 8 segmentos y sus 123 puntos no se modifican.

SEGURIDAD (invariantes de este nodo):
  - No existe ningun cliente de accion ni de servicio: el nodo no puede
    ejecutar nada.
  - No publica en /joint_states, /fake_joint_states, /kuka_bridge/*,
    /move_action, /execute_trajectory ni en ningun follow_joint_trajectory.
    Esos nombres estan prohibidos por _assert_safe_topic() y el nodo se
    niega a arrancar si se configuran.
  - No usa EnableMove ni ninguna via del bridge TCP/IP.
"""

import queue
import threading
import time
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from builtin_interfaces.msg import Duration as DurationMsg
from moveit_msgs.msg import (
    DisplayRobotState,
    DisplayTrajectory,
    RobotState,
    RobotTrajectory,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .trajectory_contract import (
    DEFAULT_BOUNDARY_TOLERANCE_RAD,
    PREVIEW_STATUS_CANCELLED,
    PREVIEW_STATUS_COMPLETED,
    PREVIEW_STATUS_ERROR,
    PREVIEW_STATUS_PLAYING,
    ContractError,
    PreviewRequest,
    PreviewSequence,
    build_preview_sequence,
    build_preview_status,
    dumps_json,
    loads_json,
    parse_preview_request,
    peek_preview_request_id,
)

#: Fragmentos de nombre de topico que este nodo NUNCA debe usar: son la ruta
#: fisica (bridge TCP/IP, controlador, ejecucion de MoveIt) o el estado real
#: del robot. Si un parametro los contiene, el nodo no arranca.
FORBIDDEN_TOPIC_FRAGMENTS = (
    'kuka_bridge',
    'joint_command',
    'joint_states',
    'follow_joint_trajectory',
    'execute_trajectory',
    'move_action',
    'controller',
    'enable_move',
)


class UnsafeTopicError(RuntimeError):
    """Un parametro apunta a un topico de la ruta fisica del robot."""


def _assert_safe_topic(topic: str, purpose: str):
    lowered = topic.lower()
    for fragment in FORBIDDEN_TOPIC_FRAGMENTS:
        if fragment in lowered:
            raise UnsafeTopicError(
                f'El topico "{topic}" configurado para {purpose} contiene '
                f'"{fragment}", que pertenece a la ruta fisica del robot. '
                'La previsualizacion NUNCA debe publicar ahi.')


def sec_to_duration_msg(seconds: float) -> DurationMsg:
    duration = DurationMsg()
    total_ns = int(round(float(seconds) * 1e9))
    duration.sec = int(total_ns // 1_000_000_000)
    duration.nanosec = int(total_ns % 1_000_000_000)
    return duration


class TrajectoryPreviewNode(Node):
    """Reproductor visual. Solo dibuja: no ejecuta y no comanda nada."""

    def __init__(self):
        super().__init__('kuka_trajectory_preview_node')

        self._declare_parameters()
        self._load_parameters()

        self._callback_group = ReentrantCallbackGroup()

        qos = QoSProfile(
            depth=self._qos_depth,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._status_pub = self.create_publisher(
            String, self._status_topic, qos)
        self._display_pub = self.create_publisher(
            DisplayTrajectory, self._display_topic, qos)
        self._robot_state_pub = (
            self.create_publisher(DisplayRobotState, self._robot_state_topic,
                                  qos)
            if self._publish_robot_state else None)

        self.create_subscription(
            String, self._request_topic, self._request_callback, qos,
            callback_group=self._callback_group)

        self._request_queue: 'queue.Queue[str]' = queue.Queue()
        self._cancel_current = threading.Event()
        self._shutdown = threading.Event()
        self._current_request_id: Optional[str] = None
        self._worker = threading.Thread(
            target=self._worker_loop, name='preview_worker', daemon=True)
        self._worker.start()

        robot_state_info = (self._robot_state_topic
                            if self._publish_robot_state else 'deshabilitado')
        self.get_logger().info(
            '\n'
            '╔══════════════════════════════════════════════════════════╗\n'
            '║   kuka_trajectory_preview_node iniciado                  ║\n'
            '╠══════════════════════════════════════════════════════════╣\n'
            f'║  Request:     {self._request_topic:<43}║\n'
            f'║  Status:      {self._status_topic:<43}║\n'
            f'║  RViz path:   {self._display_topic:<43}║\n'
            f'║  RobotState:  {robot_state_info:<43}║\n'
            f'║  Modo:        {"secuencia continua (T1..TN)":<43}║\n'
            f'║  Velocidad:   {str(self._playback_rate) + "x":<43}║\n'
            '╚══════════════════════════════════════════════════════════╝\n'
            'Solo PREVISUALIZA. No ejecuta, no comanda el robot real y no\n'
            'publica en ningun topico del bridge TCP/IP.\n')

    # ─────────────────────────────────────────────────────────────────
    # Parametros
    # ─────────────────────────────────────────────────────────────────

    def _declare_parameters(self):
        self.declare_parameter(
            'request_topic', '/kuka_moveit/trajectory_preview/request_json')
        self.declare_parameter(
            'status_topic', '/kuka_moveit/trajectory_preview/status_json')
        self.declare_parameter('qos_depth', 10)

        # Mecanismo nativo de MoveIt2/RViz2.
        self.declare_parameter(
            'display_trajectory_topic', '/display_planned_path')
        # Mecanismo exclusivo de preview (topico propio, nadie mas lo usa).
        self.declare_parameter(
            'robot_state_topic',
            '/kuka_moveit/trajectory_preview/robot_state')
        self.declare_parameter('publish_robot_state', True)

        self.declare_parameter('playback_rate', 1.0)
        # Tolerancia para dar por duplicada la pose de una frontera.
        self.declare_parameter(
            'boundary_tolerance_rad', DEFAULT_BOUNDARY_TOLERANCE_RAD)
        # Hueco anadido al reloj global SOLO entre segmentos cuya frontera NO
        # estaba duplicada. 0.0 = secuencia continua sin pausas.
        self.declare_parameter('inter_segment_gap_sec', 0.0)
        self.declare_parameter('publish_segment_status', True)
        self.declare_parameter('robot_model_name', '')
        self.declare_parameter('joint_names', [
            'joint_a1', 'joint_a2', 'joint_a3',
            'joint_a4', 'joint_a5', 'joint_a6'])

    def _load_parameters(self):
        gp = self.get_parameter
        self._request_topic = gp('request_topic').value
        self._status_topic = gp('status_topic').value
        self._qos_depth = int(gp('qos_depth').value)

        self._display_topic = gp('display_trajectory_topic').value
        self._robot_state_topic = gp('robot_state_topic').value
        self._publish_robot_state = bool(gp('publish_robot_state').value)

        # Guardia de seguridad sobre TODOS los topicos de salida.
        _assert_safe_topic(self._display_topic, 'la trayectoria de RViz')
        _assert_safe_topic(self._robot_state_topic, 'el estado de RViz')
        _assert_safe_topic(self._status_topic, 'el estado de preview')

        rate = float(gp('playback_rate').value)
        if rate <= 0.0:
            self.get_logger().warn(
                f'playback_rate={rate} invalido; se usa 1.0.')
            rate = 1.0
        self._playback_rate = rate

        self._boundary_tolerance_rad = max(
            0.0, float(gp('boundary_tolerance_rad').value))
        self._inter_segment_gap = max(
            0.0, float(gp('inter_segment_gap_sec').value))
        self._publish_segment_status = bool(gp('publish_segment_status').value)
        self._robot_model_name = gp('robot_model_name').value or ''
        self._default_joint_names = list(gp('joint_names').value)

    # ─────────────────────────────────────────────────────────────────
    # Entrada
    # ─────────────────────────────────────────────────────────────────

    def _request_callback(self, msg: String):
        # Una peticion nueva cancela la reproduccion en curso.
        if self._current_request_id is not None:
            self.get_logger().info(
                'Nueva peticion de preview: se cancela la reproduccion en '
                'curso.')
            self._cancel_current.set()
        self._request_queue.put(msg.data)

    def _worker_loop(self):
        while not self._shutdown.is_set():
            try:
                text = self._request_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            self._cancel_current.clear()
            try:
                self._play_request(text)
            except Exception as exc:                      # noqa: BLE001
                self.get_logger().error(f'Excepcion en la preview: {exc}')
                self._publish_status(build_preview_status(
                    self._current_request_id or '',
                    PREVIEW_STATUS_ERROR,
                    f'Excepcion interna: {exc}'))
            finally:
                self._current_request_id = None
                self._request_queue.task_done()

    # ─────────────────────────────────────────────────────────────────
    # Reproduccion
    # ─────────────────────────────────────────────────────────────────

    def _play_request(self, text: str):
        try:
            payload = loads_json(text)
            request = parse_preview_request(
                payload, self._default_joint_names)
        except ContractError as exc:
            request_id = ''
            try:
                # La identidad puede venir en el sobre o dentro de
                # "trajectory": se busca en los dos niveles.
                request_id = peek_preview_request_id(loads_json(text))
            except ContractError:
                pass
            self.get_logger().error(f'Peticion de preview invalida: {exc}')
            self._publish_status(build_preview_status(
                request_id, PREVIEW_STATUS_ERROR, str(exc)))
            return

        self._current_request_id = request.request_id
        for warning in request.warnings:
            self.get_logger().warn(f'  aviso: {warning}')

        self.get_logger().info(
            f'Preview "{request.request_id}": {len(request.segments)} '
            f'segmentos, {request.total_point_count} puntos, '
            f'{request.total_duration_sec:.3f} s.')

        # ── Secuencia visual continua (solo en memoria) ─────────────────
        # Los segmentos originales NO se tocan: se construye una vista unica
        # que omite la pose duplicada de cada frontera y encadena los tiempos
        # en un reloj global.
        sequence = build_preview_sequence(
            request,
            boundary_tolerance_rad=self._boundary_tolerance_rad,
            inter_segment_gap_sec=self._inter_segment_gap)

        if not sequence.frames:
            message = 'La secuencia no contiene ninguna pose reproducible.'
            self.get_logger().error(message)
            self._publish_status(build_preview_status(
                request.request_id, PREVIEW_STATUS_ERROR, message))
            return

        self.get_logger().info(
            f'Secuencia continua: {sequence.point_count} poses '
            f'({request.total_point_count} originales - '
            f'{sequence.dropped_boundary_count} fronteras duplicadas), '
            f'{sequence.duration_sec:.3f} s de reloj global.')

        extra = {
            'segment_count': len(request.segments),
            'trajectory_point_count': request.total_point_count,
            'preview_point_count': sequence.point_count,
            'dropped_boundary_points': sequence.dropped_boundary_count,
            'total_duration_sec': sequence.duration_sec,
            'playback_rate': self._playback_rate,
            'display_trajectory_topic': self._display_topic,
            'warnings': request.warnings,
        }
        if request.preview_id:
            extra['preview_id'] = request.preview_id
        self._publish_status(build_preview_status(
            request.request_id, PREVIEW_STATUS_PLAYING, extra=extra))

        # ── RViz: UNA sola publicacion con la trayectoria completa ───────
        # Publicar un DisplayTrajectory por segmento hacia que RViz reiniciase
        # su animacion en cada frontera: esa era la causa del parpadeo.
        self._display_pub.publish(self._build_display_trajectory(sequence))

        # ── Reproduccion punto a punto con un unico reloj global ────────
        self._play_sequence(request, sequence)

        if self._cancel_current.is_set():
            self._publish_status(build_preview_status(
                request.request_id, PREVIEW_STATUS_CANCELLED,
                'Reproduccion cancelada por una peticion nueva.'))
            self.get_logger().info('Preview cancelada.')
            return

        self._publish_status(build_preview_status(
            request.request_id, PREVIEW_STATUS_COMPLETED,
            extra={
                'segment_count': len(request.segments),
                'trajectory_point_count': request.total_point_count,
                'preview_point_count': sequence.point_count,
            }))
        self.get_logger().info(
            f'Preview "{request.request_id}" completada.')

    def _play_sequence(self,
                       request: PreviewRequest,
                       sequence: PreviewSequence):
        """
        Recorre TODAS las poses con un unico reloj monotono.

        El reloj arranca una sola vez y nunca se reinicia entre segmentos: la
        ultima pose de Ti continua directamente hacia T(i+1). Los status por
        segmento son diagnostico y no alteran nada de lo que se dibuja.
        """
        start = time.monotonic()
        for frame in sequence.frames:
            target = start + frame.time_from_start_sec / self._playback_rate
            while True:
                remaining = target - time.monotonic()
                if remaining <= 0.0 or self._cancel_current.is_set():
                    break
                time.sleep(min(remaining, 0.02))
            if self._cancel_current.is_set():
                return

            if frame.is_segment_start:
                self._announce_segment(request, frame)

            if self._robot_state_pub is not None:
                self._robot_state_pub.publish(
                    self._build_display_robot_state(
                        sequence.joint_names, frame.positions_rad))

    def _announce_segment(self, request: PreviewRequest, frame):
        """Status y log al entrar en un segmento. Solo diagnostico."""
        segment = request.segments[frame.segment_index]
        self.get_logger().info(
            f'[{segment.segment_id}] {segment.from_point} -> '
            f'{segment.to_point}: {len(segment.points)} puntos, '
            f't global {frame.time_from_start_sec:.3f} s.')
        if not self._publish_segment_status:
            return
        self._publish_status(build_preview_status(
            request.request_id, PREVIEW_STATUS_PLAYING,
            extra={
                'segment_index': frame.segment_index,
                'segment_id': segment.segment_id,
                'from': segment.from_point,
                'to': segment.to_point,
                'duration_sec': segment.duration_sec,
                'global_time_sec': frame.time_from_start_sec,
            }))

    # ─────────────────────────────────────────────────────────────────
    # Construccion de los mensajes de visualizacion
    # ─────────────────────────────────────────────────────────────────

    def _build_display_trajectory(self,
                                  sequence: PreviewSequence,
                                  ) -> DisplayTrajectory:
        """
        Reconstruye UNA sola RobotTrajectory continua con toda la secuencia.

        Se publica una unica vez al empezar. Antes se publicaba un
        DisplayTrajectory por segmento y RViz reiniciaba la animacion (y el
        rastro) en cada frontera: ese era el parpadeo.

        trajectory_start es la primera pose de la secuencia; los tiempos son
        los del reloj global, ya encadenados y sin la pose duplicada de las
        fronteras. Ningun valor articular se recalcula.
        """
        display = DisplayTrajectory()
        display.model_id = self._robot_model_name

        first_frame = sequence.frames[0]
        joint_state = JointState()
        joint_state.name = list(sequence.joint_names)
        joint_state.position = list(first_frame.positions_rad)
        start_state = RobotState()
        start_state.joint_state = joint_state
        start_state.is_diff = False
        display.trajectory_start = start_state

        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = list(sequence.joint_names)
        for frame in sequence.frames:
            traj_point = JointTrajectoryPoint()
            traj_point.positions = list(frame.positions_rad)
            traj_point.velocities = list(frame.velocities_rad_s)
            traj_point.accelerations = list(frame.accelerations_rad_s2)
            traj_point.time_from_start = sec_to_duration_msg(
                frame.time_from_start_sec)
            joint_trajectory.points.append(traj_point)

        robot_trajectory = RobotTrajectory()
        robot_trajectory.joint_trajectory = joint_trajectory
        display.trajectory.append(robot_trajectory)

        return display

    @staticmethod
    def _build_display_robot_state(joint_names: List[str],
                                   positions_rad: List[float],
                                   ) -> DisplayRobotState:
        joint_state = JointState()
        joint_state.name = list(joint_names)
        joint_state.position = list(positions_rad)

        state = RobotState()
        state.joint_state = joint_state
        state.is_diff = False

        message = DisplayRobotState()
        message.state = state
        return message

    # ─────────────────────────────────────────────────────────────────
    # Salida
    # ─────────────────────────────────────────────────────────────────

    def _publish_status(self, status: Dict[str, Any]):
        message = String()
        message.data = dumps_json(status)
        self._status_pub.publish(message)

    def destroy_node(self):
        self._shutdown.set()
        self._cancel_current.set()
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = TrajectoryPreviewNode()
    except UnsafeTopicError as exc:
        print(f'[kuka_trajectory_preview_node] CONFIGURACION INSEGURA: {exc}')
        rclpy.shutdown()
        raise SystemExit(1)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
