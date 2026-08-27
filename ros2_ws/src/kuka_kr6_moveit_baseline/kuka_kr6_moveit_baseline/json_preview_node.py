#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
json_preview_node.py

REPRODUCE en RViz una secuencia de trayectoria grabada en JSON, para VER las
configuraciones articulares que produjo cada condicion.

POR QUE EXISTE
  El analizador (continuity_analyzer_node) mide y escribe CSV, pero es un
  OBSERVADOR PASIVO: no publica nada. Esa invariante se mantiene. Este nodo es
  el que publica, y esta separado a proposito.

QUE HACE
  Publica UNA SOLA RobotTrajectory con los 199 waypoints de los 14 segmentos
  concatenados y un RELOJ GLOBAL CONTINUO, en /display_planned_path. RViz la
  anima de principio a fin, sin cortes ni reinicios entre segmentos.

  El tiempo de cada segmento REINICIA en 0 en el archivo (asi lo entrega
  MoveIt2 y asi viene en el JSON). Aqui se acumula:
      t_global = t_local + suma de las duraciones de los segmentos previos
  Los valores articulares NO se tocan: ni se interpolan, ni se remuestrean,
  ni se suavizan. Se publica exactamente lo que hay en el archivo.

QUE NO HACE
  - NO mueve el robot. No hay controlador, ni cliente de accion, ni topico de
    ejecucion. La animacion de RViz es una visualizacion, no un movimiento.
  - NO planifica.
  - NO escribe ningun archivo.
  - NO modifica el JSON: se abre en SOLO LECTURA.

ADVERTENCIA DE PROCEDENCIA
  Si el JSON es del sistema AFINADO (TG2), lo que ves es la condicion AFINADA.
  El banner lo dice en cada reproduccion.
"""

import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from builtin_interfaces.msg import Duration
from moveit_msgs.msg import DisplayTrajectory, RobotState, RobotTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .continuity_metrics import DECLARED_TIP, JOINT_NAMES
from .trajectory_json_reader import JsonTrajectoryError
from .trajectory_json_reader import load as load_json_sequence

#: Topicos prohibidos: este nodo NO puede publicar en nada que ejecute.
FORBIDDEN_TOPIC_FRAGMENTS = (
    'follow_joint_trajectory', 'joint_states', 'controller',
    'kuka_bridge', 'eki', 'command',
)


def to_duration(seconds: float) -> Duration:
    msg = Duration()
    msg.sec = int(seconds)
    msg.nanosec = int(round((seconds - msg.sec) * 1e9))
    if msg.nanosec >= 1000000000:
        msg.sec += 1
        msg.nanosec -= 1000000000
    return msg


def build_display_trajectory(sequence, time_scale: float = 1.0,
                             segment: int = -1):
    """
    Construye el DisplayTrajectory de una secuencia grabada.

    UNA sola RobotTrajectory con todos los segmentos concatenados y un RELOJ
    GLOBAL CONTINUO, para que RViz la anime de principio a fin sin cortes ni
    reinicios entre segmentos.

    Los valores articulares NO se tocan: ni se interpolan, ni se remuestrean,
    ni se suavizan. Lo unico que se construye es el reloj:
        t_global = t_local + suma de las duraciones de los segmentos previos
    `time_scale` estira o comprime SOLO la animacion; no altera posiciones.

    Devuelve (DisplayTrajectory, numero_de_puntos, duracion_total).
    """
    segments = sequence.segments
    if segment >= 0:
        # `segment` es 1-BASED y se resuelve por ID (T1 = el primero), que es
        # como se nombran en el archivo y en la tabla comparativa. Se busca
        # PRIMERO por id; el indice 0-based es solo un respaldo, y nunca se
        # mezclan los dos criterios: hacerlo seleccionaba dos segmentos.
        wanted = f'T{segment}'
        match = [s for s in segments if s.id == wanted]
        if not match:
            match = [s for s in segments if s.index == segment - 1]
        if not match:
            raise ValueError(
                f'No existe el segmento {wanted}. Disponibles: '
                f'{[s.id for s in segments]}')
        segments = match

    scale = max(0.05, float(time_scale))
    traj = JointTrajectory()
    traj.joint_names = list(JOINT_NAMES)
    offset = 0.0
    for seg in segments:
        for wp in seg.waypoints:
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in wp.positions_rad]
            # Velocidades y aceleraciones TAL CUAL si vienen; si no, se
            # omiten. NO se inventan ceros.
            if len(wp.velocities_rad_s) == 6:
                point.velocities = [float(v) for v in wp.velocities_rad_s]
            if len(wp.accelerations_rad_s2) == 6:
                point.accelerations = [
                    float(v) for v in wp.accelerations_rad_s2]
            point.time_from_start = to_duration(
                (wp.time_from_start_s + offset) * scale)
            traj.points.append(point)
        offset += seg.duration_sec

    robot_traj = RobotTrajectory()
    robot_traj.joint_trajectory = traj

    start = RobotState()
    js = JointState()
    js.name = list(JOINT_NAMES)
    js.position = [float(v) for v in segments[0].waypoints[0].positions_rad]
    start.joint_state = js
    start.is_diff = False

    display = DisplayTrajectory()
    display.model_id = 'kuka_kr6r900sixx'
    display.trajectory_start = start
    display.trajectory.append(robot_traj)
    return display, len(traj.points), offset * scale


def build_display_from_groups(groups, time_scale: float = 1.0,
                              gap_sec: float = 0.0):
    """
    Concatena VARIOS planes en UNA sola trayectoria con reloj global continuo.

    Es lo que hace falta para ver la tarea completa animada de un tiron: si se
    publica un plan por segmento, RViz reinicia la animacion en cada uno y solo
    se ve el ultimo gesto.

    `groups` es una lista de listas de Waypoint. Las listas vacias (segmentos
    no resueltos) se saltan; la animacion salta ese tramo, que es exactamente
    lo que se quiere ver.

    Devuelve (DisplayTrajectory, n_puntos, duracion_total, n_segmentos_usados).
    """
    scale = max(0.05, float(time_scale))
    traj = JointTrajectory()
    traj.joint_names = list(JOINT_NAMES)
    offset = 0.0
    used = 0
    first = None
    last_t = -1.0
    for wps in groups:
        if not wps:
            continue
        used += 1
        if first is None:
            first = wps[0]
        for wp in wps:
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in wp.positions_rad]
            if len(wp.velocities_rad_s) == 6:
                point.velocities = [float(v) for v in wp.velocities_rad_s]
            if len(wp.accelerations_rad_s2) == 6:
                point.accelerations = [
                    float(v) for v in wp.accelerations_rad_s2]
            t = (wp.time_from_start_s + offset) * scale
            # El reloj debe ser ESTRICTAMENTE creciente o RViz descarta puntos.
            if t <= last_t:
                t = last_t + 1.0e-4
            last_t = t
            point.time_from_start = to_duration(t)
            traj.points.append(point)
        offset += wps[-1].time_from_start_s + gap_sec

    if first is None:
        raise ValueError('Ningun segmento se resolvio: no hay nada que animar.')

    robot_traj = RobotTrajectory()
    robot_traj.joint_trajectory = traj

    start = RobotState()
    js = JointState()
    js.name = list(JOINT_NAMES)
    js.position = [float(v) for v in first.positions_rad]
    start.joint_state = js
    start.is_diff = False

    display = DisplayTrajectory()
    display.model_id = 'kuka_kr6r900sixx'
    display.trajectory_start = start
    display.trajectory.append(robot_traj)
    return display, len(traj.points), last_t, used


def preview_qos() -> QoSProfile:
    """QoS de visualizacion: TRANSIENT_LOCAL para que RViz la conserve."""
    return QoSProfile(
        depth=1,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class JsonPreviewNode(Node):
    """Publica una secuencia grabada para que RViz la anime."""

    def __init__(self):
        super().__init__('kr6_baseline_json_preview')

        self.declare_parameter('input_json', '')
        self.declare_parameter('display_topic', '/display_planned_path')
        self.declare_parameter('loop', True)
        self.declare_parameter('loop_pause_sec', 2.0)
        # Estira o comprime el reloj SOLO para la animacion. 1.0 = tiempos del
        # archivo. Sube a 3.0 para verlo despacio. NO altera las posiciones ni
        # el CSV: es un factor de reproduccion, nada mas.
        self.declare_parameter('time_scale', 1.0)
        self.declare_parameter('segment', -1)   # -1 = toda la secuencia

        gp = self.get_parameter
        self._input_json = str(gp('input_json').value or '').strip()
        topic = str(gp('display_topic').value)
        self._loop = bool(gp('loop').value)
        self._pause = float(gp('loop_pause_sec').value)
        self._time_scale = max(0.05, float(gp('time_scale').value))
        self._segment = int(gp('segment').value)

        lowered = topic.lower()
        for fragment in FORBIDDEN_TOPIC_FRAGMENTS:
            if fragment in lowered:
                raise ValueError(
                    f'display_topic {topic!r} contiene {fragment!r}. Este nodo '
                    'solo puede publicar en un topico de VISUALIZACION.')

        self._publisher = self.create_publisher(
            DisplayTrajectory, topic, preview_qos())
        self._stop = threading.Event()

        if not self._input_json:
            self.get_logger().error(
                'Falta input_json. Este nodo no hace nada sin un archivo.')
            self._stop.set()
            return
        threading.Thread(target=self._run, daemon=True).start()

    # ─────────────────────────────────────────────────────────────────

    def _build(self, sequence):
        return build_display_trajectory(
            sequence, self._time_scale, self._segment)

    def _run(self):
        try:
            sequence = load_json_sequence(self._input_json)
        except (JsonTrajectoryError, OSError, ValueError) as exc:
            self.get_logger().error(f'\nPREVISUALIZACION ABORTADA\n{exc}')
            self._stop.set()
            return

        display, n_points, total = self._build(sequence)
        events = ', '.join(
            f"{e.get('action')}@{e.get('at_point')}"
            for e in sequence.gripper_events) or '(ninguno)'

        self.get_logger().info(
            '\n'
            '╔══════════════════════════════════════════════════════════════╗\n'
            '║  PREVISUALIZACION DE UNA SECUENCIA GRABADA                   ║\n'
            '╠══════════════════════════════════════════════════════════════╣\n'
            '║  *** PROCEDENCIA EXTERNA: NO la genero el baseline ***       ║\n'
            f'║  source  : {sequence.source[:49]:<49}║\n'
            f'║  md5     : {sequence.md5:<49}║\n'
            f'║  tip     : {DECLARED_TIP:<49}║\n'
            '╠══════════════════════════════════════════════════════════════╣\n'
            f'║  waypoints publicados : {n_points:<37}║\n'
            f'║  duracion de la animacion: {total:>8.3f} s{"":<26}║\n'
            f'║  factor de reproduccion : {self._time_scale:>8.2f}x{"":<27}║\n'
            f'║  bucle   : {("si" if self._loop else "no"):<49}║\n'
            '╚══════════════════════════════════════════════════════════════╝\n'
            f'Eventos de gripper (METADATO, no se actuan): {events}\n'
            '\n'
            'EN RVIZ: panel Displays -> MotionPlanning -> Planned Path\n'
            '   Trajectory Topic  = display_planned_path\n'
            '   Show Trail        = true    <- imprescindible para ver el rastro\n'
            '   Loop Animation    = true\n'
            '   State Display Time = 0.05 s o mas lento\n'
            '\n'
            'El robot NO se mueve: esto es una VISUALIZACION. No hay controlador,\n'
            'ni cliente de accion, ni ningun topico de ejecucion.\n')

        while rclpy.ok() and not self._stop.is_set():
            self._publisher.publish(display)
            self.get_logger().info(
                f'Publicada la secuencia ({n_points} waypoints, '
                f'{total:.3f} s). Mira RViz.')
            if not self._loop:
                break
            waited = 0.0
            step = 0.2
            limit = total + self._pause
            while rclpy.ok() and not self._stop.is_set() and waited < limit:
                time.sleep(step)
                waited += step
        if not self._loop:
            self.get_logger().info(
                'Publicacion unica hecha. El nodo sigue vivo para que RViz '
                'conserve la trayectoria (QoS TRANSIENT_LOCAL). Ctrl+C para '
                'terminar.')

    def stop(self):
        self._stop.set()


def main(args=None):
    rclpy.init(args=args)
    node = JsonPreviewNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
