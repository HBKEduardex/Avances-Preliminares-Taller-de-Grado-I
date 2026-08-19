#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
trajectory_logger_node.py

Registrador PASIVO de las trayectorias generadas por MoveIt2 para el
KUKA KR6 R900.

QUE HACE:
  Se suscribe a /display_planned_path [moveit_msgs/msg/DisplayTrajectory],
  el topico en el que el planning pipeline de move_group publica CADA plan
  que resuelve con exito. Por cada plan nuevo:

    1. Numera la trayectoria automaticamente (TRAYECTORIA 1, 2, 3, ...).
    2. Imprime en terminal sus puntos (P0 → P1 → ... → Pn).
    3. Anade una fila por punto al CSV de la sesion.

QUE NO HACE:
  - No publica en ningun topico.
  - No llama a ningun servicio ni accion.
  - No modifica el estado del robot, de MoveIt2 ni de RViz.
  - No interpola, no deriva y no inventa puntos ni valores: solo copia lo
    que realmente entrego el planificador.

UNIDADES ORIGINALES (ROS 2 / MoveIt2):
  posicion rad | velocidad rad/s | aceleracion rad/s^2 | esfuerzo N*m | tiempo s
  El CSV guarda ademas la conversion a grados, conservando siempre el original.

CIERRE:
  Ctrl+C cierra el nodo, hace flush + fsync del CSV e imprime un resumen.
  El CSV se escribe de forma incremental (una escritura por trayectoria),
  de modo que los datos ya capturados nunca se pierden.
"""

import csv
import os
from datetime import datetime

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from moveit_msgs.msg import DisplayTrajectory

from .trajectory_record import (
    build_csv_header,
    build_csv_rows,
    extract_trajectory,
    format_session_summary,
    format_trajectory_block,
    trajectory_signature,
)


# ─────────────────────────────────────────────────────────────────────────────
# Escritor CSV (un archivo por conjunto de joint names)
# ─────────────────────────────────────────────────────────────────────────────

class CsvSink:
    """
    Archivo CSV abierto en modo incremental.

    La cabecera se escribe al crear el archivo, usando los joint names reales
    de la primera trayectoria recibida. Tras cada trayectoria se hace
    flush + fsync, por lo que un Ctrl+C nunca deja el archivo a medias.
    """

    def __init__(self, path: str, header: list):
        self.path = path
        self._file = open(path, 'w', newline='', encoding='utf-8')
        self._writer = csv.writer(self._file)
        self._writer.writerow(header)
        self._sync()

    def write_rows(self, rows: list):
        self._writer.writerows(rows)
        self._sync()

    def _sync(self):
        self._file.flush()
        os.fsync(self._file.fileno())

    def close(self):
        if self._file is not None and not self._file.closed:
            self._sync()
            self._file.close()


# ─────────────────────────────────────────────────────────────────────────────
# Nodo
# ─────────────────────────────────────────────────────────────────────────────

class TrajectoryLoggerNode(Node):
    """Observador independiente de los planes publicados por MoveIt2."""

    def __init__(self):
        super().__init__('kuka_trajectory_logger_node')

        self._declare_parameters()
        self._load_parameters()

        # ── Estado de la sesion ───────────────────────────────────────
        self._trajectory_count = 0
        self._summary_rows = []            # (id, n_puntos, duracion)
        self._sinks = {}                   # tuple(joint_names) → CsvSink
        self._closed = False

        # Deteccion de republicaciones identicas
        self._last_signature = None
        self._last_signature_time = 0.0

        # Avisos que solo deben emitirse una vez
        self._warned_effort = False
        self._warned_multi_dof = False

        # ── Directorio y nombre base del CSV ──────────────────────────
        self._session_stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._output_dir = self._resolve_output_directory()
        os.makedirs(self._output_dir, exist_ok=True)

        # ── Suscripcion (unica interfaz ROS 2 que usa este nodo) ──────
        # VOLATILE a proposito: evita que una publicacion antigua retenida
        # por el middleware se registre como trayectoria nueva al arrancar.
        qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            DisplayTrajectory,
            self._trajectory_topic,
            self._display_trajectory_callback,
            qos)

        # Aviso unico si nadie publica en el topico (move_group apagado)
        self._publisher_check_timer = self.create_timer(
            5.0, self._publisher_check_callback)

        self.get_logger().info(
            '\n'
            '╔══════════════════════════════════════════════════════════╗\n'
            '║   kuka_trajectory_logger_node iniciado                   ║\n'
            '╠══════════════════════════════════════════════════════════╣\n'
            f'║  Topico:      {self._trajectory_topic:<43}║\n'
            f'║  Tipo:        {"moveit_msgs/msg/DisplayTrajectory":<43}║\n'
            f'║  Puntos:      {("si" if self._print_points else "no"):<43}║\n'
            f'║  Velocidades: {("si" if self._save_velocities else "no"):<43}║\n'
            f'║  Acelerac.:   {("si" if self._save_accelerations else "no"):<43}║\n'
            f'║  Esfuerzos:   {("si" if self._save_effort else "no"):<43}║\n'
            '╚══════════════════════════════════════════════════════════╝\n'
            f'\nDirectorio de salida: {self._output_dir}\n'
            'El CSV se crea con la primera trayectoria capturada.\n'
            'Observador pasivo: no publica, no ejecuta y no altera el sistema.\n'
            'Esperando planificaciones de MoveIt2... (Ctrl+C para terminar)\n')

    # ─────────────────────────────────────────────────────────────────
    # Parametros
    # ─────────────────────────────────────────────────────────────────

    def _declare_parameters(self):
        self.declare_parameter('trajectory_topic', '/display_planned_path')
        self.declare_parameter('output_directory', '')
        self.declare_parameter('print_points', True)
        self.declare_parameter('save_velocities', True)
        self.declare_parameter('save_accelerations', True)
        self.declare_parameter('save_effort', False)
        self.declare_parameter('duplicate_window_sec', 0.5)

    def _load_parameters(self):
        gp = self.get_parameter
        self._trajectory_topic = gp('trajectory_topic').value
        self._output_directory_param = gp('output_directory').value
        self._print_points = gp('print_points').value
        self._save_velocities = gp('save_velocities').value
        self._save_accelerations = gp('save_accelerations').value
        self._save_effort = gp('save_effort').value
        self._duplicate_window_sec = gp('duplicate_window_sec').value

    def _resolve_output_directory(self) -> str:
        """
        Directorio donde se guardan los CSV.

        - Si el parametro output_directory tiene valor, se usa tal cual.
        - Si esta vacio (por defecto), se usa <workspace>/trajectory_logs
          cuando se detecta el workspace del proyecto montado en el
          contenedor (~/taller1/ros2_ws); si no, <cwd>/trajectory_logs.
        """
        if self._output_directory_param:
            return os.path.abspath(
                os.path.expanduser(self._output_directory_param))

        workspace = os.path.join(os.path.expanduser('~'), 'taller1', 'ros2_ws')
        if os.path.isdir(workspace):
            return os.path.join(workspace, 'trajectory_logs')
        return os.path.join(os.getcwd(), 'trajectory_logs')

    # ─────────────────────────────────────────────────────────────────
    # Aviso si el topico no tiene publicadores
    # ─────────────────────────────────────────────────────────────────

    def _publisher_check_callback(self):
        """Se ejecuta una sola vez, 5 s despues del arranque."""
        self._publisher_check_timer.cancel()
        publishers = self.get_publishers_info_by_topic(self._trajectory_topic)

        if not publishers:
            self.get_logger().warn(
                f'Nadie publica todavia en {self._trajectory_topic}. '
                'Verifique que move_group (kuka_bridge_system.launch.py o '
                'demo.launch.py) este en ejecucion. El logger seguira '
                'esperando.')
            return

        names = ', '.join(info.node_name for info in publishers)
        self.get_logger().info(
            f'Publicador(es) detectado(s) en {self._trajectory_topic}: {names}')

        # Aviso de incompatibilidad de QoS: con un publicador BEST_EFFORT y
        # una suscripcion RELIABLE no se recibiria ningun mensaje.
        for info in publishers:
            if info.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT:
                self.get_logger().warn(
                    f'El publicador {info.node_name} usa QoS BEST_EFFORT y '
                    'esta suscripcion es RELIABLE: no se recibiran mensajes. '
                    'Reporte este caso para ajustar la QoS del logger.')

    # ─────────────────────────────────────────────────────────────────
    # Callback principal
    # ─────────────────────────────────────────────────────────────────

    def _display_trajectory_callback(self, msg: DisplayTrajectory):
        """
        Un mensaje = un plan resuelto por el planning pipeline de MoveIt2.

        DisplayTrajectory.trajectory es un array; el pipeline publica
        normalmente un unico elemento. Si llegasen varios, cada segmento no
        vacio se registra como una trayectoria propia (columna segment_index).
        """
        now_s = self.get_clock().now().nanoseconds * 1e-9

        for segment_index, robot_trajectory in enumerate(msg.trajectory):
            joint_trajectory = robot_trajectory.joint_trajectory

            if (robot_trajectory.multi_dof_joint_trajectory.points
                    and not self._warned_multi_dof):
                self._warned_multi_dof = True
                self.get_logger().warn(
                    'El plan contiene multi_dof_joint_trajectory. Este nodo '
                    'registra unicamente joint_trajectory (juntas del grupo '
                    '"manipulator"). Los datos multi-DOF no se guardan.')

            if not joint_trajectory.points:
                self.get_logger().warn(
                    'Plan recibido sin puntos en joint_trajectory: ignorado '
                    '(no se numera como trayectoria).')
                continue

            # ── Descartar republicaciones identicas del mismo plan ────
            signature = trajectory_signature(joint_trajectory)
            if (signature == self._last_signature and
                    (now_s - self._last_signature_time)
                    < self._duplicate_window_sec):
                self.get_logger().debug(
                    'Mensaje duplicado del mismo plan ignorado '
                    f'(ventana {self._duplicate_window_sec:.2f} s).')
                continue
            self._last_signature = signature
            self._last_signature_time = now_s

            self._trajectory_count += 1
            record = extract_trajectory(
                joint_trajectory,
                trajectory_id=self._trajectory_count,
                segment_index=segment_index,
                received_time_iso=datetime.now().isoformat(
                    sep=' ', timespec='milliseconds'),
                received_time_ros_s=now_s,
                source_topic=self._trajectory_topic,
            )

            if (record.has_efforts and not self._save_effort
                    and not self._warned_effort):
                self._warned_effort = True
                self.get_logger().warn(
                    'La trayectoria contiene esfuerzos (effort). Ejecute el '
                    'nodo con save_effort:=true para guardarlos en el CSV.')

            csv_path = self._write_record(record)

            self.get_logger().info(format_trajectory_block(
                record,
                print_points=self._print_points,
                show_velocities=self._save_velocities,
                show_accelerations=self._save_accelerations,
                show_efforts=self._save_effort,
                csv_path=csv_path))

            self._summary_rows.append(
                (record.trajectory_id, record.point_count, record.duration_s))

    # ─────────────────────────────────────────────────────────────────
    # Escritura CSV
    # ─────────────────────────────────────────────────────────────────

    def _write_record(self, record) -> str:
        """Escribe las filas de la trayectoria y devuelve la ruta del CSV."""
        key = tuple(record.joint_names)
        sink = self._sinks.get(key)

        if sink is None:
            sink = CsvSink(
                self._build_csv_path(len(self._sinks)),
                build_csv_header(
                    record.joint_names,
                    save_velocities=self._save_velocities,
                    save_accelerations=self._save_accelerations,
                    save_effort=self._save_effort))
            self._sinks[key] = sink
            if len(self._sinks) > 1:
                self.get_logger().warn(
                    'Nuevo conjunto de joint names detectado '
                    f'({", ".join(record.joint_names)}). Se abrio un CSV '
                    f'adicional para no mezclar cabeceras: {sink.path}')
            else:
                self.get_logger().info(f'CSV de la sesion: {sink.path}')

        sink.write_rows(build_csv_rows(
            record,
            save_velocities=self._save_velocities,
            save_accelerations=self._save_accelerations,
            save_effort=self._save_effort))
        return sink.path

    def _build_csv_path(self, sink_index: int) -> str:
        """moveit_trajectories_YYYYmmdd_HHMMSS.csv (nunca sobrescribe)."""
        suffix = '' if sink_index == 0 else f'_set{sink_index + 1}'
        name = f'moveit_trajectories_{self._session_stamp}{suffix}.csv'
        path = os.path.join(self._output_dir, name)

        # Salvaguarda: si el archivo ya existiese, se anade un contador.
        counter = 1
        while os.path.exists(path):
            name = (f'moveit_trajectories_{self._session_stamp}{suffix}'
                    f'_{counter}.csv')
            path = os.path.join(self._output_dir, name)
            counter += 1
        return path

    # ─────────────────────────────────────────────────────────────────
    # Cierre limpio
    # ─────────────────────────────────────────────────────────────────

    def close(self):
        """Cierra los CSV e imprime el resumen. Idempotente."""
        if self._closed:
            return
        self._closed = True

        paths = []
        for sink in self._sinks.values():
            paths.append(sink.path)
            try:
                sink.close()
            except OSError as exc:
                self.get_logger().error(f'Error cerrando {sink.path}: {exc}')

        # print() ademas del logger: al recibir SIGINT el logger puede haberse
        # cerrado antes de que se vacie la salida.
        print(format_session_summary(self._summary_rows, paths), flush=True)


# ─────────────────────────────────────────────────────────────────────────────
# Punto de entrada
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TrajectoryLoggerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
