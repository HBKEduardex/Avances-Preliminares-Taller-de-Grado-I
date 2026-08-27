#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
continuity_analyzer_node.py

Analizador de continuidad articular de la condicion MOVEIT2 BASE.

Escucha las trayectorias que el pipeline de MoveIt2 publica al resolver un
plan, calcula las metricas de discontinuidad y de proximidad a singularidad, e
imprime un resumen por terminal y un CSV con marca de tiempo.

DOS MODOS DE ENTRADA:

  MODO TOPICO (por defecto, comportamiento historico)
    entrada : /display_planned_path   (moveit_msgs/msg/DisplayTrajectory)
    salida  : terminal + CSV baseline_continuity_*.csv

  MODO ARCHIVO (opcional, NO se activa por defecto)
    entrada : un JSON de secuencia externo (parametro input_json)
    salida  : terminal + CSV afinada_externa_continuity_*.csv

    ADVERTENCIA: el JSON lo genero el SISTEMA AFINADO en TG2. Las metricas
    que salen de ahi caracterizan la CONDICION AFINADA, no la base. Por eso
    el CSV, su nombre y la consola llevan condition = AFINADA/ORIGEN EXTERNO.
    El archivo se abre en SOLO LECTURA.

  salida  : analysis_output/ dentro de este paquete, en ambos modos

Es un OBSERVADOR PASIVO:
  - no publica en ningun topico;
  - no tiene clientes de accion ni de servicio;
  - no planifica, no ejecuta y no modifica la trayectoria;
  - no puede comunicarse con el robot fisico por ninguna via.

Este nodo es PROPIO del paquete baseline. kuka_trajectory_logger sirvio de
referencia de implementacion, pero NO se importa, NO se modifica y NO se
depende de el: el baseline debe quedar congelado (propiedad 1c del encargo).
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

from .continuity_metrics import (
    DECLARED_TIP,
    JOINT_NAMES,
    Waypoint,
    analyze,
    csv_header,
    csv_rows,
    format_report,
    reorder_to_canonical,
)
from .sequence_analysis import (
    CONDITION_RECORDED,
    analyze_sequence,
)
from .trajectory_json_reader import JsonTrajectoryError
from .trajectory_json_reader import load as load_json_sequence


def duration_to_sec(duration_msg) -> float:
    """builtin_interfaces/msg/Duration -> segundos."""
    return float(duration_msg.sec) + float(duration_msg.nanosec) * 1e-9


def default_output_directory() -> str:
    """
    analysis_output/ DENTRO de este paquete.

    Se resuelve desde la ruta real de este archivo. Con --symlink-install el
    modulo instalado es un enlace al fuente, asi que realpath cae en
    src/kuka_kr6_moveit_baseline/ y el CSV queda junto al codigo, visible
    desde el host. Sin symlink cae en el share instalado.

    En ningun caso se escribe fuera del paquete.
    """
    here = os.path.dirname(os.path.realpath(__file__))
    package_root = os.path.dirname(here)
    return os.path.join(package_root, 'analysis_output')


class ContinuityAnalyzerNode(Node):
    """Observador pasivo de los planes de MoveIt2."""

    def __init__(self):
        super().__init__('kr6_baseline_continuity_analyzer')

        self.declare_parameter('trajectory_topic', '/display_planned_path')
        self.declare_parameter('output_directory', '')
        self.declare_parameter('write_csv', True)
        self.declare_parameter('print_report', True)
        self.declare_parameter('duplicate_window_sec', 0.5)
        # ── MODO ARCHIVO (J.3). Vacio = DESACTIVADO. El comportamiento por
        #    defecto del nodo y del launch NO cambia.
        self.declare_parameter('input_json', '')

        gp = self.get_parameter
        self._topic = gp('trajectory_topic').value
        self._write_csv = bool(gp('write_csv').value)
        self._print_report = bool(gp('print_report').value)
        self._duplicate_window = float(gp('duplicate_window_sec').value)
        self._input_json = str(gp('input_json').value or '').strip()

        configured = gp('output_directory').value
        self._output_dir = configured or default_output_directory()
        if self._write_csv:
            os.makedirs(self._output_dir, exist_ok=True)

        self._session_stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._count = 0
        self._last_signature = None
        self._last_signature_time = 0.0
        # Estado del modo archivo: el nodo termina en vez de girar.
        self._file_mode_done = False
        self._file_mode_failed = False

        if self._input_json:
            # MODO ARCHIVO: no se suscribe a nada. Analiza y termina.
            self._run_file_mode()
            return

        qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            DisplayTrajectory, self._topic, self._callback, qos)

        self.get_logger().info(
            '\n'
            '╔══════════════════════════════════════════════════════════════╗\n'
            '║  ANALIZADOR DE CONTINUIDAD — CONDICION MOVEIT2 BASE          ║\n'
            '╠══════════════════════════════════════════════════════════════╣\n'
            f'║  Modo   : {"TOPICO (en vivo)":<50}║\n'
            f'║  Topico : {self._topic:<50}║\n'
            f'║  CSV    : {("si" if self._write_csv else "no"):<50}║\n'
            f'║  Salida : {self._output_dir[:50]:<50}║\n'
            '╚══════════════════════════════════════════════════════════════╝\n'
            'Observador pasivo: no publica, no ejecuta y no toca el robot.\n'
            'Esperando planificaciones de MoveIt2... (Ctrl+C para terminar)\n')

    @property
    def file_mode_finished(self) -> bool:
        return self._file_mode_done or self._file_mode_failed

    @property
    def file_mode_failed(self) -> bool:
        return self._file_mode_failed

    # ── MODO ARCHIVO ─────────────────────────────────────────────────────

    def _run_file_mode(self) -> None:
        """
        Analiza una secuencia grabada en JSON. SOLO LECTURA sobre el archivo.

        Todo lo que sale de aqui se etiqueta AFINADA/ORIGEN EXTERNO: la
        trayectoria NO la genero el baseline.
        """
        try:
            sequence = load_json_sequence(self._input_json)
        except (JsonTrajectoryError, OSError, ValueError) as exc:
            self.get_logger().error(
                '\n╔══════════════════════════════════════════════════════════╗'
                '\n║  MODO ARCHIVO ABORTADO                                   ║'
                '\n╚══════════════════════════════════════════════════════════╝'
                f'\n{exc}\n'
                'No se produce ningun CSV. Corrige el archivo o la ruta.')
            self._file_mode_failed = True
            return

        banner = self._provenance_block(sequence)
        self.get_logger().info(
            '\n'
            '╔══════════════════════════════════════════════════════════════╗\n'
            '║  ANALIZADOR DE CONTINUIDAD — MODO ARCHIVO                    ║\n'
            '╠══════════════════════════════════════════════════════════════╣\n'
            '║  *** ATENCION: PROCEDENCIA EXTERNA ***                       ║\n'
            '║  Esta trayectoria NO fue generada por la condicion base.     ║\n'
            f'║  condition = {CONDITION_RECORDED:<48}║\n'
            '╠══════════════════════════════════════════════════════════════╣\n'
            f'║  Archivo : {os.path.basename(sequence.path)[:49]:<49}║\n'
            f'║  md5     : {sequence.md5:<49}║\n'
            f'║  source  : {sequence.source[:49]:<49}║\n'
            f'║  fecha   : {sequence.generated_at[:49]:<49}║\n'
            f'║  tip     : {DECLARED_TIP:<49}║\n'
            '╚══════════════════════════════════════════════════════════════╝\n'
            f'{banner}')

        metadata = {
            'source_file': sequence.path,
            'source_md5': sequence.md5,
            'declared_tip': DECLARED_TIP,
        }
        result = analyze_sequence(
            [s.waypoints for s in sequence.segments],
            [s.id for s in sequence.segments],
            condition=CONDITION_RECORDED,
            metadata=metadata)

        csv_path = ''
        if self._write_csv:
            csv_path = self._write_sequence(result, sequence)
        if self._print_report:
            for item in result.segments:
                self.get_logger().info(format_report(item))
        self.get_logger().info(
            f'\nMODO ARCHIVO COMPLETADO — {len(result.segments)} segmentos, '
            f'{result.total_waypoints} waypoints.\n'
            f'CSV: {csv_path or "(no escrito)"}\n'
            f'RECORDATORIO: condition = {CONDITION_RECORDED}. '
            'Estas metricas caracterizan la condicion AFINADA, no la base.')
        self._file_mode_done = True

    @staticmethod
    def _provenance_block(sequence) -> str:
        pm = sequence.planner_metadata
        lines = ['CONTROL DE INTEGRIDAD DEL ARCHIVO']
        for check in sequence.checks:
            lines.append(f'  - {check}')
        lines.append('')
        lines.append('HUELLA DEL SISTEMA AFINADO (planner_metadata del archivo)')
        for key in ('velocity_scaling', 'acceleration_scaling',
                    'planning_attempts', 'allowed_planning_time_sec',
                    'joint_goal_tolerance_rad', 'segment_chaining'):
            if key in pm:
                lines.append(f'  - {key:26s} = {pm[key]}')
        lines.append('')
        lines.append('MAPEO DE SEGMENTOS (from_point/to_point vienen a null; '
                     'se resuelve POR INDICE)')
        lines.append(f'  desviacion maxima extremos vs source_points: '
                     f'{sequence.max_endpoint_deviation_deg:.6f} deg')
        lines.append('  (coincide con joint_goal_tolerance_rad = 0.2 deg del '
                     'sistema afinado; es su tolerancia, no ruido)')
        if sequence.limit_violations:
            lines.append('')
            lines.append('*** VIOLACIONES DE LIMITES DEL URDF BASELINE ***')
            for v in sequence.limit_violations[:20]:
                lines.append(f'    {v}')
        lines.append('')
        lines.append('EVENTOS DE GRIPPER (metadato de la tarea; NO se actuan)')
        lines.append(f'  estado inicial: {sequence.gripper_initial_state}')
        for event in sequence.gripper_events:
            lines.append(f'  - {event}')
        lines.append('  El baseline NO controla el gripper. La comparacion es '
                     'de trayectorias articulares, no de la tarea completa.')
        return '\n'.join(lines)

    def _write_sequence(self, result, sequence) -> str:
        """Un unico CSV con los 14 segmentos concatenados."""
        stem = os.path.splitext(os.path.basename(sequence.path))[0]
        name = (f'afinada_externa_continuity_{self._session_stamp}'
                f'_{stem}.csv')
        path = os.path.join(self._output_dir, name)
        try:
            with open(path, 'w', newline='') as handle:
                writer = csv.writer(handle)
                writer.writerow(csv_header())
                for item in result.segments:
                    writer.writerows(csv_rows(item))
        except OSError as exc:
            self.get_logger().error(f'No se pudo escribir el CSV: {exc}')
            return ''
        return path

    # ─────────────────────────────────────────────────────────────────

    def _callback(self, msg: DisplayTrajectory):
        if not msg.trajectory:
            return
        joint_trajectory = msg.trajectory[0].joint_trajectory
        if not joint_trajectory.points:
            return

        # Descartar republicaciones identicas del MISMO plan.
        signature = (
            tuple(joint_trajectory.joint_names),
            len(joint_trajectory.points),
            tuple(round(v, 6) for v in joint_trajectory.points[-1].positions),
        )
        now = self.get_clock().now().nanoseconds * 1e-9
        same_plan = signature == self._last_signature
        within_window = now - self._last_signature_time < self._duplicate_window
        if same_plan and within_window:
            return
        self._last_signature = signature
        self._last_signature_time = now

        waypoints = []
        for i, point in enumerate(joint_trajectory.points):
            positions = reorder_to_canonical(
                joint_trajectory.joint_names, point.positions)
            if positions is None:
                self.get_logger().warn(
                    'La trayectoria no contiene los seis joints del grupo '
                    f'{JOINT_NAMES}; se ignora.')
                return
            waypoints.append(Waypoint(
                index=i,
                time_from_start_s=duration_to_sec(point.time_from_start),
                positions_rad=positions,
                velocities_rad_s=(reorder_to_canonical(
                    joint_trajectory.joint_names, point.velocities) or []),
                accelerations_rad_s2=(reorder_to_canonical(
                    joint_trajectory.joint_names, point.accelerations) or []),
            ))

        self._count += 1
        analysis = analyze(waypoints, trajectory_id=self._count)

        csv_path = ''
        if self._write_csv:
            csv_path = self._write(analysis)
        if self._print_report:
            self.get_logger().info(format_report(analysis, csv_path))

    def _write(self, analysis) -> str:
        name = (f'baseline_continuity_{self._session_stamp}'
                f'_traj{analysis.trajectory_id:03d}.csv')
        path = os.path.join(self._output_dir, name)
        try:
            with open(path, 'w', newline='') as handle:
                writer = csv.writer(handle)
                writer.writerow(csv_header())
                writer.writerows(csv_rows(analysis))
        except OSError as exc:
            self.get_logger().error(f'No se pudo escribir el CSV: {exc}')
            return ''
        return path


def main(args=None):
    rclpy.init(args=args)
    node = ContinuityAnalyzerNode()
    exit_code = 0
    try:
        if node.file_mode_finished:
            # MODO ARCHIVO: el trabajo ya esta hecho en el constructor.
            # No hay nada que escuchar, asi que no se gira.
            exit_code = 1 if node.file_mode_failed else 0
        else:
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    main()
