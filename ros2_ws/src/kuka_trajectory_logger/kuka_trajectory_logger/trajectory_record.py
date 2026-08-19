#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
trajectory_record.py

Logica pura (sin ROS) del paquete kuka_trajectory_logger.

Responsabilidades:
  - Convertir un trajectory_msgs/msg/JointTrajectory en una estructura de datos
    propia (TrajectoryRecord) SIN interpolar, sin recalcular y sin inventar
    valores.
  - Construir la cabecera y las filas del CSV de forma dinamica a partir de los
    joint names realmente recibidos en el mensaje.
  - Construir el bloque de texto que se imprime en terminal.
  - Construir una firma (signature) que permite detectar republicaciones
    identicas del mismo plan.

REGLA DE UNIDADES (unidades originales de ROS 2 / MoveIt2):
  - posiciones      : rad     (juntas rotacionales)
  - velocidades     : rad/s
  - aceleraciones   : rad/s^2
  - esfuerzos       : N*m     (juntas rotacionales)
  - tiempos         : s

Las columnas *_deg, *_deg_s y *_deg_s2 son una CONVERSION ADICIONAL de
conveniencia. Los valores originales en radianes SIEMPRE se conservan.

Si un campo no viene en el mensaje (por ejemplo effort), se registra NaN.
Nunca se calcula ni se estima un valor ausente.
"""

import math
from dataclasses import dataclass, field
from typing import List, Sequence, Tuple

# ─────────────────────────────────────────────────────────────────────────────
# Constantes
# ─────────────────────────────────────────────────────────────────────────────

RAD_TO_DEG = 180.0 / math.pi
NAN = float('nan')


# ─────────────────────────────────────────────────────────────────────────────
# Helpers de conversion
# ─────────────────────────────────────────────────────────────────────────────

def duration_to_sec(duration_msg) -> float:
    """Convierte builtin_interfaces/msg/Duration a segundos (float)."""
    return float(duration_msg.sec) + float(duration_msg.nanosec) * 1e-9


def rad_to_deg(value: float) -> float:
    """rad → deg. NaN se propaga como NaN (no se inventa un valor)."""
    return value * RAD_TO_DEG


def value_at(values: Sequence, index: int) -> float:
    """
    Devuelve values[index] como float, o NaN si el campo no existe
    o es mas corto que el numero de joints.

    MoveIt2 puede publicar arrays vacios (por ejemplo effort). En ese caso
    el dato NO existe y se registra NaN.
    """
    if values is None:
        return NAN
    if index >= len(values):
        return NAN
    return float(values[index])


def short_joint_label(joint_name: str) -> str:
    """
    Etiqueta corta para mostrar en terminal: 'joint_a1' → 'A1'.

    Solo afecta a la impresion en pantalla. En el CSV se usa SIEMPRE el
    nombre real del joint tal y como viene en el mensaje.
    """
    label = joint_name
    if label.startswith('joint_'):
        label = label[len('joint_'):]
    if label.endswith('_joint'):
        label = label[:-len('_joint')]
    return label.upper() if label else joint_name


# ─────────────────────────────────────────────────────────────────────────────
# Estructuras de datos
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class TrajectoryPointRecord:
    """Un punto de la trayectoria, exactamente como lo entrego el planificador."""

    index: int
    time_from_start_s: float
    positions_rad: List[float] = field(default_factory=list)
    velocities_rad_s: List[float] = field(default_factory=list)
    accelerations_rad_s2: List[float] = field(default_factory=list)
    efforts: List[float] = field(default_factory=list)


@dataclass
class TrajectoryRecord:
    """Una trayectoria completa generada por MoveIt2."""

    trajectory_id: int
    segment_index: int
    received_time_iso: str
    received_time_ros_s: float
    joint_names: List[str]
    points: List[TrajectoryPointRecord]
    source_topic: str = ''

    @property
    def point_count(self) -> int:
        return len(self.points)

    @property
    def duration_s(self) -> float:
        """time_from_start del ultimo punto (duracion planificada)."""
        if not self.points:
            return NAN
        return self.points[-1].time_from_start_s

    @property
    def has_velocities(self) -> bool:
        return any(
            any(not math.isnan(v) for v in p.velocities_rad_s)
            for p in self.points)

    @property
    def has_accelerations(self) -> bool:
        return any(
            any(not math.isnan(a) for a in p.accelerations_rad_s2)
            for p in self.points)

    @property
    def has_efforts(self) -> bool:
        return any(
            any(not math.isnan(e) for e in p.efforts)
            for p in self.points)


# ─────────────────────────────────────────────────────────────────────────────
# Extraccion desde el mensaje ROS
# ─────────────────────────────────────────────────────────────────────────────

def extract_trajectory(joint_trajectory,
                       trajectory_id: int,
                       segment_index: int,
                       received_time_iso: str,
                       received_time_ros_s: float,
                       source_topic: str = '') -> TrajectoryRecord:
    """
    Convierte un trajectory_msgs/msg/JointTrajectory en un TrajectoryRecord.

    Copia literal de los datos del mensaje. No interpola, no deriva y no
    completa campos ausentes: lo que no viene, queda como NaN.
    """
    joint_names = list(joint_trajectory.joint_names)
    n_joints = len(joint_names)

    points: List[TrajectoryPointRecord] = []
    for i, point in enumerate(joint_trajectory.points):
        points.append(TrajectoryPointRecord(
            index=i,
            time_from_start_s=duration_to_sec(point.time_from_start),
            positions_rad=[value_at(point.positions, j) for j in range(n_joints)],
            velocities_rad_s=[value_at(point.velocities, j) for j in range(n_joints)],
            accelerations_rad_s2=[
                value_at(point.accelerations, j) for j in range(n_joints)],
            efforts=[value_at(point.effort, j) for j in range(n_joints)],
        ))

    return TrajectoryRecord(
        trajectory_id=trajectory_id,
        segment_index=segment_index,
        received_time_iso=received_time_iso,
        received_time_ros_s=received_time_ros_s,
        joint_names=joint_names,
        points=points,
        source_topic=source_topic,
    )


def trajectory_signature(joint_trajectory, decimals: int = 6) -> Tuple:
    """
    Firma del contenido de una trayectoria.

    Se usa unicamente para descartar republicaciones identicas del MISMO plan
    (mensajes repetidos). Dos planificaciones distintas producen firmas
    distintas porque OMPL/RRTConnect no genera dos veces exactamente los
    mismos puntos con los mismos tiempos.
    """
    parts: List = [tuple(joint_trajectory.joint_names),
                   len(joint_trajectory.points)]
    for point in joint_trajectory.points:
        parts.append((
            round(duration_to_sec(point.time_from_start), 6),
            tuple(round(float(v), decimals) for v in point.positions),
        ))
    return tuple(parts)


# ─────────────────────────────────────────────────────────────────────────────
# CSV
# ─────────────────────────────────────────────────────────────────────────────

BASE_CSV_COLUMNS = [
    'trajectory_id',
    'trajectory_received_time_iso',
    'trajectory_received_time_ros_s',
    'segment_index',
    'point_index',
    'point_count',
    'time_from_start_s',
    'trajectory_duration_s',
]


def build_csv_header(joint_names: Sequence[str],
                     save_velocities: bool = True,
                     save_accelerations: bool = True,
                     save_effort: bool = False) -> List[str]:
    """
    Cabecera CSV construida dinamicamente con los joint names reales.

    Para cada joint se generan (segun parametros):
        <joint>_position_rad        <joint>_position_deg
        <joint>_velocity_rad_s      <joint>_velocity_deg_s
        <joint>_acceleration_rad_s2 <joint>_acceleration_deg_s2
        <joint>_effort
    """
    header = list(BASE_CSV_COLUMNS)
    for name in joint_names:
        header.append(f'{name}_position_rad')
        header.append(f'{name}_position_deg')
        if save_velocities:
            header.append(f'{name}_velocity_rad_s')
            header.append(f'{name}_velocity_deg_s')
        if save_accelerations:
            header.append(f'{name}_acceleration_rad_s2')
            header.append(f'{name}_acceleration_deg_s2')
        if save_effort:
            header.append(f'{name}_effort')
    return header


def build_csv_rows(record: TrajectoryRecord,
                   save_velocities: bool = True,
                   save_accelerations: bool = True,
                   save_effort: bool = False) -> List[List]:
    """Una fila por punto de trayectoria, en el orden de build_csv_header()."""
    rows: List[List] = []
    duration = record.duration_s
    n_joints = len(record.joint_names)

    for point in record.points:
        row: List = [
            record.trajectory_id,
            record.received_time_iso,
            record.received_time_ros_s,
            record.segment_index,
            point.index,
            record.point_count,
            point.time_from_start_s,
            duration,
        ]
        for j in range(n_joints):
            pos = point.positions_rad[j]
            row.append(pos)
            row.append(rad_to_deg(pos))
            if save_velocities:
                vel = point.velocities_rad_s[j]
                row.append(vel)
                row.append(rad_to_deg(vel))
            if save_accelerations:
                acc = point.accelerations_rad_s2[j]
                row.append(acc)
                row.append(rad_to_deg(acc))
            if save_effort:
                row.append(point.efforts[j])
        rows.append(row)

    return rows


# ─────────────────────────────────────────────────────────────────────────────
# Salida por terminal
# ─────────────────────────────────────────────────────────────────────────────

SEPARATOR = '=' * 70


def _format_value_pair(value_rad: float, unit_rad: str, unit_deg: str) -> str:
    """'0.123456 rad = 7.0735 °' o 'sin dato' si es NaN."""
    if math.isnan(value_rad):
        return f'{"sin dato":>24}'
    return (f'{value_rad:>12.6f} {unit_rad} = '
            f'{rad_to_deg(value_rad):>10.4f} {unit_deg}')


def format_trajectory_block(record: TrajectoryRecord,
                            print_points: bool = True,
                            show_velocities: bool = True,
                            show_accelerations: bool = True,
                            show_efforts: bool = False,
                            csv_path: str = '') -> str:
    """Bloque de texto que se imprime en terminal por cada trayectoria."""
    lines: List[str] = []
    lines.append('')
    lines.append(SEPARATOR)
    lines.append(f'TRAYECTORIA {record.trajectory_id}')
    lines.append(SEPARATOR)
    lines.append('')
    lines.append(f'Fuente:             {record.source_topic} '
                 f'[moveit_msgs/msg/DisplayTrajectory]')
    lines.append(f'Recibida:           {record.received_time_iso}')
    lines.append(f'Tiempo ROS:         {record.received_time_ros_s:.6f} s')
    if record.segment_index > 0:
        lines.append(f'Segmento del msg:   {record.segment_index}')
    lines.append(f'Numero de puntos:   {record.point_count}')
    lines.append(f'Duracion total:     {record.duration_s:.3f} s')
    lines.append(f'Velocidades:        '
                 f'{"si" if record.has_velocities else "no (NaN)"}')
    lines.append(f'Aceleraciones:      '
                 f'{"si" if record.has_accelerations else "no (NaN)"}')
    lines.append(f'Esfuerzos:          '
                 f'{"si" if record.has_efforts else "no (NaN)"}')
    lines.append('')
    lines.append('Joint names:')
    for name in record.joint_names:
        lines.append(f'  {name}')

    if print_points:
        for point in record.points:
            lines.append('')
            if point.index == record.point_count - 1:
                lines.append(f'Punto {point.index} (punto final)')
            else:
                lines.append(f'Punto {point.index}')
            lines.append(f't = {point.time_from_start_s:.3f} s')
            for j, name in enumerate(record.joint_names):
                label = short_joint_label(name)
                segment = (f'  {label:<4} = '
                           f'{_format_value_pair(point.positions_rad[j], "rad", "°")}')
                if show_velocities:
                    segment += ('   v = ' + _format_value_pair(
                        point.velocities_rad_s[j], 'rad/s', '°/s'))
                if show_accelerations:
                    segment += ('   a = ' + _format_value_pair(
                        point.accelerations_rad_s2[j], 'rad/s²', '°/s²'))
                if show_efforts:
                    effort = point.efforts[j]
                    segment += ('   e = sin dato' if math.isnan(effort)
                                else f'   e = {effort:>12.6f} N·m')
                lines.append(segment)

    lines.append('')
    if csv_path:
        lines.append(f'CSV: {csv_path}  (+{record.point_count} filas)')
    lines.append(SEPARATOR)
    return '\n'.join(lines)


def format_session_summary(summary_rows: Sequence[Tuple[int, int, float]],
                           csv_paths: Sequence[str]) -> str:
    """
    Resumen final que se imprime al cerrar el nodo con Ctrl+C.

    summary_rows: secuencia de (trajectory_id, point_count, duration_s).
    """
    lines: List[str] = []
    lines.append('')
    lines.append(SEPARATOR)
    lines.append('RESUMEN DE LA SESION')
    lines.append(SEPARATOR)

    if not summary_rows:
        lines.append('')
        lines.append('No se capturo ninguna trayectoria.')
        lines.append(SEPARATOR)
        return '\n'.join(lines)

    lines.append('')
    total_points = 0
    for traj_id, point_count, duration in summary_rows:
        total_points += point_count
        lines.append(f'Trayectoria {traj_id:<4} {point_count:>5} puntos'
                     f'    duracion {duration:.3f} s')
    lines.append('')
    lines.append(f'Total: {len(summary_rows)} trayectorias, '
                 f'{total_points} puntos.')
    lines.append('')
    for path in csv_paths:
        lines.append(f'CSV: {path}')
    lines.append(SEPARATOR)
    return '\n'.join(lines)
