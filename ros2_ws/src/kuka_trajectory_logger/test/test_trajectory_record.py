#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tests de la logica pura de kuka_trajectory_logger.

No requieren ROS 2 en ejecucion: los mensajes se simulan con objetos
equivalentes (misma estructura de campos que
trajectory_msgs/msg/JointTrajectory).

    pytest-3 test/test_trajectory_record.py
"""

import math

from kuka_trajectory_logger.trajectory_record import (
    RAD_TO_DEG,
    build_csv_header,
    build_csv_rows,
    duration_to_sec,
    extract_trajectory,
    format_trajectory_block,
    short_joint_label,
    trajectory_signature,
    value_at,
)

JOINTS = ['joint_a1', 'joint_a2', 'joint_a3',
          'joint_a4', 'joint_a5', 'joint_a6']


# ─────────────────────────────────────────────────────────────────────────────
# Mensajes simulados
# ─────────────────────────────────────────────────────────────────────────────

class FakeDuration:
    def __init__(self, sec, nanosec):
        self.sec = sec
        self.nanosec = nanosec


class FakePoint:
    def __init__(self, positions, velocities=None, accelerations=None,
                 effort=None, time_sec=0.0):
        self.positions = positions
        self.velocities = velocities if velocities is not None else []
        self.accelerations = (accelerations
                              if accelerations is not None else [])
        self.effort = effort if effort is not None else []
        sec = int(time_sec)
        self.time_from_start = FakeDuration(
            sec, int(round((time_sec - sec) * 1e9)))


class FakeJointTrajectory:
    def __init__(self, joint_names, points):
        self.joint_names = joint_names
        self.points = points


def make_trajectory():
    """Trayectoria de 3 puntos con posiciones, velocidades y aceleraciones."""
    return FakeJointTrajectory(
        JOINTS,
        [
            FakePoint([0.0] * 6, [0.0] * 6, [0.1] * 6, time_sec=0.0),
            FakePoint([0.1] * 6, [0.2] * 6, [0.0] * 6, time_sec=1.5),
            FakePoint([0.2] * 6, [0.0] * 6, [-0.1] * 6, time_sec=3.0),
        ])


def make_record(trajectory=None, trajectory_id=1):
    return extract_trajectory(
        trajectory if trajectory is not None else make_trajectory(),
        trajectory_id=trajectory_id,
        segment_index=0,
        received_time_iso='2026-08-18 18:35:00.123',
        received_time_ros_s=1755541200.123456,
        source_topic='/display_planned_path')


# ─────────────────────────────────────────────────────────────────────────────
# Helpers basicos
# ─────────────────────────────────────────────────────────────────────────────

def test_duration_to_sec():
    assert duration_to_sec(FakeDuration(2, 500000000)) == 2.5


def test_value_at_devuelve_nan_si_falta_el_campo():
    assert value_at([], 0) != value_at([], 0)          # NaN != NaN
    assert math.isnan(value_at([1.0], 3))
    assert value_at([1.0, 2.0], 1) == 2.0


def test_short_joint_label():
    assert short_joint_label('joint_a1') == 'A1'
    assert short_joint_label('shoulder_joint') == 'SHOULDER'


# ─────────────────────────────────────────────────────────────────────────────
# Extraccion de la trayectoria
# ─────────────────────────────────────────────────────────────────────────────

def test_extract_conserva_puntos_y_duracion():
    record = make_record()
    assert record.point_count == 3
    assert record.joint_names == JOINTS
    assert record.duration_s == 3.0
    assert record.has_velocities
    assert record.has_accelerations
    assert not record.has_efforts


def test_extract_no_inventa_valores_ausentes():
    """Sin velocidades ni aceleraciones en el mensaje → NaN, nunca ceros."""
    trajectory = FakeJointTrajectory(
        JOINTS, [FakePoint([0.0] * 6, time_sec=0.0)])
    record = make_record(trajectory)
    point = record.points[0]
    assert all(math.isnan(v) for v in point.velocities_rad_s)
    assert all(math.isnan(a) for a in point.accelerations_rad_s2)
    assert all(math.isnan(e) for e in point.efforts)
    assert not record.has_velocities


def test_extract_no_agrega_puntos_intermedios():
    trajectory = make_trajectory()
    record = make_record(trajectory)
    assert record.point_count == len(trajectory.points)
    assert [p.time_from_start_s for p in record.points] == [0.0, 1.5, 3.0]


# ─────────────────────────────────────────────────────────────────────────────
# Firma / duplicados
# ─────────────────────────────────────────────────────────────────────────────

def test_signature_igual_para_el_mismo_plan():
    assert trajectory_signature(make_trajectory()) == \
        trajectory_signature(make_trajectory())


def test_signature_distinta_si_cambia_un_punto():
    other = make_trajectory()
    other.points[1].positions = [0.9] * 6
    assert trajectory_signature(make_trajectory()) != \
        trajectory_signature(other)


# ─────────────────────────────────────────────────────────────────────────────
# CSV
# ─────────────────────────────────────────────────────────────────────────────

def test_header_dinamico_con_nombres_reales():
    header = build_csv_header(JOINTS)
    assert header[0] == 'trajectory_id'
    assert 'joint_a1_position_rad' in header
    assert 'joint_a1_position_deg' in header
    assert 'joint_a6_velocity_deg_s' in header
    assert 'joint_a3_acceleration_rad_s2' in header
    assert 'joint_a1_effort' not in header       # save_effort=False


def test_header_y_filas_tienen_la_misma_longitud():
    record = make_record()
    header = build_csv_header(record.joint_names)
    rows = build_csv_rows(record)
    assert len(rows) == record.point_count
    assert all(len(row) == len(header) for row in rows)


def test_conversion_a_grados_conserva_el_valor_original():
    record = make_record()
    header = build_csv_header(record.joint_names)
    row = build_csv_rows(record)[1]
    i_rad = header.index('joint_a1_position_rad')
    i_deg = header.index('joint_a1_position_deg')
    assert row[i_rad] == 0.1
    assert abs(row[i_deg] - 0.1 * RAD_TO_DEG) < 1e-9


def test_columnas_opcionales_desactivadas():
    record = make_record()
    header = build_csv_header(record.joint_names,
                              save_velocities=False,
                              save_accelerations=False)
    rows = build_csv_rows(record,
                          save_velocities=False,
                          save_accelerations=False)
    assert 'joint_a1_velocity_rad_s' not in header
    assert all(len(row) == len(header) for row in rows)


# ─────────────────────────────────────────────────────────────────────────────
# Terminal
# ─────────────────────────────────────────────────────────────────────────────

def test_bloque_de_terminal_contiene_lo_esperado():
    text = format_trajectory_block(make_record(trajectory_id=7),
                                   csv_path='/tmp/x.csv')
    assert 'TRAYECTORIA 7' in text
    assert 'Numero de puntos:   3' in text
    assert 'joint_a1' in text
    assert 'Punto 0' in text
    assert 'Punto 2 (punto final)' in text
    assert '/tmp/x.csv' in text


def test_bloque_sin_puntos_cuando_print_points_es_false():
    text = format_trajectory_block(make_record(), print_points=False)
    assert 'TRAYECTORIA 1' in text
    assert 'Punto 0' not in text
