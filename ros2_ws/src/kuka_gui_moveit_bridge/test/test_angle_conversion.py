#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_angle_conversion.py

Pruebas unitarias para las funciones de conversion de angulos.
"""

import math
import pytest
import sys
import os

sys.path.insert(
    0,
    os.path.join(os.path.dirname(__file__), '..', 'kuka_gui_moveit_bridge')
)

from transform_utils import deg_to_rad, rad_to_deg, validate_float_array


def test_deg_to_rad_zero():
    assert deg_to_rad(0.0) == pytest.approx(0.0, abs=1e-9)

def test_deg_to_rad_180():
    assert deg_to_rad(180.0) == pytest.approx(math.pi, abs=1e-9)

def test_deg_to_rad_minus_90():
    assert deg_to_rad(-90.0) == pytest.approx(-math.pi / 2.0, abs=1e-9)

def test_deg_to_rad_360():
    assert deg_to_rad(360.0) == pytest.approx(2.0 * math.pi, abs=1e-9)

def test_deg_to_rad_45():
    assert deg_to_rad(45.0) == pytest.approx(math.pi / 4.0, abs=1e-9)

def test_rad_to_deg_pi():
    assert rad_to_deg(math.pi) == pytest.approx(180.0, abs=1e-9)

def test_rad_to_deg_zero():
    assert rad_to_deg(0.0) == pytest.approx(0.0, abs=1e-9)

def test_rad_to_deg_half_pi():
    assert rad_to_deg(math.pi / 2.0) == pytest.approx(90.0, abs=1e-9)

def test_rad_to_deg_minus_pi_over_2():
    assert rad_to_deg(-math.pi / 2.0) == pytest.approx(-90.0, abs=1e-9)

def test_roundtrip_deg_rad():
    for angle in [-180.0, -90.0, -45.0, 0.0, 45.0, 90.0, 180.0, 360.0]:
        result = rad_to_deg(deg_to_rad(angle))
        assert result == pytest.approx(angle, abs=1e-9)

def test_tolerance_12_deg_to_rad():
    result = deg_to_rad(12.0)
    expected = 12.0 * math.pi / 180.0
    assert result == pytest.approx(expected, abs=1e-9)

def test_tolerance_1_deg_to_rad():
    result = deg_to_rad(1.0)
    expected = math.pi / 180.0
    assert result == pytest.approx(expected, abs=1e-9)

def test_validate_array_correct_size():
    ok, err = validate_float_array([0.0, -30.0, 60.0, 0.0, 30.0, 0.0], 6)
    assert ok is True

def test_validate_array_wrong_size_less():
    ok, err = validate_float_array([0.0, -30.0, 60.0], 6)
    assert ok is False

def test_validate_array_wrong_size_more():
    ok, err = validate_float_array([0.0] * 7, 6)
    assert ok is False

def test_validate_array_empty():
    ok, err = validate_float_array([], 6)
    assert ok is False

def test_validate_array_nan():
    ok, err = validate_float_array([0.0, float('nan'), 0.0, 0.0, 0.0, 0.0], 6)
    assert ok is False

def test_validate_array_inf():
    ok, err = validate_float_array([0.0, float('inf'), 0.0, 0.0, 0.0, 0.0], 6)
    assert ok is False

def test_validate_array_negative_inf():
    ok, err = validate_float_array([float('-inf'), 0.0, 0.0, 0.0, 0.0, 0.0], 6)
    assert ok is False

def test_joint_state_reorder_and_convert():
    joint_names = ['joint_a1', 'joint_a2', 'joint_a3',
                   'joint_a4', 'joint_a5', 'joint_a6']
    cache = {
        'joint_a6': math.pi,
        'joint_a3': math.pi / 2.0,
        'joint_a1': 0.0,
        'joint_a4': -math.pi / 2.0,
        'joint_a2': math.pi / 4.0,
        'joint_a5': -math.pi,
    }
    positions_rad = [cache[n] for n in joint_names]
    positions_deg = [rad_to_deg(r) for r in positions_rad]
    expected_deg = [0.0, 45.0, 90.0, -90.0, -180.0, 180.0]
    for i, (got, exp) in enumerate(zip(positions_deg, expected_deg)):
        assert got == pytest.approx(exp, abs=1e-6)

def test_joint_state_missing_joint_defaults_zero():
    joint_names = ['joint_a1', 'joint_a2', 'joint_a3',
                   'joint_a4', 'joint_a5', 'joint_a6']
    cache = {}
    positions_rad = [cache.get(n, 0.0) for n in joint_names]
    positions_deg = [rad_to_deg(r) for r in positions_rad]
    assert all(v == pytest.approx(0.0, abs=1e-9) for v in positions_deg)
