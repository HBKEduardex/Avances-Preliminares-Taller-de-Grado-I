#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_cartesian_state.py

Pruebas para la conversión del estado cartesiano (TF) a grados.
"""

import math
import pytest
import sys
import os

sys.path.insert(
    0,
    os.path.join(os.path.dirname(__file__), '..', 'kuka_gui_moveit_bridge')
)

from transform_utils import quaternion_to_kuka_abc_deg, normalize_angle_deg

def test_normalize_angle():
    assert normalize_angle_deg(0.0) == pytest.approx(0.0, abs=1e-5)
    assert normalize_angle_deg(180.0) == pytest.approx(180.0, abs=1e-5)
    assert normalize_angle_deg(360.0) == pytest.approx(0.0, abs=1e-5)
    assert normalize_angle_deg(359.0) == pytest.approx(-1.0, abs=1e-5)
    assert normalize_angle_deg(-181.0) == pytest.approx(179.0, abs=1e-5)

def test_quaternion_to_kuka_abc_identity():
    # q = (0, 0, 0, 1) -> (0, 0, 0)
    a, b, c = quaternion_to_kuka_abc_deg(0.0, 0.0, 0.0, 1.0)
    assert a == pytest.approx(0.0, abs=1e-5)
    assert b == pytest.approx(0.0, abs=1e-5)
    assert c == pytest.approx(0.0, abs=1e-5)

def test_quaternion_to_kuka_abc_x90():
    # R(x, 90) -> q = (sin(45), 0, 0, cos(45))
    s = math.sqrt(2)/2
    a, b, c = quaternion_to_kuka_abc_deg(s, 0.0, 0.0, s)
    assert a == pytest.approx(0.0, abs=1e-5)
    assert b == pytest.approx(0.0, abs=1e-5)
    assert c == pytest.approx(90.0, abs=1e-5)

def test_quaternion_to_kuka_abc_y90():
    # R(y, 90) -> q = (0, sin(45), 0, cos(45))
    s = math.sqrt(2)/2
    a, b, c = quaternion_to_kuka_abc_deg(0.0, s, 0.0, s)
    assert a == pytest.approx(0.0, abs=1e-5)
    assert b == pytest.approx(90.0, abs=1e-5)
    assert c == pytest.approx(0.0, abs=1e-5)

def test_quaternion_to_kuka_abc_z90():
    # R(z, 90) -> q = (0, 0, sin(45), cos(45))
    s = math.sqrt(2)/2
    a, b, c = quaternion_to_kuka_abc_deg(0.0, 0.0, s, s)
    assert a == pytest.approx(90.0, abs=1e-5)
    assert b == pytest.approx(0.0, abs=1e-5)
    assert c == pytest.approx(0.0, abs=1e-5)
