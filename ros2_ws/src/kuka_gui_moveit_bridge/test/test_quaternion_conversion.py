#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_quaternion_conversion.py
"""

import math
import pytest
import sys
import os

sys.path.insert(
    0,
    os.path.join(os.path.dirname(__file__), '..', 'kuka_gui_moveit_bridge')
)

from transform_utils import kuka_abc_deg_to_quaternion, quaternion_to_kuka_abc_deg, normalize_quaternion

def quat_norm(q):
    x, y, z, w = q
    return math.sqrt(x*x + y*y + z*z + w*w)

def assert_quaternion_approx(q_got, q_exp, tol=1e-6, label=''):
    x, y, z, w = q_got
    xe, ye, ze, we = q_exp
    same_sign = (
        abs(x - xe) < tol and abs(y - ye) < tol and
        abs(z - ze) < tol and abs(w - we) < tol
    )
    opp_sign = (
        abs(x + xe) < tol and abs(y + ye) < tol and
        abs(z + ze) < tol and abs(w + we) < tol
    )
    assert same_sign or opp_sign

def test_identity_rotation():
    q = kuka_abc_deg_to_quaternion(0.0, 0.0, 0.0)
    assert_quaternion_approx(q, (0.0, 0.0, 0.0, 1.0))
    a, b, c = quaternion_to_kuka_abc_deg(*q)
    assert a == pytest.approx(0.0, abs=1e-5)
    assert b == pytest.approx(0.0, abs=1e-5)
    assert c == pytest.approx(0.0, abs=1e-5)

def test_rotation_z_180():
    q = kuka_abc_deg_to_quaternion(180.0, 0.0, 0.0)
    assert_quaternion_approx(q, (0.0, 0.0, 1.0, 0.0))

def test_rotation_y_180():
    q = kuka_abc_deg_to_quaternion(0.0, 180.0, 0.0)
    assert_quaternion_approx(q, (0.0, 1.0, 0.0, 0.0))

def test_rotation_x_180():
    q = kuka_abc_deg_to_quaternion(0.0, 0.0, 180.0)
    assert_quaternion_approx(q, (1.0, 0.0, 0.0, 0.0))

def test_rotation_z_90():
    q = kuka_abc_deg_to_quaternion(90.0, 0.0, 0.0)
    s = math.sqrt(2.0) / 2.0
    assert_quaternion_approx(q, (0.0, 0.0, s, s))

def test_rotation_y_90():
    q = kuka_abc_deg_to_quaternion(0.0, 90.0, 0.0)
    s = math.sqrt(2.0) / 2.0
    assert_quaternion_approx(q, (0.0, s, 0.0, s))

def test_rotation_x_90():
    q = kuka_abc_deg_to_quaternion(0.0, 0.0, 90.0)
    s = math.sqrt(2.0) / 2.0
    assert_quaternion_approx(q, (s, 0.0, 0.0, s))

def test_real_project_case():
    q = kuka_abc_deg_to_quaternion(0.0, -90.0, 180.0)
    assert quat_norm(q) == pytest.approx(1.0, abs=1e-6)

def test_quaternion_always_normalized():
    test_cases = [
        (0.0, 0.0, 0.0),
        (45.0, 30.0, 15.0),
        (90.0, -45.0, 120.0),
        (180.0, 180.0, 180.0),
        (-90.0, 0.0, 0.0),
        (0.0, -90.0, 180.0),
    ]
    for a, b, c in test_cases:
        q = kuka_abc_deg_to_quaternion(a, b, c)
        assert quat_norm(q) == pytest.approx(1.0, abs=1e-6)

def test_normalize_quaternion():
    q = normalize_quaternion(1.0, 1.0, 1.0, 1.0)
    assert quat_norm(q) == pytest.approx(1.0, abs=1e-6)

def test_quaternion_to_kuka_abc():
    q = (0.0, 0.0, 0.0, 1.0)
    a, b, c = quaternion_to_kuka_abc_deg(*q)
    assert a == pytest.approx(0.0, abs=1e-5)
    assert b == pytest.approx(0.0, abs=1e-5)
    assert c == pytest.approx(0.0, abs=1e-5)
