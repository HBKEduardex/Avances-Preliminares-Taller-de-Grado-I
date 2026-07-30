#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_target_comparison.py

Pruebas de la lógica ALREADY_AT_TARGET.
"""

import math
import pytest
import sys
import os

sys.path.insert(
    0,
    os.path.join(os.path.dirname(__file__), '..', 'kuka_gui_moveit_bridge')
)

from transform_utils import (
    check_already_at_target_joint,
    check_already_at_target_cartesian,
    quaternion_angular_distance_deg
)

def test_angular_distance_identical():
    # Misma orientación
    d = quaternion_angular_distance_deg((0,0,0,1), (0,0,0,1))
    assert d == pytest.approx(0.0, abs=1e-5)

def test_angular_distance_opposite_sign():
    # q y -q son la misma rotación 3D
    d = quaternion_angular_distance_deg((0,0,0,1), (0,0,0,-1))
    assert d == pytest.approx(0.0, abs=1e-5)

def test_angular_distance_90_deg():
    s = math.sqrt(2)/2
    d = quaternion_angular_distance_deg((0,0,0,1), (s,0,0,s))
    assert d == pytest.approx(90.0, abs=1e-5)

def test_already_at_target_joint_exact():
    current_rad = {
        'j1': 0.0, 'j2': 1.0, 'j3': -0.5
    }
    # target in degrees
    target_deg = [0.0, 1.0 * 180/math.pi, -0.5 * 180/math.pi]
    names = ['j1', 'j2', 'j3']
    
    assert check_already_at_target_joint(current_rad, target_deg, names, tol_deg=0.1)

def test_already_at_target_joint_outside_tol():
    current_rad = {
        'j1': 0.0, 'j2': 1.0, 'j3': -0.5
    }
    # target diff by 2 degrees
    target_deg = [2.0, 1.0 * 180/math.pi, -0.5 * 180/math.pi]
    names = ['j1', 'j2', 'j3']
    
    assert not check_already_at_target_joint(current_rad, target_deg, names, tol_deg=0.1)
    # But it should pass if tol is 3 degrees
    assert check_already_at_target_joint(current_rad, target_deg, names, tol_deg=3.0)

def test_already_at_target_cartesian_exact():
    c_xyz = (0.5, 0.0, 0.5)
    c_q = (0, 0, 0, 1)
    
    assert check_already_at_target_cartesian(
        c_xyz, c_q, c_xyz, c_q, pos_tol_m=0.01, orient_tol_deg=1.0)

def test_already_at_target_cartesian_pos_fail():
    c_xyz = (0.5, 0.0, 0.5)
    c_q = (0, 0, 0, 1)
    t_xyz = (0.55, 0.0, 0.5)
    
    assert not check_already_at_target_cartesian(
        c_xyz, c_q, t_xyz, c_q, pos_tol_m=0.01, orient_tol_deg=1.0)

def test_already_at_target_cartesian_orient_fail():
    c_xyz = (0.5, 0.0, 0.5)
    c_q = (0, 0, 0, 1)
    s = math.sqrt(2)/2
    t_q = (s, 0, 0, s) # 90 degrees apart
    
    assert not check_already_at_target_cartesian(
        c_xyz, c_q, c_xyz, t_q, pos_tol_m=0.01, orient_tol_deg=1.0)
