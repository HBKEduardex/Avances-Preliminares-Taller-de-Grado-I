#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_gui_validation.py

Pruebas para validación de límites en la GUI (y en el bridge).
"""

import math
import pytest
import sys
import os

sys.path.insert(
    0,
    os.path.join(os.path.dirname(__file__), '..', 'kuka_gui_moveit_bridge')
)

from transform_utils import validate_joint_limits

def test_validate_joint_limits_ok():
    names = ['joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 'joint_a5', 'joint_a6']
    values_deg = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    ok, err = validate_joint_limits(values_deg, names)
    assert ok is True
    assert err is None

def test_validate_joint_limits_out_of_bounds_high():
    names = ['joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 'joint_a5', 'joint_a6']
    values_deg = [171.0, 0.0, 0.0, 0.0, 0.0, 0.0] # Limit A1 is 170
    
    ok, err = validate_joint_limits(values_deg, names)
    assert ok is False
    assert "REJECTED_JOINT_LIMIT" in err
    assert "A1=171" in err

def test_validate_joint_limits_out_of_bounds_low():
    names = ['joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 'joint_a5', 'joint_a6']
    values_deg = [0.0, -191.0, 0.0, 0.0, 0.0, 0.0] # Limit A2 is -190
    
    ok, err = validate_joint_limits(values_deg, names)
    assert ok is False
    assert "REJECTED_JOINT_LIMIT" in err
    assert "A2=-191" in err

def test_validate_joint_limits_missing_name():
    names = ['unknown_joint']
    values_deg = [1000.0]
    
    ok, err = validate_joint_limits(values_deg, names)
    # If the joint is not in the limits dict, it should pass without validating
    assert ok is True
    assert err is None
