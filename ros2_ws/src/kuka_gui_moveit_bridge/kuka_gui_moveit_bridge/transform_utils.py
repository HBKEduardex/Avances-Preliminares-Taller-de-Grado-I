#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
transform_utils.py

Funciones de conversión centralizadas para kuka_gui_moveit_bridge.

Importadas por:
  - kuka_moveit_bridge_node.py
  - kuka_bridge_test_gui_node.py
  - archivos de test

REGLA: Todos los ángulos recibidos/publicados hacia la GUI están en GRADOS.
       MoveIt trabaja en radianes. La conversión ocurre en el bridge.
"""

import math

# ─────────────────────────────────────────────────────────────────────────────
# Límites articulares reales del URDF kr6r900sixx_macro.xacro
# ─────────────────────────────────────────────────────────────────────────────
JOINT_LIMITS_DEG = {
    'joint_a1': (-170.0,  170.0),
    'joint_a2': (-190.0,   45.0),
    'joint_a3': (-120.0,  156.0),
    'joint_a4': (-185.0,  185.0),
    'joint_a5': (-120.0,  120.0),
    'joint_a6': (-350.0,  350.0),
}

JOINT_NAMES_ORDERED = [
    'joint_a1', 'joint_a2', 'joint_a3',
    'joint_a4', 'joint_a5', 'joint_a6',
]

JOINT_LABELS = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6']

# ─────────────────────────────────────────────────────────────────────────────
# Tabla de errores MoveIt
# ─────────────────────────────────────────────────────────────────────────────
MOVEIT_ERROR_DESCRIPTIONS = {
    1:     'SUCCESS',
    -1:    'FAILURE',
    -2:    'PLANNING_FAILED',
    -3:    'INVALID_MOTION_PLAN',
    -4:    'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE',
    -5:    'CONTROL_FAILED',
    -6:    'UNABLE_TO_AQUIRE_SENSOR_DATA',
    -7:    'TIMED_OUT',
    -8:    'PREEMPTED',
    -10:   'START_STATE_IN_COLLISION',
    -11:   'START_STATE_VIOLATES_PATH_CONSTRAINTS',
    -12:   'START_STATE_INVALID',
    -13:   'GOAL_IN_COLLISION',
    -14:   'GOAL_VIOLATES_PATH_CONSTRAINTS',
    -15:   'GOAL_CONSTRAINTS_VIOLATED',
    -16:   'INVALID_GROUP_NAME',
    -17:   'INVALID_GOAL_CONSTRAINTS',
    -18:   'INVALID_ROBOT_STATE',
    -19:   'INVALID_LINK_NAME',
    -20:   'INVALID_OBJECT_NAME',
    -21:   'FRAME_TRANSFORM_FAILURE',
    -22:   'COLLISION_CHECKING_UNAVAILABLE',
    -23:   'ROBOT_STATE_STALE',
    -24:   'SENSOR_INFO_STALE',
    -25:   'COMMUNICATION_FAILURE',
    -26:   'CRASH',
    -27:   'ABORT',
    -28:   'NO_IK_SOLUTION',
    -31:   'NO_IK_SOLUTION',
    -32:   'KINEMATIC_BOUNDS_VIOLATED',
    99999: 'FAILURE (catastrofico: sin solucion IK o colision en goal)',
}


# ─────────────────────────────────────────────────────────────────────────────
# Conversión básica de ángulos
# ─────────────────────────────────────────────────────────────────────────────

def deg_to_rad(degrees: float) -> float:
    """Convierte grados a radianes."""
    return degrees * math.pi / 180.0


def rad_to_deg(radians: float) -> float:
    """Convierte radianes a grados."""
    return radians * 180.0 / math.pi


def normalize_angle_deg(angle_deg: float) -> float:
    """Normaliza un ángulo al rango [-180, 180] grados."""
    angle = angle_deg % 360.0
    if angle > 180.0:
        angle -= 360.0
    return angle


# ─────────────────────────────────────────────────────────────────────────────
# Cuaterniones
# ─────────────────────────────────────────────────────────────────────────────

def normalize_quaternion(x: float, y: float, z: float, w: float) -> tuple:
    """
    Normaliza un cuaternión. Retorna (x, y, z, w).
    Si la norma es cero, retorna la identidad (0, 0, 0, 1).
    """
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm < 1e-10:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def kuka_abc_deg_to_quaternion(a_deg: float, b_deg: float,
                                c_deg: float) -> tuple:
    """
    KUKA ABC en grados → cuaternión normalizado (x, y, z, w).

    Convención KUKA:
      A = rotación alrededor de Z (yaw)
      B = rotación alrededor de Y (pitch)
      C = rotación alrededor de X (roll)
      Composición: R = Rz(A) * Ry(B) * Rx(C)  (Euler ZYX extrínseco)

    Entradas en GRADOS. Salida: (x, y, z, w) normalizado.
    """
    a = deg_to_rad(a_deg)   # yaw   (Z)
    b = deg_to_rad(b_deg)   # pitch (Y)
    c = deg_to_rad(c_deg)   # roll  (X)

    cy, sy = math.cos(a / 2.0), math.sin(a / 2.0)
    cp, sp = math.cos(b / 2.0), math.sin(b / 2.0)
    cr, sr = math.cos(c / 2.0), math.sin(c / 2.0)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return normalize_quaternion(x, y, z, w)


def quaternion_to_kuka_abc_deg(qx: float, qy: float,
                                qz: float, qw: float) -> tuple:
    """
    Cuaternión → KUKA ABC en grados (rango [-180, 180]).

    Convención: R = Rz(A) * Ry(B) * Rx(C)  (Euler ZYX)
    Retorna (A_deg, B_deg, C_deg).

    NOTA:
    - A = 0, B = 0, C = 0  representa la identidad (efector alineado con base_link).
    - A = 0, B = 0, C = 0  NO significa "mantener la orientación actual".
    - La orientación actual debe leerse desde TF (base_link -> link_6).
    """
    qx, qy, qz, qw = normalize_quaternion(qx, qy, qz, qw)

    # B = pitch alrededor de Y
    sinp = 2.0 * (qw * qy - qz * qx)
    
    if sinp >= 0.999999: # Gimbal lock at North Pole (+90 deg)
        b_rad = math.pi / 2.0
        c_rad = 0.0
        a_rad = 2.0 * math.atan2(qz, qw)
    elif sinp <= -0.999999: # Gimbal lock at South Pole (-90 deg)
        b_rad = -math.pi / 2.0
        c_rad = 0.0
        a_rad = -2.0 * math.atan2(qz, qw)
    else:
        b_rad = math.asin(sinp)
        
        # C = roll alrededor de X
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        c_rad = math.atan2(sinr_cosp, cosr_cosp)
        
        # A = yaw alrededor de Z
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        a_rad = math.atan2(siny_cosp, cosy_cosp)

    a_deg = normalize_angle_deg(rad_to_deg(a_rad))
    b_deg = normalize_angle_deg(rad_to_deg(b_rad))
    c_deg = normalize_angle_deg(rad_to_deg(c_rad))

    return (a_deg, b_deg, c_deg)


def quaternion_angular_distance_deg(q1: tuple, q2: tuple) -> float:
    """
    Distancia angular entre dos cuaterniones en grados.

    q1, q2: tuplas (x, y, z, w).
    Retorna el ángulo de rotación entre ellos, en grados [0, 180].

    Nota: usa |dot product| para manejar la doble cobertura del cuaternión.
    No compares ángulos A, B, C directamente para esta distancia.
    """
    q1 = normalize_quaternion(*q1)
    q2 = normalize_quaternion(*q2)

    dot = abs(q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3])
    dot = min(1.0, dot)   # clamp numérico
    angle_rad = 2.0 * math.acos(dot)
    return rad_to_deg(angle_rad)


# ─────────────────────────────────────────────────────────────────────────────
# Validación de datos
# ─────────────────────────────────────────────────────────────────────────────

def validate_float_array(data, expected_size: int) -> tuple:
    """
    Valida un arreglo de floats.
    Retorna (True, None) si es válido.
    Retorna (False, mensaje_error) si es inválido.
    """
    if len(data) != expected_size:
        return False, (
            f'REJECTED_INVALID_SIZE: se esperaban {expected_size} valores, '
            f'se recibieron {len(data)}'
        )
    for i, v in enumerate(data):
        if not isinstance(v, (int, float)):
            return False, (
                f'REJECTED_INVALID_VALUE: elemento [{i}]={v!r} no es numerico'
            )
        if math.isnan(v):
            return False, f'REJECTED_NAN: elemento [{i}] es NaN'
        if math.isinf(v):
            return False, f'REJECTED_INF: elemento [{i}] es Inf'
    return True, None


def validate_joint_limits(values_deg: list, joint_names: list,
                           labels=None) -> tuple:
    """
    Verifica que los valores articulares estén dentro de los límites del URDF.
    Retorna (True, None) o (False, mensaje_de_error).

    Compara en GRADOS (no en radianes).
    """
    if labels is None:
        labels = JOINT_LABELS
    for i, jname in enumerate(joint_names):
        if i >= len(values_deg):
            break
        val_deg = values_deg[i]
        if jname in JOINT_LIMITS_DEG:
            lo, hi = JOINT_LIMITS_DEG[jname]
            if val_deg < lo or val_deg > hi:
                return False, (
                    f'REJECTED_JOINT_LIMIT: {labels[i]}={val_deg:.4f} deg, '
                    f'limite permitido=[{lo:.1f}, {hi:.1f}] deg'
                )
    return True, None


# ─────────────────────────────────────────────────────────────────────────────
# Detección ALREADY_AT_TARGET
# ─────────────────────────────────────────────────────────────────────────────

def check_already_at_target_joint(current_rad: dict, target_deg: list,
                                   joint_names: list,
                                   tol_deg: float) -> bool:
    """
    Detecta si el robot ya está en el objetivo articular.

    current_rad : dict  {joint_name: posicion_en_radianes}
    target_deg  : list  [A1, A2, A3, A4, A5, A6] en GRADOS
    tol_deg     : float tolerancia en GRADOS

    Retorna True si TODOS los joints están dentro de la tolerancia.

    Comparación interna en radianes.
    Los mensajes de estado informan en grados.
    """
    tol_rad = deg_to_rad(tol_deg)
    for i, jname in enumerate(joint_names):
        current = current_rad.get(jname, None)
        if current is None:
            return False
        target = deg_to_rad(target_deg[i])
        if abs(current - target) > tol_rad:
            return False
    return True


def check_already_at_target_cartesian(
        current_xyz: tuple, current_q: tuple,
        target_xyz: tuple, target_q: tuple,
        pos_tol_m: float, orient_tol_deg: float) -> bool:
    """
    Detecta si el robot ya está en el objetivo cartesiano.

    current_xyz, target_xyz : tuplas (x, y, z) en metros
    current_q,  target_q    : tuplas (x, y, z, w) cuaterniones normalizados

    pos_tol_m      : tolerancia de posición en metros
    orient_tol_deg : tolerancia angular en GRADOS

    Retorna True si posición Y orientación están dentro de tolerancias.

    La diferencia de orientación se calcula con cuaterniones (no con A, B, C).
    """
    dx = current_xyz[0] - target_xyz[0]
    dy = current_xyz[1] - target_xyz[1]
    dz = current_xyz[2] - target_xyz[2]
    pos_dist = math.sqrt(dx*dx + dy*dy + dz*dz)

    orient_dist = quaternion_angular_distance_deg(current_q, target_q)

    return pos_dist <= pos_tol_m and orient_dist <= orient_tol_deg


def build_diff_for_log(current_rad: dict, target_deg: list,
                       joint_names: list, labels=None) -> str:
    """
    Construye un string con la diferencia articular (para logs).
    Convierte a grados para mostrar al usuario.
    """
    if labels is None:
        labels = JOINT_LABELS
    lines = []
    for i, jname in enumerate(joint_names):
        current_deg = rad_to_deg(current_rad.get(jname, 0.0))
        diff_deg = abs(current_deg - target_deg[i])
        lines.append(
            f'  {labels[i]}: actual={current_deg:.3f}°, '
            f'objetivo={target_deg[i]:.3f}°, diff={diff_deg:.3f}°'
        )
    return '\n'.join(lines)
