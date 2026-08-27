#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_fk_node.py

PRUEBA OBLIGATORIA §7.0 — valida la cinematica reimplementada contra la TF
real que publica robot_state_publisher.

POR QUE ESTE NODO Y NO tf2_echo.
  `ros2 run tf2_ros tf2_echo` imprime TRES DECIMALES, es decir resuelve 1 mm.
  La tolerancia de aceptacion es 1e-6 m = 0.001 mm, mil veces mas fina. Con
  tf2_echo NO se puede verificar esa tolerancia: sirve para descartar un error
  grosero, no para validar la cinematica.

  Este nodo lee la TF por la API, con la precision completa del float64 del
  mensaje, y compara contra forward_kinematics_tip() de este paquete usando
  los valores articulares REALES de /joint_states en ese instante.

QUE COMPARA
  Para cada tip (link_6, flange, tool0), en base_link y en world:
    - posicion, con tolerancia 1e-6 m
    - orientacion, como angulo de la rotacion residual, tolerancia 1e-5 rad

SALIDA
  Un informe por terminal y un codigo de salida: 0 si TODO pasa, 1 si algo
  falla. Si falla, NINGUNA metrica de Jacobiano del estudio es valida.

Observador pasivo: no publica, no planifica, no ejecuta, no toca el robot.
"""

import math
import sys
import threading
import time

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

import tf2_ros
from sensor_msgs.msg import JointState

from .continuity_metrics import (
    AXES,
    JOINT_NAMES,
    forward_kinematics_tip,
    reorder_to_canonical,
)

#: Tolerancias de aceptacion. Ver README seccion 7.0.
POSITION_TOLERANCE_M = 1.0e-6
ORIENTATION_TOLERANCE_RAD = 1.0e-5

TIPS = ('link_6', 'flange', 'tool0')


def quat_to_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rotation_angle(Ra, Rb) -> float:
    """Angulo de la rotacion residual Ra^T Rb, en radianes."""
    c = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return float(math.acos(max(-1.0, min(1.0, c))))


class VerifyFkNode(Node):
    """Compara la FK del paquete contra la TF real. Termina al acabar."""

    def __init__(self):
        super().__init__('kr6_baseline_verify_fk')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('timeout_sec', 20.0)

        gp = self.get_parameter
        self.base_frame = gp('base_frame').value
        self.world_frame = gp('world_frame').value
        self._timeout = float(gp('timeout_sec').value)

        self._buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buffer, self)
        self._joints = None
        self.create_subscription(
            JointState, '/joint_states', self._on_joints, 10)

        self.passed = False
        self.done = threading.Event()
        threading.Thread(target=self._run, daemon=True).start()

    def _on_joints(self, msg: JointState):
        positions = reorder_to_canonical(list(msg.name), list(msg.position))
        if positions is not None:
            self._joints = positions

    def _lookup(self, parent, child):
        try:
            tf = self._buffer.lookup_transform(
                parent, child, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException, tf2_ros.TransformException):
            return None
        t = tf.transform.translation
        r = tf.transform.rotation
        T = np.eye(4)
        T[:3, :3] = quat_to_matrix(r.x, r.y, r.z, r.w)
        T[:3, 3] = [t.x, t.y, t.z]
        return T

    def _run(self):
        try:
            self._run_inner()
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().error(f'Verificacion abortada: {exc}')
        finally:
            self.done.set()

    def _run_inner(self):
        deadline = time.monotonic() + self._timeout
        while rclpy.ok() and self._joints is None:
            if time.monotonic() > deadline:
                self.get_logger().error(
                    'No llego ningun /joint_states con los seis joints del '
                    f'grupo {JOINT_NAMES} en {self._timeout:.0f} s. '
                    '¿Esta el sistema levantado?')
                return
            time.sleep(0.1)

        q = list(self._joints)
        deg = [math.degrees(v) for v in q]
        lines = [
            '',
            '╔' + '═' * 74 + '╗',
            '║  §7.0 VERIFICACION DE LA CINEMATICA CONTRA LA TF REAL'.ljust(75) + '║',
            '╠' + '═' * 74 + '╣',
            ('║  Tolerancias: posicion <= 1e-6 m   orientacion <= 1e-5 rad'
             ).ljust(75) + '║',
            ('║  (tf2_echo imprime 3 decimales = 1 mm: NO sirve para esto)'
             ).ljust(75) + '║',
            '╚' + '═' * 74 + '╝',
            '',
            'ESTADO ARTICULAR LEIDO DE /joint_states:',
            '  ' + '  '.join(f'{a}={d:+10.6f}°' for a, d in zip(AXES, deg)),
            '',
        ]

        a5 = deg[4]
        if abs(a5) < 1e-3:
            lines.append('  A5 = 0 -> HOME BASELINE. Correcto para esta prueba.')
        else:
            lines.append(f'  *** AVISO: A5 = {a5:.6f}°, NO es el HOME baseline '
                         '(A5 = 0). La prueba sigue siendo valida como')
            lines.append('      verificacion de la FK, pero NO estas en la '
                         'configuracion singular que el estudio documenta.')
        lines.append('')

        header = (f'{"referencia":11s} {"tip":8s} '
                  f'{"|d posicion| [m]":>18s} {"|d orient| [rad]":>18s}  '
                  f'veredicto')
        lines.append(header)
        lines.append('─' * len(header))

        all_ok = True
        found_any = False
        for ref in (self.base_frame, self.world_frame):
            for tip in TIPS:
                T_tf = self._lookup(ref, tip)
                if T_tf is None:
                    lines.append(f'{ref:11s} {tip:8s} '
                                 f'{"TF NO DISPONIBLE":>38s}  ---')
                    continue
                found_any = True
                T_fk = forward_kinematics_tip(q, tip)
                if ref == self.world_frame:
                    T_wb = self._lookup(self.world_frame, self.base_frame)
                    if T_wb is None:
                        lines.append(f'{ref:11s} {tip:8s} '
                                     f'{"world->base NO DISPONIBLE":>38s}  ---')
                        continue
                    T_fk = T_wb @ T_fk
                dp = float(np.linalg.norm(T_tf[:3, 3] - T_fk[:3, 3]))
                dr = rotation_angle(T_tf[:3, :3], T_fk[:3, :3])
                ok = dp <= POSITION_TOLERANCE_M and dr <= ORIENTATION_TOLERANCE_RAD
                all_ok = all_ok and ok
                lines.append(f'{ref:11s} {tip:8s} {dp:18.12e} {dr:18.12e}  '
                             f'{"PASA" if ok else "*** FALLA ***"}')

        lines.append('')
        if not found_any:
            lines.append('*** NO SE PUDO LEER NINGUNA TF. ¿Esta corriendo '
                         'robot_state_publisher? ***')
            all_ok = False
        elif all_ok:
            lines.append('╔' + '═' * 74 + '╗')
            lines.append('║  RESULTADO: PASA'.ljust(75) + '║')
            lines.append(('║  La cinematica de continuity_metrics.py coincide '
                          'con la TF real.').ljust(75) + '║')
            lines.append(('║  Las metricas de Jacobiano del estudio quedan '
                          'validadas EMPIRICAMENTE.').ljust(75) + '║')
            lines.append('╚' + '═' * 74 + '╝')
        else:
            lines.append('╔' + '═' * 74 + '╗')
            lines.append('║  RESULTADO: FALLA'.ljust(75) + '║')
            lines.append(('║  NO CONTINUES. Ninguna metrica de Jacobiano del '
                          'estudio es valida.').ljust(75) + '║')
            lines.append(('║  Ver README seccion 7.0, apartado "Que hacer si '
                          'falla".').ljust(75) + '║')
            lines.append('╚' + '═' * 74 + '╝')
        self.passed = all_ok
        self.get_logger().info('\n'.join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = VerifyFkNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        while rclpy.ok() and not node.done.wait(timeout=0.2):
            pass
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        ok = node.passed
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
