#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
kuka_moveit_bridge_node.py

Nodo puente entre GUI externa y MoveIt2 para el robot KUKA KR6 (kr6r900sixx).

REGLA DE UNIDADES:
  - Todos los ángulos recibidos desde la GUI están en GRADOS.
  - Este nodo convierte internamente grados → radianes antes de enviar a MoveIt.
  - Todos los ángulos publicados hacia la GUI están en GRADOS.
  - La GUI NUNCA envía ni recibe radianes.

Tópicos de entrada (GUI → Bridge):
  /kuka_bridge/joint_command_deg      [std_msgs/Float64MultiArray]
      data: [A1, A2, A3, A4, A5, A6]   (todos en grados)

  /kuka_bridge/cartesian_command_deg  [std_msgs/Float64MultiArray]
      data: [X(m), Y(m), Z(m), A(°), B(°), C(°)]

Tópicos de salida (Bridge → GUI):
  /kuka_bridge/status                 [std_msgs/String]
  /kuka_bridge/joint_state_deg        [std_msgs/Float64MultiArray]
      data: [A1, A2, A3, A4, A5, A6]   (grados)
  /kuka_bridge/cartesian_state_deg    [std_msgs/Float64MultiArray]
      data: [X(m), Y(m), Z(m), A(°), B(°), C(°)]   (desde TF)

Servicios utilizados:
  /compute_ik  [moveit_msgs/srv/GetPositionIK]  (semilla = estado actual)

Acción MoveIt:
  /move_action  [moveit_msgs/action/MoveGroup]
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState as SensorJointState
from geometry_msgs.msg import PoseStamped

from builtin_interfaces.msg import Duration as BuiltinDuration

import tf2_ros

from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    PlanningOptions,
    WorkspaceParameters,
    RobotState as MoveItRobotState,
    MoveItErrorCodes,
)

from .transform_utils import (
    deg_to_rad, rad_to_deg,
    kuka_abc_deg_to_quaternion,
    quaternion_to_kuka_abc_deg,
    quaternion_angular_distance_deg,
    validate_float_array,
    validate_joint_limits,
    check_already_at_target_joint,
    check_already_at_target_cartesian,
    build_diff_for_log,
    JOINT_LABELS,
    MOVEIT_ERROR_DESCRIPTIONS,
)


# ─────────────────────────────────────────────────────────────────────────────
# Nodo principal
# ─────────────────────────────────────────────────────────────────────────────

class KukaMoveitBridgeNode(Node):
    """
    Puente entre GUI externa y MoveIt2 para KUKA KR6.

    Responsabilidades:
    - Esperar estado real del robot antes de aceptar comandos.
    - Publicar estado articular en grados y estado cartesiano desde TF.
    - Detectar ALREADY_AT_TARGET (no mover si ya está en objetivo).
    - Usar estado actual como semilla de IK y como start_state.
    - Gestionar un único objetivo a la vez.
    """

    def __init__(self):
        super().__init__('kuka_moveit_bridge_node')

        # ── Parámetros ────────────────────────────────────────────────
        self._declare_parameters()
        self._load_parameters()

        # ── Estado de concurrencia ────────────────────────────────────
        self._busy = False
        self._busy_lock = threading.Lock()

        # ── Cache de joint states ─────────────────────────────────────
        self._joint_positions: dict = {}     # nombre → rad
        self._last_joint_state_stamp = None  # rclpy.Time
        self._joint_state_lock = threading.Lock()

        # ── Estado de readiness ───────────────────────────────────────
        self._is_ready = False

        # ── TF2 ─────────────────────────────────────────────────────
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer, self)

        # ── Publicadores ──────────────────────────────────────────────
        self._status_pub = self.create_publisher(
            String, self._status_topic, 10)
        self._joint_deg_pub = self.create_publisher(
            Float64MultiArray, self._joint_state_deg_topic, 10)
        self._cartesian_state_pub = self.create_publisher(
            Float64MultiArray, self._cartesian_state_deg_topic, 10)

        # ── Suscriptores ──────────────────────────────────────────────
        self.create_subscription(
            Float64MultiArray,
            self._joint_cmd_topic,
            self._joint_command_callback,
            10)
        self.create_subscription(
            Float64MultiArray,
            self._cartesian_cmd_topic,
            self._cartesian_command_callback,
            10)
        self.create_subscription(
            SensorJointState,
            self._joint_state_input_topic,
            self._joint_state_callback,
            10)

        # ── Action client MoveIt ──────────────────────────────────────
        self._action_client = ActionClient(
            self, MoveGroup, self._move_action_name)

        # ── Cliente del servicio /compute_ik ─────────────────────────
        self._ik_client = self.create_client(
            GetPositionIK, self._compute_ik_service)

        # ── Timers ────────────────────────────────────────────────────
        # Verificar readiness (1 Hz)
        self.create_timer(1.0, self._readiness_timer_callback)

        # Publicar estado articular en grados (10 Hz)
        self.create_timer(0.1, self._publish_joint_state_deg_callback)

        # Publicar estado cartesiano desde TF (configurable Hz)
        rate = self._cartesian_state_publish_rate_hz
        self.create_timer(1.0 / rate, self._publish_cartesian_state_callback)

        self._publish_status('WAITING_FOR_ROBOT_STATE')
        self.get_logger().info(
            '╔══════════════════════════════════════════════════╗\n'
            '║   kuka_moveit_bridge_node iniciado              ║\n'
            '╠══════════════════════════════════════════════════╣\n'
            f'║  Planning group: {self._planning_group:<32}║\n'
            f'║  Base frame:     {self._base_frame:<32}║\n'
            f'║  End effector:   {self._end_effector:<32}║\n'
            f'║  Action:         {self._move_action_name:<32}║\n'
            f'║  IK service:     {self._compute_ik_service:<32}║\n'
            f'║  Plan only:      {str(self._plan_only):<32}║\n'
            '╚══════════════════════════════════════════════════╝\n'
            '\n'
            'IMPORTANTE: La GUI trabaja en GRADOS.\n'
            'El nodo convierte internamente a radianes para MoveIt.\n'
            'A=B=C=0 es la identidad, NO la orientacion actual del robot.\n'
        )

    # ─────────────────────────────────────────────────────────────────
    # Declaración y carga de parámetros
    # ─────────────────────────────────────────────────────────────────

    def _declare_parameters(self):
        # Grupos y frames
        self.declare_parameter('planning_group', 'manipulator')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('end_effector_link', 'link_6')
        self.declare_parameter('move_action_name', '/move_action')

        # Tópicos
        self.declare_parameter('joint_command_topic',
                               '/kuka_bridge/joint_command_deg')
        self.declare_parameter('cartesian_command_topic',
                               '/kuka_bridge/cartesian_command_deg')
        self.declare_parameter('status_topic', '/kuka_bridge/status')
        self.declare_parameter('joint_state_input_topic', '/joint_states')
        self.declare_parameter('joint_state_degrees_topic',
                               '/kuka_bridge/joint_state_deg')
        self.declare_parameter('cartesian_state_degrees_topic',
                               '/kuka_bridge/cartesian_state_deg')

        # Planificación
        self.declare_parameter('planning_time', 10.0)
        self.declare_parameter('planning_attempts', 10)
        self.declare_parameter('velocity_scaling', 0.1)
        self.declare_parameter('acceleration_scaling', 0.1)
        self.declare_parameter('cartesian_state_publish_rate_hz', 10.0)

        # Tolerancias para MoveIt (en grados, conversión interna a rad)
        self.declare_parameter('position_tolerance_m', 0.002)
        self.declare_parameter('orientation_tolerance_deg', 1.0)
        self.declare_parameter('joint_tolerance_deg', 0.2)

        # Tolerancias ALREADY_AT_TARGET
        self.declare_parameter('already_at_target_joint_tolerance_deg', 0.2)
        self.declare_parameter('already_at_target_position_tolerance_m', 0.002)
        self.declare_parameter(
            'already_at_target_orientation_tolerance_deg', 1.0)

        # Staleness
        self.declare_parameter('maximum_joint_state_age_sec', 1.0)

        # /compute_ik
        self.declare_parameter('compute_ik_service', '/compute_ik')
        self.declare_parameter('ik_timeout_sec', 2.0)
        self.declare_parameter('avoid_collisions', True)

        # Modo
        self.declare_parameter('plan_only', False)
        self.declare_parameter('reject_commands_while_busy', True)

        # Joints
        self.declare_parameter('joint_names', [
            'joint_a1', 'joint_a2', 'joint_a3',
            'joint_a4', 'joint_a5', 'joint_a6',
        ])

    def _load_parameters(self):
        gp = self.get_parameter
        self._planning_group = gp('planning_group').value
        self._base_frame = gp('base_frame').value
        self._end_effector = gp('end_effector_link').value
        self._move_action_name = gp('move_action_name').value

        self._joint_cmd_topic = gp('joint_command_topic').value
        self._cartesian_cmd_topic = gp('cartesian_command_topic').value
        self._status_topic = gp('status_topic').value
        self._joint_state_input_topic = gp('joint_state_input_topic').value
        self._joint_state_deg_topic = gp('joint_state_degrees_topic').value
        self._cartesian_state_deg_topic = gp(
            'cartesian_state_degrees_topic').value

        self._planning_time = gp('planning_time').value
        self._planning_attempts = gp('planning_attempts').value
        self._velocity_scaling = gp('velocity_scaling').value
        self._acceleration_scaling = gp('acceleration_scaling').value
        self._cartesian_state_publish_rate_hz = gp(
            'cartesian_state_publish_rate_hz').value

        self._position_tolerance_m = gp('position_tolerance_m').value
        self._orientation_tolerance_deg = gp('orientation_tolerance_deg').value
        self._joint_tolerance_deg = gp('joint_tolerance_deg').value

        self._aat_joint_tol_deg = gp(
            'already_at_target_joint_tolerance_deg').value
        self._aat_pos_tol_m = gp(
            'already_at_target_position_tolerance_m').value
        self._aat_orient_tol_deg = gp(
            'already_at_target_orientation_tolerance_deg').value

        self._max_joint_state_age_sec = gp('maximum_joint_state_age_sec').value

        self._compute_ik_service = gp('compute_ik_service').value
        self._ik_timeout_sec = gp('ik_timeout_sec').value
        self._avoid_collisions = gp('avoid_collisions').value

        self._plan_only = gp('plan_only').value
        self._reject_while_busy = gp('reject_commands_while_busy').value
        self._joint_names = gp('joint_names').value

    # ─────────────────────────────────────────────────────────────────
    # Timer: verificar readiness
    # ─────────────────────────────────────────────────────────────────

    def _readiness_timer_callback(self):
        """
        Verifica periódicamente si el sistema está listo para aceptar comandos.
        READY requiere: joint_states válidos + TF disponible + /move_action UP.
        """
        if self._is_ready:
            return

        # 1) ¿Tenemos joint_states con los 6 joints?
        with self._joint_state_lock:
            has_all = all(
                n in self._joint_positions for n in self._joint_names)
            stamp = self._last_joint_state_stamp

        if not has_all or stamp is None:
            self._publish_status('WAITING_FOR_ROBOT_STATE')
            return

        # 2) ¿Son frescos?
        if not self._is_joint_state_fresh():
            self._publish_status('WAITING_FOR_ROBOT_STATE')
            return

        # 3) ¿TF disponible?
        tf_ok = self._get_current_tf_pose() is not None
        if not tf_ok:
            self._publish_status('WAITING_FOR_ROBOT_STATE')
            return

        # 4) ¿/move_action está UP?
        if not self._action_client.server_is_ready():
            self._publish_status('WAITING_FOR_ROBOT_STATE')
            return

        self._is_ready = True
        self._publish_status('READY')
        self.get_logger().info(
            'Sistema LISTO: joint_states validos, TF disponible, '
            '/move_action UP.')

    # ─────────────────────────────────────────────────────────────────
    # Timer: publicar estado articular en grados
    # ─────────────────────────────────────────────────────────────────

    def _publish_joint_state_deg_callback(self):
        """Publica el estado articular en GRADOS (A1–A6)."""
        with self._joint_state_lock:
            positions_rad = [
                self._joint_positions.get(jname, 0.0)
                for jname in self._joint_names
            ]
        positions_deg = [rad_to_deg(r) for r in positions_rad]
        msg = Float64MultiArray()
        msg.data = positions_deg
        self._joint_deg_pub.publish(msg)

    # ─────────────────────────────────────────────────────────────────
    # Timer: publicar estado cartesiano desde TF
    # ─────────────────────────────────────────────────────────────────

    def _publish_cartesian_state_callback(self):
        """
        Publica la pose actual del efector final en grados.
        Lee desde TF: base_link → link_6.
        Publica TF_UNAVAILABLE en /status si TF no está disponible.
        Nunca publica datos inventados.
        """
        result = self._get_current_tf_pose()
        if result is None:
            msg = String()
            msg.data = 'TF_UNAVAILABLE'
            self._status_pub.publish(msg)
            return

        (x, y, z), (qx, qy, qz, qw) = result
        a_deg, b_deg, c_deg = quaternion_to_kuka_abc_deg(qx, qy, qz, qw)

        msg = Float64MultiArray()
        msg.data = [x, y, z, a_deg, b_deg, c_deg]
        self._cartesian_state_pub.publish(msg)

    # ─────────────────────────────────────────────────────────────────
    # Callback: joint_states
    # ─────────────────────────────────────────────────────────────────

    def _joint_state_callback(self, msg: SensorJointState):
        """Actualiza cache de posiciones articulares (en radianes)."""
        with self._joint_state_lock:
            for name, pos in zip(msg.name, msg.position):
                self._joint_positions[name] = pos
            self._last_joint_state_stamp = self.get_clock().now()

    # ─────────────────────────────────────────────────────────────────
    # Callback: comando articular
    # ─────────────────────────────────────────────────────────────────

    def _joint_command_callback(self, msg: Float64MultiArray):
        """Recibe comando articular [A1..A6] en GRADOS."""
        data = list(msg.data)

        ok, err = validate_float_array(data, 6)
        if not ok:
            self.get_logger().error(err)
            self._publish_status(err)
            return

        ok, err = validate_joint_limits(data, self._joint_names)
        if not ok:
            self.get_logger().error(err)
            self._publish_status(err)
            return

        if not self._is_ready:
            self._publish_status('WAITING_FOR_ROBOT_STATE')
            return

        if not self._is_joint_state_fresh():
            self._publish_status('ROBOT_STATE_STALE')
            return

        # Log
        joints_str = '  ' + '  '.join(
            f'{JOINT_LABELS[i]}={data[i]:.3f}°' for i in range(6))
        self.get_logger().info(f'Comando articular recibido:\n{joints_str}')
        self._publish_status(
            'RECEIVED_JOINT_COMMAND_DEG: ' +
            ', '.join(f'{JOINT_LABELS[i]}={data[i]:.3f}' for i in range(6))
        )

        # ALREADY_AT_TARGET antes de ocupar el nodo
        with self._joint_state_lock:
            current = dict(self._joint_positions)
        if check_already_at_target_joint(
                current, data, self._joint_names, self._aat_joint_tol_deg):
            diff = build_diff_for_log(current, data, self._joint_names)
            self.get_logger().info(
                f'ALREADY_AT_TARGET (articular, tol={self._aat_joint_tol_deg}°)\n'
                f'{diff}')
            self._publish_status('ALREADY_AT_TARGET')
            return

        if not self._check_and_set_busy():
            return

        thread = threading.Thread(
            target=self._execute_joint_goal,
            args=(data,),
            daemon=True)
        thread.start()

    # ─────────────────────────────────────────────────────────────────
    # Callback: comando cartesiano
    # ─────────────────────────────────────────────────────────────────

    def _cartesian_command_callback(self, msg: Float64MultiArray):
        """Recibe comando cartesiano [X(m), Y(m), Z(m), A(°), B(°), C(°)]."""
        data = list(msg.data)

        ok, err = validate_float_array(data, 6)
        if not ok:
            self.get_logger().error(err)
            self._publish_status(err)
            return

        if not self._is_ready:
            self._publish_status('WAITING_FOR_ROBOT_STATE')
            return

        if not self._is_joint_state_fresh():
            self._publish_status('ROBOT_STATE_STALE')
            return

        x, y, z = data[0], data[1], data[2]
        a_deg, b_deg, c_deg = data[3], data[4], data[5]

        self.get_logger().info(
            f'Comando cartesiano recibido:\n'
            f'  X={x:.4f} m  Y={y:.4f} m  Z={z:.4f} m\n'
            f'  A={a_deg:.4f}° (Rz)  B={b_deg:.4f}° (Ry)  C={c_deg:.4f}° (Rx)'
        )
        self._publish_status(
            f'RECEIVED_CARTESIAN_COMMAND_DEG: '
            f'X={x:.4f} Y={y:.4f} Z={z:.4f} '
            f'A={a_deg:.4f} B={b_deg:.4f} C={c_deg:.4f}'
        )

        # Verificar ALREADY_AT_TARGET antes de bloquear
        tf_result = self._get_current_tf_pose()
        if tf_result is not None:
            (cx, cy_tf, cz), (cqx, cqy, cqz, cqw) = tf_result
            tq = kuka_abc_deg_to_quaternion(a_deg, b_deg, c_deg)
            if check_already_at_target_cartesian(
                    (cx, cy_tf, cz), (cqx, cqy, cqz, cqw),
                    (x, y, z), tq,
                    self._aat_pos_tol_m, self._aat_orient_tol_deg):
                self.get_logger().info(
                    f'ALREADY_AT_TARGET (cartesiano, '
                    f'pos_tol={self._aat_pos_tol_m*1000:.1f}mm, '
                    f'orient_tol={self._aat_orient_tol_deg:.1f}°)')
                self._publish_status('ALREADY_AT_TARGET')
                return

        if not self._check_and_set_busy():
            return

        thread = threading.Thread(
            target=self._execute_cartesian_goal_via_ik,
            args=(x, y, z, a_deg, b_deg, c_deg),
            daemon=True)
        thread.start()

    # ─────────────────────────────────────────────────────────────────
    # Estado del robot
    # ─────────────────────────────────────────────────────────────────

    def _is_joint_state_fresh(self) -> bool:
        """Verifica que el último /joint_states no sea demasiado antiguo."""
        with self._joint_state_lock:
            stamp = self._last_joint_state_stamp
        if stamp is None:
            return False
        age_sec = (self.get_clock().now() - stamp).nanoseconds / 1e9
        return age_sec < self._max_joint_state_age_sec

    def _get_current_tf_pose(self):
        """
        Obtiene la pose actual del efector desde TF (base_link → link_6).
        Retorna ((x, y, z), (qx, qy, qz, qw)) o None si no disponible.
        No bloquea el hilo (timeout=0).
        """
        try:
            tf_stamped = self._tf_buffer.lookup_transform(
                self._base_frame,
                self._end_effector,
                rclpy.time.Time()   # latest available, no wait
            )
            t = tf_stamped.transform.translation
            r = tf_stamped.transform.rotation
            return (t.x, t.y, t.z), (r.x, r.y, r.z, r.w)
        except tf2_ros.TransformException:
            return None

    def _build_current_robot_state(self) -> MoveItRobotState:
        """
        Construye un MoveItRobotState con las posiciones actuales de /joint_states.

        Busca cada joint por nombre (no asume orden).
        Las posiciones están en RADIANES (MoveIt las requiere así).
        Este estado se usa como start_state en todos los goals a MoveIt.
        """
        with self._joint_state_lock:
            positions = dict(self._joint_positions)

        js = SensorJointState()
        js.name = list(self._joint_names)
        js.position = [positions.get(n, 0.0) for n in self._joint_names]
        js.velocity = [0.0] * len(self._joint_names)
        js.effort = [0.0] * len(self._joint_names)

        state = MoveItRobotState()
        state.joint_state = js
        state.is_diff = False
        return state

    # ─────────────────────────────────────────────────────────────────
    # Control de concurrencia
    # ─────────────────────────────────────────────────────────────────

    def _check_and_set_busy(self) -> bool:
        with self._busy_lock:
            if self._busy:
                self.get_logger().warn(
                    'BUSY: nuevo comando rechazado, hay un objetivo activo.')
                self._publish_status('BUSY')
                return False
            self._busy = True
        return True

    def _release_busy(self):
        with self._busy_lock:
            self._busy = False

    # ─────────────────────────────────────────────────────────────────
    # Ejecutar objetivo articular (en hilo secundario)
    # ─────────────────────────────────────────────────────────────────

    def _execute_joint_goal(self, values_deg: list):
        """
        Construye y envía un objetivo articular a MoveIt.
        Convierte grados → radianes internamente.
        Usa el estado articular actual como start_state.
        """
        try:
            self._publish_status('CONVERTING_DEGREES_TO_RADIANS')
            tol_rad = deg_to_rad(self._joint_tolerance_deg)

            joint_constraints = []
            for i, jname in enumerate(self._joint_names):
                val_rad = deg_to_rad(values_deg[i])
                jc = JointConstraint()
                jc.joint_name = jname
                jc.position = val_rad
                jc.tolerance_above = tol_rad
                jc.tolerance_below = tol_rad
                jc.weight = 1.0
                joint_constraints.append(jc)

            constraints = Constraints()
            constraints.name = 'joint_goal'
            constraints.joint_constraints = joint_constraints

            log_str = ', '.join(
                f'{JOINT_LABELS[i]}={values_deg[i]:.3f}°'
                for i in range(6))
            self.get_logger().info(
                f'Enviando objetivo articular a /move_action: {log_str}')

            self._send_moveit_goal(constraints, 'articular')

        except Exception as e:
            self.get_logger().error(f'Excepcion en objetivo articular: {e}')
            self._publish_status(f'FAILURE: {e}')
            self._release_busy()

    # ─────────────────────────────────────────────────────────────────
    # Ejecutar objetivo cartesiano via IK (en hilo secundario)
    # ─────────────────────────────────────────────────────────────────

    def _execute_cartesian_goal_via_ik(
            self, x: float, y: float, z: float,
            a_deg: float, b_deg: float, c_deg: float):
        """
        Flujo completo para objetivo cartesiano:
        1. Convertir ABC → cuaternión
        2. Llamar /compute_ik con el estado actual como semilla
        3. Si IK tiene éxito, verificar ALREADY_AT_TARGET (articular)
        4. Enviar objetivo articular (de la solución IK) a /move_action

        Ventaja frente a PositionConstraint: KDL recibe la semilla actual,
        favoreciendo la solución más cercana y evitando cambios de
        configuración innecesarios.
        """
        try:
            # ── 1. Conversión ABC (grados) → cuaternión ───────────────
            self._publish_status('CONVERTING_DEGREES_TO_RADIANS')
            self.get_logger().info(
                f'ABC → cuaternion: A={a_deg}°, B={b_deg}°, C={c_deg}°')
            qx, qy, qz, qw = kuka_abc_deg_to_quaternion(a_deg, b_deg, c_deg)
            self.get_logger().info(
                f'Cuaternion: x={qx:.4f} y={qy:.4f} z={qz:.4f} w={qw:.4f}')

            # ── 2. Llamar /compute_ik ─────────────────────────────────
            self._publish_status('COMPUTING_IK')

            if not self._ik_client.service_is_ready():
                self.get_logger().warn(
                    '/compute_ik no disponible. Esperando...')
                timeout_secs = 3.0
                t0 = time.time()
                while not self._ik_client.service_is_ready():
                    if time.time() - t0 > timeout_secs:
                        self.get_logger().error(
                            '/compute_ik no disponible tras timeout.')
                        self._publish_status(
                            'IK_FAILED: servicio /compute_ik no disponible')
                        self._release_busy()
                        return
                    time.sleep(0.1)

            ik_req = GetPositionIK.Request()
            ik_req.ik_request.group_name = self._planning_group
            ik_req.ik_request.ik_link_name = self._end_effector
            ik_req.ik_request.avoid_collisions = self._avoid_collisions
            ik_req.ik_request.timeout = BuiltinDuration(
                sec=int(self._ik_timeout_sec),
                nanosec=0)

            pose = PoseStamped()
            pose.header.frame_id = self._base_frame
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            ik_req.ik_request.pose_stamped = pose

            # Semilla = estado articular actual (no ceros)
            ik_req.ik_request.robot_state = self._build_current_robot_state()

            ik_future = self._ik_client.call_async(ik_req)

            # Esperar resultado sin bloquear el executor
            timeout_s = self._ik_timeout_sec + 2.0
            t0 = time.time()
            while rclpy.ok() and not ik_future.done():
                if time.time() - t0 > timeout_s:
                    self.get_logger().error('Timeout esperando /compute_ik')
                    self._publish_status(
                        'IK_FAILED: timeout en /compute_ik')
                    self._release_busy()
                    return
                time.sleep(0.01)

            ik_resp = ik_future.result()

            if ik_resp is None:
                self.get_logger().error('/compute_ik no retorno respuesta')
                self._publish_status('IK_FAILED: sin respuesta de /compute_ik')
                self._release_busy()
                return

            ik_code = ik_resp.error_code.val
            if ik_code != 1:  # != SUCCESS
                desc = MOVEIT_ERROR_DESCRIPTIONS.get(ik_code, f'code={ik_code}')
                msg = f'MOVEIT_ERROR: NO_IK_SOLUTION ({desc})'
                self.get_logger().error(
                    f'IK fallo: code={ik_code} ({desc}) '
                    f'para X={x:.3f} Y={y:.3f} Z={z:.3f} '
                    f'A={a_deg:.1f}° B={b_deg:.1f}° C={c_deg:.1f}°')
                self._publish_status(msg)
                self._release_busy()
                return

            # ── 3. Extraer solución de joint_state ───────────────────
            self._publish_status('IK_SUCCEEDED')
            sol_js = ik_resp.solution.joint_state

            # Mapeo nombre → posición (la respuesta puede tener más joints)
            ik_positions = {}
            for name, pos in zip(sol_js.name, sol_js.position):
                ik_positions[name] = pos

            # Verificar que tenemos todos los joints
            missing = [n for n in self._joint_names if n not in ik_positions]
            if missing:
                self.get_logger().error(
                    f'IK solution incompleta, faltan joints: {missing}')
                self._publish_status(f'IK_FAILED: solucion incompleta')
                self._release_busy()
                return

            ik_values_rad = [ik_positions[n] for n in self._joint_names]
            ik_values_deg = [rad_to_deg(r) for r in ik_values_rad]

            log_ik = ', '.join(
                f'{JOINT_LABELS[i]}={ik_values_deg[i]:.3f}°'
                for i in range(6))
            self.get_logger().info(
                f'Solucion IK: {log_ik}')

            # ── 4. Verificar ALREADY_AT_TARGET (articular, IK vs actual) ─
            with self._joint_state_lock:
                current_rad = dict(self._joint_positions)
            if check_already_at_target_joint(
                    current_rad, ik_values_deg,
                    self._joint_names, self._aat_joint_tol_deg):
                self.get_logger().info(
                    'ALREADY_AT_TARGET (solucion IK == estado actual)')
                self._publish_status('ALREADY_AT_TARGET')
                self._release_busy()
                return

            # ── 5. Construir JointConstraints y enviar a /move_action ─
            tol_rad = deg_to_rad(self._joint_tolerance_deg)
            joint_constraints = []
            for i, jname in enumerate(self._joint_names):
                jc = JointConstraint()
                jc.joint_name = jname
                jc.position = ik_values_rad[i]
                jc.tolerance_above = tol_rad
                jc.tolerance_below = tol_rad
                jc.weight = 1.0
                joint_constraints.append(jc)

            constraints = Constraints()
            constraints.name = 'cartesian_ik_goal'
            constraints.joint_constraints = joint_constraints

            self._send_moveit_goal(constraints, 'cartesiano_via_ik')

        except Exception as e:
            self.get_logger().error(f'Excepcion en objetivo cartesiano IK: {e}')
            self._publish_status(f'FAILURE: {e}')
            self._release_busy()

    # ─────────────────────────────────────────────────────────────────
    # Envío del goal a /move_action y espera del resultado
    # ─────────────────────────────────────────────────────────────────

    def _send_moveit_goal(self, constraints: Constraints, goal_type: str):
        """
        Construye el MoveGroup goal con start_state explícito y lo envía.
        Comparte código entre goals articulares y cartesianos (via IK).

        El start_state se construye desde el último /joint_states válido.
        NO usa is_diff=True (evita el warning de MoveIt).
        """
        # Workspace
        ws = WorkspaceParameters()
        ws.header.frame_id = self._base_frame
        ws.min_corner.x = ws.min_corner.y = ws.min_corner.z = -2.0
        ws.max_corner.x = ws.max_corner.y = ws.max_corner.z = 2.0

        # MotionPlanRequest con start_state explícito
        request = MotionPlanRequest()
        request.group_name = self._planning_group
        request.num_planning_attempts = self._planning_attempts
        request.allowed_planning_time = self._planning_time
        request.max_velocity_scaling_factor = self._velocity_scaling
        request.max_acceleration_scaling_factor = self._acceleration_scaling
        request.goal_constraints = [constraints]
        request.workspace_parameters = ws
        request.start_state = self._build_current_robot_state()

        options = PlanningOptions()
        options.plan_only = self._plan_only
        options.replan = False
        options.look_around = False

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = options

        # Verificar /move_action
        self.get_logger().info(
            f'Verificando /move_action para objetivo {goal_type}...')
        if not self._action_client.server_is_ready():
            self.get_logger().error('/move_action no disponible')
            self._publish_status('ACTION_SERVER_UNAVAILABLE')
            self._release_busy()
            return

        self.get_logger().info(
            f'Enviando objetivo {goal_type} a /move_action '
            f'(plan_only={self._plan_only})...')
        self._publish_status('PLANNING')

        send_future = self._action_client.send_goal_async(goal)
        while rclpy.ok() and not send_future.done():
            time.sleep(0.01)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Objetivo rechazado por /move_action')
            self._publish_status('GOAL_REJECTED')
            self._release_busy()
            return

        self.get_logger().info(
            'Objetivo aceptado. Esperando resultado...')
        self._publish_status('EXECUTING')

        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            time.sleep(0.05)

        result_obj = result_future.result()
        if result_obj is None:
            self.get_logger().error('Resultado nulo de /move_action')
            self._publish_status('MOVEIT_ERROR: resultado nulo')
            self._release_busy()
            return

        error_code = result_obj.result.error_code.val
        if error_code == 1:   # SUCCESS
            self.get_logger().info('Movimiento completado: SUCCEEDED')
            self._publish_status('SUCCEEDED')
        else:
            desc = MOVEIT_ERROR_DESCRIPTIONS.get(
                error_code, f'ERROR_DESCONOCIDO (code={error_code})')
            msg = f'MOVEIT_ERROR: code={error_code} description={desc}'
            self.get_logger().error(msg)
            self._publish_status(msg)

        self._release_busy()

    # ─────────────────────────────────────────────────────────────────
    # Publicación de estado
    # ─────────────────────────────────────────────────────────────────

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)


# ─────────────────────────────────────────────────────────────────────────────
# Punto de entrada
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = KukaMoveitBridgeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
