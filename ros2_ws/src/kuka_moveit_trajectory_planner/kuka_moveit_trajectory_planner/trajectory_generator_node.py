#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
trajectory_generator_node.py

Genera con MoveIt2 una trayectoria por cada transicion entre configuraciones
articulares reales capturadas del KUKA por la GUI externa.

    P1 -> P2   =>  T1
    P2 -> P3   =>  T2
    ...
    P(N-1) -> PN => T(N-1)

Cada transicion es un SEGMENTO INDEPENDIENTE. Los segmentos no se concatenan,
no se re-optimizan, no se suavizan y no se re-muestrean.

Contrato ROS 2 (std_msgs/msg/String con JSON):
    entrada : /kuka_moveit/trajectory_generation/request_json
    salida  : /kuka_moveit/trajectory_generation/result_json

SEGURIDAD (invariantes de este nodo):
  - planning_options.plan_only SIEMPRE es True. No hay parametro para
    desactivarlo.
  - No se crea ningun cliente de /execute_trajectory ni de
    /joint_trajectory_controller/follow_joint_trajectory.
  - No se publica en /joint_states, /fake_joint_states ni en ningun topico
    del bridge TCP/IP.
  - No se escribe ningun archivo: el resultado se publica y el otro entorno
    (GUI/TCP-IP) es quien lo guarda.

El estado inicial de cada segmento se envia EXPLICITAMENTE en el
MotionPlanRequest, por lo que este nodo no necesita /joint_states y no depende
de donde este el robot real ni el modelo de RViz.
"""

import queue
import threading
import time
from typing import Any, Dict, List, Optional, Sequence

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    PlanningOptions,
    RobotState,
    WorkspaceParameters,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .trajectory_contract import (
    ContractError,
    GenerationRequest,
    SourcePoint,
    build_error_result,
    build_ok_result,
    build_segment,
    build_trajectory_point,
    deg_to_rad,
    dumps_json,
    loads_json,
    parse_generation_request,
    rad_to_deg,
)

# Descripciones de moveit_msgs/msg/MoveItErrorCodes mas frecuentes.
MOVEIT_ERROR_DESCRIPTIONS = {
    1: 'SUCCESS',
    99999: 'FAILURE',
    -1: 'PLANNING_FAILED',
    -2: 'INVALID_MOTION_PLAN',
    -3: 'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE',
    -4: 'CONTROL_FAILED',
    -5: 'UNABLE_TO_AQUIRE_SENSOR_DATA',
    -6: 'TIMED_OUT',
    -7: 'PREEMPTED',
    -10: 'START_STATE_IN_COLLISION',
    -11: 'START_STATE_VIOLATES_PATH_CONSTRAINTS',
    -12: 'GOAL_IN_COLLISION',
    -13: 'GOAL_VIOLATES_PATH_CONSTRAINTS',
    -14: 'GOAL_CONSTRAINTS_VIOLATED',
    -15: 'INVALID_GROUP_NAME',
    -16: 'INVALID_GOAL_CONSTRAINTS',
    -17: 'INVALID_ROBOT_STATE',
    -18: 'INVALID_LINK_NAME',
    -19: 'INVALID_OBJECT_NAME',
    -21: 'FRAME_TRANSFORM_FAILURE',
    -22: 'COLLISION_CHECKING_UNAVAILABLE',
    -23: 'ROBOT_STATE_STALE',
    -24: 'SENSOR_INFO_STALE',
    -25: 'COMMUNICATION_FAILURE',
    -31: 'NO_IK_SOLUTION',
}

#: Encadenamiento entre segmentos (parametro segment_chaining).
CHAIN_PREVIOUS_END = 'previous_trajectory_end'
CHAIN_SOURCE_POINT = 'source_point'


def duration_to_sec(duration_msg) -> float:
    """builtin_interfaces/msg/Duration -> segundos."""
    return float(duration_msg.sec) + float(duration_msg.nanosec) * 1e-9


class TrajectoryGeneratorNode(Node):
    """Planificador por segmentos. Solo planifica: nunca ejecuta."""

    def __init__(self):
        super().__init__('kuka_trajectory_generator_node')

        self._declare_parameters()
        self._load_parameters()

        # Un unico grupo reentrante: el hilo trabajador bloquea esperando el
        # resultado de la accion mientras el executor sigue procesando.
        self._callback_group = ReentrantCallbackGroup()

        qos = QoSProfile(
            depth=self._qos_depth,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._result_pub = self.create_publisher(
            String, self._result_topic, qos)
        self.create_subscription(
            String, self._request_topic, self._request_callback, qos,
            callback_group=self._callback_group)

        # Cliente de la accion de PLANIFICACION de MoveIt2. Es el unico
        # cliente de accion del nodo (no existe cliente de ejecucion).
        self._action_client = ActionClient(
            self, MoveGroup, self._move_action_name,
            callback_group=self._callback_group)

        # Procesamiento serializado: una peticion a la vez.
        self._request_queue: 'queue.Queue[str]' = queue.Queue()
        self._busy = False
        self._busy_lock = threading.Lock()
        self._shutdown = threading.Event()
        self._worker = threading.Thread(
            target=self._worker_loop, name='generation_worker', daemon=True)
        self._worker.start()

        self.get_logger().info(
            '\n'
            '╔══════════════════════════════════════════════════════════╗\n'
            '║   kuka_trajectory_generator_node iniciado                ║\n'
            '╠══════════════════════════════════════════════════════════╣\n'
            f'║  Grupo:       {self._planning_group:<43}║\n'
            f'║  Accion:      {self._move_action_name:<43}║\n'
            f'║  Request:     {self._request_topic:<43}║\n'
            f'║  Result:      {self._result_topic:<43}║\n'
            f'║  plan_only:   {"True (fijo, no configurable)":<43}║\n'
            f'║  Encadenado:  {self._segment_chaining:<43}║\n'
            '╚══════════════════════════════════════════════════════════╝\n'
            'Solo PLANIFICA. No ejecuta, no mueve el robot y no guarda '
            'archivos.\n'
            'El resultado se publica en JSON para que la GUI lo almacene.\n')

    # ─────────────────────────────────────────────────────────────────
    # Parametros
    # ─────────────────────────────────────────────────────────────────

    def _declare_parameters(self):
        self.declare_parameter(
            'request_topic', '/kuka_moveit/trajectory_generation/request_json')
        self.declare_parameter(
            'result_topic', '/kuka_moveit/trajectory_generation/result_json')
        self.declare_parameter('qos_depth', 10)

        self.declare_parameter('planning_group', 'manipulator')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('move_action_name', '/move_action')
        self.declare_parameter('action_wait_timeout_sec', 30.0)

        # Pipeline y planner: vacio = el que ya esta configurado en el
        # proyecto (ompl / RRTConnectkConfigDefault). NO se cambia nada.
        self.declare_parameter('planning_pipeline_id', '')
        self.declare_parameter('planner_id', '')

        self.declare_parameter('planning_time', 10.0)
        self.declare_parameter('planning_attempts', 10)
        self.declare_parameter('velocity_scaling', 0.1)
        self.declare_parameter('acceleration_scaling', 0.1)
        self.declare_parameter('joint_goal_tolerance_deg', 0.2)
        self.declare_parameter('endpoint_warning_tolerance_deg', 0.5)

        self.declare_parameter('segment_chaining', CHAIN_PREVIOUS_END)
        self.declare_parameter('joint_names', [
            'joint_a1', 'joint_a2', 'joint_a3',
            'joint_a4', 'joint_a5', 'joint_a6'])
        self.declare_parameter('log_points', False)

    def _load_parameters(self):
        gp = self.get_parameter
        self._request_topic = gp('request_topic').value
        self._result_topic = gp('result_topic').value
        self._qos_depth = int(gp('qos_depth').value)

        self._planning_group = gp('planning_group').value
        self._base_frame = gp('base_frame').value
        self._move_action_name = gp('move_action_name').value
        self._action_wait_timeout = float(gp('action_wait_timeout_sec').value)

        self._pipeline_id = gp('planning_pipeline_id').value or ''
        self._planner_id = gp('planner_id').value or ''

        self._planning_time = float(gp('planning_time').value)
        self._planning_attempts = int(gp('planning_attempts').value)
        self._velocity_scaling = float(gp('velocity_scaling').value)
        self._acceleration_scaling = float(gp('acceleration_scaling').value)
        self._joint_goal_tolerance_rad = deg_to_rad(
            float(gp('joint_goal_tolerance_deg').value))
        self._endpoint_tolerance_rad = deg_to_rad(
            float(gp('endpoint_warning_tolerance_deg').value))

        chaining = gp('segment_chaining').value
        if chaining not in (CHAIN_PREVIOUS_END, CHAIN_SOURCE_POINT):
            self.get_logger().warn(
                f'segment_chaining="{chaining}" no reconocido; se usa '
                f'"{CHAIN_PREVIOUS_END}".')
            chaining = CHAIN_PREVIOUS_END
        self._segment_chaining = chaining

        self._default_joint_names = list(gp('joint_names').value)
        self._log_points = bool(gp('log_points').value)

    # ─────────────────────────────────────────────────────────────────
    # Entrada
    # ─────────────────────────────────────────────────────────────────

    def _request_callback(self, msg: String):
        """Encola la peticion; el trabajo pesado ocurre en el hilo worker."""
        with self._busy_lock:
            busy = self._busy or not self._request_queue.empty()
        if busy:
            # Nunca se procesan dos secuencias a la vez: el resultado seria
            # ambiguo para la GUI.
            request_id = self._peek_request_id(msg.data)
            self.get_logger().warn(
                f'Peticion "{request_id}" rechazada: ya hay una generacion en '
                'curso.')
            self._publish_result(build_error_result(
                request_id,
                'BUSY: ya hay una generacion de trayectorias en curso.'))
            return
        self._request_queue.put(msg.data)

    @staticmethod
    def _peek_request_id(text: str) -> str:
        """request_id sin validar el resto (solo para responder errores)."""
        try:
            payload = loads_json(text)
            value = payload.get('request_id', '')
            return value if isinstance(value, str) else ''
        except ContractError:
            return ''

    def _worker_loop(self):
        while not self._shutdown.is_set():
            try:
                text = self._request_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            with self._busy_lock:
                self._busy = True
            try:
                self._process_request(text)
            except Exception as exc:                      # noqa: BLE001
                self.get_logger().error(f'Excepcion procesando peticion: {exc}')
                self._publish_result(build_error_result(
                    self._peek_request_id(text),
                    f'Excepcion interna: {exc}'))
            finally:
                with self._busy_lock:
                    self._busy = False
                self._request_queue.task_done()

    # ─────────────────────────────────────────────────────────────────
    # Procesamiento de una peticion completa
    # ─────────────────────────────────────────────────────────────────

    def _process_request(self, text: str):
        # 1. Validar contrato
        try:
            payload = loads_json(text)
            request = parse_generation_request(
                payload, self._default_joint_names)
        except ContractError as exc:
            request_id = self._peek_request_id(text)
            self.get_logger().error(f'Peticion invalida: {exc}')
            self._publish_result(build_error_result(request_id, str(exc)))
            return

        self.get_logger().info(
            f'Peticion "{request.request_id}": {len(request.points)} puntos '
            f'=> {request.segment_count} segmentos.')
        for warning in request.warnings:
            self.get_logger().warn(f'  aviso: {warning}')

        # 2. Comprobar que MoveIt2 esta disponible
        if not self._action_client.wait_for_server(
                timeout_sec=self._action_wait_timeout):
            message = (
                f'El action server {self._move_action_name} no esta '
                'disponible. Levante primero el sistema MoveIt2.')
            self.get_logger().error(message)
            self._publish_result(build_error_result(
                request.request_id, message))
            return

        # 3. Planificar segmento a segmento
        segments: List[Dict[str, Any]] = []
        # Estado articular (rad) desde el que arranca el siguiente segmento.
        current_start_rad: List[float] = list(request.points[0].joints_rad)

        for index, from_point, to_point in request.segment_pairs():
            segment_id = f'T{index}'
            self.get_logger().info(
                f'[{segment_id}] Planificando {from_point.id} -> '
                f'{to_point.id} ...')

            try:
                segment, end_rad = self._plan_segment(
                    request, index, from_point, to_point, current_start_rad)
            except _SegmentPlanningError as exc:
                self.get_logger().error(
                    f'[{segment_id}] {from_point.id} -> {to_point.id}: '
                    f'{exc.message}')
                self.get_logger().error(
                    'Secuencia abortada. La GUI NO debe considerarla '
                    'ejecutable.')
                self._publish_result(build_error_result(
                    request.request_id,
                    exc.message,
                    failed_segment=segment_id,
                    from_point=from_point.id,
                    to_point=to_point.id,
                    error_code=exc.error_code,
                    extra={
                        'completed_segments': len(segments),
                        'segment_count_expected': request.segment_count,
                    },
                ))
                return

            segments.append(segment)
            self.get_logger().info(
                f'[{segment_id}] OK: {segment["point_count"]} puntos, '
                f'{segment["duration_sec"]:.3f} s.')

            # Requisito: el final de un segmento es el inicio del siguiente.
            if self._segment_chaining == CHAIN_PREVIOUS_END:
                current_start_rad = end_rad
            else:
                current_start_rad = list(to_point.joints_rad)

        # 4. Publicar resultado
        result = build_ok_result(
            request, segments, self._planner_metadata(request))
        self._publish_result(result)
        summary = result['summary']
        self.get_logger().info(
            f'Peticion "{request.request_id}" completada: '
            f'{summary["segment_count"]} segmentos, '
            f'{summary["trajectory_point_count"]} puntos, '
            f'{summary["total_duration_sec"]:.3f} s totales. '
            'Resultado publicado (este contenedor NO lo guarda).')

    def _planner_metadata(self, request: GenerationRequest) -> Dict[str, Any]:
        """Metadatos REALMENTE usados en la peticion (no los del config)."""
        return {
            # Cadena vacia = pipeline/planner por defecto del proyecto
            # (ompl / RRTConnectkConfigDefault). No se altera la config.
            'planning_pipeline': self._pipeline_id,
            'planner_id': self._planner_id,
            'group': self._planning_group,
            'plan_only': True,
            'velocity_scaling': self._effective_velocity_scaling(request),
            'acceleration_scaling': self._effective_acceleration_scaling(
                request),
            'planning_attempts': self._planning_attempts,
            'allowed_planning_time_sec': self._planning_time,
            'joint_goal_tolerance_rad': self._joint_goal_tolerance_rad,
            'segment_chaining': self._segment_chaining,
        }

    # ─────────────────────────────────────────────────────────────────
    # Planificacion de un segmento
    # ─────────────────────────────────────────────────────────────────

    def _plan_segment(self,
                      request: GenerationRequest,
                      index: int,
                      from_point: SourcePoint,
                      to_point: SourcePoint,
                      start_rad: Sequence[float]):
        """
        Planifica Pi -> P(i+1) y devuelve (segmento_json, ultimo_estado_rad).

        Lanza _SegmentPlanningError si MoveIt2 no entrega un plan valido.
        """
        goal = self._build_goal(request, to_point, start_rad)

        if not self._action_client.server_is_ready():
            raise _SegmentPlanningError(
                f'{self._move_action_name} dejo de estar disponible.')

        send_future = self._action_client.send_goal_async(goal)
        if not self._wait_for(send_future, self._planning_time + 30.0):
            raise _SegmentPlanningError(
                'Tiempo agotado esperando la aceptacion del objetivo por '
                f'{self._move_action_name}.')
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise _SegmentPlanningError(
                f'Objetivo rechazado por {self._move_action_name}.')

        result_future = goal_handle.get_result_async()
        if not self._wait_for(result_future, self._planning_time + 60.0):
            raise _SegmentPlanningError(
                'Tiempo agotado esperando el resultado de la planificacion.')
        result_obj = result_future.result()
        if result_obj is None:
            raise _SegmentPlanningError(
                f'Resultado nulo de {self._move_action_name}.')

        move_result = result_obj.result
        error_code = int(move_result.error_code.val)
        if error_code != 1:
            description = MOVEIT_ERROR_DESCRIPTIONS.get(
                error_code, 'ERROR_DESCONOCIDO')
            raise _SegmentPlanningError(
                f'MoveIt2 fallo la planificacion: code={error_code} '
                f'({description}).',
                error_code=error_code)

        joint_trajectory = move_result.planned_trajectory.joint_trajectory
        if not joint_trajectory.points:
            raise _SegmentPlanningError(
                'MoveIt2 devolvio SUCCESS pero la trayectoria esta vacia.',
                error_code=error_code)

        segment, end_rad = self._extract_segment(
            request, index, from_point, to_point, joint_trajectory,
            planning_time_sec=float(getattr(move_result, 'planning_time', 0.0)))

        self._check_endpoints(request, segment, start_rad, to_point, end_rad)

        return segment, end_rad

    def _build_goal(self,
                    request: GenerationRequest,
                    to_point: SourcePoint,
                    start_rad: Sequence[float]) -> MoveGroup.Goal:
        """MotionPlanRequest con start_state EXPLICITO y objetivo articular."""
        # ── Estado inicial explicito ────────────────────────────────────
        joint_state = JointState()
        joint_state.name = list(request.joint_names)
        joint_state.position = [float(v) for v in start_rad]

        start_state = RobotState()
        start_state.joint_state = joint_state
        start_state.is_diff = False

        # ── Objetivo articular ──────────────────────────────────────────
        joint_constraints = []
        for name, value_rad in zip(request.joint_names, to_point.joints_rad):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = float(value_rad)
            constraint.tolerance_above = self._joint_goal_tolerance_rad
            constraint.tolerance_below = self._joint_goal_tolerance_rad
            constraint.weight = 1.0
            joint_constraints.append(constraint)

        constraints = Constraints()
        constraints.name = f'goal_{to_point.id}'
        constraints.joint_constraints = joint_constraints

        workspace = WorkspaceParameters()
        workspace.header.frame_id = self._base_frame
        workspace.min_corner.x = -2.0
        workspace.min_corner.y = -2.0
        workspace.min_corner.z = -2.0
        workspace.max_corner.x = 2.0
        workspace.max_corner.y = 2.0
        workspace.max_corner.z = 2.0

        motion_request = MotionPlanRequest()
        motion_request.group_name = self._planning_group
        motion_request.num_planning_attempts = self._planning_attempts
        motion_request.allowed_planning_time = self._planning_time
        motion_request.max_velocity_scaling_factor = (
            self._effective_velocity_scaling(request))
        motion_request.max_acceleration_scaling_factor = (
            self._effective_acceleration_scaling(request))
        motion_request.goal_constraints = [constraints]
        motion_request.workspace_parameters = workspace
        motion_request.start_state = start_state
        # Si estan vacios NO se tocan los campos: move_group aplica el
        # pipeline y el planner por defecto del proyecto (ompl /
        # RRTConnectkConfigDefault), exactamente igual que hoy.
        if self._pipeline_id:
            motion_request.pipeline_id = self._pipeline_id
        if self._planner_id:
            motion_request.planner_id = self._planner_id

        options = PlanningOptions()
        # INVARIANTE DE SEGURIDAD: este entorno solo planifica.
        options.plan_only = True
        options.replan = False
        options.look_around = False

        goal = MoveGroup.Goal()
        goal.request = motion_request
        goal.planning_options = options
        return goal

    def _effective_velocity_scaling(self, request: GenerationRequest) -> float:
        """La peticion puede sobrescribir el valor del config."""
        if request.velocity_scaling is not None:
            return float(request.velocity_scaling)
        return self._velocity_scaling

    def _effective_acceleration_scaling(self,
                                        request: GenerationRequest) -> float:
        if request.acceleration_scaling is not None:
            return float(request.acceleration_scaling)
        return self._acceleration_scaling

    def _wait_for(self, future, timeout_sec: float) -> bool:
        """Espera activa (el executor multihilo sigue procesando)."""
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not self._shutdown.is_set():
            if future.done():
                return True
            if time.monotonic() > deadline:
                return False
            time.sleep(0.01)
        return future.done()

    # ─────────────────────────────────────────────────────────────────
    # Extraccion de la trayectoria tal cual la entrego MoveIt2
    # ─────────────────────────────────────────────────────────────────

    def _extract_segment(self,
                         request: GenerationRequest,
                         index: int,
                         from_point: SourcePoint,
                         to_point: SourcePoint,
                         joint_trajectory,
                         planning_time_sec: float):
        """
        Copia literal de trajectory_msgs/msg/JointTrajectory al JSON.

        Lo unico que se hace sobre los datos es REORDENAR las columnas al
        orden canonico de request.joint_names (MoveIt2 puede devolver otro
        orden). Los valores no se tocan; el orden original queda registrado
        en el campo moveit_joint_names del segmento.
        """
        moveit_names = list(joint_trajectory.joint_names)
        try:
            order = [moveit_names.index(name) for name in request.joint_names]
        except ValueError as exc:
            missing = [n for n in request.joint_names if n not in moveit_names]
            raise _SegmentPlanningError(
                'La trayectoria de MoveIt2 no contiene los joints '
                f'{missing}. joint_names devueltos: {moveit_names}.') from exc

        extra = [n for n in moveit_names if n not in request.joint_names]
        if extra:
            self.get_logger().warn(
                f'[T{index}] MoveIt2 devolvio joints adicionales {extra}; no '
                'forman parte de joint_names y no se incluyen en el '
                'resultado.')

        trajectory_points: List[Dict[str, Any]] = []
        for point_index, point in enumerate(joint_trajectory.points):
            positions = self._reorder(point.positions, order)
            if positions is None:
                raise _SegmentPlanningError(
                    f'El punto {point_index} de la trayectoria no trae '
                    'posiciones para todos los joints.')
            trajectory_points.append(build_trajectory_point(
                index=point_index,
                time_from_start_sec=duration_to_sec(point.time_from_start),
                positions_rad=positions,
                # Vacias si MoveIt2 no las entrego: nunca se inventan ceros.
                velocities_rad_s=self._reorder(point.velocities, order) or [],
                accelerations_rad_s2=(
                    self._reorder(point.accelerations, order) or []),
            ))

        segment = build_segment(
            segment_index=index,
            from_point_id=from_point.id,
            to_point_id=to_point.id,
            trajectory_points=trajectory_points,
            moveit_joint_names=moveit_names,
            planning_time_sec=planning_time_sec,
        )

        if self._log_points:
            for tp in trajectory_points:
                self.get_logger().info(
                    f'  [T{index}] #{tp["index"]:03d} '
                    f't={tp["time_from_start_sec"]:.3f}s '
                    'deg=[' + ', '.join(
                        f'{v:.3f}' for v in tp['positions_deg']) + ']')

        end_rad = list(trajectory_points[-1]['positions_rad'])
        return segment, end_rad

    @staticmethod
    def _reorder(values: Sequence[float],
                 order: Sequence[int]) -> Optional[List[float]]:
        """
        Reordena un array del punto al orden canonico.

        Devuelve None si el array esta vacio o es mas corto de lo necesario
        (dato ausente): el llamador decide si eso es un error o una lista
        vacia.
        """
        if values is None or len(values) == 0:
            return None
        if max(order) >= len(values):
            return None
        return [float(values[i]) for i in order]

    def _check_endpoints(self,
                         request: GenerationRequest,
                         segment: Dict[str, Any],
                         start_rad: Sequence[float],
                         to_point: SourcePoint,
                         end_rad: Sequence[float]):
        """
        Avisa (no modifica) si MoveIt2 no arranco o no termino donde se pidio.

        Ocurre, por ejemplo, cuando un punto capturado del KUKA queda fuera de
        los limites del URDF: el adaptador FixStartStateBounds ajusta el
        estado inicial silenciosamente. El dato se conserva tal cual; el aviso
        queda en el log y en el JSON de resultado.
        """
        first_rad = segment['trajectory_points'][0]['positions_rad']
        tolerance = self._endpoint_tolerance_rad
        segment_id = segment['segment_id']

        start_dev = [abs(a - b) for a, b in zip(first_rad, start_rad)]
        if start_dev and max(start_dev) > tolerance:
            message = (
                f'{segment_id}: el primer punto devuelto por MoveIt2 difiere '
                f'del estado inicial pedido hasta '
                f'{rad_to_deg(max(start_dev)):.3f} deg (posible ajuste por '
                'limites articulares).')
            self.get_logger().warn(message)
            request.warnings.append(message)

        goal_dev = [abs(a - b) for a, b in zip(end_rad, to_point.joints_rad)]
        if goal_dev and max(goal_dev) > tolerance:
            message = (
                f'{segment_id}: el ultimo punto difiere del objetivo '
                f'{to_point.id} hasta {rad_to_deg(max(goal_dev)):.3f} deg '
                '(tolerancia del objetivo articular).')
            self.get_logger().warn(message)
            request.warnings.append(message)

    # ─────────────────────────────────────────────────────────────────
    # Salida
    # ─────────────────────────────────────────────────────────────────

    def _publish_result(self, result: Dict[str, Any]):
        try:
            text = dumps_json(result)
        except ValueError as exc:
            self.get_logger().error(
                f'No se pudo serializar el resultado: {exc}')
            text = dumps_json(build_error_result(
                str(result.get('request_id', '')),
                f'No se pudo serializar el resultado: {exc}'))
        message = String()
        message.data = text
        self._result_pub.publish(message)

    def destroy_node(self):
        self._shutdown.set()
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)
        return super().destroy_node()


class _SegmentPlanningError(Exception):
    """Fallo de un segmento concreto. Aborta la secuencia completa."""

    def __init__(self, message: str, error_code: Optional[int] = None):
        super().__init__(message)
        self.message = message
        self.error_code = error_code


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGeneratorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
