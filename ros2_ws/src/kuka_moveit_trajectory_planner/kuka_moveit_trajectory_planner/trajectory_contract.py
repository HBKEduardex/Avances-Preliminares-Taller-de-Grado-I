#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
trajectory_contract.py

Logica pura (sin ROS) del contrato JSON entre la GUI/TCP-IP externa y este
entorno MoveIt2.

Responsabilidades:
  - Validar y normalizar la peticion de GENERACION recibida en
    /kuka_moveit/trajectory_generation/request_json
  - Construir el JSON de RESULTADO publicado en
    /kuka_moveit/trajectory_generation/result_json
  - Validar y normalizar la peticion de PREVISUALIZACION recibida en
    /kuka_moveit/trajectory_preview/request_json

REGLAS DE UNIDADES:
  - La GUI envia SIEMPRE grados (joints_deg).
  - MoveIt2 trabaja SIEMPRE en radianes.
  - El resultado guarda AMBAS representaciones: los radianes son el dato
    original entregado por MoveIt2; los grados son una conversion de
    conveniencia para la GUI.

REGLAS DE FIDELIDAD (impuestas por el encargo):
  - No se interpola, no se suaviza, no se re-muestrea y no se recalcula nada.
  - Si MoveIt2 no entrega velocidades o aceleraciones, el campo se devuelve
    como lista vacia. NUNCA se inventa un cero.
  - Cada transicion Pi -> P(i+1) es un SEGMENTO independiente. Los segmentos
    no se concatenan ni se optimizan.
"""

import json
import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

# ─────────────────────────────────────────────────────────────────────────────
# Constantes del contrato
# ─────────────────────────────────────────────────────────────────────────────

SCHEMA_VERSION = 1

RAD_TO_DEG = 180.0 / math.pi
DEG_TO_RAD = math.pi / 180.0

#: Orden canonico de los joints del KUKA KR6 R900 (URDF kr6r900sixx_macro.xacro)
DEFAULT_JOINT_NAMES = [
    'joint_a1',
    'joint_a2',
    'joint_a3',
    'joint_a4',
    'joint_a5',
    'joint_a6',
]

#: Unico modo de planificacion admitido en este entorno.
#: (Sin Pilz, sin LIN/CIRC/SCIRC, sin deteccion de figuras.)
PLANNER_MODE_BASE = 'moveit_base'

#: Estado inicial de la garra por defecto.
DEFAULT_GRIPPER_INITIAL_STATE = 'open'

STATUS_OK = 'ok'
STATUS_ERROR = 'error'

#: Tolerancia (rad) para considerar que la primera pose de un segmento es la
#: MISMA que la ultima ya reproducida. Las fronteras que genera MoveIt2 son
#: identicas bit a bit; la tolerancia solo cubre reserializaciones del JSON.
DEFAULT_BOUNDARY_TOLERANCE_RAD = 1e-6

PREVIEW_STATUS_PLAYING = 'playing'
PREVIEW_STATUS_COMPLETED = 'completed'
PREVIEW_STATUS_CANCELLED = 'cancelled'
PREVIEW_STATUS_ERROR = 'error'


class ContractError(ValueError):
    """Peticion JSON invalida: no se puede procesar y debe devolverse error."""


# ─────────────────────────────────────────────────────────────────────────────
# Helpers basicos
# ─────────────────────────────────────────────────────────────────────────────

def deg_to_rad(value: float) -> float:
    return float(value) * DEG_TO_RAD


def rad_to_deg(value: float) -> float:
    return float(value) * RAD_TO_DEG


def deg_list_to_rad(values: Sequence[float]) -> List[float]:
    return [deg_to_rad(v) for v in values]


def rad_list_to_deg(values: Sequence[Optional[float]]) -> List[Optional[float]]:
    """rad -> deg conservando None (dato ausente) como None."""
    return [None if v is None else rad_to_deg(v) for v in values]


def loads_json(text: str) -> Dict[str, Any]:
    """json.loads con error de contrato en vez de excepcion cruda."""
    try:
        payload = json.loads(text)
    except Exception as exc:                      # noqa: BLE001
        raise ContractError(f'JSON invalido: {exc}') from exc
    if not isinstance(payload, dict):
        raise ContractError(
            'El JSON raiz debe ser un objeto, no '
            f'{type(payload).__name__}')
    return payload


def dumps_json(payload: Dict[str, Any]) -> str:
    """Serializacion estable (sin NaN: JSON estandar no admite NaN)."""
    return json.dumps(payload, ensure_ascii=False, allow_nan=False)


def _as_float_list(values: Any, context: str) -> List[float]:
    if not isinstance(values, (list, tuple)):
        raise ContractError(f'{context}: se esperaba una lista de numeros')
    out: List[float] = []
    for i, v in enumerate(values):
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            raise ContractError(
                f'{context}[{i}]: se esperaba un numero, se recibio '
                f'{type(v).__name__}')
        fv = float(v)
        if math.isnan(fv) or math.isinf(fv):
            raise ContractError(f'{context}[{i}]: valor no finito ({v})')
        out.append(fv)
    return out


def _as_optional_float_list(values: Any, context: str) -> List[Optional[float]]:
    """Lista que admite null (dato que MoveIt2 no entrego)."""
    if values is None:
        return []
    if not isinstance(values, (list, tuple)):
        raise ContractError(f'{context}: se esperaba una lista o null')
    out: List[Optional[float]] = []
    for i, v in enumerate(values):
        if v is None:
            out.append(None)
            continue
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            raise ContractError(
                f'{context}[{i}]: se esperaba un numero o null')
        fv = float(v)
        if math.isnan(fv) or math.isinf(fv):
            raise ContractError(f'{context}[{i}]: valor no finito ({v})')
        out.append(fv)
    return out


# ─────────────────────────────────────────────────────────────────────────────
# Peticion de GENERACION
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class SourcePoint:
    """Configuracion articular real capturada del KUKA por la GUI."""

    id: str
    joints_deg: List[float]

    @property
    def joints_rad(self) -> List[float]:
        return deg_list_to_rad(self.joints_deg)

    def to_dict(self) -> Dict[str, Any]:
        return {
            'id': self.id,
            'joints_deg': list(self.joints_deg),
            # Conversion de conveniencia: es exactamente joints_deg en radianes.
            'joints_rad': self.joints_rad,
        }


@dataclass
class GenerationRequest:
    """Peticion normalizada de generacion de trayectorias."""

    schema_version: int
    request_id: str
    joint_names: List[str]
    points: List[SourcePoint]
    gripper: Dict[str, Any]
    planner_mode: str
    execute: bool
    velocity_scaling: Optional[float] = None
    acceleration_scaling: Optional[float] = None
    warnings: List[str] = field(default_factory=list)

    @property
    def segment_count(self) -> int:
        return max(0, len(self.points) - 1)

    def segment_pairs(self) -> List[Tuple[int, SourcePoint, SourcePoint]]:
        """
        Parejas consecutivas: (indice base 1, Pi, P(i+1)).

        T1 = P1 -> P2, T2 = P2 -> P3, ... T(N-1) = P(N-1) -> PN.
        La posicion final de un segmento es exactamente el inicio del
        siguiente, porque ambos son el MISMO punto de la lista.
        """
        return [
            (i + 1, self.points[i], self.points[i + 1])
            for i in range(self.segment_count)
        ]


def parse_generation_request(
        payload: Dict[str, Any],
        default_joint_names: Optional[Sequence[str]] = None,
) -> GenerationRequest:
    """
    Valida la peticion de generacion y la normaliza.

    Lanza ContractError si la peticion no puede procesarse. Los problemas que
    NO impiden planificar (por ejemplo un evento de garra que referencia un
    punto inexistente) se devuelven como warnings.
    """
    default_names = list(default_joint_names or DEFAULT_JOINT_NAMES)
    warnings: List[str] = []

    # ── schema_version ───────────────────────────────────────────────────
    schema_version = payload.get('schema_version', SCHEMA_VERSION)
    if isinstance(schema_version, bool) or not isinstance(schema_version, int):
        raise ContractError('schema_version debe ser un entero')
    if schema_version != SCHEMA_VERSION:
        warnings.append(
            f'schema_version={schema_version} distinto del soportado '
            f'({SCHEMA_VERSION}); se procesa igualmente.')

    # ── request_id ───────────────────────────────────────────────────────
    request_id = payload.get('request_id', '')
    if request_id is None:
        request_id = ''
    if not isinstance(request_id, str):
        raise ContractError('request_id debe ser una cadena')

    # ── joint_names ──────────────────────────────────────────────────────
    raw_names = payload.get('joint_names', default_names)
    if not isinstance(raw_names, (list, tuple)) or not raw_names:
        raise ContractError('joint_names debe ser una lista no vacia')
    joint_names = []
    for i, name in enumerate(raw_names):
        if not isinstance(name, str) or not name:
            raise ContractError(f'joint_names[{i}] debe ser una cadena no vacia')
        joint_names.append(name)
    if len(set(joint_names)) != len(joint_names):
        raise ContractError('joint_names contiene nombres repetidos')

    # ── points ───────────────────────────────────────────────────────────
    raw_points = payload.get('points')
    if not isinstance(raw_points, (list, tuple)):
        raise ContractError('points debe ser una lista')
    if len(raw_points) < 2:
        raise ContractError(
            'Se necesitan al menos 2 puntos para generar un segmento '
            f'(recibidos: {len(raw_points)})')

    points: List[SourcePoint] = []
    seen_ids = set()
    for i, raw in enumerate(raw_points):
        if not isinstance(raw, dict):
            raise ContractError(f'points[{i}] debe ser un objeto')
        point_id = raw.get('id', f'P{i + 1}')
        if not isinstance(point_id, str) or not point_id:
            raise ContractError(f'points[{i}].id debe ser una cadena no vacia')
        if point_id in seen_ids:
            warnings.append(
                f'points[{i}].id="{point_id}" esta repetido; los eventos de '
                'garra que lo referencien son ambiguos.')
        seen_ids.add(point_id)

        joints_deg = _as_float_list(
            raw.get('joints_deg'), f'points[{i}].joints_deg')
        if len(joints_deg) != len(joint_names):
            raise ContractError(
                f'points[{i}].joints_deg tiene {len(joints_deg)} valores y '
                f'joint_names tiene {len(joint_names)} nombres')
        points.append(SourcePoint(id=point_id, joints_deg=joints_deg))

    # ── gripper (no afecta a MoveIt: solo se conserva y se devuelve) ─────
    gripper = _normalize_gripper(payload.get('gripper'), seen_ids, warnings)

    # ── planner ──────────────────────────────────────────────────────────
    raw_planner = payload.get('planner', {})
    if raw_planner is None:
        raw_planner = {}
    if not isinstance(raw_planner, dict):
        raise ContractError('planner debe ser un objeto')

    planner_mode = raw_planner.get('mode', PLANNER_MODE_BASE)
    if not isinstance(planner_mode, str):
        raise ContractError('planner.mode debe ser una cadena')
    if planner_mode != PLANNER_MODE_BASE:
        raise ContractError(
            f'planner.mode="{planner_mode}" no soportado. Este entorno solo '
            f'admite "{PLANNER_MODE_BASE}" (pipeline base ya configurado en '
            'el proyecto).')

    execute = raw_planner.get('execute', False)
    if execute is None:
        execute = False
    if not isinstance(execute, bool):
        raise ContractError('planner.execute debe ser booleano')
    if execute:
        raise ContractError(
            'planner.execute=true no esta permitido: este entorno SOLO '
            'planifica y previsualiza. La ejecucion pertenece al entorno '
            'TCP/IP.')

    velocity_scaling = _optional_scaling(
        raw_planner.get('velocity_scaling'), 'planner.velocity_scaling')
    acceleration_scaling = _optional_scaling(
        raw_planner.get('acceleration_scaling'),
        'planner.acceleration_scaling')

    return GenerationRequest(
        schema_version=schema_version,
        request_id=request_id,
        joint_names=joint_names,
        points=points,
        gripper=gripper,
        planner_mode=planner_mode,
        execute=execute,
        velocity_scaling=velocity_scaling,
        acceleration_scaling=acceleration_scaling,
        warnings=warnings,
    )


def _optional_scaling(value: Any, context: str) -> Optional[float]:
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ContractError(f'{context} debe ser un numero entre 0 y 1')
    fv = float(value)
    if not (0.0 < fv <= 1.0):
        raise ContractError(f'{context}={fv} fuera del rango (0, 1]')
    return fv


def _normalize_gripper(raw: Any,
                       known_point_ids: set,
                       warnings: List[str]) -> Dict[str, Any]:
    """
    Normaliza el bloque de garra.

    La garra NO interviene en el calculo de MoveIt. El bloque se conserva tal
    cual para que el otro entorno pueda almacenarlo y ejecutarlo despues.
    Si no se envia, se asume la garra ABIERTA y sin eventos.
    """
    if raw is None:
        return {
            'initial_state': DEFAULT_GRIPPER_INITIAL_STATE,
            'events': [],
        }
    if not isinstance(raw, dict):
        raise ContractError('gripper debe ser un objeto')

    initial_state = raw.get('initial_state', DEFAULT_GRIPPER_INITIAL_STATE)
    if initial_state is None:
        initial_state = DEFAULT_GRIPPER_INITIAL_STATE
    if not isinstance(initial_state, str):
        raise ContractError('gripper.initial_state debe ser una cadena')

    raw_events = raw.get('events', [])
    if raw_events is None:
        raw_events = []
    if not isinstance(raw_events, (list, tuple)):
        raise ContractError('gripper.events debe ser una lista')

    events: List[Dict[str, Any]] = []
    for i, event in enumerate(raw_events):
        if not isinstance(event, dict):
            raise ContractError(f'gripper.events[{i}] debe ser un objeto')
        at_point = event.get('at_point')
        action = event.get('action')
        if not isinstance(at_point, str) or not at_point:
            raise ContractError(
                f'gripper.events[{i}].at_point debe ser una cadena no vacia')
        if not isinstance(action, str) or not action:
            raise ContractError(
                f'gripper.events[{i}].action debe ser una cadena no vacia')
        if at_point not in known_point_ids:
            warnings.append(
                f'gripper.events[{i}].at_point="{at_point}" no corresponde a '
                'ningun punto de la lista; se conserva sin modificar.')
        # Se copia el evento COMPLETO: si la GUI anade campos extra
        # (por ejemplo una anchura de garra), no se pierden.
        events.append(dict(event))

    normalized = dict(raw)
    normalized['initial_state'] = initial_state
    normalized['events'] = events
    return normalized


# ─────────────────────────────────────────────────────────────────────────────
# Construccion del RESULTADO de generacion
# ─────────────────────────────────────────────────────────────────────────────

def build_trajectory_point(index: int,
                           time_from_start_sec: float,
                           positions_rad: Sequence[Optional[float]],
                           velocities_rad_s: Sequence[Optional[float]],
                           accelerations_rad_s2: Sequence[Optional[float]],
                           ) -> Dict[str, Any]:
    """
    Un punto de trayectoria tal y como lo entrego MoveIt2.

    Las listas de velocidad y aceleracion se devuelven VACIAS si MoveIt2 no
    las entrego. Nunca se rellenan con ceros.
    """
    return {
        'index': index,
        'time_from_start_sec': float(time_from_start_sec),
        'positions_rad': list(positions_rad),
        'positions_deg': rad_list_to_deg(positions_rad),
        'velocities_rad_s': list(velocities_rad_s),
        'accelerations_rad_s2': list(accelerations_rad_s2),
    }


def build_segment(segment_index: int,
                  from_point_id: str,
                  to_point_id: str,
                  trajectory_points: List[Dict[str, Any]],
                  moveit_joint_names: Sequence[str],
                  planning_time_sec: Optional[float] = None,
                  ) -> Dict[str, Any]:
    """Un segmento independiente Ti = P(i) -> P(i+1)."""
    duration = (trajectory_points[-1]['time_from_start_sec']
                if trajectory_points else 0.0)
    segment: Dict[str, Any] = {
        'segment_id': f'T{segment_index}',
        'from': from_point_id,
        'to': to_point_id,
        'duration_sec': duration,
        'point_count': len(trajectory_points),
        # Orden de joints tal y como lo devolvio MoveIt2 (informativo).
        # Los valores de trajectory_points estan reordenados al orden
        # canonico de joint_names del resultado.
        'moveit_joint_names': list(moveit_joint_names),
        'trajectory_points': trajectory_points,
    }
    if planning_time_sec is not None:
        segment['planning_time_sec'] = float(planning_time_sec)
    return segment


def build_ok_result(request: GenerationRequest,
                    segments: List[Dict[str, Any]],
                    planner_metadata: Dict[str, Any],
                    ) -> Dict[str, Any]:
    """JSON de resultado correcto para /trajectory_generation/result_json."""
    total_points = sum(len(s['trajectory_points']) for s in segments)
    total_duration = sum(float(s['duration_sec']) for s in segments)
    result = {
        'schema_version': SCHEMA_VERSION,
        'request_id': request.request_id,
        'status': STATUS_OK,
        'joint_names': list(request.joint_names),
        'source_points': [p.to_dict() for p in request.points],
        'gripper': request.gripper,
        'planner_metadata': planner_metadata,
        'segments': segments,
        'summary': {
            'source_point_count': len(request.points),
            'segment_count': len(segments),
            'trajectory_point_count': total_points,
            'total_duration_sec': total_duration,
        },
    }
    if request.warnings:
        result['warnings'] = list(request.warnings)
    return result


def build_error_result(request_id: str,
                       message: str,
                       failed_segment: Optional[str] = None,
                       from_point: Optional[str] = None,
                       to_point: Optional[str] = None,
                       error_code: Optional[int] = None,
                       extra: Optional[Dict[str, Any]] = None,
                       ) -> Dict[str, Any]:
    """
    JSON de error. La GUI NO debe considerar ejecutable esta secuencia.

    Nunca se devuelve status "ok" con segmentos incompletos.
    """
    result: Dict[str, Any] = {
        'schema_version': SCHEMA_VERSION,
        'request_id': request_id or '',
        'status': STATUS_ERROR,
        'message': message,
    }
    if failed_segment is not None:
        result['failed_segment'] = failed_segment
    if from_point is not None:
        result['from_point'] = from_point
    if to_point is not None:
        result['to_point'] = to_point
    if error_code is not None:
        result['moveit_error_code'] = int(error_code)
    if extra:
        result.update(extra)
    return result


# ─────────────────────────────────────────────────────────────────────────────
# Peticion de PREVISUALIZACION
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class PreviewPoint:
    time_from_start_sec: float
    positions_rad: List[float]
    velocities_rad_s: List[float]
    accelerations_rad_s2: List[float]


@dataclass
class PreviewSegment:
    segment_id: str
    from_point: str
    to_point: str
    points: List[PreviewPoint]

    @property
    def duration_sec(self) -> float:
        return self.points[-1].time_from_start_sec if self.points else 0.0


@dataclass
class PreviewRequest:
    request_id: str
    joint_names: List[str]
    segments: List[PreviewSegment]
    warnings: List[str] = field(default_factory=list)
    #: Identificador propio de la peticion de preview (formato envuelto).
    preview_id: str = ''

    @property
    def total_duration_sec(self) -> float:
        return sum(s.duration_sec for s in self.segments)

    @property
    def total_point_count(self) -> int:
        return sum(len(s.points) for s in self.segments)


def extract_trajectory_payload(payload: Dict[str, Any]) -> Dict[str, Any]:
    """
    Devuelve el objeto que contiene REALMENTE la trayectoria.

    La GUI puede enviar la peticion de preview en dos formatos:

      envuelto (el que produce el otro entorno al releer un archivo guardado):
        {"schema_version": 1, "preview_id": "...", "request_id": "...",
         "source_file": "...",
         "trajectory": {"joint_names": [...], "segments": [...], ...}}

      directo (el resultado de la generacion tal cual):
        {"joint_names": [...], "segments": [...], ...}

    En el formato envuelto TODO lo relativo a la trayectoria (joint_names,
    segments, summary, schema_version) vive dentro de "trajectory"; en el
    directo, el propio payload es la trayectoria.
    """
    if 'trajectory' not in payload:
        return payload
    inner = payload['trajectory']
    if inner is None:
        return payload
    if not isinstance(inner, dict):
        raise ContractError(
            'trajectory debe ser un objeto con joint_names y segments '
            f'(se recibio {type(inner).__name__})')
    return inner


def _first_string(*candidates: Any) -> str:
    """Primera cadena no vacia de la lista (o cadena vacia)."""
    for value in candidates:
        if isinstance(value, str) and value:
            return value
    return ''


def peek_preview_request_id(payload: Dict[str, Any]) -> str:
    """
    request_id de una peticion de preview SIN validar el resto.

    Solo se usa para poder responder un status de error con el identificador
    correcto. Mira el nivel raiz y, si no esta, dentro de "trajectory".
    """
    inner = payload.get('trajectory')
    if not isinstance(inner, dict):
        inner = {}
    return _first_string(payload.get('request_id'), inner.get('request_id'))


def parse_preview_request(
        payload: Dict[str, Any],
        default_joint_names: Optional[Sequence[str]] = None,
) -> PreviewRequest:
    """
    Valida una secuencia guardada previamente (el mismo JSON que produce la
    generacion) para reproducirla en RViz2.

    Admite los dos formatos descritos en extract_trajectory_payload():
    envuelto ({"trajectory": {...}}) y directo ({"segments": [...]}).
    TODOS los datos de la trayectoria (joint_names, segments, summary,
    schema_version) se leen del mismo nivel, nunca mezclados.

    Tolerancias deliberadas:
      - Si un punto no trae positions_rad pero si positions_deg, se convierte.
      - time_from_start_sec admite el alias time_from_start.
    No se interpola ni se completa ningun punto que falte.
    """
    default_names = list(default_joint_names or DEFAULT_JOINT_NAMES)
    warnings: List[str] = []

    # La trayectoria puede venir envuelta: se resuelve UNA vez y todo lo
    # relativo a ella se lee de aqui.
    trajectory = extract_trajectory_payload(payload)

    schema_version = trajectory.get(
        'schema_version', payload.get('schema_version', SCHEMA_VERSION))
    if isinstance(schema_version, int) and not isinstance(schema_version, bool):
        if schema_version != SCHEMA_VERSION:
            warnings.append(
                f'schema_version={schema_version} distinto del soportado '
                f'({SCHEMA_VERSION}); se procesa igualmente.')

    # Identidad: la del sobre manda; si no la trae, la de la trayectoria.
    for key in ('request_id', 'preview_id'):
        for source in (payload, trajectory):
            if key in source and source[key] is not None \
                    and not isinstance(source[key], str):
                raise ContractError(f'{key} debe ser una cadena')
    request_id = _first_string(
        payload.get('request_id'), trajectory.get('request_id'))
    preview_id = _first_string(
        payload.get('preview_id'), trajectory.get('preview_id'))

    # joint_names y segments SIEMPRE del nivel de la trayectoria (el nivel
    # raiz solo se usa como respaldo del formato directo).
    raw_names = trajectory.get('joint_names')
    if raw_names is None:
        raw_names = payload.get('joint_names', default_names)
    if not isinstance(raw_names, (list, tuple)) or not raw_names:
        raise ContractError('joint_names debe ser una lista no vacia')
    joint_names = [str(n) for n in raw_names]
    n_joints = len(joint_names)

    raw_segments = trajectory.get('segments')
    if not isinstance(raw_segments, (list, tuple)) or not raw_segments:
        where = '' if trajectory is payload else ' dentro de "trajectory"'
        raise ContractError(f'segments debe ser una lista no vacia{where}')

    segments: List[PreviewSegment] = []
    for si, raw_segment in enumerate(raw_segments):
        if not isinstance(raw_segment, dict):
            raise ContractError(f'segments[{si}] debe ser un objeto')
        segment_id = raw_segment.get('segment_id', f'T{si + 1}')
        raw_points = raw_segment.get('trajectory_points')
        if not isinstance(raw_points, (list, tuple)) or not raw_points:
            raise ContractError(
                f'segments[{si}].trajectory_points debe ser una lista no vacia')

        points: List[PreviewPoint] = []
        previous_time = None
        for pi, raw_point in enumerate(raw_points):
            context = f'segments[{si}].trajectory_points[{pi}]'
            if not isinstance(raw_point, dict):
                raise ContractError(f'{context} debe ser un objeto')

            # ── Posiciones ───────────────────────────────────────────────
            if raw_point.get('positions_rad') is not None:
                positions = _as_float_list(
                    raw_point.get('positions_rad'), f'{context}.positions_rad')
            elif raw_point.get('positions_deg') is not None:
                positions = deg_list_to_rad(_as_float_list(
                    raw_point.get('positions_deg'),
                    f'{context}.positions_deg'))
            else:
                raise ContractError(
                    f'{context}: falta positions_rad (o positions_deg)')
            if len(positions) != n_joints:
                raise ContractError(
                    f'{context}: {len(positions)} posiciones para '
                    f'{n_joints} joints')

            # ── Tiempo ───────────────────────────────────────────────────
            time_value = raw_point.get('time_from_start_sec')
            if time_value is None:
                time_value = raw_point.get('time_from_start')
            if time_value is None:
                raise ContractError(f'{context}: falta time_from_start_sec')
            if isinstance(time_value, bool) or not isinstance(
                    time_value, (int, float)):
                raise ContractError(
                    f'{context}.time_from_start_sec debe ser un numero')
            time_sec = float(time_value)
            if math.isnan(time_sec) or math.isinf(time_sec) or time_sec < 0.0:
                raise ContractError(
                    f'{context}.time_from_start_sec invalido ({time_value})')
            if previous_time is not None and time_sec < previous_time:
                raise ContractError(
                    f'{context}.time_from_start_sec={time_sec} es menor que '
                    f'el del punto anterior ({previous_time})')
            previous_time = time_sec

            # ── Velocidades y aceleraciones (opcionales) ────────────────
            velocities = _sanitize_optional_series(
                raw_point.get('velocities_rad_s'),
                f'{context}.velocities_rad_s', n_joints, warnings)
            accelerations = _sanitize_optional_series(
                raw_point.get('accelerations_rad_s2'),
                f'{context}.accelerations_rad_s2', n_joints, warnings)

            points.append(PreviewPoint(
                time_from_start_sec=time_sec,
                positions_rad=positions,
                velocities_rad_s=velocities,
                accelerations_rad_s2=accelerations,
            ))

        segments.append(PreviewSegment(
            segment_id=str(segment_id),
            from_point=str(raw_segment.get('from', '')),
            to_point=str(raw_segment.get('to', '')),
            points=points,
        ))

    return PreviewRequest(
        request_id=request_id,
        joint_names=joint_names,
        segments=segments,
        warnings=warnings,
        preview_id=preview_id,
    )


def _sanitize_optional_series(raw: Any,
                              context: str,
                              n_joints: int,
                              warnings: List[str]) -> List[float]:
    """
    Devuelve una serie completa o una lista VACIA.

    Un JointTrajectoryPoint de ROS 2 no admite huecos: o el array tiene un
    valor por joint, o va vacio. Si la secuencia guardada trae null o una
    longitud incorrecta, se descarta la serie entera (con warning) en vez de
    inventar valores.
    """
    if raw is None:
        return []
    values = _as_optional_float_list(raw, context)
    if not values:
        return []
    if len(values) != n_joints or any(v is None for v in values):
        warnings.append(
            f'{context}: serie incompleta; se previsualiza sin ella.')
        return []
    return [float(v) for v in values]


def build_preview_status(request_id: str,
                         status: str,
                         message: Optional[str] = None,
                         extra: Optional[Dict[str, Any]] = None,
                         ) -> Dict[str, Any]:
    """JSON de estado para /kuka_moveit/trajectory_preview/status_json."""
    payload: Dict[str, Any] = {
        'schema_version': SCHEMA_VERSION,
        'request_id': request_id or '',
        'status': status,
    }
    if message:
        payload['message'] = message
    if extra:
        payload.update(extra)
    return payload


# ─────────────────────────────────────────────────────────────────────────────
# Secuencia continua de PREVISUALIZACION (solo runtime)
# ─────────────────────────────────────────────────────────────────────────────
#
# La secuencia guardada tiene N segmentos independientes y cada uno reinicia su
# time_from_start_sec en 0.0, ademas de repetir la pose de la frontera:
#
#     Ti[-1] == T(i+1)[0]
#
# Eso es CORRECTO en los datos de MoveIt2 y no se toca. Pero reproducirlo tal
# cual en RViz hace que el robot repita una pose y que el reloj vuelva a cero
# en cada frontera. build_preview_sequence() construye, SOLO EN MEMORIA y SOLO
# para dibujar, una unica secuencia continua:
#
#   - la primera pose repetida de cada frontera se omite (si de verdad coincide);
#   - los tiempos se trasladan a un reloj global monotono que nunca vuelve a 0.
#
# No se interpola, no se recalcula ninguna posicion, velocidad o aceleracion, y
# los segmentos originales quedan intactos.


@dataclass
class PreviewFrame:
    """Una pose de la reproduccion continua (dato copiado, no recalculado)."""

    #: Tiempo en el reloj GLOBAL de la reproduccion (no el del segmento).
    time_from_start_sec: float
    positions_rad: List[float]
    velocities_rad_s: List[float]
    accelerations_rad_s2: List[float]
    #: Trazabilidad hacia el dato original (para el status por segmento).
    segment_index: int
    segment_id: str
    source_point_index: int
    #: True en la primera pose que se reproduce de cada segmento.
    is_segment_start: bool


@dataclass
class PreviewSequence:
    """Las N trayectorias vistas como una sola secuencia visual continua."""

    joint_names: List[str]
    frames: List[PreviewFrame]
    #: Poses de frontera omitidas por estar duplicadas (una por frontera).
    dropped_boundary_count: int = 0

    @property
    def point_count(self) -> int:
        return len(self.frames)

    @property
    def duration_sec(self) -> float:
        return self.frames[-1].time_from_start_sec if self.frames else 0.0

    @property
    def times(self) -> List[float]:
        return [f.time_from_start_sec for f in self.frames]


def same_pose(a: Sequence[float],
              b: Sequence[float],
              tolerance_rad: float = DEFAULT_BOUNDARY_TOLERANCE_RAD) -> bool:
    """True si las dos poses articulares son la misma dentro de la tolerancia."""
    if len(a) != len(b):
        return False
    return all(abs(float(x) - float(y)) <= tolerance_rad for x, y in zip(a, b))


def build_preview_sequence(
        request: PreviewRequest,
        boundary_tolerance_rad: float = DEFAULT_BOUNDARY_TOLERANCE_RAD,
        inter_segment_gap_sec: float = 0.0,
) -> PreviewSequence:
    """
    Aplana los segmentos en una unica secuencia visual continua.

    Reglas (solo afectan a la reproduccion, nunca al dato guardado):

    1. El primer segmento aporta TODOS sus puntos.
    2. Para T2..TN, si la primera pose coincide con la ultima ya reproducida
       dentro de boundary_tolerance_rad, se omite ESE punto y solo ese. Si no
       coincide, no se omite nada. Un segmento de un solo punto nunca se queda
       sin poses.
    3. Los tiempos se trasladan a un reloj global monotono no decreciente. Al
       omitir una frontera, el reloj se ancla en la pose omitida, de modo que
       el siguiente punto conserva exactamente su separacion temporal original.

    Las posiciones, velocidades y aceleraciones se copian tal cual.
    """
    frames: List[PreviewFrame] = []
    dropped = 0
    last_global_time = 0.0

    for segment_index, segment in enumerate(request.segments):
        points = segment.points
        if not points:
            continue

        skip_first = False
        if frames and len(points) > 1:
            skip_first = same_pose(
                frames[-1].positions_rad,
                points[0].positions_rad,
                boundary_tolerance_rad)

        emitted = points[1:] if skip_first else points
        if skip_first:
            dropped += 1

        # ── Traslacion al reloj global ──────────────────────────────────
        if not frames:
            # La reproduccion siempre arranca en 0.0.
            offset = -points[0].time_from_start_sec
        elif skip_first:
            # El punto omitido ES la pose ya dibujada: el reloj se ancla en
            # ella, asi el siguiente punto mantiene su separacion original.
            offset = last_global_time - points[0].time_from_start_sec
        else:
            gap = last_global_time + inter_segment_gap_sec
            offset = gap - emitted[0].time_from_start_sec

        first_source_index = 1 if skip_first else 0
        for local_index, point in enumerate(emitted):
            global_time = offset + point.time_from_start_sec
            # Garantia de monotonia: el reloj nunca retrocede.
            if frames and global_time < last_global_time:
                global_time = last_global_time
            frames.append(PreviewFrame(
                time_from_start_sec=global_time,
                # Copias: los objetos originales no se comparten ni se mutan.
                positions_rad=list(point.positions_rad),
                velocities_rad_s=list(point.velocities_rad_s),
                accelerations_rad_s2=list(point.accelerations_rad_s2),
                segment_index=segment_index,
                segment_id=segment.segment_id,
                source_point_index=first_source_index + local_index,
                is_segment_start=(local_index == 0),
            ))
            last_global_time = global_time

    return PreviewSequence(
        joint_names=list(request.joint_names),
        frames=frames,
        dropped_boundary_count=dropped,
    )
