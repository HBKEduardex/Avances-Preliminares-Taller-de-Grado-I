#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
trajectory_json_reader.py

Lector de secuencias de trayectoria en JSON, PROPIO del paquete baseline.

═══════════════════════════════════════════════════════════════════════════
  ADVERTENCIA DE PROCEDENCIA — LEER ANTES DE USAR CUALQUIER METRICA
═══════════════════════════════════════════════════════════════════════════
  Los archivos que lee este modulo fueron generados por el SISTEMA AFINADO
  en el entorno TG2. NO son salida de la condicion base.

  Cualquier metrica calculada sobre ellos caracteriza la CONDICION AFINADA.
  Etiquetarla como baseline invalida la comparacion del estudio.

  Por eso toda salida derivada de este modulo lleva
      condition = AFINADA / ORIGEN EXTERNO
  en el nombre de archivo, en la cabecera del CSV y en la consola.
═══════════════════════════════════════════════════════════════════════════

═══════════════════════════════════════════════════════════════════════════
  PROHIBICION DURA — cartesian_diagnostic  (decision #10.4 del operador)
═══════════════════════════════════════════════════════════════════════════
  El campo  source_points[k].cartesian_diagnostic  NO SE USA JAMAS:
    - ni como objetivo de planificacion,
    - ni como referencia de pose,
    - ni para validar ninguna cinematica.

  MOTIVO, medido: en P1 ese campo da X=674.677551, Y=-2.331047, Z=885.201538,
  mientras que la FK de ESTE paquete para los mismos valores articulares da
  [525.0000, -0.0000, 890.0055] mm en link_6, flange Y tool0 (los tres
  comparten origen). La diferencia es un offset de

        [+149.678, -2.331, -4.804] mm      modulo 149.773 mm

  que corresponde a una transformacion de TCP que TG2 aplica y que el URDF de
  taller1 NO declara. Usar esos valores como objetivo apuntaria a poses
  desplazadas ~150 mm respecto a la tarea real en el modelo del baseline.

  El lector lo expone SOLO como metadato documental, en el campo
  `cartesian_diagnostic_raw`, y ninguna funcion de este paquete lo consume.
  Si alguna vez hace falta una pose cartesiana de Pk, se DERIVA de los joints
  con continuity_metrics.forward_kinematics_tip(). Nunca de aqui.
═══════════════════════════════════════════════════════════════════════════

Este modulo es autonomo: no importa nada de TG2 ni de otros paquetes de
taller1, y no depende de ROS (se puede probar sin move_group).
"""

import hashlib
import json
import math
import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from .continuity_metrics import AXES, JOINT_NAMES, Waypoint

#: Version del esquema que este lector entiende.
SUPPORTED_SCHEMA_VERSIONS = (1,)

#: Tolerancia de la verificacion cruzada rad <-> deg. Si se supera, se ABORTA.
#: El archivo de referencia cierra en 0.000e+00, asi que cualquier
#: discrepancia real es un fallo de integridad, no ruido numerico.
RAD_DEG_CROSS_CHECK_TOLERANCE_DEG = 1.0e-9

#: Limites articulares en grados, CONGELADOS desde urdf/kr6r900sixx_macro.xacro
#: de este paquete (atributos lower/upper de cada <limit>).
JOINT_LIMITS_DEG: List[Tuple[float, float]] = [
    (-170.0, 170.0),    # joint_a1
    (-190.0, 45.0),     # joint_a2
    (-120.0, 156.0),    # joint_a3
    (-185.0, 185.0),    # joint_a4
    (-120.0, 120.0),    # joint_a5
    (-350.0, 350.0),    # joint_a6
]


class JsonTrajectoryError(Exception):
    """El archivo no cumple el contrato. Se aborta en vez de continuar."""


@dataclass
class RecordedSegment:
    """Un segmento Tk de la secuencia grabada."""

    id: str
    index: int                       # 0-based
    waypoints: List[Waypoint]
    duration_sec: float
    execution_profile: Dict[str, Any] = field(default_factory=dict)
    #: indices 0-based de los source_points que este segmento une
    from_point_index: int = -1
    to_point_index: int = -1
    #: desviacion maxima [deg] entre los extremos reales y los source_points
    start_deviation_deg: float = 0.0
    end_deviation_deg: float = 0.0


@dataclass
class RecordedSequence:
    """Una secuencia completa leida de un JSON externo."""

    path: str
    md5: str
    schema_version: int
    source: str
    generated_at: str
    joint_names: List[str]
    #: P1..PN en RADIANES, derivados de joints_deg
    source_points_rad: List[List[float]]
    source_point_ids: List[str]
    segments: List[RecordedSegment]
    gripper_initial_state: str
    gripper_events: List[Dict[str, Any]]
    planner_metadata: Dict[str, Any]
    summary: Dict[str, Any]
    #: SOLO DOCUMENTAL. Ver la prohibicion dura de la cabecera. No consumir.
    cartesian_diagnostic_raw: List[Optional[Dict[str, float]]] = field(
        default_factory=list)
    #: hallazgos del control de integridad, para reportar en consola
    checks: List[str] = field(default_factory=list)
    limit_violations: List[Tuple[str, int, str, float]] = field(
        default_factory=list)

    @property
    def total_waypoints(self) -> int:
        return sum(len(s.waypoints) for s in self.segments)

    @property
    def max_endpoint_deviation_deg(self) -> float:
        if not self.segments:
            return 0.0
        return max(max(s.start_deviation_deg, s.end_deviation_deg)
                   for s in self.segments)


# ─────────────────────────────────────────────────────────────────────────────
# Carga y control de integridad
# ─────────────────────────────────────────────────────────────────────────────

def file_md5(path: str) -> str:
    digest = hashlib.md5()
    with open(path, 'rb') as handle:
        for block in iter(lambda: handle.read(65536), b''):
            digest.update(block)
    return digest.hexdigest()


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise JsonTrajectoryError(message)


def _cross_check_units(point: Dict[str, Any], where: str) -> None:
    """
    Verificacion cruzada rad <-> deg. FALLA RUIDOSAMENTE si difieren.

    DECISION DEL OPERADOR: positions_rad es la FUENTE (unidad nativa de ROS) y
    positions_deg es la VERIFICACION. No hay conversion implicita en ningun
    sentido: si las dos representaciones no coinciden, el archivo esta
    corrupto o fue escrito por otro productor, y seguir seria inventar datos.
    """
    rad = point.get('positions_rad')
    deg = point.get('positions_deg')
    if rad is None or deg is None:
        raise JsonTrajectoryError(
            f'{where}: falta positions_rad o positions_deg. La verificacion '
            'cruzada de unidades es obligatoria y no puede omitirse.')
    worst = 0.0
    worst_axis = ''
    for i, (r, g) in enumerate(zip(rad, deg)):
        diff = abs(math.degrees(float(r)) - float(g))
        if diff > worst:
            worst = diff
            worst_axis = AXES[i] if i < len(AXES) else f'idx{i}'
    if worst > RAD_DEG_CROSS_CHECK_TOLERANCE_DEG:
        raise JsonTrajectoryError(
            f'{where}: VERIFICACION CRUZADA DE UNIDADES FALLIDA. '
            f'|degrees(positions_rad) - positions_deg| = {worst:.6e} deg en '
            f'{worst_axis}, por encima de la tolerancia '
            f'{RAD_DEG_CROSS_CHECK_TOLERANCE_DEG:.1e} deg. '
            'Se aborta: continuar significaria elegir una de las dos '
            'representaciones sin saber cual es correcta.')


def load(path: str) -> RecordedSequence:
    """
    Lee y VALIDA un JSON de secuencia. Aborta ante cualquier incoherencia.

    El archivo se abre en SOLO LECTURA. Este modulo nunca escribe, mueve ni
    renombra nada junto al archivo de origen.
    """
    path = os.path.abspath(path)
    _require(os.path.isfile(path), f'No existe el archivo JSON: {path}')

    with open(path, 'r', encoding='utf-8') as handle:
        raw = json.load(handle)

    checks: List[str] = []
    md5 = file_md5(path)

    version = raw.get('schema_version')
    _require(version in SUPPORTED_SCHEMA_VERSIONS,
             f'schema_version {version!r} no soportada. Soportadas: '
             f'{SUPPORTED_SCHEMA_VERSIONS}.')
    checks.append(f'schema_version = {version} (soportada)')

    joint_names = list(raw.get('joint_names') or [])
    _require(joint_names == JOINT_NAMES,
             f'joint_names del archivo {joint_names} no coincide con el orden '
             f'canonico del grupo {JOINT_NAMES}.')
    checks.append(f'joint_names coinciden con el orden canonico ({len(JOINT_NAMES)})')

    raw_points = raw.get('source_points') or []
    _require(len(raw_points) >= 2,
             f'Se necesitan al menos 2 source_points; hay {len(raw_points)}.')
    source_points_rad = []
    source_point_ids = []
    cartesian_raw: List[Optional[Dict[str, float]]] = []
    for k, sp in enumerate(raw_points):
        deg = sp.get('joints_deg')
        _require(isinstance(deg, list) and len(deg) == 6,
                 f'source_points[{k}] no tiene exactamente 6 joints_deg.')
        source_points_rad.append([math.radians(float(v)) for v in deg])
        source_point_ids.append(str(sp.get('id') or f'P{k + 1}'))
        # SOLO DOCUMENTAL. Ver la prohibicion dura de la cabecera del modulo.
        cartesian_raw.append(sp.get('cartesian_diagnostic'))
    checks.append(f'source_points: {len(source_points_rad)}, todos con 6 componentes')

    raw_segments = raw.get('segments') or []
    _require(bool(raw_segments), 'El archivo no contiene segmentos.')
    _require(len(raw_segments) == len(source_points_rad) - 1,
             f'Incoherencia estructural: {len(raw_segments)} segmentos para '
             f'{len(source_points_rad)} source_points. El contrato exige '
             f'{len(source_points_rad) - 1}.')
    checks.append(f'segmentos: {len(raw_segments)} = source_points - 1 (coherente)')

    segments: List[RecordedSegment] = []
    limit_violations: List[Tuple[str, int, str, float]] = []
    rad_deg_worst = 0.0

    for k, seg in enumerate(raw_segments):
        seg_id = str(seg.get('id') or f'T{k + 1}')
        raw_wps = seg.get('trajectory_points') or []
        _require(bool(raw_wps), f'{seg_id}: sin trajectory_points.')

        waypoints: List[Waypoint] = []
        for i, point in enumerate(raw_wps):
            where = f'{seg_id} punto {i}'
            # Verificacion cruzada OBLIGATORIA. Aborta si falla.
            _cross_check_units(point, where)
            rad = [float(v) for v in point['positions_rad']]
            deg = [float(v) for v in point['positions_deg']]
            _require(len(rad) == 6, f'{where}: positions_rad no tiene 6 valores.')
            rad_deg_worst = max(
                rad_deg_worst,
                max(abs(math.degrees(r) - g) for r, g in zip(rad, deg)))

            for j, value_deg in enumerate(deg):
                low, high = JOINT_LIMITS_DEG[j]
                if value_deg < low or value_deg > high:
                    limit_violations.append(
                        (seg_id, i, JOINT_NAMES[j], value_deg))

            # Velocidades y aceleraciones: se usan TAL CUAL si vienen.
            # NO se recalculan y NO se rellenan con ceros inventados.
            velocities = point.get('velocities_rad_s') or []
            accelerations = point.get('accelerations_rad_s2') or []
            waypoints.append(Waypoint(
                index=i,
                time_from_start_s=float(point.get('time_from_start_sec', 0.0)),
                positions_rad=rad,
                velocities_rad_s=[float(v) for v in velocities],
                accelerations_rad_s2=[float(v) for v in accelerations],
            ))

        # from_point / to_point vienen a null en el archivo de referencia.
        # Se resuelven POR INDICE segun el contrato: Tk une Pk con P(k+1).
        start_dev = max(
            abs(math.degrees(a - b)) for a, b in
            zip(waypoints[0].positions_rad, source_points_rad[k]))
        end_dev = max(
            abs(math.degrees(a - b)) for a, b in
            zip(waypoints[-1].positions_rad, source_points_rad[k + 1]))

        segments.append(RecordedSegment(
            id=seg_id,
            index=k,
            waypoints=waypoints,
            duration_sec=float(seg.get('duration_sec', 0.0)),
            execution_profile=dict(seg.get('execution_profile') or {}),
            from_point_index=k,
            to_point_index=k + 1,
            start_deviation_deg=start_dev,
            end_deviation_deg=end_dev,
        ))

    checks.append(
        f'verificacion cruzada rad/deg: peor caso {rad_deg_worst:.3e} deg '
        f'(tolerancia {RAD_DEG_CROSS_CHECK_TOLERANCE_DEG:.1e}) — SUPERADA')
    total = sum(len(s.waypoints) for s in segments)
    checks.append(f'waypoints totales: {total}, todos con 6 componentes')
    if limit_violations:
        checks.append(
            f'*** {len(limit_violations)} VIOLACIONES de limites del URDF ***')
    else:
        checks.append('limites del URDF baseline: 0 violaciones')

    gripper = raw.get('gripper') or {}
    sequence = RecordedSequence(
        path=path,
        md5=md5,
        schema_version=int(version),
        source=str(raw.get('source') or 'desconocido'),
        generated_at=str(raw.get('generated_at') or ''),
        joint_names=joint_names,
        source_points_rad=source_points_rad,
        source_point_ids=source_point_ids,
        segments=segments,
        gripper_initial_state=str(gripper.get('initial_state') or ''),
        gripper_events=list(gripper.get('events') or []),
        planner_metadata=dict(raw.get('planner_metadata') or {}),
        summary=dict(raw.get('summary') or {}),
        cartesian_diagnostic_raw=cartesian_raw,
        checks=checks,
        limit_violations=limit_violations,
    )
    return sequence
