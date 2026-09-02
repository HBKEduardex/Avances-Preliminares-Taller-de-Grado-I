#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
trajectory_json_writer.py

Serializa un plan generado por el BASELINE al mismo contrato JSON
(schema_version 1) que produce el sistema afinado, para que las dos
condiciones puedan compararse con la MISMA herramienta y el MISMO lector.

═══════════════════════════════════════════════════════════════════════════
  QUE ES ESTE ARCHIVO Y QUE NO ES
═══════════════════════════════════════════════════════════════════════════
  ES     : la salida de MoveIt2 POR DEFECTO planificando la misma tarea,
           desde los mismos puntos enseñados P1..PN. Es la CONDICION BASE
           regenerada, no recuperada.

  NO ES  : la trayectoria preliminar historica. Esa no quedo almacenada.
           Redactar el resultado como "se recupero la preliminar" seria
           falso; lo correcto es "se regenero la condicion base".

  NO ES  : telemetria. Son puntos PLANIFICADOS, no medidos en el robot.

  Por eso `source` y `condition` van marcados en el propio archivo: si
  alguien lo abre sin contexto, no puede confundirlo con uno afinado.
═══════════════════════════════════════════════════════════════════════════

DE DONDE SALE CADA CAMPO
  ESPEJADO       : source_points, gripper, from_point/to_point y
                    execution_profile se COPIAN del JSON de entrada. Describen
                    la TAREA, no el plan, y mantienen el archivo estructural-
                    mente identico a uno valido para los consumidores externos
                    (GUI por TCP/IP incluida).
  source_points   : COPIADOS LITERALMENTE del JSON de entrada. Es lo que
                    hace valida la comparacion: misma tarea, mismas metas.
  trajectory_points: los waypoints que devolvio /move_action. No se
                    recalculan, no se rellenan, no se interpolan.
  positions_deg   : derivados de positions_rad con math.degrees(), para que
                    la verificacion cruzada del lector cierre en 0.000e+00.

Modulo autonomo: no importa ROS ni TG2. Solo escribe donde se le indica y
nunca junto al JSON de origen (prohibicion J.0).
"""

import json
import math
import os
import uuid
from datetime import datetime
from typing import Any, Dict, List, Optional, Sequence

from .kuka_pipeline_limits import EXPERIMENTAL_PTP_VELOCITY_PCT

SCHEMA_VERSION = 1

#: Marca de procedencia. Cualquier consumidor debe poder distinguir este
#: archivo de uno del sistema afinado sin leer el resto del contenido.
BASELINE_SOURCE = 'kuka_kr6_moveit_baseline/BASELINE_SIN_AFINAR'
BASELINE_CONDITION = (
    'BASELINE — MoveIt2 por defecto, sin afinamiento. Puntos PLANIFICADOS, '
    'no telemetria fisica.')

#: Nota #10.4: cartesian_diagnostic se arrastra SOLO como documentacion.
CARTESIAN_DIAGNOSTIC_NOTE = (
    'El campo source_points[k].cartesian_diagnostic se copia literalmente del '
    'JSON de origen UNICAMENTE como documentacion. Prohibicion #10.4: no se '
    'uso como objetivo de planificacion, ni como referencia de pose, ni para '
    'validar ninguna cinematica. Las metas fueron siempre joints_deg.')


class JsonTrajectoryWriteError(Exception):
    """No se pudo escribir el archivo. Se aborta en vez de continuar."""


def default_filename(cartesian: bool, when: Optional[datetime] = None,
                     ptp_velocity_pct: float = EXPERIMENTAL_PTP_VELOCITY_PCT,
                     executable: bool = True) -> str:
    """
    Nombre autoexplicativo: variante + condicion de velocidad + marca temporal.

    Un archivo que NO pasa el preflight lleva `_raw` y jamas el sufijo de
    velocidad, para que no pueda confundirse con uno listo para el robot.
    """
    when = when or datetime.now()
    variant = 'cartesiano' if cartesian else 'articular'
    if not executable:
        return f'baseline_{variant}_raw_{when:%Y%m%d_%H%M%S}.json'
    tag = f'vel{ptp_velocity_pct:g}'.replace('.', 'p')
    return f'baseline_{variant}_{tag}_{when:%Y%m%d_%H%M%S}.json'


def _read_raw_source_points(path: str) -> Dict[str, Any]:
    """
    Relee el JSON de origen en SOLO LECTURA para copiar los bloques que
    definen la tarea. No se modifica, no se mueve, no se renombra.
    """
    with open(path, 'r', encoding='utf-8') as handle:
        raw = json.load(handle)
    return raw


def _waypoint_to_dict(waypoint) -> Dict[str, Any]:
    """
    Un waypoint planificado -> dict del contrato.

    positions_rad es la FUENTE; positions_deg se DERIVA. Velocidades y
    aceleraciones se copian tal cual: si el planificador no las entrego, se
    escribe la lista vacia en vez de ceros inventados.
    """
    rad = [float(v) for v in waypoint.positions_rad]
    return {
        'positions_rad': rad,
        'positions_deg': [math.degrees(v) for v in rad],
        'positions_deg_source': 'derived_from_positions_rad',
        'velocities_rad_s': [float(v) for v in waypoint.velocities_rad_s],
        'accelerations_rad_s2': [
            float(v) for v in waypoint.accelerations_rad_s2],
        'time_from_start_sec': float(waypoint.time_from_start_s),
    }


def build_baseline_document(sequence, groups: Sequence[Sequence[Any]],
                            segment_ids: Sequence[str],
                            failed: Sequence[str],
                            rebuilt: Sequence[str],
                            params,
                            cartesian: bool,
                            planned_at: Optional[datetime] = None,
                            ptp_velocity_pct: float =
                            EXPERIMENTAL_PTP_VELOCITY_PCT
                            ) -> Dict[str, Any]:
    """
    Construye el documento completo.

    REGLA ESTRUCTURAL: todo campo que describe la TAREA se ESPEJA del JSON de
    entrada (source_points, gripper, from_point/to_point, execution_profile).
    Solo trajectory_points es contenido nuevo. Asi el archivo es
    estructuralmente indistinguible de uno valido y lo aceptan los mismos
    consumidores, incluida la GUI externa por TCP/IP.

    Toda la metainformacion propia del baseline va bajo UNA sola clave
    adicional, `baseline_metadata`, para no introducir campos sueltos que un
    validador externo pueda rechazar.

    `params` se lee por atributos (duck typing) para no importar replan_core y
    mantener este modulo sin ROS.
    """
    if len(groups) != len(segment_ids):
        raise JsonTrajectoryWriteError(
            f'Incoherencia interna: {len(groups)} grupos planificados para '
            f'{len(segment_ids)} segmentos. Se aborta la escritura.')

    planned_at = planned_at or datetime.now()
    raw = _read_raw_source_points(sequence.path)
    raw_segments = raw.get('segments') or []

    # ── CONDICION EXPERIMENTAL DE VELOCIDAD ──────────────────────────
    # La geometria de los source_points se conserva intacta. Lo unico que se
    # reescribe es el metadato de velocidad KUKA, y SOLO donde ya existia:
    # crear el campo donde el esquema de origen no lo trae seria inventar
    # estructura. Se registra exactamente que se toco.
    raw_points = []
    velocity_fields_set: List[str] = []
    for point in (raw.get('source_points') or []):
        copy = dict(point)
        if 'incoming_kuka_ptp_velocity_pct' in copy:
            copy['incoming_kuka_ptp_velocity_pct'] = float(ptp_velocity_pct)
            velocity_fields_set.append(
                f'source_points[{copy.get("id")}]'
                f'.incoming_kuka_ptp_velocity_pct')
        raw_points.append(copy)

    segments: List[Dict[str, Any]] = []
    total_waypoints = 0
    total_duration = 0.0
    for k, seg_id in enumerate(segment_ids):
        waypoints = list(groups[k]) if k < len(groups) else []
        points = [_waypoint_to_dict(w) for w in waypoints]
        duration = points[-1]['time_from_start_sec'] if points else 0.0
        total_waypoints += len(points)
        total_duration += duration

        source_seg = raw_segments[k] if k < len(raw_segments) else {}
        segment = {
            'id': seg_id,
            # Espejados del JSON de entrada: describen la tarea, no el plan.
            'from_point': source_seg.get('from_point'),
            'to_point': source_seg.get('to_point'),
            'duration_sec': duration,
        }
        # ── CONDICION EXPERIMENTAL: velocidad PTP UNIFORME ───────────
        # NO se heredan los perfiles 30/5 del archivo de entrada. Esta prueba
        # exige la MISMA velocidad en todos los segmentos para que ninguna
        # diferencia entre baseline y afinado pueda atribuirse a la velocidad.
        # Las demas claves del perfil de origen, si las hubiera, se conservan.
        profile = dict(source_seg.get('execution_profile') or {})
        profile['kuka_ptp_velocity_pct'] = float(ptp_velocity_pct)
        segment['execution_profile'] = profile
        velocity_fields_set.append(
            f'segments[{seg_id}].execution_profile.kuka_ptp_velocity_pct')
        segment['trajectory_points'] = points
        segments.append(segment)

    complete = not failed
    variant = 'cartesiano' if cartesian else 'articular'
    document = {
        'schema_version': SCHEMA_VERSION,
        'request_id': str(uuid.uuid4()),
        'generated_at': planned_at.strftime('%Y-%m-%dT%H:%M:%S'),
        'generated_at_date': planned_at.strftime('%Y-%m-%d'),
        'generated_at_time': planned_at.strftime('%H:%M:%S'),
        'source': BASELINE_SOURCE,
        'source_points': raw_points,
        'joint_names': list(sequence.joint_names),
        # Espejado del origen. El baseline no controla el gripper.
        'gripper': raw.get('gripper') or {'initial_state': '', 'events': []},
        # Exactamente las mismas claves que produce el sistema afinado.
        'planner_metadata': {
            'planning_pipeline': '',
            'planner_id': '',
            'group': getattr(params, 'group', ''),
            'plan_only': True,
            'velocity_scaling': getattr(params, 'vel_scale', None),
            'acceleration_scaling': getattr(params, 'acc_scale', None),
            'planning_attempts': getattr(params, 'attempts', None),
            'allowed_planning_time_sec': getattr(
                params, 'planning_time', None),
            'joint_goal_tolerance_rad': getattr(params, 'joint_tol', None),
            'segment_chaining': 'previous_trajectory_end',
        },
        'segments': segments,
        'summary': {
            'num_source_points': len(raw_points),
            'num_gripper_events': len(sequence.gripper_events),
            'num_segments': len(segments),
            'num_trajectory_points': total_waypoints,
            'total_duration_sec': total_duration,
        },
        # ── UNICA clave fuera del contrato original ──────────────────
        'baseline_metadata': {
            'condition': BASELINE_CONDITION,
            'schema_status': 'COMPLETO' if complete else 'INCOMPLETO',
            'input_json': sequence.path,
            'input_md5': sequence.md5,
            'input_source': sequence.source,
            'input_generated_at': sequence.generated_at,
            'goal_variant': variant,
            'goals_taken_from': 'source_points[k].joints_deg',
            'intermediate_waypoints_reused': False,
            'planned_by': '/move_action (move_group), plan_only=True',
            'base_frame': getattr(params, 'base_frame', ''),
            'tip_frame': getattr(params, 'tip_frame', ''),
            'position_tolerance_m': getattr(params, 'pos_tol', None),
            'orientation_tolerance_rad': getattr(params, 'ori_tol', None),
            'failed_segments': list(failed),
            'chain_rebuilt_segments': list(rebuilt),
            'resolved_segments': len(segments) - len(failed),
            'mirrored_from_input': [
                'source_points (geometria, ids, captured_at)', 'gripper',
                'segments[k].from_point', 'segments[k].to_point'],
            'experimental_velocity_condition': {
                'kuka_ptp_velocity_pct': float(ptp_velocity_pct),
                'applied_to': 'TODOS los segmentos, sin excepcion',
                'inherited_from_input': False,
                'note': (
                    'Condicion de EJECUCION fisica. NO afecta a '
                    'velocities_rad_s, accelerations_rad_s2 ni '
                    'time_from_start_sec, que conservan el resultado real de '
                    'MoveIt sin escalar.'),
                'fields_set': velocity_fields_set,
            },
            'cartesian_diagnostic_note': CARTESIAN_DIAGNOSTIC_NOTE,
            'warning': (
                '' if complete else
                'Archivo INCOMPLETO: los segmentos listados en '
                'failed_segments se escribieron con trajectory_points vacio '
                'porque el baseline no los resolvio. No se invento ningun '
                'punto. El lector estricto rechazara este archivo, y con '
                'razon: la comparacion exige un plan completo.'),
        },
    }
    return document


def write_baseline_sequence(out_dir: str, sequence, groups, segment_ids,
                            failed, rebuilt, params, cartesian: bool,
                            filename: Optional[str] = None,
                            planned_at: Optional[datetime] = None,
                            ptp_velocity_pct: float =
                            EXPERIMENTAL_PTP_VELOCITY_PCT,
                            executable: bool = True,
                            document: Optional[Dict[str, Any]] = None) -> str:
    """
    Escribe el plan del baseline y devuelve la RUTA ABSOLUTA del archivo.

    Nunca sobrescribe: si el nombre ya existe, se aborta. Nunca escribe en el
    directorio del JSON de origen (prohibicion J.0).
    """
    planned_at = planned_at or datetime.now()
    out_dir = os.path.abspath(os.path.expanduser(out_dir))
    source_dir = os.path.dirname(os.path.abspath(sequence.path))
    if out_dir == source_dir:
        raise JsonTrajectoryWriteError(
            f'Destino invalido: {out_dir} es el directorio del JSON de '
            'origen. Prohibido escribir junto a los archivos de referencia.')

    if document is None:
        document = build_baseline_document(
            sequence, groups, segment_ids, failed, rebuilt, params, cartesian,
            planned_at=planned_at, ptp_velocity_pct=ptp_velocity_pct)
    if not executable:
        document['baseline_metadata']['schema_status'] = 'RAW_NO_EJECUTABLE'

    os.makedirs(out_dir, exist_ok=True)
    name = filename or default_filename(
        cartesian, planned_at, ptp_velocity_pct, executable)
    out_path = os.path.join(out_dir, name)
    if os.path.exists(out_path):
        raise JsonTrajectoryWriteError(
            f'Ya existe {out_path}. No se sobrescribe ningun plan anterior.')

    with open(out_path, 'w', encoding='utf-8') as handle:
        json.dump(document, handle, indent=2, ensure_ascii=False)
        handle.write('\n')
    return out_path
