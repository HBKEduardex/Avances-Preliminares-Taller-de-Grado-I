#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
baseline_preflight.py

Validacion OFFLINE de un JSON de trayectoria contra el contrato de ejecucion
del pipeline KUKA. NO conecta con el robot, NO envia nada, NO abre EKI.

Comprueba, sobre el archivo ya escrito:
  - JSON bien formado y schema reconocido
  - NaN / Inf
  - soft limits de posicion
  - salto articular entre puntos consecutivos (intra-segmento Y en uniones)
  - execution_profile presente y a la velocidad experimental en TODOS los
    segmentos
  - eventos de garra referidos a source_points existentes
  - marcas de tiempo monotonas
  - joint_names y su orden
  - compatibilidad con el ejecutor por lotes

El PRIMER PUNTO de la trayectoria se reporta aparte: el salto entre la
posicion REAL del robot y ese punto no se puede conocer sin AxisActual, y
este modulo no inventa esa lectura.
"""

import json
import math
from typing import Any, Dict, List, Optional, Tuple

from .continuity_metrics import AXES, JOINT_NAMES
from .kuka_pipeline_limits import (
    EXPERIMENTAL_PTP_VELOCITY_PCT,
    MAX_BATCH_POINTS,
    MAX_JOINT_DELTA_DEG,
    PIPELINE_JOINT_LIMITS_DEG,
    violations_deg,
)


class PreflightResult:
    """Resultado de la validacion. `ok` decide si el JSON es ejecutable."""

    def __init__(self):
        self.errors: List[str] = []
        self.warnings: List[str] = []
        self.stats: Dict[str, Any] = {}
        self.first_point_deg: List[float] = []

    @property
    def ok(self) -> bool:
        return not self.errors

    def error(self, text: str) -> None:
        self.errors.append(text)

    def warn(self, text: str) -> None:
        self.warnings.append(text)


def _worst_delta(points: List[List[float]]
                 ) -> Tuple[float, Optional[str], Optional[int]]:
    worst, axis, where = 0.0, None, None
    for i in range(1, len(points)):
        for j in range(6):
            delta = abs(points[i][j] - points[i - 1][j])
            if delta > worst:
                worst, axis, where = delta, AXES[j], i - 1
    return worst, axis, where


def preflight(path: str,
              ptp_velocity_pct: float = EXPERIMENTAL_PTP_VELOCITY_PCT
              ) -> PreflightResult:
    """Valida el JSON en `path`. No lo modifica."""
    result = PreflightResult()
    try:
        with open(path, 'r', encoding='utf-8') as handle:
            doc = json.load(handle)
    except (OSError, ValueError) as exc:
        result.error(f'JSON ilegible: {exc}')
        return result
    return preflight_document(doc, ptp_velocity_pct)


def preflight_document(doc: Dict[str, Any],
                       ptp_velocity_pct: float = EXPERIMENTAL_PTP_VELOCITY_PCT
                       ) -> PreflightResult:
    """
    Valida un documento YA construido, ANTES de decidir como se llamara.

    Asi el archivo nace con el nombre correcto: uno que no pasa el contrato
    jamas llega a llamarse como si estuviera listo para el robot.
    """
    result = PreflightResult()
    if doc.get('schema_version') != 1:
        result.error(f'schema_version inesperada: {doc.get("schema_version")}')

    names = list(doc.get('joint_names') or [])
    if names != JOINT_NAMES:
        result.error(f'joint_names {names} != orden canonico {JOINT_NAMES}')

    segments = doc.get('segments') or []
    if not segments:
        result.error('El archivo no contiene segmentos.')
        return result

    # ── recorrido punto a punto ─────────────────────────────────────
    nan_inf = 0
    limit_hits: List[Tuple[str, int, str, float, Tuple[float, float]]] = []
    time_faults: List[str] = []
    flat: List[List[float]] = []
    per_segment_worst: List[Tuple[str, float, Optional[str], Optional[int]]] = []
    empty_segments: List[str] = []

    for seg in segments:
        seg_id = str(seg.get('id') or '?')
        points = seg.get('trajectory_points') or []
        if not points:
            empty_segments.append(seg_id)
            continue
        deg: List[List[float]] = []
        last_time = -1.0
        for i, point in enumerate(points):
            values = [float(v) for v in point.get('positions_deg') or []]
            if len(values) != 6:
                result.error(f'{seg_id} punto {i}: no tiene 6 componentes.')
                continue
            for v in values:
                if math.isnan(v) or math.isinf(v):
                    nan_inf += 1
            for j, value, bounds in violations_deg(values):
                limit_hits.append((seg_id, i, AXES[j], value, bounds))
            stamp = float(point.get('time_from_start_sec', 0.0))
            if stamp < last_time:
                time_faults.append(f'{seg_id} punto {i}: tiempo no monotono')
            last_time = stamp
            deg.append(values)
        flat.extend(deg)
        per_segment_worst.append((seg_id,) + _worst_delta(deg))

    if empty_segments:
        result.error(
            f'Segmentos sin puntos (no resueltos): {empty_segments}. '
            'La trayectoria esta incompleta y no es ejecutable.')

    if nan_inf:
        result.error(f'{nan_inf} valores NaN/Inf en positions_deg.')

    if limit_hits:
        shown = limit_hits[:5]
        detail = '; '.join(
            f'{s} wp{i} {a}={v:.3f} fuera de [{b[0]:.0f}, {b[1]:.0f}]'
            for s, i, a, v, b in shown)
        more = f' (+{len(limit_hits) - len(shown)} mas)' if len(
            limit_hits) > len(shown) else ''
        result.error(
            f'{len(limit_hits)} violaciones de soft limits: {detail}{more}')

    for fault in time_faults:
        result.error(fault)

    # ── saltos intra-segmento ───────────────────────────────────────
    worst_delta, worst_axis, worst_seg, worst_pair = 0.0, None, None, None
    delta_faults = 0
    for seg_id, delta, axis, index in per_segment_worst:
        if delta > worst_delta:
            worst_delta, worst_axis, worst_seg = delta, axis, seg_id
            worst_pair = (index, None if index is None else index + 1)
    for seg in segments:
        points = seg.get('trajectory_points') or []
        for i in range(1, len(points)):
            a = points[i - 1].get('positions_deg') or []
            b = points[i].get('positions_deg') or []
            for j in range(min(6, len(a), len(b))):
                if abs(float(b[j]) - float(a[j])) > MAX_JOINT_DELTA_DEG:
                    delta_faults += 1

    # ── saltos en las uniones entre segmentos ───────────────────────
    join_worst, join_axis, join_where = 0.0, None, None
    for k in range(1, len(segments)):
        prev = segments[k - 1].get('trajectory_points') or []
        curr = segments[k].get('trajectory_points') or []
        if not prev or not curr:
            continue
        a = prev[-1].get('positions_deg') or []
        b = curr[0].get('positions_deg') or []
        for j in range(min(6, len(a), len(b))):
            delta = abs(float(b[j]) - float(a[j]))
            if delta > join_worst:
                join_worst, join_axis = delta, AXES[j]
                join_where = f'{segments[k - 1].get("id")}->{segments[k].get("id")}'
            if delta > MAX_JOINT_DELTA_DEG:
                delta_faults += 1

    if delta_faults:
        result.error(
            f'{delta_faults} pares eje-punto con |dq| > '
            f'{MAX_JOINT_DELTA_DEG:.0f} deg. Maximo {worst_delta:.4f} deg en '
            f'{worst_axis} ({worst_seg}, puntos {worst_pair}).')

    # ── velocidad PTP: TODOS los segmentos, sin excepcion ───────────
    at_target, other, missing = 0, [], []
    for seg in segments:
        seg_id = str(seg.get('id') or '?')
        profile = seg.get('execution_profile')
        if not isinstance(profile, dict) or 'kuka_ptp_velocity_pct' not in profile:
            missing.append(seg_id)
            continue
        value = profile.get('kuka_ptp_velocity_pct')
        try:
            value = float(value)
        except (TypeError, ValueError):
            other.append((seg_id, value))
            continue
        if not 0.0 < value <= 100.0:
            other.append((seg_id, value))
        elif abs(value - ptp_velocity_pct) > 1e-9:
            other.append((seg_id, value))
        else:
            at_target += 1

    if missing:
        result.error(f'Segmentos sin execution_profile valido: {missing}')
    if other:
        result.error(
            f'Segmentos con velocidad distinta de {ptp_velocity_pct}%: {other}')

    # ── eventos de garra ────────────────────────────────────────────
    point_ids = {str(p.get('id')) for p in (doc.get('source_points') or [])}
    events = ((doc.get('gripper') or {}).get('events') or [])
    bad_events = [e for e in events if str(e.get('at_point')) not in point_ids]
    if bad_events:
        result.error(f'Eventos de garra que apuntan a puntos inexistentes: '
                     f'{bad_events}')

    # ── lotes ───────────────────────────────────────────────────────
    oversized = [str(s.get('id')) for s in segments
                 if len(s.get('trajectory_points') or []) > MAX_BATCH_POINTS]
    if oversized:
        result.warn(
            f'{len(oversized)} segmentos con mas de {MAX_BATCH_POINTS} puntos: '
            f'el ejecutor por lotes tendra que trocearlos (comportamiento '
            f'normal, no es un fallo).')

    if flat:
        result.first_point_deg = list(flat[0])

    result.stats = {
        'segments': len(segments),
        'points': len(flat),
        'nan_inf': nan_inf,
        'soft_limit_violations': len(limit_hits),
        'delta_violations': delta_faults,
        'max_joint_delta_deg': worst_delta,
        'max_delta_axis': worst_axis,
        'max_delta_segment': worst_seg,
        'max_delta_points': worst_pair,
        'max_join_delta_deg': join_worst,
        'max_join_delta_axis': join_axis,
        'max_join_where': join_where,
        'segments_at_target_velocity': at_target,
        'segments_other_velocity': len(other),
        'segments_missing_profile': len(missing),
        'ptp_velocity_pct': ptp_velocity_pct,
        'gripper_events': len(events),
        'segments_over_batch': len(oversized),
    }
    return result


def format_report(result: PreflightResult, path: str) -> str:
    """Informe legible en consola y en el log de la GUI."""
    s = result.stats
    lines = [
        '=' * 74,
        'PREFLIGHT OFFLINE — contrato de ejecucion KUKA',
        f'archivo: {path}',
        '=' * 74,
        f'  Segmentos                   : {s.get("segments")}',
        f'  Puntos                      : {s.get("points")}',
        f'  NaN/Inf                     : {s.get("nan_inf")}',
        f'  Soft-limit violations       : {s.get("soft_limit_violations")}',
        f'  Delta violations >{MAX_JOINT_DELTA_DEG:.0f}deg      : '
        f'{s.get("delta_violations")}',
        f'  Maximum joint delta         : '
        f'{s.get("max_joint_delta_deg", 0.0):.4f} deg'
        f'  [{s.get("max_delta_axis")}, {s.get("max_delta_segment")}, '
        f'puntos {s.get("max_delta_points")}]',
        f'  Maximo delta en uniones     : '
        f'{s.get("max_join_delta_deg", 0.0):.4f} deg'
        f'  [{s.get("max_join_delta_axis")}, {s.get("max_join_where")}]',
        '',
        f'  KUKA PTP velocity requested : {s.get("ptp_velocity_pct")}%',
        f'  Segments total              : {s.get("segments")}',
        f'  Segments at {s.get("ptp_velocity_pct")}%            : '
        f'{s.get("segments_at_target_velocity")}',
        f'  Segments at another velocity: {s.get("segments_other_velocity")}',
        f'  Segments without profile    : {s.get("segments_missing_profile")}',
        '',
        f'  Gripper events              : {s.get("gripper_events")}',
        f'  Segmentos > lote de {MAX_BATCH_POINTS}      : '
        f'{s.get("segments_over_batch")} (se trocean, no es fallo)',
    ]
    if result.first_point_deg:
        values = ', '.join(f'{v:.3f}' for v in result.first_point_deg)
        lines += [
            '',
            '  PRIMER PUNTO (no validable offline):',
            f'    [{values}]',
            '    El salto entre la posicion REAL del robot y este punto '
            'requiere AxisActual.',
            '    Queda PENDIENTE para el dia de la prueba. No se inventa la '
            'lectura.',
        ]
    if result.warnings:
        lines += [''] + [f'  AVISO: {w}' for w in result.warnings]
    lines += ['', '  RESULTADO: ' + ('OK — JSON EJECUTABLE' if result.ok
                                     else 'RECHAZADO — NO EJECUTABLE')]
    if result.errors:
        lines += [f'    - {e}' for e in result.errors]
    lines.append('=' * 74)
    return '\n'.join(lines)


def limits_table() -> str:
    """Tabla MoveIt (URDF) vs pipeline KUKA."""
    from .kuka_pipeline_limits import URDF_JOINT_LIMITS_DEG
    lines = [f'{"eje":4s} {"MoveIt (URDF)":>18s} {"pipeline KUKA":>18s}  '
             f'coincide']
    for i, axis in enumerate(AXES):
        u, p = URDF_JOINT_LIMITS_DEG[i], PIPELINE_JOINT_LIMITS_DEG[i]
        same = 'SI' if u == p else 'NO'
        lines.append(f'{axis:4s} {f"[{u[0]:.0f}, {u[1]:.0f}]":>18s} '
                     f'{f"[{p[0]:.0f}, {p[1]:.0f}]":>18s}  {same}')
    return '\n'.join(lines)
