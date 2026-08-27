#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sequence_analysis.py

Analisis de SECUENCIAS multisegmento y tabla comparativa entre condiciones.

Las tres condiciones que se comparan sobre la MISMA tarea:

  AFINADA/ORIGEN EXTERNO  trayectoria GRABADA por el sistema afinado (TG2),
                          leida de un JSON. No la genero el baseline.
  BASELINE-A              plan del baseline con metas ARTICULARES.
  BASELINE-B              plan del baseline con metas CARTESIANAS derivadas de
                          los mismos joints por FK de este paquete.

RELOJ. El tiempo de cada segmento REINICIA en 0: asi lo entrega MoveIt2 en
`time_from_start` y asi viene en el JSON. Se conservan las dos formas:
  - `time_from_start_s`  tiempo LOCAL del segmento (columna 3 del CSV)
  - `global_time_s`      tiempo ACUMULADO de la secuencia
El acumulado se construye sumando la duracion de los segmentos previos; no se
interpola ni se reparametriza nada.
"""

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

from .continuity_metrics import (
    AXES,
    TrajectoryAnalysis,
    Waypoint,
    analyze,
)

CONDITION_RECORDED = 'AFINADA/ORIGEN EXTERNO'
CONDITION_BASELINE_JOINT = 'BASELINE-A (metas articulares)'
CONDITION_BASELINE_CART = 'BASELINE-B (metas cartesianas por FK)'

#: Marcas de comparabilidad que acompañan a las cabeceras de columna.
COMPARABILITY_LEGEND = """\
┌────────────────────────────────────────────────────────────────────────────┐
│  MARCAS DE COMPARABILIDAD  (tabla completa: README seccion 8.9)     │
├────────────────────────────────────────────────────────────────────────────┤
│  !  = NO COMPARABLE entre condiciones. Valor DESCRIPTIVO de su fila.        │
│       Leer esta columna en horizontal produce conclusiones falsas.          │
│         t[s]     el afinado corrio con velocity_scaling = 0.1 y el baseline │
│                  con 1.0: hasta 10x de diferencia SOLO por el escalado.     │
│         wp       TOTG remuestrea a resample_dt = 0.1 s FIJO, asi que el     │
│                  numero de waypoints es t[s]/0.1: es un proxy del tiempo,   │
│                  no de la complejidad del camino. Hereda el mismo problema. │
│         jerkMax  derivado numericamente de una aceleracion bang-bang; su    │
│                  valor lo fija dt y la saturacion, no el movimiento.        │
│                                                                             │
│  ~  = COMPARABLE CON RESERVA. Depende de la densidad de muestreo.           │
│         dmax     con menos waypoints los saltos individuales son mayores    │
│                  aunque el camino sea el mismo. Verificado: al diezmar a    │
│                  1/2 y 1/4, dmax crece x1.99 y x3.58.                       │
│                                                                             │
│  (sin marca) = COMPARABLE. Invarianza verificada numericamente.             │
│         recorrido, directa, ratio: 0.0000% de variacion al diezmar a 1/2 y  │
│         a 1/4 la trayectoria de referencia.                                 │
│         saltos, condJ: geometricos, independientes del muestreo y del tip.  │
└────────────────────────────────────────────────────────────────────────────┘"""

#: Aviso obligatorio en toda tabla comparativa. Va literal.
COMPARISON_DISCLAIMER = """\
╔════════════════════════════════════════════════════════════════════════════╗
║  COMPARACION INDICATIVA, NO ENSAYO CONTROLADO                              ║
╠════════════════════════════════════════════════════════════════════════════╣
║  La fila AFINADA es una trayectoria GRABADA por el sistema afinado en el   ║
║  entorno TG2 el 2026-08-22, con velocity_scaling=0.1, acceleration_        ║
║  scaling=0.1, planning_attempts=10, allowed_planning_time=10.0 s y         ║
║  tolerancia de meta de 0.2 grados.                                         ║
║                                                                            ║
║  Las filas BASELINE son planes GENERADOS AHORA por la condicion base con   ║
║  sus propios defectos y tolerancia de 0.0057 grados.                       ║
║                                                                            ║
║  Difieren el entorno, la fecha, el planificador efectivo y la tolerancia.  ║
║  NO CORRIERON BAJO EL MISMO PROCEDIMIENTO.                                 ║
╚════════════════════════════════════════════════════════════════════════════╝"""


@dataclass
class SequenceAnalysis:
    """Analisis de una secuencia completa bajo UNA condicion."""

    condition: str
    segments: List[TrajectoryAnalysis] = field(default_factory=list)
    #: segmentos que el planificador NO consiguio resolver (solo baseline)
    failed_segment_ids: List[str] = field(default_factory=list)
    #: Q.1 — segmentos POSTERIORES a un fallo. Su estado inicial ya no viene
    #: del plan real anterior, sino de la meta teorica, asi que el encadenado
    #: deja de ser equivalente al de la condicion afinada (J-D3).
    chain_rebuilt_segment_ids: List[str] = field(default_factory=list)
    metadata: Dict[str, str] = field(default_factory=dict)

    @property
    def total_waypoints(self) -> int:
        return sum(s.waypoint_count for s in self.segments)

    @property
    def total_time_s(self) -> float:
        return sum(s.total_time_s for s in self.segments)

    @property
    def total_path_deg(self) -> float:
        return sum(s.total_path_deg for s in self.segments)

    @property
    def total_direct_deg(self) -> float:
        return sum(s.total_direct_deg for s in self.segments)

    @property
    def total_events(self) -> int:
        return sum(len(s.events) for s in self.segments)

    @property
    def max_condition_number(self) -> float:
        values = [s.max_condition_number for s in self.segments]
        return max(values) if values else float('nan')


def analyze_sequence(segment_waypoints: Sequence[Sequence[Waypoint]],
                     segment_ids: Sequence[str],
                     condition: str,
                     metadata: Optional[Dict[str, str]] = None,
                     ) -> SequenceAnalysis:
    """
    Analiza cada segmento por separado y encadena el reloj global.

    NO recalcula velocidades ni aceleraciones: se usan las que traiga cada
    waypoint. Si un segmento no las trae, quedan como nan en el CSV.
    """
    meta = dict(metadata or {})
    meta['condition'] = condition
    result = SequenceAnalysis(condition=condition, metadata=meta)
    offset = 0.0
    broken = False
    for k, (wps, sid) in enumerate(zip(segment_waypoints, segment_ids)):
        if not wps:
            result.failed_segment_ids.append(sid)
            # Q.1 — a partir de aqui la cadena queda reconstruida.
            broken = True
            # Q.4 — el reloj global NO avanza: un segmento sin plan no tiene
            # duracion. Los global_time_s posteriores quedan comprimidos
            # respecto al tiempo real de la tarea. Documentado en el README.
            continue
        if broken:
            result.chain_rebuilt_segment_ids.append(sid)
        item = analyze(list(wps),
                       trajectory_id=k + 1,
                       segment_id=sid,
                       global_time_offset_s=offset,
                       metadata=meta)
        result.segments.append(item)
        offset += item.total_time_s
    return result


# ─────────────────────────────────────────────────────────────────────────────
# J.4 — tabla comparativa
# ─────────────────────────────────────────────────────────────────────────────

def _f(value: float, width: int = 9, prec: int = 3) -> str:
    if value is None:
        return f'{"-":>{width}}'
    if isinstance(value, float) and math.isinf(value):
        return f'{"inf":>{width}}'
    if isinstance(value, float) and math.isnan(value):
        return f'{"nan":>{width}}'
    return f'{value:>{width}.{prec}f}'


def _short(condition: str) -> str:
    if condition == CONDITION_RECORDED:
        return 'AFINADA(grab)'
    if condition == CONDITION_BASELINE_JOINT:
        return 'BASELINE-A'
    if condition == CONDITION_BASELINE_CART:
        return 'BASELINE-B'
    return condition[:13]


def comparison_table(runs: Sequence[SequenceAnalysis],
                     segment_ids: Sequence[str]) -> str:
    """
    Tabla comparativa: TRES filas por segmento (una por condicion) mas un
    resumen agregado. Encabezada SIEMPRE por el aviso de comparacion
    indicativa.
    """
    out: List[str] = [COMPARISON_DISCLAIMER, '', COMPARABILITY_LEGEND, '']
    out.append('LEYENDA DE LAS FILAS')
    for run in runs:
        out.append(f'  {_short(run.condition):14s} = {run.condition}')
        src = run.metadata.get('source_file', '')
        if src:
            out.append(f'  {"":14s}   origen: {src}')
            out.append(f'  {"":14s}   md5   : {run.metadata.get("source_md5", "")}')
        out.append(f'  {"":14s}   tip declarado: '
                   f'{run.metadata.get("declared_tip", "-")}')
    out.append('')

    by_segment: Dict[str, Dict[str, TrajectoryAnalysis]] = {}
    for run in runs:
        for item in run.segments:
            by_segment.setdefault(item.segment_id, {})[run.condition] = item

    head = (f'{"seg":5s} {"condicion":14s} {"wp!":>4s} {"t[s]!":>8s} '
            f'{"dmax[deg]~":>10s} {"eje":>4s} {"saltos":>7s} '
            f'{"recorrido":>10s} {"directa":>9s} {"ratio":>8s} '
            f'{"condJ max":>12s} {"@wp":>5s} {"jerkMax!":>9s}')
    out.append('─' * len(head))
    out.append('TABLA POR SEGMENTO')
    out.append('─' * len(head))
    out.append(head)
    out.append('─' * len(head))

    for sid in segment_ids:
        entries = by_segment.get(sid, {})
        for run in runs:
            item = entries.get(run.condition)
            if item is None:
                out.append(f'{sid:5s} {_short(run.condition):14s} '
                           f'{"*** SEGMENTO NO RESUELTO ***":>60s}')
                continue
            jm = max((v for v in item.max_jerk_per_axis
                      if not math.isnan(v)), default=float('nan'))
            flag = ''
            if sid in run.chain_rebuilt_segment_ids:
                flag = '  <<< CADENA RECONSTRUIDA'
            out.append(
                f'{sid:5s} {_short(run.condition):14s} '
                f'{item.waypoint_count:4d} {_f(item.total_time_s, 8)} '
                f'{_f(item.max_delta_deg, 10)} {item.max_delta_axis:>4s} '
                f'{len(item.events):7d} '
                f'{_f(item.total_path_deg, 10)} {_f(item.total_direct_deg, 9)} '
                f'{_f(item.total_ratio, 8)} '
                f'{_f(item.max_condition_number, 12, 1)} '
                f'{item.max_condition_index:5d} {_f(jm, 9)}{flag}')
        out.append('')

    out.append('─' * len(head))
    out.append('RESUMEN AGREGADO DE LA SECUENCIA')
    out.append('─' * len(head))
    out.append(f'{"":5s} {"condicion":14s} {"wp":>4s} {"t[s]":>8s} '
               f'{"":10s} {"":4s} {"saltos":>7s} '
               f'{"recorrido":>10s} {"directa":>9s} {"ratio":>8s} '
               f'{"condJ max":>12s} {"fallos":>6s}')
    for run in runs:
        ratio = (run.total_path_deg / run.total_direct_deg
                 if run.total_direct_deg > 1e-9 else float('inf'))
        out.append(
            f'{"":5s} {_short(run.condition):14s} '
            f'{run.total_waypoints:4d} {_f(run.total_time_s, 8)} '
            f'{"":10s} {"":4s} {run.total_events:7d} '
            f'{_f(run.total_path_deg, 10)} {_f(run.total_direct_deg, 9)} '
            f'{_f(ratio, 8)} {_f(run.max_condition_number, 12, 1)} '
            f'{len(run.failed_segment_ids):6d}')
    out.append('')

    out.append('─' * len(head))
    out.append('DELTA ARTICULAR MAXIMO POR EJE [deg] (peor de toda la secuencia)')
    out.append('─' * len(head))
    axis_head = ' '.join(f'{a:>10s}' for a in AXES)
    out.append(f'{"":5s} {"condicion":14s} {axis_head}')
    for run in runs:
        worst = [0.0] * 6
        for item in run.segments:
            for row in item.deltas_deg:
                for j in range(6):
                    worst[j] = max(worst[j], row[j])
        cells = ' '.join(_f(v, 10) for v in worst)
        out.append(f'{"":5s} {_short(run.condition):14s} {cells}')
    out.append('')

    out.append('─' * len(head))
    out.append('RECORRIDO ACUMULADO POR EJE [deg]')
    out.append('─' * len(head))
    out.append(f'{"":5s} {"condicion":14s} {axis_head}')
    for run in runs:
        total = [0.0] * 6
        for item in run.segments:
            for j in range(6):
                total[j] += item.path_length_deg[j]
        cells = ' '.join(_f(v, 10) for v in total)
        out.append(f'{"":5s} {_short(run.condition):14s} {cells}')
    out.append('')

    # ── Q.3 — recuento de fallos y de cadena reconstruida ────────────────
    out.append('─' * len(head))
    out.append('Q — INTEGRIDAD DEL ENCADENADO (J-D3)')
    out.append('─' * len(head))
    any_broken = False
    for run in runs:
        if not run.failed_segment_ids and not run.chain_rebuilt_segment_ids:
            out.append(f'  {_short(run.condition):14s} cadena INTACTA: los '
                       f'{len(run.segments)} segmentos encadenados desde el '
                       'plan real anterior.')
            continue
        any_broken = True
        out.append(f'  {_short(run.condition):14s} '
                   f'{len(run.failed_segment_ids)} segmento(s) NO RESUELTO(S): '
                   f'{run.failed_segment_ids}')
        out.append(f'  {"":14s} '
                   f'{len(run.chain_rebuilt_segment_ids)} segmento(s) con '
                   f'CADENA RECONSTRUIDA: {run.chain_rebuilt_segment_ids}')
    if any_broken:
        out.append('')
        out.append('  AVISO. Tras un segmento no resuelto, el estado inicial '
                   'del siguiente pasa a ser la META TEORICA, un estado en el')
        out.append('  que el baseline nunca estuvo. A partir de ese punto el '
                   'encadenado YA NO ES EQUIVALENTE al de la condicion')
        out.append('  afinada (J-D3), y los segmentos marcados no deben '
                   'compararse en pie de igualdad con ella.')
        out.append('  Ademas el RELOJ GLOBAL no avanza durante el segmento '
                   'fallido: los global_time_s posteriores quedan comprimidos')
        out.append('  respecto al tiempo real de la tarea. El tiempo LOCAL de '
                   'cada segmento (columna t[s]) no se ve afectado.')
    out.append('')
    out.append('NOTA: "saltos" = eventos de cambio de configuracion detectados '
               '(near_180, sign_change, a5_abrupt, singular).')
    out.append('NOTA: t[s] es el tiempo LOCAL de cada segmento; el reloj '
               'global acumulado esta en la columna global_time_s del CSV.')
    out.append('NOTA: jerkMax es DERIVADO, no entregado por MoveIt. Ver '
               'JERK_NOTE en continuity_metrics.py antes de citarlo.')
    return '\n'.join(out)
