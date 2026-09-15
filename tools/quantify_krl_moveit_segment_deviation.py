#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
quantify_krl_moveit_segment_deviation.py

Cuantifica la DESVIACION DE LA EVOLUCION ARTICULAR ENTRE KRL Y MoveIt2
AFINADO comparando SEGMENTOS EQUIVALENTES de la tarea, con el progreso de
cada segmento medido por RECORRIDO ARTICULAR y no por indice de muestra.

═══════════════════════════════════════════════════════════════════════════
  POR QUE EXISTE ESTA HERRAMIENTA
═══════════════════════════════════════════════════════════════════════════
  quantify_krl_moveit_deviation.py normaliza por indice global de muestra.
  Eso introduce un desfase artificial: el log de KRL contiene muestras
  repetidas mientras el robot esta detenido (12.8 % en el cuadrado, 22.2 % en
  el pick and place), que la trayectoria planificada no tiene. Una misma
  fraccion de progreso no corresponde a la misma fase de la tarea, y el error
  instantaneo se dispara en las transiciones.

  Aqui se elimina ese efecto por construccion:
    1. se descartan las muestras estacionarias de KRL,
    2. se parte la tarea en segmentos equivalentes,
    3. dentro de cada segmento el progreso es RECORRIDO ARTICULAR NORMALIZADO.

  Asi la duracion, la frecuencia de muestreo, las esperas y el numero de
  puntos dejan de influir.
═══════════════════════════════════════════════════════════════════════════

QUE NO SE HACE
  NO se usa DTW ni ningun metodo que deforme las curvas para minimizar el
  error. NO se usa el tiempo como eje. Estas metricas NO son "tracking error"
  ni "error de seguimiento": se comparan una ejecucion fisica registrada y
  una trayectoria planificada, que son fuentes de naturaleza distinta.

COMO SE SEGMENTA KRL, Y POR QUE ES LEGITIMO
  Los puntos ENSEÑADOS de la tarea (source_points del JSON) son las
  configuraciones objetivo conocidas. Se localiza en que muestra de KRL se
  alcanzo cada uno, mediante una asignacion MONOTONA optima por programacion
  dinamica: indices i1 < i2 < ... < iK que minimizan la suma de distancias a
  P1..PK.

  Esto NO es DTW y no puede sesgar el resultado, por una razon concreta: la
  asignacion solo mira el registro KRL y los puntos enseñados. NUNCA mira la
  curva de MoveIt2. Es imposible que "ajuste" una fuente a la otra, porque
  desconoce la otra.

  Una busqueda voraz hacia delante NO sirve: la tarea revisita
  configuraciones (en el pick and place P5 vuelve a la pose de P3) y el
  emparejado voraz se queda atrapado, con residuos de hasta 16 grados. La
  asignacion monotona optima baja ese residuo a 0.014 grados.

  CORROBORACION INDEPENDIENTE: en el pick and place, los indices asignados a
  P4, P7, P10 y P13 caen exactamente sobre las paradas detectadas en el log
  (donde se acciona la garra). Dos criterios independientes coinciden.

  Si el emparejado no es seguro, la herramienta SE DETIENE y lo reporta, en
  vez de inventar una correspondencia.

Offline y de solo lectura. No toca los CSV ni los JSON de entrada, ni ninguna
herramienta existente.
"""

import argparse
import json
import os
import sys
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

import compare_planned_trajectories as base              # noqa: E402
import compare_krl_moveit_trajectories as krlmv          # noqa: E402

AXES = base.AXES

#: Puntos de la malla comun POR SEGMENTO.
SEGMENT_GRID_POINTS = 200

#: Umbral de variacion conjunta por debajo del cual una muestra se considera
#: estacionaria. 0.0 = solo repeticiones EXACTAS. Es el defecto a proposito:
#: cualquier valor mayor es un umbral elegido, y elegirlo cambia el resultado.
DEFAULT_STATIONARY_EPS = 0.0

#: Residuo maximo admisible al localizar un punto enseñado en el log, en
#: grados. Por encima de esto el emparejado no es seguro y se aborta.
DEFAULT_MATCH_TOLERANCE_DEG = 0.5

ERROR_CONVENTION = 'error = q_MoveIt2_afinado - q_KRL'
METRIC_NAME = 'Desviacion de la evolucion articular entre KRL y MoveIt2 afinado'


class SegmentationError(Exception):
    """El mapeo de segmentos no es seguro. Se aborta en vez de inventarlo."""


# ═══════════════════════════════════════════════════════════════════════════
# Paso 2 — muestras estacionarias
# ═══════════════════════════════════════════════════════════════════════════

def drop_stationary(q_deg: np.ndarray, eps: float = DEFAULT_STATIONARY_EPS
                    ) -> Tuple[np.ndarray, np.ndarray, int]:
    """
    Descarta las muestras sin avance articular apreciable.

    dq_norm(k) = ||q(k) - q(k-1)||. Se conserva la muestra k si su variacion
    respecto a la anterior supera eps, y SIEMPRE la primera y la ultima del
    registro. De cada racha estacionaria sobrevive su primera muestra: las
    demas son identicas, asi que no se pierde ninguna informacion geometrica.

    Las posiciones NO se suavizan ni se modifican de ninguna forma.
    """
    if len(q_deg) < 2:
        return q_deg, np.arange(len(q_deg)), 0
    dq = np.linalg.norm(np.diff(q_deg, axis=0), axis=1)
    keep = np.zeros(len(q_deg), dtype=bool)
    keep[0] = True
    keep[-1] = True
    keep[1:] |= dq > eps
    return q_deg[keep], np.where(keep)[0], int((~keep).sum())


# ═══════════════════════════════════════════════════════════════════════════
# Paso 3 — segmentacion
# ═══════════════════════════════════════════════════════════════════════════

def monotone_match(q_deg: np.ndarray, targets: List[np.ndarray]
                   ) -> Tuple[List[int], List[float]]:
    """
    Localiza en que muestra se alcanzo cada punto enseñado, en orden.

    Programacion dinamica sobre i1 < i2 < ... < iK minimizando la suma de
    distancias. Solo intervienen el registro KRL y los puntos enseñados.
    """
    n, k_total = len(q_deg), len(targets)
    if n < k_total:
        raise SegmentationError(
            f'El registro tiene {n} muestras utiles para {k_total} puntos '
            'enseñados. No hay muestras suficientes para una asignacion '
            'estrictamente creciente.')

    dist = np.stack([np.linalg.norm(q_deg - t, axis=1) for t in targets])
    cost = np.full((k_total, n), np.inf)
    back = np.zeros((k_total, n), dtype=int)
    cost[0, 0] = dist[0, 0]           # P1 se ancla en la primera muestra
    for k in range(1, k_total):
        best, best_j = np.inf, -1
        for i in range(1, n):
            if cost[k - 1, i - 1] < best:
                best, best_j = cost[k - 1, i - 1], i - 1
            if best < np.inf:
                cost[k, i] = best + dist[k, i]
                back[k, i] = best_j

    last = int(np.argmin(cost[k_total - 1]))
    if not np.isfinite(cost[k_total - 1, last]):
        raise SegmentationError(
            'No existe ninguna asignacion estrictamente creciente de los '
            'puntos enseñados sobre el registro.')

    idx = [0] * k_total
    idx[k_total - 1] = last
    for k in range(k_total - 1, 0, -1):
        last = back[k, last]
        idx[k - 1] = last
    residuals = [float(np.linalg.norm(q_deg[idx[k]] - targets[k]))
                 for k in range(k_total)]
    return idx, residuals


def build_segment_map(q_krl: np.ndarray, document: Dict,
                      tolerance: float) -> Tuple[List[int], List[float],
                                                 List[Dict]]:
    """
    Mapeo explicito de segmentos KRL <-> MoveIt2. Aborta si no es seguro.
    """
    points = document.get('source_points') or []
    segments = document.get('segments') or []
    if len(segments) != len(points) - 1:
        raise SegmentationError(
            f'El JSON declara {len(segments)} segmentos para {len(points)} '
            f'puntos enseñados; el contrato exige {len(points) - 1}.')

    targets = [np.asarray(p['joints_deg'], dtype=float) for p in points]
    idx, residuals = monotone_match(q_krl, targets)

    for k in range(1, len(idx)):
        if idx[k] <= idx[k - 1]:
            raise SegmentationError(
                f'El emparejado no es estrictamente creciente en '
                f'{points[k].get("id")} (indice {idx[k]} <= {idx[k - 1]}).')

    worst = max(residuals)
    if worst > tolerance:
        offender = points[int(np.argmax(residuals))].get('id')
        raise SegmentationError(
            f'Residuo de emparejado {worst:.4f} deg en {offender}, por encima '
            f'de la tolerancia {tolerance:.4f} deg. El registro no pasa lo '
            'bastante cerca de ese punto enseñado como para asociarlo con '
            'seguridad. Se aborta: inventar la correspondencia falsearia todo '
            'el analisis.')

    mapping = []
    for k, segment in enumerate(segments):
        mapping.append({
            'segmento_comparado': segment.get('id') or f'T{k + 1}',
            'inicio_KRL': int(idx[k]),
            'fin_KRL': int(idx[k + 1]),
            'segmento_MoveIt2': segment.get('id') or f'T{k + 1}',
            'descripcion': (
                f'{points[k].get("id")} -> {points[k + 1].get("id")}'),
            'n_muestras_KRL': int(idx[k + 1] - idx[k] + 1),
            'n_puntos_MoveIt2': len(segment.get('trajectory_points') or []),
            'residuo_inicio_deg': residuals[k],
            'residuo_fin_deg': residuals[k + 1],
        })
    return idx, residuals, mapping


# ═══════════════════════════════════════════════════════════════════════════
# Paso 4 y 5 — progreso por recorrido articular y remuestreo
# ═══════════════════════════════════════════════════════════════════════════

def arclength_progress(q_deg: np.ndarray) -> Optional[np.ndarray]:
    """
    Progreso por recorrido articular acumulado, normalizado a [0, 1].

    s(0) = 0;  s(k) = s(k-1) + ||q(k) - q(k-1)||;  p = s / s(final).
    Devuelve None si el segmento no avanza: normalizar dividiendo por cero no
    es un caso a parchear, es un segmento que no se puede comparar.
    """
    if len(q_deg) < 2:
        return None
    step = np.linalg.norm(np.diff(q_deg, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(step)])
    if s[-1] <= 0.0:
        return None
    return s / s[-1]


def resample_segment(q_deg: np.ndarray, grid: np.ndarray
                     ) -> Optional[np.ndarray]:
    """Interpola los seis ejes sobre la malla de progreso del segmento."""
    p = arclength_progress(q_deg)
    if p is None:
        return None
    # p puede repetir valores si dos muestras consecutivas coinciden; np.interp
    # lo tolera mientras sea no decreciente, que lo es por construccion.
    return np.column_stack(
        [np.interp(grid, p, q_deg[:, j]) for j in range(len(AXES))])


# ═══════════════════════════════════════════════════════════════════════════
# Paso 6 — metricas
# ═══════════════════════════════════════════════════════════════════════════

def metric_rows(error: np.ndarray, segment_id: Optional[str] = None
                ) -> List[Dict]:
    """MAE, RMSE y maximo absoluto por eje."""
    rows = []
    for j, axis in enumerate(AXES):
        column = error[:, j]
        row = {}
        if segment_id is not None:
            row['segmento'] = segment_id
        row.update({
            'eje': axis,
            'mae_deg': float(np.mean(np.abs(column))),
            'rmse_deg': float(np.sqrt(np.mean(column ** 2))),
            'max_abs_deg': float(np.max(np.abs(column))),
        })
        rows.append(row)
    return rows


# ═══════════════════════════════════════════════════════════════════════════
# Figuras
# ═══════════════════════════════════════════════════════════════════════════

def _rc():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        'font.size': 10, 'axes.titlesize': 11, 'axes.labelsize': 10,
        'xtick.labelsize': 9, 'ytick.labelsize': 9, 'legend.fontsize': 10,
        'axes.grid': True, 'grid.alpha': 0.3, 'grid.linewidth': 0.5,
        'axes.spines.top': False, 'axes.spines.right': False,
        'figure.facecolor': 'white', 'savefig.facecolor': 'white',
    })
    return plt


def _segment_axis(ax, n_segments: int, ids: List[str]) -> None:
    """Separadores y etiquetas de segmento en el eje horizontal."""
    for k in range(1, n_segments):
        ax.axvline(k, color='#bdc3c7', linestyle='-', linewidth=0.8)
    ax.set_xlim(0, n_segments)
    step = 1 if n_segments <= 10 else 2
    ticks = list(range(0, n_segments, step))
    ax.set_xticks([k + 0.5 for k in ticks])
    ax.set_xticklabels([ids[k] for k in ticks], fontsize=8)


def make_evolution_figure(grid, per_segment, ids, out_png, task, dpi):
    """Evolucion KRL y MoveIt2 alineadas por progreso de cada segmento."""
    plt = _rc()
    from matplotlib.gridspec import GridSpec
    n = len(ids)
    fig = plt.figure(figsize=(13.5, 9.6))
    gs = GridSpec(2, 3, figure=fig, hspace=0.55, wspace=0.30,
                  top=0.800, bottom=0.160, left=0.075, right=0.985)
    title = ('Evolución articular por segmentos: ejecución KRL registrada '
             'frente a\ntrayectoria afinada planificada por MoveIt2')
    if task:
        title += f' — {task}'
    fig.suptitle(title, fontsize=15, fontweight='bold', y=0.965)
    fig.text(0.075, 0.845,
             'Progreso de cada segmento medido por RECORRIDO ARTICULAR '
             'normalizado, no por índice de muestra ni por tiempo',
             fontsize=12, fontweight='bold', ha='left')

    for j, axis in enumerate(AXES):
        ax = fig.add_subplot(gs[j // 3, j % 3])
        for k in range(n):
            x = k + grid
            ax.plot(x, per_segment[k]['krl'][:, j],
                    label='KRL' if k == 0 else None, **base.STYLE['krl'])
            ax.plot(x, per_segment[k]['moveit'][:, j],
                    label='MoveIt2 Afinado' if k == 0 else None,
                    **base.STYLE['moveit_tuned'])
        _segment_axis(ax, n, ids)
        ax.set_title(f'{axis}', fontweight='bold', pad=8)
        ax.set_xlabel('Segmento  (progreso articular 0→1)')
        ax.set_ylabel(f'{axis} [°]')
        if j == 0:
            ax.legend(loc='best', framealpha=0.92, fontsize=9)

    fig.text(
        0.5, 0.098,
        'Desviación de la evolución articular entre fuentes de distinta '
        'naturaleza: KRL es ejecución física REGISTRADA y MoveIt2 Afinado es '
        'trayectoria PLANIFICADA.\n'
        'NO es error de seguimiento ni precisión del controlador. No se usa '
        'tiempo, ni DTW, ni ninguna deformación de las curvas.\n'
        'Cada segmento se normaliza por su propio recorrido articular, de modo '
        'que las pausas y la distinta velocidad no introducen desfase.',
        ha='center', va='top', fontsize=10, style='italic', color='#444444')

    os.makedirs(os.path.dirname(out_png) or '.', exist_ok=True)
    fig.savefig(out_png, dpi=dpi)
    plt.close(fig)
    return out_png


def make_error_figure(grid, per_segment, ids, out_png, task, dpi):
    """Error articular frente al progreso normalizado de cada segmento."""
    plt = _rc()
    from matplotlib.gridspec import GridSpec
    n = len(ids)
    fig = plt.figure(figsize=(13.5, 9.6))
    gs = GridSpec(2, 3, figure=fig, hspace=0.55, wspace=0.30,
                  top=0.800, bottom=0.160, left=0.075, right=0.985)
    title = ('Desviación de la evolución articular por segmentos entre KRL '
             'y MoveIt2 afinado')
    if task:
        title += f'\n{task}'
    fig.suptitle(title, fontsize=15, fontweight='bold', y=0.965)
    fig.text(0.075, 0.845,
             f'Δqᵢ = qᵢ,MoveIt2 − qᵢ,KRL [°] sobre el progreso articular '
             f'normalizado de cada segmento ({len(grid)} puntos por segmento)',
             fontsize=12, fontweight='bold', ha='left')

    for j, axis in enumerate(AXES):
        ax = fig.add_subplot(gs[j // 3, j % 3])
        ax.axhline(0.0, color='#7f8c8d', linestyle=':', linewidth=1.0)
        for k in range(n):
            err = per_segment[k]['moveit'][:, j] - per_segment[k]['krl'][:, j]
            ax.plot(k + grid, err, color='#1f4e79', linestyle='-',
                    linewidth=1.5, alpha=0.95)
        _segment_axis(ax, n, ids)
        ax.set_title(f'{axis}', fontweight='bold', pad=8)
        ax.set_xlabel('Segmento  (progreso articular 0→1)')
        ax.set_ylabel(f'Δ{axis} [°]')

    fig.text(
        0.5, 0.098,
        f'{ERROR_CONVENTION}.  Fuentes de distinta naturaleza: KRL es '
        'ejecución física REGISTRADA, MoveIt2 Afinado es trayectoria '
        'PLANIFICADA.\n'
        'NO es error de seguimiento, error temporal ni precisión del '
        'controlador.\n'
        'El progreso es recorrido articular dentro de cada segmento: las '
        'pausas del registro ya no producen desfase.',
        ha='center', va='top', fontsize=10, style='italic', color='#444444')

    os.makedirs(os.path.dirname(out_png) or '.', exist_ok=True)
    fig.savefig(out_png, dpi=dpi)
    plt.close(fig)
    return out_png


# ═══════════════════════════════════════════════════════════════════════════
# Programa principal
# ═══════════════════════════════════════════════════════════════════════════

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=('Cuantifica la desviacion de la evolucion articular '
                     'entre KRL y MoveIt2 afinado por segmentos equivalentes, '
                     'con progreso por recorrido articular. Offline.'),
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--krl', required=True)
    ap.add_argument('--moveit', required=True)
    ap.add_argument('--task-name', default='')
    ap.add_argument('--output-dir',
                    default='resultados_desviacion_segmentos')
    ap.add_argument('--grid-points', type=int, default=SEGMENT_GRID_POINTS)
    ap.add_argument('--stationary-eps', type=float,
                    default=DEFAULT_STATIONARY_EPS,
                    help='Variacion conjunta [deg] por debajo de la cual una '
                         'muestra se considera estacionaria. Por defecto 0.0: '
                         'solo repeticiones exactas.')
    ap.add_argument('--match-tolerance', type=float,
                    default=DEFAULT_MATCH_TOLERANCE_DEG,
                    help='Residuo maximo admisible al localizar un punto '
                         'enseñado. Por encima, se aborta.')
    ap.add_argument('--dpi', type=int, default=300)
    args = ap.parse_args(argv)

    task = args.task_name.strip()
    print('=' * 78)
    print(METRIC_NAME.upper())
    if task:
        print(f'TAREA: {task}')
    print('=' * 78)
    print('NO es error de seguimiento, error temporal ni precision del '
          'controlador.')
    print(f'  KRL             = {krlmv.DATA_NATURE_KRL}')
    print(f'  MoveIt2 Afinado = {krlmv.DATA_NATURE_MOVEIT}')
    print(f'  convenio        : {ERROR_CONVENTION}')

    try:
        krl = krlmv.load_krl_csv(args.krl, 'KRL')
        moveit = base.load_trajectory(args.moveit, 'MoveIt2 Afinado')
        with open(os.path.abspath(args.moveit), 'r', encoding='utf-8') as fh:
            document = json.load(fh)
    except Exception as exc:                          # noqa: BLE001
        print(f'\nERROR: {exc}')
        return 1

    if krl.joint_names != moveit.joint_names:
        print('\n*** ABORTADO: las fuentes no declaran el mismo conjunto de '
              'articulaciones. ***')
        return 1

    n_original = krl.n
    q_clean, kept_idx, removed = drop_stationary(krl.q_deg,
                                                 args.stationary_eps)
    print(f'\nKRL:             {krl.path}')
    print(f'  columnas      : {krl.extra["axis_columns"]}  (grados)')
    print(f'  N original    : {n_original}')
    print(f'  estacionarias eliminadas: {removed} '
          f'({100.0 * removed / max(1, n_original):.1f} %)')
    print(f'  criterio      : ||q(k)-q(k-1)|| <= {args.stationary_eps:g} deg; '
          'se conservan siempre la primera y la ultima muestra')
    print(f'  N tras limpiar: {len(q_clean)}')
    print(f'MoveIt2 Afinado: {moveit.path}')
    print(f'  N waypoints   : {moveit.n}   segmentos: '
          f'{len(document.get("segments") or [])}')

    try:
        idx, residuals, mapping = build_segment_map(
            q_clean, document, args.match_tolerance)
    except SegmentationError as exc:
        print(f'\n*** ANALISIS DETENIDO — SEGMENTACION NO SEGURA ***\n{exc}')
        return 2

    print(f'\nMAPEO DE SEGMENTOS  (residuo maximo {max(residuals):.5f} deg, '
          f'tolerancia {args.match_tolerance:g} deg)')
    print(f'  {"segmento":9s} {"inicio_KRL":>10s} {"fin_KRL":>8s} '
          f'{"MoveIt2":>8s} {"n_KRL":>6s} {"n_MV":>5s}  descripcion')
    for row in mapping:
        print(f'  {row["segmento_comparado"]:9s} {row["inicio_KRL"]:10d} '
              f'{row["fin_KRL"]:8d} {row["segmento_MoveIt2"]:>8s} '
              f'{row["n_muestras_KRL"]:6d} {row["n_puntos_MoveIt2"]:5d}  '
              f'{row["descripcion"]}')

    grid = np.linspace(0.0, 1.0, int(args.grid_points))
    segments = document.get('segments') or []
    per_segment = []
    ids = []
    failed = []
    for k, segment in enumerate(segments):
        seg_id = segment.get('id') or f'T{k + 1}'
        q_k = q_clean[idx[k]:idx[k + 1] + 1]
        q_m = np.array(
            [[float(v) for v in p['positions_deg']]
             for p in segment.get('trajectory_points') or []], dtype=float)
        rs_k = resample_segment(q_k, grid)
        rs_m = resample_segment(q_m, grid)
        if rs_k is None or rs_m is None:
            which = 'KRL' if rs_k is None else 'MoveIt2'
            failed.append((seg_id, which))
            continue
        per_segment.append({'id': seg_id, 'krl': rs_k, 'moveit': rs_m})
        ids.append(seg_id)

    if failed:
        print('\n*** ANALISIS DETENIDO ***')
        for seg_id, which in failed:
            print(f'  {seg_id}: el tramo de {which} no avanza (recorrido '
                  'articular nulo). No se puede normalizar por recorrido.')
        return 2

    out_dir = os.path.abspath(args.output_dir)
    os.makedirs(out_dir, exist_ok=True)

    seg_rows: List[Dict] = []
    stacked = []
    for item in per_segment:
        error = item['moveit'] - item['krl']
        seg_rows.extend(metric_rows(error, item['id']))
        stacked.append(error)
    global_error = np.vstack(stacked)
    global_rows = metric_rows(global_error)

    csv_segments = base.write_csv(
        os.path.join(out_dir, 'desviacion_segmentos.csv'), seg_rows)
    csv_global = base.write_csv(
        os.path.join(out_dir, 'desviacion_global.csv'), global_rows)
    png_evolution = make_evolution_figure(
        grid, per_segment, ids,
        os.path.join(out_dir, 'desviacion_segmentos.png'), task, args.dpi)
    png_error = make_error_figure(
        grid, per_segment, ids,
        os.path.join(out_dir, 'error_segmentos.png'), task, args.dpi)

    meta = {
        'metric_name': METRIC_NAME,
        'task': task or None,
        'execution_date': datetime.now().isoformat(timespec='seconds'),
        'not_a': ('No es error de seguimiento, error temporal, precision del '
                  'controlador ni error fisico. No se usa DTW, ni ninguna '
                  'deformacion de las curvas, ni el tiempo como eje.'),
        'error_convention': ERROR_CONVENTION,
        'krl': {
            'path': krl.path,
            'data_nature': krlmv.DATA_NATURE_KRL,
            'axis_columns': krl.extra['axis_columns'],
            'units': 'grados',
            'n_original_samples': n_original,
            'n_stationary_removed': removed,
            'n_samples_used': int(len(q_clean)),
            'stationary_criterion': (
                f'||q(k)-q(k-1)|| <= {args.stationary_eps:g} deg; se '
                'conservan siempre la primera y la ultima muestra. Las '
                'posiciones no se suavizan.'),
        },
        'moveit_tuned': {
            'path': moveit.path,
            'data_nature': krlmv.DATA_NATURE_MOVEIT,
            'units': moveit.units_note,
            'n_waypoints': moveit.n,
            'n_segments': len(segments),
        },
        'segmentation': {
            'krl_method': ('asignacion monotona optima de los puntos '
                           'enseñados (source_points) sobre el registro, por '
                           'programacion dinamica. Solo usa KRL y los puntos '
                           'enseñados: nunca la curva de MoveIt2.'),
            'moveit_method': 'estructura segments[] del JSON, directamente.',
            'match_tolerance_deg': args.match_tolerance,
            'max_residual_deg': max(residuals),
            'residuals_deg': residuals,
            'map': mapping,
        },
        'progress_definition': (
            's(k) = s(k-1) + ||q(k)-q(k-1)||;  p = s / s(final), por segmento '
            'y por fuente, independientemente.'),
        'interpolation_points_per_segment': int(args.grid_points),
        'outputs': {
            'segments_csv': csv_segments, 'global_csv': csv_global,
            'evolution_png': png_evolution, 'error_png': png_error,
        },
    }
    meta_path = os.path.join(
        out_dir, 'desviacion_segmentos_metadatos.json')
    with open(meta_path, 'w', encoding='utf-8') as handle:
        json.dump(meta, handle, indent=2, ensure_ascii=False, default=str)

    print(f'\nCSV segmentos: {csv_segments}')
    print(f'CSV global:    {csv_global}')
    print(f'PNG evolucion: {png_evolution}')
    print(f'PNG error:     {png_error}')
    print(f'Metadatos:     {meta_path}')

    print('\n' + '=' * 62)
    print(f'RESUMEN GLOBAL POR EJE{f" — {task}" if task else ""}')
    print('=' * 62)
    print(f'{"Eje":5s} {"MAE [deg]":>12s} {"RMSE [deg]":>12s} '
          f'{"Maximo [deg]":>14s}')
    print('-' * 62)
    for row in global_rows:
        print(f'{row["eje"]:5s} {row["mae_deg"]:12.4f} '
              f'{row["rmse_deg"]:12.4f} {row["max_abs_deg"]:14.4f}')
    print('-' * 62)
    print(f'{len(per_segment)} segmentos comparados x {len(grid)} puntos = '
          f'{len(global_error)} puntos por eje.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
