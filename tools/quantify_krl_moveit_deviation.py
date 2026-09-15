#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
quantify_krl_moveit_deviation.py

Cuantifica la DESVIACION ARTICULAR ENTRE KRL Y MoveIt2 AFINADO para una misma
tarea, sobre una malla comun de progreso normalizado.

═══════════════════════════════════════════════════════════════════════════
  QUE MIDE Y QUE NO MIDE ESTA HERRAMIENTA
═══════════════════════════════════════════════════════════════════════════
  MIDE   : la diferencia entre la evolucion articular registrada durante la
           ejecucion fisica (KRL) y la evolucion articular de la trayectoria
           planificada y afinada (MoveIt2), ambas parametrizadas por su
           propio progreso normalizado.

  NO MIDE: error temporal, error de seguimiento del controlador, precision
           del controlador ni error fisico. No se usan marcas de tiempo, no
           se alinean temporalmente las fuentes y no se comparan velocidades,
           aceleraciones ni jerk.

  La denominacion correcta de estas metricas es
        "Desviacion articular entre KRL y MoveIt2 afinado"
  porque las dos fuentes son de naturaleza distinta: una es una ejecucion
  fisica registrada y la otra una trayectoria planificada.
═══════════════════════════════════════════════════════════════════════════

RESERVA METODOLOGICA QUE AFECTA A LOS NUMEROS
  El progreso normalizado es una fraccion de MUESTRAS, no de tarea. El log de
  KRL contiene muestras estacionarias (el robot quieto mientras el log sigue
  grabando) que la trayectoria planificada no tiene. Por tanto una misma
  fraccion de progreso NO corresponde necesariamente a la misma fase de la
  tarea en las dos fuentes, y parte de la desviacion medida procede de esa
  diferencia de reparto, no de una diferencia geometrica. La herramienta
  informa cuantas muestras de KRL son repeticiones exactas de la anterior
  para que esa reserva sea cuantificable. No se filtra nada.

REUTILIZACION
  La lectura de los dos formatos se IMPORTA de las herramientas existentes,
  que no se modifican:
    - compare_krl_moveit_trajectories.load_krl_csv  (CSV del bridge)
    - compare_planned_trajectories.load_trajectory  (JSON del contrato)

Offline y de solo lectura. No toca los CSV ni los JSON de entrada.
"""

import argparse
import json
import os
import sys
from typing import Dict, List

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

import compare_planned_trajectories as base              # noqa: E402
import compare_krl_moveit_trajectories as krlmv          # noqa: E402

AXES = base.AXES

#: Puntos de la malla comun de progreso normalizado.
COMMON_GRID_POINTS = 1000

#: Convenio de signo del error, fijado y documentado.
ERROR_CONVENTION = 'error = q_MoveIt2_afinado - q_KRL'

METRIC_NAME = 'Desviacion articular entre KRL y MoveIt2 afinado'


def normalised_progress(traj) -> np.ndarray:
    """
    Progreso normalizado p(k) = k / (N - 1), en [0, 1].

    Trajectory.progress ya aplica esa definicion pero en porcentaje, que es lo
    que usan las figuras del proyecto. Aqui se divide entre 100 para dejarlo
    en [0, 1]: es la MISMA definicion, solo cambia la escala.
    """
    return np.asarray(traj.progress, dtype=float) / 100.0


def resample_to_grid(traj, grid: np.ndarray) -> np.ndarray:
    """
    Interpola las seis articulaciones sobre la malla comun.

    Interpolacion lineal y nada mas: sin suavizado, sin filtros y sin
    alineamiento temporal. `progress` es estrictamente creciente por
    construccion, que es lo que np.interp necesita.
    """
    p = normalised_progress(traj)
    return np.column_stack(
        [np.interp(grid, p, traj.q_deg[:, j]) for j in range(len(AXES))])


def count_repeated_samples(traj) -> int:
    """Muestras identicas a la anterior. Diagnostico, no filtro."""
    if traj.n < 2:
        return 0
    return int(np.sum(np.all(np.diff(traj.q_deg, axis=0) == 0.0, axis=1)))


def deviation_rows(error: np.ndarray) -> List[Dict]:
    """MAE, RMSE y maximo absoluto por eje, en grados."""
    rows = []
    for j, axis in enumerate(AXES):
        column = error[:, j]
        rows.append({
            'eje': axis,
            'mae_deg': float(np.mean(np.abs(column))),
            'rmse_deg': float(np.sqrt(np.mean(column ** 2))),
            'max_abs_deg': float(np.max(np.abs(column))),
            # Auxiliar, NO metrica principal: un error medio firmado cercano a
            # cero puede esconder desviaciones grandes que se compensan.
            'mean_signed_deg_auxiliar': float(np.mean(column)),
        })
    return rows


def point_rows(grid: np.ndarray, error: np.ndarray) -> List[Dict]:
    """Una fila por punto de la malla comun."""
    rows = []
    for k in range(len(grid)):
        row = {'progreso': float(grid[k])}
        for j, axis in enumerate(AXES):
            row[f'{axis}_error_deg'] = float(error[k, j])
        rows.append(row)
    return rows


def make_deviation_figure(grid: np.ndarray, error: np.ndarray,
                          krl, moveit, out_png: str, task: str,
                          dpi: int = 300) -> str:
    """
    Seis subgraficos de desviacion articular frente al progreso normalizado.

    Misma identidad visual que las figuras de compare_planned_trajectories:
    mismos rcParams, misma tipografia, mismo tratamiento de titulo y nota al
    pie. El eje horizontal va en porcentaje, como el resto de figuras del
    proyecto; los CSV guardan el progreso en [0, 1].
    """
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.gridspec import GridSpec

    plt.rcParams.update({
        'font.size': 10, 'axes.titlesize': 11, 'axes.labelsize': 10,
        'xtick.labelsize': 9, 'ytick.labelsize': 9, 'legend.fontsize': 10,
        'axes.grid': True, 'grid.alpha': 0.3, 'grid.linewidth': 0.5,
        'axes.spines.top': False, 'axes.spines.right': False,
        'figure.facecolor': 'white', 'savefig.facecolor': 'white',
    })

    fig = plt.figure(figsize=(13.5, 9.4))
    gs = GridSpec(2, 3, figure=fig, hspace=0.52, wspace=0.30,
                  top=0.805, bottom=0.165, left=0.075, right=0.985)

    title = ('Desviación articular entre la ejecución KRL registrada\n'
             'y la trayectoria afinada planificada por MoveIt2')
    if task:
        title += f' — {task}'
    fig.suptitle(title, fontsize=15, fontweight='bold', y=0.965)

    fig.text(0.075, 0.845,
             f'Δqᵢ(p) = qᵢ,MoveIt2(p) − qᵢ,KRL(p) [°] sobre malla común de '
             f'{len(grid)} puntos de progreso normalizado',
             fontsize=12, fontweight='bold', ha='left')

    percent = grid * 100.0
    for j, axis in enumerate(AXES):
        ax = fig.add_subplot(gs[j // 3, j % 3])
        ax.axhline(0.0, color='#7f8c8d', linestyle=':', linewidth=1.0)
        ax.plot(percent, error[:, j], color='#1f4e79', linestyle='-',
                linewidth=1.6, alpha=0.95)
        ax.set_title(f'{axis}', fontweight='bold', pad=8)
        ax.set_xlabel('Progreso normalizado [%]')
        ax.set_ylabel(f'Δ{axis} [°]')
        ax.set_xlim(0, 100)

    fig.text(
        0.5, 0.100,
        'Desviación articular entre fuentes de distinta naturaleza: KRL es '
        'ejecución física REGISTRADA y MoveIt2 Afinado es trayectoria '
        'PLANIFICADA.\n'
        'NO es error de seguimiento, error temporal ni precisión del '
        'controlador: no se usan marcas de tiempo ni alineamiento temporal.\n'
        f'N(KRL) = {krl.n}   N(MoveIt2 Afinado) = {moveit.n}   '
        f'malla común = {len(grid)} puntos.  '
        'El progreso es fracción de MUESTRAS, no de tarea.',
        ha='center', va='top', fontsize=10, style='italic', color='#444444')

    os.makedirs(os.path.dirname(out_png) or '.', exist_ok=True)
    fig.savefig(out_png, dpi=dpi)
    plt.close(fig)
    return out_png


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=('Cuantifica la desviacion articular entre una ejecucion '
                     'KRL registrada y una trayectoria MoveIt2 afinada, sobre '
                     'progreso normalizado. Offline, solo lectura.'),
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--krl', required=True, help='CSV del log de ejecucion KRL')
    ap.add_argument('--moveit', required=True,
                    help='JSON afinado de MoveIt2 para la MISMA tarea')
    ap.add_argument('--task-name', default='')
    ap.add_argument('--output-dir', default='resultados_desviacion_krl_moveit')
    ap.add_argument('--grid-points', type=int, default=COMMON_GRID_POINTS,
                    help=f'Puntos de la malla comun (defecto '
                         f'{COMMON_GRID_POINTS})')
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
    except Exception as exc:                          # noqa: BLE001
        print(f'\nERROR: {exc}')
        return 1

    if krl.joint_names != moveit.joint_names:
        print('\n*** ABORTADO: las dos fuentes no declaran el mismo conjunto '
              'de articulaciones. ***')
        return 1
    if krl.n < 2 or moveit.n < 2:
        print('\n*** ABORTADO: se necesitan al menos 2 muestras por fuente '
              'para definir el progreso normalizado. ***')
        return 1

    print(f'\nKRL:             {krl.path}')
    print(f'  columnas      : {krl.extra["axis_columns"]}')
    print('  unidades      : grados (el CSV ya los da en grados)')
    print(f'  N             : {krl.n}')
    repeated = count_repeated_samples(krl)
    print(f'  muestras identicas a la anterior: {repeated} '
          f'({100.0 * repeated / max(1, krl.n - 1):.1f} %)')
    print(f'MoveIt2 Afinado: {moveit.path}')
    print('  unidades      : positions_rad -> grados')
    print(f'  N             : {moveit.n}')

    if repeated:
        print('\n  RESERVA: el log de KRL contiene muestras estacionarias que '
              'la trayectoria')
        print('  planificada no tiene. El progreso normalizado es fraccion de '
              'MUESTRAS, no de')
        print('  tarea, asi que parte de la desviacion medida procede de ese '
              'reparto distinto')
        print('  y no de una diferencia geometrica. No se ha filtrado nada.')

    grid = np.linspace(0.0, 1.0, int(args.grid_points))
    q_krl = resample_to_grid(krl, grid)
    q_moveit = resample_to_grid(moveit, grid)
    error = q_moveit - q_krl

    out_dir = os.path.abspath(args.output_dir)
    os.makedirs(out_dir, exist_ok=True)

    rows = deviation_rows(error)
    csv_summary = base.write_csv(
        os.path.join(out_dir, 'desviacion_articular_resumen.csv'), rows)
    csv_points = base.write_csv(
        os.path.join(out_dir, 'desviacion_articular_puntos.csv'),
        point_rows(grid, error))
    png = make_deviation_figure(
        grid, error, krl, moveit,
        os.path.join(out_dir, 'desviacion_articular.png'), task, args.dpi)

    meta = {
        'metric_name': METRIC_NAME,
        'task': task or None,
        'not_a': ('No es error de seguimiento, error temporal, precision del '
                  'controlador ni error fisico. No se usan marcas de tiempo '
                  'ni alineamiento temporal.'),
        'error_convention': ERROR_CONVENTION,
        'common_grid_points': int(args.grid_points),
        'progress_definition': 'p(k) = k / (N - 1), en [0, 1], por fuente',
        'interpolation': 'lineal (numpy.interp), sin suavizado',
        'krl': {
            'path': krl.path,
            'data_nature': krlmv.DATA_NATURE_KRL,
            'axis_columns': krl.extra['axis_columns'],
            'units': 'grados',
            'n_samples': krl.n,
            'n_total_samples': krl.extra['n_total_samples'],
            'sample_criterion': krl.extra['sample_criterion'],
            'repeated_samples': repeated,
        },
        'moveit_tuned': {
            'path': moveit.path,
            'data_nature': krlmv.DATA_NATURE_MOVEIT,
            'units': moveit.units_note,
            'n_waypoints': moveit.n,
        },
        'caveat': ('El progreso normalizado es fraccion de MUESTRAS, no de '
                   'tarea. Parte de la desviacion procede del reparto '
                   'distinto de muestras entre las dos fuentes.'),
        'outputs': {'png': png, 'summary_csv': csv_summary,
                    'points_csv': csv_points},
    }
    meta_path = os.path.join(out_dir, 'desviacion_articular_metadatos.json')
    with open(meta_path, 'w', encoding='utf-8') as handle:
        json.dump(meta, handle, indent=2, ensure_ascii=False, default=str)

    print(f'\nPNG generado: {png}')
    print(f'CSV resumen:  {csv_summary}')
    print(f'CSV puntos:   {csv_points}')
    print(f'Metadatos:    {meta_path}')

    print('\n' + '=' * 62)
    print(f'{METRIC_NAME.upper()}{f" — {task}" if task else ""}')
    print('=' * 62)
    print(f'{"Eje":5s} {"MAE [deg]":>12s} {"RMSE [deg]":>12s} '
          f'{"Maximo [deg]":>14s}')
    print('-' * 62)
    for row in rows:
        print(f'{row["eje"]:5s} {row["mae_deg"]:12.4f} '
              f'{row["rmse_deg"]:12.4f} {row["max_abs_deg"]:14.4f}')
    print('-' * 62)
    print(f'{ERROR_CONVENTION}. Malla comun de {len(grid)} puntos de progreso '
          f'normalizado.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
