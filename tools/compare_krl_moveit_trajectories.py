#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
compare_krl_moveit_trajectories.py

Compara la evolucion articular entre la EJECUCION KRL REGISTRADA y la
TRAYECTORIA AFINADA PLANIFICADA por MoveIt2, para una misma tarea.

═══════════════════════════════════════════════════════════════════════════
  LAS DOS FUENTES NO SON DE LA MISMA NATURALEZA
═══════════════════════════════════════════════════════════════════════════
  KRL              : posiciones articulares REGISTRADAS durante la ejecucion
                     fisica del robot (columnas axis_actual.A1..A6 del log
                     del bridge).

  MoveIt2 Afinado  : puntos articulares de la trayectoria PLANIFICADA y
                     almacenada en el JSON del contrato.

  Ninguna de las dos es "telemetria" de la otra, y ninguna es "planificacion"
  de la otra. Toda salida de esta herramienta lo dice explicitamente: en la
  consola, en la figura, en los CSV y en los metadatos.
═══════════════════════════════════════════════════════════════════════════

FUERA DE ALCANCE, A PROPOSITO
  Esta herramienta compara EVOLUCION Y CONFIGURACION ARTICULAR. No compara
  tiempo, velocidad, aceleracion, jerk ni frecuencia. El CSV de KRL trae
  marcas de tiempo, pero enfrentarlas a los tiempos del JSON planificado
  seria comparar dos cosas distintas. Ese analisis pertenece al estudio
  comparativo fisico, con los CSV fisicos de AMBAS condiciones.

REUTILIZACION
  El estilo de figura, la lectura de limites, el calculo de metricas y la
  escritura de CSV se IMPORTAN de compare_planned_trajectories.py, para que
  las dos familias de figuras sean visualmente la misma. Esta herramienta no
  duplica ninguna de esas piezas.

Offline y de solo lectura. No toca los CSV ni los JSON de entrada.
"""

import argparse
import csv
import json
import os
import sys
from typing import Dict, List, Optional, Tuple

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

import compare_planned_trajectories as base   # noqa: E402

AXES = base.AXES

#: Columnas del log del bridge con la posicion articular REGISTRADA.
#: Van en GRADOS: la primera muestra de los dos archivos es el HOME del
#: proyecto, [0, -90, 90, ~0, ~0, ~0], que solo tiene sentido en grados.
#: El log no publica una version en radianes, asi que no hay verificacion
#: cruzada posible y no se inventa ninguna.
KRL_AXIS_COLUMNS = [f'axis_actual.A{i}' for i in range(1, 7)]

#: Columnas que PODRIAN indicar movimiento. Se auditan, no se asumen.
MOTION_CANDIDATE_COLUMNS = (
    'in_motion', 'moving', 'motion_state', 'move_executed', 'move_ready')

DATA_NATURE_KRL = 'ejecucion fisica registrada'
DATA_NATURE_MOVEIT = 'trayectoria planificada por MoveIt2'


class KrlLoadError(Exception):
    """El CSV no tiene la forma esperada. Se aborta en vez de adivinar."""


def audit_motion_columns(rows: List[Dict[str, str]]
                         ) -> Tuple[Optional[str], List[str]]:
    """
    Busca una columna de movimiento EXPLICITA Y DISCRIMINANTE.

    Una columna que existe pero vale lo mismo en todas las filas no separa
    nada: informarla como criterio seria falso. Solo se considera utilizable
    si toma mas de un valor.

    Devuelve (columna_utilizable_o_None, notas_de_la_auditoria).
    """
    notes: List[str] = []
    usable: Optional[str] = None
    for column in MOTION_CANDIDATE_COLUMNS:
        if column not in rows[0]:
            continue
        values = {row.get(column) for row in rows}
        if len(values) == 1:
            only = next(iter(values))
            notes.append(
                f'{column}: presente, pero CONSTANTE (= {only!r}) en las '
                f'{len(rows)} muestras. No discrimina movimiento.')
        else:
            notes.append(
                f'{column}: presente y variable, valores {sorted(values)}. '
                'Utilizable como criterio explicito.')
            if usable is None:
                usable = column
    if not notes:
        notes.append(
            'Ninguna de las columnas candidatas '
            f'{list(MOTION_CANDIDATE_COLUMNS)} existe en el archivo.')
    return usable, notes


def load_krl_csv(path: str, label: str,
                 motion_column: Optional[str] = None,
                 motion_value: Optional[str] = None):
    """
    Lee las posiciones articulares registradas de un CSV del bridge.

    Sin filtro por defecto: se usan TODAS las muestras. Solo se descartan
    muestras si el operador nombra explicitamente la columna y el valor, y la
    columna resulta ser discriminante. No hay ningun detector de movimiento
    deducido de los datos: fabricar uno para que las curvas se parezcan seria
    construir el resultado.
    """
    path = os.path.abspath(path)
    if not os.path.isfile(path):
        raise KrlLoadError(f'No existe el CSV KRL: {path}')

    with open(path, 'r', newline='', encoding='utf-8-sig') as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise KrlLoadError(f'{path}: sin filas de datos.')

    missing = [c for c in KRL_AXIS_COLUMNS if c not in rows[0]]
    if missing:
        raise KrlLoadError(
            f'{path}: faltan las columnas de posicion articular {missing}. '
            f'Cabecera encontrada: {list(rows[0])[:12]}...')

    usable, notes = audit_motion_columns(rows)
    n_total = len(rows)
    criterion = 'todas las muestras (no hay criterio de movimiento utilizable)'

    selected = rows
    if motion_column is not None:
        if motion_column not in rows[0]:
            raise KrlLoadError(
                f'{path}: la columna {motion_column!r} no existe.')
        values = {row.get(motion_column) for row in rows}
        if len(values) == 1:
            raise KrlLoadError(
                f'{path}: {motion_column!r} es constante (= '
                f'{next(iter(values))!r}). Filtrar por ella dejaria 0 o todas '
                'las muestras; no es un criterio.')
        if motion_value is None:
            raise KrlLoadError(
                f'--motion-column requiere --motion-value. Valores presentes '
                f'en {motion_column!r}: {sorted(values)}')
        selected = [r for r in rows if r.get(motion_column) == motion_value]
        if not selected:
            raise KrlLoadError(
                f'{path}: ninguna muestra con {motion_column} == '
                f'{motion_value!r}.')
        criterion = f'columna {motion_column} == {motion_value!r}'

    q_deg = np.array(
        [[float(row[c]) for c in KRL_AXIS_COLUMNS] for row in selected],
        dtype=float)

    traj = base.Trajectory(
        path=path, label=label, q_deg=q_deg,
        joint_names=list(base.JOINT_NAMES),
        source_format='CSV del bridge (log de ejecucion KRL)',
        units_note=('columnas axis_actual.A1..A6, en GRADOS. El log no '
                    'publica radianes: no hay verificacion cruzada posible '
                    'y no se inventa ninguna.'),
        extra={
            'data_nature': DATA_NATURE_KRL,
            'axis_columns': list(KRL_AXIS_COLUMNS),
            'n_total_samples': n_total,
            'n_used_samples': len(selected),
            'sample_criterion': criterion,
            'motion_column_audit': notes,
            'usable_motion_column_found': usable,
        })
    return traj


def krl_metrics(traj, limits, condition_label: str, data_nature: str
                ) -> List[Dict]:
    """
    Metricas por eje, reutilizando el calculo de compare_planned_trajectories.

    Solo se renombran dos cosas: `trajectory` -> `condition`, y el recorrido
    pierde el adjetivo "planned". La formula es la misma,
    L_i = suma |q_i(k) - q_i(k-1)|, pero llamar "planificado" al recorrido de
    una ejecucion registrada seria falsear la procedencia del dato.
    """
    rows = base.metrics(traj, limits)
    out = []
    for row in rows:
        new = {'condition': condition_label, 'data_nature': data_nature}
        for key, value in row.items():
            if key == 'trajectory':
                continue
            if key == 'planned_accumulated_joint_path_deg':
                new['accumulated_joint_path_deg'] = value
            else:
                new[key] = value
        out.append(new)
    return out


def krl_detail_rows(traj, condition_label: str) -> List[Dict]:
    """Detalle por muestra, con `condition` en vez de `trajectory`."""
    rows = base.detail_rows(traj)
    out = []
    for row in rows:
        new = {'condition': condition_label}
        new.update({k: v for k, v in row.items() if k != 'trajectory'})
        out.append(new)
    return out


def build_footer(krl, moveit) -> str:
    """Nota metodologica de la figura. Cuatro lineas, sobria."""
    return (
        'KRL: posiciones articulares REGISTRADAS durante la ejecución física.  '
        'MoveIt2 Afinado: puntos de trayectoria PLANIFICADOS.\n'
        'El eje horizontal es el progreso normalizado de cada fuente por '
        'separado; NO representa tiempo.\n'
        f'N(KRL) = {krl.n}   N(MoveIt2 Afinado) = {moveit.n}\n'
        'Δq depende del muestreo físico en KRL y de la discretización en '
        'MoveIt2: no son equivalentes. Indicador COMPLEMENTARIO.')


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=('Compara la evolucion articular entre la ejecucion KRL '
                     'registrada y la trayectoria afinada planificada por '
                     'MoveIt2. Offline, solo lectura.'),
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--krl', required=True,
                    help='CSV del log de ejecucion KRL')
    ap.add_argument('--moveit', required=True,
                    help='JSON afinado de MoveIt2 para la MISMA tarea')
    ap.add_argument('--task-name', default='',
                    help='Nombre de la tarea, p. ej. "CUADRADO"')
    ap.add_argument('--output-dir', default='resultados_krl_vs_moveit')
    ap.add_argument('--label-krl', default='KRL')
    ap.add_argument('--label-moveit', default='MoveIt2 Afinado')
    ap.add_argument('--motion-column', default=None,
                    help='Columna de movimiento a usar como filtro. Por '
                         'defecto NO se filtra nada.')
    ap.add_argument('--motion-value', default=None,
                    help='Valor de --motion-column que indica movimiento.')
    ap.add_argument('--limits-source', default=None)
    ap.add_argument('--no-limits', action='store_true')
    ap.add_argument('--dpi', type=int, default=300)
    ap.add_argument('--prefix', default=None)
    args = ap.parse_args(argv)

    task = args.task_name.strip()
    suffix = f'_{task.lower().replace(" ", "_")}' if task else ''
    prefix = args.prefix or f'comparacion_krl_moveit{suffix}'

    print('=' * 78)
    print('COMPARACION KRL (ejecucion registrada) vs MoveIt2 AFINADO '
          '(planificado)')
    if task:
        print(f'TAREA: {task}')
    print('=' * 78)
    print('Las dos fuentes NO son de la misma naturaleza:')
    print(f'  KRL             = {DATA_NATURE_KRL}')
    print(f'  MoveIt2 Afinado = {DATA_NATURE_MOVEIT}')

    try:
        krl = load_krl_csv(args.krl, args.label_krl,
                           args.motion_column, args.motion_value)
        moveit = base.load_trajectory(args.moveit, args.label_moveit)
    except Exception as exc:                          # noqa: BLE001
        print(f'\nERROR: {exc}')
        return 1

    print(f'\n{krl.label}: {krl.path}')
    print(f'  formato   : {krl.source_format}')
    print(f'  unidades  : {krl.units_note}')
    print(f'  columnas  : {krl.extra["axis_columns"]}')
    print('\n  Auditoria de columnas de movimiento (seccion 6 del encargo):')
    for note in krl.extra['motion_column_audit']:
        print(f'    - {note}')
    print(f'  N total    = {krl.extra["n_total_samples"]}')
    print(f'  N usadas   = {krl.extra["n_used_samples"]}')
    print(f'  criterio   = {krl.extra["sample_criterion"]}')

    print(f'\n{moveit.label}: {moveit.path}')
    print(f'  formato   : {moveit.source_format}')
    print(f'  unidades  : {moveit.units_note}')
    print(f'  N waypoints = {moveit.n}')

    if krl.joint_names != moveit.joint_names:
        print('\n*** ABORTADO: las dos fuentes no declaran el mismo conjunto '
              'de articulaciones. ***')
        return 1

    print(f'\nN_KRL = {krl.n}    N_MoveIt2 = {moveit.n}')
    if krl.n != moveit.n:
        print('  Numero de muestras DISTINTO. Por eso el eje horizontal es el '
              'progreso normalizado')
        print('  de cada fuente por separado: la muestra k de KRL NO es el '
              'waypoint k de MoveIt2.')
        print('  No se interpola ninguna fuente al tamaño de la otra.')

    wrap = base.report_wraparound(krl) + base.report_wraparound(moveit)
    if wrap:
        print('\nAvisos de wrap-around (NO se modifico ningun valor):')
        for note in wrap:
            print(note)
    else:
        print('\nNingun eje excede ±180°: no hubo que tratar wrap-around.')

    limits = None
    limits_src = 'no utilizados (--no-limits)'
    if not args.no_limits:
        src = args.limits_source or os.path.join(
            base._REPO, base.DEFAULT_LIMITS_SOURCE)
        try:
            limits, limits_src = base.read_joint_limits(src)
            print(f'\nLimites obtenidos desde: {limits_src}')
            for axis, (lo, hi) in zip(AXES, limits):
                print(f'  {axis}: [{lo:8.2f}, {hi:8.2f}] deg')
        except Exception as exc:                      # noqa: BLE001
            print(f'\nAVISO: no se pudieron leer los limites ({exc}).')
            limits, limits_src = None, f'ERROR: {exc}'

    out_dir = os.path.abspath(args.output_dir)
    os.makedirs(out_dir, exist_ok=True)

    rows_krl = krl_metrics(krl, limits, krl.label, DATA_NATURE_KRL)
    rows_mv = krl_metrics(moveit, limits, moveit.label, DATA_NATURE_MOVEIT)
    summary = [r for j in range(len(AXES)) for r in (rows_krl[j], rows_mv[j])]
    csv_summary = base.write_csv(
        os.path.join(out_dir, f'{prefix}_resumen.csv'), summary)
    csv_detail = base.write_csv(
        os.path.join(out_dir, f'{prefix}_detalle.csv'),
        krl_detail_rows(krl, krl.label) + krl_detail_rows(moveit, moveit.label))

    title = ('Comparación de la evolución articular entre la ejecución KRL '
             'registrada\ny la trayectoria afinada planificada por MoveIt2')
    if task:
        title += f' — {task}'
    png = base.make_figure(
        [(krl, 'krl'), (moveit, 'moveit_tuned')], limits,
        os.path.join(out_dir, f'{prefix}_articular.png'),
        dpi=args.dpi, title=title, footer=build_footer(krl, moveit),
        panel_b_label=(
            '(b)  Variación articular entre muestras/puntos consecutivos de '
            'cada fuente,  Δqᵢ(k) = |qᵢ(k) − qᵢ(k−1)| [°]'))

    meta = {
        'task': task or None,
        'comparison': ('Comparacion de la evolucion articular entre la '
                       'ejecucion KRL registrada y la trayectoria afinada '
                       'planificada por MoveIt2.'),
        'scope_note': ('Solo evolucion y configuracion articular. NO compara '
                       'tiempo, velocidad, aceleracion, jerk ni frecuencia: '
                       'esas magnitudes pertenecen al estudio comparativo '
                       'fisico.'),
        'krl': {
            'path': krl.path,
            'data_nature': DATA_NATURE_KRL,
            'axis_columns': krl.extra['axis_columns'],
            'units': krl.units_note,
            'n_total_samples': krl.extra['n_total_samples'],
            'n_used_samples': krl.extra['n_used_samples'],
            'sample_criterion': krl.extra['sample_criterion'],
            'motion_column_audit': krl.extra['motion_column_audit'],
        },
        'moveit_tuned': {
            'path': moveit.path,
            'data_nature': DATA_NATURE_MOVEIT,
            'units': moveit.units_note,
            'n_waypoints': moveit.n,
            'extra': moveit.extra,
        },
        'joint_limits_source': limits_src,
        'joint_limits_deg': (
            {a: list(v) for a, v in zip(AXES, limits)} if limits else None),
        'outputs': {'png': png, 'summary_csv': csv_summary,
                    'detail_csv': csv_detail},
    }
    meta_path = os.path.join(out_dir, f'{prefix}_metadatos.json')
    with open(meta_path, 'w', encoding='utf-8') as handle:
        json.dump(meta, handle, indent=2, ensure_ascii=False, default=str)

    print(f'\nPNG generado: {png}')
    print(f'CSV resumen:  {csv_summary}')
    print(f'CSV detalle:  {csv_detail}')
    print(f'Metadatos:    {meta_path}')

    print('\n' + '=' * 78)
    print(f'RESUMEN NUMERICO{f" — {task}" if task else ""}')
    print('=' * 78)
    width = 17
    head = (f'{"eje":4s} {"fuente":{width}s} {"N":>5s} {"q_min":>9s} '
            f'{"q_max":>9s} {"rango":>8s} {"max dq":>8s} {"RMS dq":>8s} '
            f'{"recorrido":>10s}')
    if limits:
        head += f' {"margen min":>11s}'
    print(head)
    print('-' * len(head))
    for j, axis in enumerate(AXES):
        for row in (rows_krl[j], rows_mv[j]):
            line = (f'{axis:4s} {row["condition"][:width]:{width}s} '
                    f'{row["n_points"]:5d} {row["q_min_deg"]:9.3f} '
                    f'{row["q_max_deg"]:9.3f} {row["q_range_deg"]:8.3f} '
                    f'{row["max_delta_deg"]:8.3f} {row["rms_delta_deg"]:8.3f} '
                    f'{row["accumulated_joint_path_deg"]:10.3f}')
            if limits:
                line += (
                    f' {row["min_joint_margin_to_configured_limits_deg"]:11.3f}')
            print(line)
        print()
    print('recorrido = recorrido articular acumulado. REGISTRADO en KRL, '
          'PLANIFICADO en MoveIt2.')
    print('margen min = margen articular minimo respecto a los limites '
          'configurados [deg]')
    print('max dq / RMS dq dependen del muestreo de cada fuente: NO son '
          'directamente equiparables.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
