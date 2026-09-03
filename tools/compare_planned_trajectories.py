#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
compare_planned_trajectories.py

Comparacion OFFLINE de la evolucion articular de dos trayectorias PLANIFICADAS
por MoveIt2, para documentar el afinamiento previo de la tarea.

═══════════════════════════════════════════════════════════════════════════
  NATURALEZA DE LOS DATOS
═══════════════════════════════════════════════════════════════════════════
  Todo lo que produce esta herramienta son DATOS ARTICULARES DE TRAYECTORIAS
  PLANIFICADAS POR MoveIt2, leidos de archivos ya existentes.

  NO son, y no deben presentarse como:
    - telemetria fisica del robot
    - posiciones medidas en el KUKA
    - velocidades, aceleraciones o jerk fisicos
    - tiempos fisicos de ejecucion
    - metricas TCP/IP, EKI ni de frecuencia de comunicacion

  No mezclar estos resultados con los CSV del estudio comparativo fisico.
═══════════════════════════════════════════════════════════════════════════

QUE HACE
  - lee dos archivos de trayectoria ya existentes
  - calcula metricas articulares y genera un PNG y dos CSV
  - NO genera trayectorias, NO planifica, NO ejecuta nada
  - NO modifica ni sobrescribe los archivos de entrada (solo lectura)
  - funciona completamente offline, sin ROS y sin el robot

FORMATOS DE ENTRADA ACEPTADOS
  1. JSON del contrato `schema_version: 1` de kuka_moveit_trajectory_planner.
     Se lee con el parser YA EXISTENTE del proyecto
     (kuka_kr6_moveit_baseline.trajectory_json_reader), que ademas valida el
     orden de los joints y la coherencia rad/deg.
  2. CSV de kuka_trajectory_logger (columnas joint_aN_position_deg/rad).

  El formato se detecta por extension. En ambos casos las articulaciones se
  mapean POR NOMBRE, nunca por posicion en la lista.

SOBRE A6 Y EL WRAP-AROUND
  Los valores se usan TAL CUAL vienen del plan. NO se normalizan a
  [-180, 180]: hacerlo alteraria la configuracion articular realmente
  almacenada, que es justamente el objeto del analisis. Si algun eje supera
  esos limites, se avisa por consola y se deja el valor intacto.
"""

import argparse
import csv
import json
import math
import os
import sys
from typing import Dict, List, Optional, Tuple

import numpy as np

# ── Reutilizacion del parser YA EXISTENTE del proyecto ─────────────────────
_REPO = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
_PKG = os.path.join(_REPO, 'ros2_ws', 'src', 'kuka_kr6_moveit_baseline')
if _PKG not in sys.path:
    sys.path.insert(0, _PKG)

try:
    from kuka_kr6_moveit_baseline.trajectory_json_reader import (  # noqa: E402
        JsonTrajectoryError, load as load_contract_json)
    HAVE_CONTRACT_READER = True
except Exception:                                     # noqa: BLE001
    HAVE_CONTRACT_READER = False

AXES = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6']
JOINT_NAMES = ['joint_a1', 'joint_a2', 'joint_a3',
               'joint_a4', 'joint_a5', 'joint_a6']

#: URDF del proyecto. Ruta relativa a la raiz del repositorio.
DEFAULT_LIMITS_SOURCE = os.path.join(
    'ros2_ws', 'src', 'kuka_kr6_support', 'urdf', 'kr6r900sixx_macro.xacro')


# ═══════════════════════════════════════════════════════════════════════════
# Carga
# ═══════════════════════════════════════════════════════════════════════════

class Trajectory:
    """Una trayectoria planificada, ya en grados y en orden canonico A1..A6."""

    def __init__(self, path, label, q_deg, joint_names, source_format,
                 units_note, extra=None):
        self.path = path
        self.label = label
        self.q_deg = np.asarray(q_deg, dtype=float)      # (N, 6)
        self.joint_names = joint_names
        self.source_format = source_format
        self.units_note = units_note
        self.extra = extra or {}

    @property
    def n(self) -> int:
        return int(self.q_deg.shape[0])

    @property
    def progress(self) -> np.ndarray:
        """Progreso normalizado DISCRETO [%]. NO es tiempo."""
        if self.n < 2:
            return np.zeros(self.n)
        return 100.0 * np.arange(self.n) / (self.n - 1)

    @property
    def deltas(self) -> np.ndarray:
        """|q(k) - q(k-1)| por eje. Fila 0 = 0 por definicion. (N, 6)"""
        d = np.zeros_like(self.q_deg)
        if self.n > 1:
            d[1:] = np.abs(np.diff(self.q_deg, axis=0))
        return d

    @property
    def delta_norm(self) -> np.ndarray:
        """Norma euclidea del cambio articular entre puntos consecutivos."""
        return np.sqrt((self.deltas ** 2).sum(axis=1))


def _contract_format_label(segments, chosen) -> str:
    base = 'JSON contrato schema_version 1'
    if not segments:
        return base
    return f'{base} (segmentos {[g.id for g in chosen]})'


def _load_contract(path: str, label: str,
                   segments: Optional[List[str]] = None) -> Trajectory:
    """JSON del contrato schema_version 1, con el parser existente."""
    if not HAVE_CONTRACT_READER:
        raise RuntimeError(
            'No se pudo importar el parser existente '
            'kuka_kr6_moveit_baseline.trajectory_json_reader. '
            f'Se buscó en: {_PKG}')
    seq = load_contract_json(path)
    chosen = seq.segments
    if segments:
        wanted = [s.upper() for s in segments]
        chosen = [g for g in seq.segments if g.id.upper() in wanted]
        missing = [w for w in wanted if w not in {g.id.upper() for g in chosen}]
        if missing:
            raise ValueError(
                f'{path}: no existen los segmentos {missing}. '
                f'Disponibles: {[g.id for g in seq.segments]}')
    q = []
    for seg in chosen:
        for wp in seg.waypoints:
            # positions_rad es la FUENTE (unidad nativa de ROS). El parser ya
            # verifico que degrees(positions_rad) == positions_deg.
            q.append([math.degrees(v) for v in wp.positions_rad])
    return Trajectory(
        path=path, label=label, q_deg=q, joint_names=list(seq.joint_names),
        source_format=_contract_format_label(segments, chosen),
        units_note=('positions_rad (radianes) -> convertido a grados; '
                    'verificacion cruzada contra positions_deg superada'),
        extra={
            'md5': seq.md5,
            'source': seq.source,
            'generated_at': seq.generated_at,
            'n_source_points': len(seq.source_points_rad),
            'n_segments': len(chosen),
            'n_segments_in_file': len(seq.segments),
            'segments_used': [g.id for g in chosen],
            'source_point_ids': list(seq.source_point_ids),
            'gripper_events': len(seq.gripper_events),
            'planner_metadata': dict(seq.planner_metadata),
            'checks': list(seq.checks),
        })


def _load_logger_csv(path: str, label: str) -> Trajectory:
    """CSV de kuka_trajectory_logger. Mapea POR NOMBRE de columna."""
    with open(path, newline='') as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError(f'{path}: el CSV no tiene filas de datos.')

    header = rows[0].keys()
    deg_cols = [f'{j}_position_deg' for j in JOINT_NAMES]
    rad_cols = [f'{j}_position_rad' for j in JOINT_NAMES]
    if all(c in header for c in rad_cols):
        q = [[math.degrees(float(r[c])) for c in rad_cols] for r in rows]
        units = 'columnas joint_aN_position_rad -> convertido a grados'
        if all(c in header for c in deg_cols):
            worst = max(
                abs(math.degrees(float(r[rc])) - float(r[dc]))
                for r in rows for rc, dc in zip(rad_cols, deg_cols))
            units += f'; coherencia con las columnas _deg: {worst:.3e} deg'
    elif all(c in header for c in deg_cols):
        q = [[float(r[c]) for c in deg_cols] for r in rows]
        units = 'columnas joint_aN_position_deg -> ya en grados, sin conversion'
    else:
        raise ValueError(
            f'{path}: no se encontraron las columnas de posicion articular '
            f'esperadas ({rad_cols} ni {deg_cols}).')

    extra = {}
    if 'trajectory_id' in header:
        ids = sorted({int(r['trajectory_id']) for r in rows})
        extra['trajectory_ids'] = ids
        if len(ids) > 1:
            extra['warning'] = (
                f'El CSV contiene {len(ids)} trayectorias distintas '
                f'({ids}) concatenadas. Se analizan como una sola serie.')
    return Trajectory(path=path, label=label, q_deg=q,
                      joint_names=list(JOINT_NAMES),
                      source_format='CSV kuka_trajectory_logger',
                      units_note=units, extra=extra)


def load_trajectory(path: str, label: str,
                    segments: Optional[List[str]] = None) -> Trajectory:
    path = os.path.abspath(path)
    if not os.path.isfile(path):
        raise FileNotFoundError(f'No existe el archivo: {path}')
    ext = os.path.splitext(path)[1].lower()
    if ext == '.json':
        return _load_contract(path, label, segments)
    if ext == '.csv':
        if segments:
            raise ValueError(
                '--segments solo aplica a los JSON del contrato, que son los '
                'unicos que declaran segmentos.')
        return _load_logger_csv(path, label)
    raise ValueError(
        f'{path}: extension {ext!r} no soportada. Se admiten .json '
        '(contrato schema_version 1) y .csv (kuka_trajectory_logger).')


# ═══════════════════════════════════════════════════════════════════════════
# Limites articulares configurados
# ═══════════════════════════════════════════════════════════════════════════

def read_joint_limits(xacro_path: str) -> Tuple[List[Tuple[float, float]], str]:
    """
    Lee lower/upper de cada <limit> del URDF del proyecto. Devuelve grados.

    NO se codifican valores a mano: se evalua la expresion del propio archivo.
    """
    import re
    src = open(xacro_path).read()
    found: Dict[str, Tuple[float, float]] = {}
    for m in re.finditer(r'<joint name="\$\{prefix\}(joint_a\d)"[^>]*>(.*?)</joint>',
                         src, re.S):
        name, body = m.groups()
        lim = re.search(r'lower="([^"]*)"\s+upper="([^"]*)"', body)
        if not lim:
            continue

        def ev(tok):
            tok = tok.strip()
            mm = re.fullmatch(r'\$\{(.*)\}', tok)
            expr = mm.group(1) if mm else tok
            return float(eval(expr, {'radians': math.radians, 'pi': math.pi}))

        found[name] = (math.degrees(ev(lim.group(1))),
                       math.degrees(ev(lim.group(2))))
    missing = [j for j in JOINT_NAMES if j not in found]
    if missing:
        raise ValueError(
            f'{xacro_path}: no se encontraron los limites de {missing}.')
    return [found[j] for j in JOINT_NAMES], xacro_path


# ═══════════════════════════════════════════════════════════════════════════
# Metricas
# ═══════════════════════════════════════════════════════════════════════════

def metrics(traj: Trajectory,
            limits: Optional[List[Tuple[float, float]]]) -> List[Dict]:
    """Una fila de metricas por articulacion."""
    out = []
    d = traj.deltas
    for j, axis in enumerate(AXES):
        col = traj.q_deg[:, j]
        dcol = d[1:, j] if traj.n > 1 else np.array([0.0])
        row = {
            'trajectory': traj.label,
            'joint': axis,
            'n_points': traj.n,
            'q_min_deg': float(col.min()),
            'q_max_deg': float(col.max()),
            'q_range_deg': float(col.max() - col.min()),
            'max_delta_deg': float(dcol.max()),
            'mean_delta_deg': float(dcol.mean()),
            'rms_delta_deg': float(np.sqrt((dcol ** 2).mean())),
            'planned_accumulated_joint_path_deg': float(dcol.sum()),
        }
        if limits is not None:
            lo, hi = limits[j]
            margin = np.minimum(col - lo, hi - col)
            row.update({
                'limit_lower_deg': lo,
                'limit_upper_deg': hi,
                'min_joint_margin_to_configured_limits_deg': float(margin.min()),
                'min_margin_at_progress_percent': float(
                    traj.progress[int(np.argmin(margin))]),
                'out_of_configured_limits_points': int((margin < 0).sum()),
            })
        out.append(row)
    return out


def detail_rows(traj: Trajectory) -> List[Dict]:
    d = traj.deltas
    dn = traj.delta_norm
    prog = traj.progress
    rows = []
    for k in range(traj.n):
        row = {'trajectory': traj.label, 'sample': k,
               'progress_percent': float(prog[k])}
        for j, axis in enumerate(AXES):
            row[f'{axis}_deg'] = float(traj.q_deg[k, j])
        for j, axis in enumerate(AXES):
            row[f'delta_{axis}_deg'] = float(d[k, j])
        row['delta_norm_deg'] = float(dn[k])
        rows.append(row)
    return rows


# ═══════════════════════════════════════════════════════════════════════════
# Figura
# ═══════════════════════════════════════════════════════════════════════════

STYLE = {
    # ── modo de DOS condiciones (comportamiento historico, sin cambios) ──
    'preliminary': {'color': '#c0392b', 'linestyle': '--', 'linewidth': 1.8,
                    'marker': 'o', 'markersize': 2.6, 'alpha': 0.95},
    'tuned': {'color': '#1f4e79', 'linestyle': '-', 'linewidth': 1.8,
              'marker': 's', 'markersize': 2.6, 'alpha': 0.95},
    # ── modo de TRES condiciones ─────────────────────────────────────────
    # Se distinguen por ESTILO DE LINEA ademas de por color, para que la
    # figura siga siendo legible impresa, en escala de grises y en PDF.
    # Sin marcadores: con tres curvas de hasta ~450 puntos, saturan.
    'raw': {'color': '#c0392b', 'linestyle': ':', 'linewidth': 1.5,
            'alpha': 0.90},
    'restricted': {'color': '#b8860b', 'linestyle': '--', 'linewidth': 1.6,
                   'alpha': 0.90},
    'tuned3': {'color': '#1f4e79', 'linestyle': '-', 'linewidth': 1.8,
               'alpha': 0.95},
    # ── KRL (ejecucion registrada) frente a MoveIt2 afinado (planificado) ─
    # Los usa tools/compare_krl_moveit_trajectories.py. Viven aqui para que
    # las dos familias de figuras compartan identidad visual.
    'krl': {'color': '#c0392b', 'linestyle': '--', 'linewidth': 1.7,
            'alpha': 0.92},
    'moveit_tuned': {'color': '#1f4e79', 'linestyle': '-', 'linewidth': 1.8,
                     'alpha': 0.95},
}


def make_figure(trajs: List[Tuple[Trajectory, str]],
                limits: Optional[List[Tuple[float, float]]],
                out_png: str, dpi: int = 300,
                title: Optional[str] = None,
                footer: Optional[str] = None,
                panel_b_label: Optional[str] = None) -> str:
    """
    Figura de N condiciones. `trajs` es [(trayectoria, clave_de_estilo), ...].

    Estructura fija: bloque (a) con la evolucion A1..A6 y bloque (b) con Δq.
    Seis subgraficos por bloque, una curva por condicion en cada uno. Nunca
    se apilan las 18 curvas en un solo eje.
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

    three = len(trajs) > 2
    fig = plt.figure(figsize=(13.5, 17.2 if three else 16.5))
    top = 0.895 if three else 0.885
    # El pie del modo de tres condiciones ocupa cuatro lineas: necesita mas
    # margen inferior o la ultima queda cortada por el borde de la figura.
    bottom = 0.105 if (three or footer is not None) else 0.085
    gs = GridSpec(4, 3, figure=fig, hspace=0.62, wspace=0.30,
                  top=top, bottom=bottom, left=0.075, right=0.985)

    if title is None:
        title = ('Comparación de configuraciones articulares planificadas '
                 'por MoveIt2\nantes y después del afinamiento previo de la '
                 'tarea')
    fig.suptitle(title, fontsize=15, fontweight='bold', y=0.978)

    label_a = 0.925 if three else 0.917
    label_b = 0.478 if three else 0.475

    fig.text(0.075, label_a,
             '(a)  Evolución articular A1–A6 [°] frente al progreso '
             'normalizado de la trayectoria [%]',
             fontsize=12, fontweight='bold', ha='left')

    for j, axis in enumerate(AXES):
        ax = fig.add_subplot(gs[j // 3, j % 3])
        for traj, key in trajs:
            ax.plot(traj.progress, traj.q_deg[:, j], label=traj.label,
                    **STYLE[key])
        if limits is not None:
            lo, hi = limits[j]
            data_min = min(t.q_deg[:, j].min() for t, _ in trajs)
            data_max = max(t.q_deg[:, j].max() for t, _ in trajs)
            span = max(1.0, data_max - data_min)
            for value in (lo, hi):
                if data_min - span <= value <= data_max + span:
                    ax.axhline(value, color='#7f8c8d', linestyle=':',
                               linewidth=1.0)
        ax.set_title(f'{axis}', fontweight='bold', pad=8)
        ax.set_xlabel('Progreso normalizado [%]')
        ax.set_ylabel(f'{axis} [°]')
        ax.set_xlim(0, 100)
        if j == 0:
            ax.legend(loc='best', framealpha=0.92,
                      fontsize=8.5 if three else 10)

    # El rotulo por defecto dice "trayectoria planificada" porque en esta
    # herramienta las dos curvas SON planificadas. Cuando una de las fuentes
    # es una ejecucion registrada, quien llama debe pasar otro rotulo: dejar
    # el de por defecto afirmaria algo falso sobre la procedencia del dato.
    if panel_b_label is None:
        panel_b_label = ('(b)  Variación articular entre puntos consecutivos '
                         'de la trayectoria planificada,  '
                         'Δqᵢ(k) = |qᵢ(k) − qᵢ(k−1)| [°]')
    fig.text(0.075, label_b, panel_b_label,
             fontsize=12, fontweight='bold', ha='left')

    for j, axis in enumerate(AXES):
        ax = fig.add_subplot(gs[2 + j // 3, j % 3])
        for traj, key in trajs:
            ax.plot(traj.progress[1:], traj.deltas[1:, j], label=traj.label,
                    **STYLE[key])
        ax.set_title(f'Δ{axis}', fontweight='bold', pad=8)
        ax.set_xlabel('Progreso normalizado [%]')
        ax.set_ylabel(f'Δ{axis} [°]')
        ax.set_xlim(0, 100)
        ax.set_ylim(bottom=0)
        if j == 0:
            ax.legend(loc='best', framealpha=0.92,
                      fontsize=8.5 if three else 10)

    # El pie se parte en lineas cortas a proposito: con las etiquetas largas
    # del modo de tres condiciones, una sola linea se sale de la figura.
    if footer is not None:
        pass
    elif three:
        counts = '   '.join(f'N({t.label}) = {t.n}' for t, _ in trajs)
        footer = (
            'Datos articulares de trayectorias PLANIFICADAS por MoveIt2. '
            'No son telemetría física del robot.\n'
            'El eje horizontal es el progreso discreto normalizado de los '
            'puntos almacenados; NO representa tiempo.\n'
            f'{counts}\n'
            'Δq depende de la discretización de cada trayectoria y es un '
            'indicador COMPLEMENTARIO.')
    else:
        counts = ' y '.join(f'N={t.n}' for t, _ in trajs)
        footer = (
            'Datos articulares de trayectorias PLANIFICADAS por MoveIt2. '
            'No son telemetría física del robot.\n'
            'El eje horizontal es el progreso discreto normalizado de los '
            f'puntos almacenados ({counts}); NO representa tiempo. '
            'Δq depende de la discretización.')
    # 2 lineas -> 0.030 (modo de dos condiciones). 4 lineas -> 0.062 (modo de
    # tres). La formula reproduce EXACTAMENTE las dos posiciones anteriores.
    lines = footer.count('\n') + 1
    fig.text(0.5, 0.030 + 0.016 * max(0, lines - 2), footer, ha='center',
             va='top', fontsize=10, style='italic', color='#444444')

    os.makedirs(os.path.dirname(out_png) or '.', exist_ok=True)
    fig.savefig(out_png, dpi=dpi)
    plt.close(fig)
    return out_png


# ═══════════════════════════════════════════════════════════════════════════
# Comparacion entre etapas (modo de tres condiciones)
# ═══════════════════════════════════════════════════════════════════════════

#: Metricas que se siguen a lo largo de las tres etapas.
#: El booleano marca las que DEPENDEN de la discretizacion y que por tanto no
#: pueden usarse por si solas como prueba de nada.
EVOLUTION_METRICS: List[Tuple[str, bool]] = [
    ('q_range_deg', False),
    ('planned_accumulated_joint_path_deg', False),
    ('min_joint_margin_to_configured_limits_deg', False),
    ('max_delta_deg', True),
    ('rms_delta_deg', True),
]


def _pct_change(before: Optional[float], after: Optional[float]
                ) -> Optional[float]:
    """
    Cambio porcentual entre dos valores. Nada mas.

    Se divide por |before| para que el signo indique la DIRECCION del cambio
    y no lo invierta un valor de partida negativo (puede pasar con el margen
    articular si un punto queda fuera de limites). Si el valor de partida es
    cero, el porcentaje no esta definido y se devuelve None: inventar un 100 %
    ahi seria una division por cero disfrazada.
    """
    if before is None or after is None:
        return None
    if abs(before) < 1e-12:
        return None
    return 100.0 * (after - before) / abs(before)


def evolution_rows(rows_raw: List[Dict], rows_res: List[Dict],
                   rows_tun: List[Dict]) -> List[Dict]:
    """
    Una fila por (articulacion, metrica) con las tres etapas y sus cambios.

    NO se interpreta nada aqui: se restan valores. Que una metrica suba en una
    etapa y baje en la siguiente es un resultado legitimo y aparece tal cual.
    """
    by_axis = {}
    for tag, rows in (('raw', rows_raw), ('restricted', rows_res),
                      ('tuned', rows_tun)):
        for row in rows:
            by_axis.setdefault(row['joint'], {})[tag] = row

    out: List[Dict] = []
    for axis in AXES:
        stage = by_axis.get(axis)
        if not stage:
            continue
        for metric, discret in EVOLUTION_METRICS:
            values = {tag: stage[tag].get(metric) for tag in
                      ('raw', 'restricted', 'tuned') if tag in stage}
            if len(values) < 3 or any(v is None for v in values.values()):
                continue
            raw, res, tun = values['raw'], values['restricted'], values['tuned']
            out.append({
                'joint': axis,
                'metric': metric,
                'depends_on_discretisation': discret,
                'raw': raw,
                'restricted': res,
                'tuned': tun,
                'raw_to_restricted_abs': res - raw,
                'raw_to_restricted_pct': _pct_change(raw, res),
                'restricted_to_tuned_abs': tun - res,
                'restricted_to_tuned_pct': _pct_change(res, tun),
                'raw_to_tuned_abs': tun - raw,
                'raw_to_tuned_pct': _pct_change(raw, tun),
            })
    return out


def print_stage_comparison(evo: List[Dict], stage_key: str, heading: str,
                           meaning: str) -> None:
    """Imprime una etapa. `meaning` dice que representa, sin adjetivos."""
    print('\n' + '─' * 78)
    print(heading)
    print(f'  {meaning}')
    print('─' * 78)
    metric_titles = {
        'q_range_deg': 'rango articular [deg]',
        'planned_accumulated_joint_path_deg': 'recorrido acumulado [deg]',
        'min_joint_margin_to_configured_limits_deg': 'margen minimo [deg]',
    }
    for metric, title in metric_titles.items():
        rows = [r for r in evo if r['metric'] == metric]
        if not rows:
            continue
        print(f'\n  {title}')
        print(f'    {"eje":4s} {"antes":>10s} {"despues":>10s} '
              f'{"cambio":>10s} {"cambio %":>10s}')
        for row in rows:
            abs_key, pct_key = f'{stage_key}_abs', f'{stage_key}_pct'
            before = row['raw'] if stage_key.startswith('raw') else \
                row['restricted']
            after = row['tuned'] if stage_key.endswith('tuned') else \
                row['restricted']
            pct = row[pct_key]
            pct_text = f'{pct:10.2f}' if pct is not None else f'{"n/d":>10s}'
            print(f'    {row["joint"]:4s} {before:10.3f} {after:10.3f} '
                  f'{row[abs_key]:10.3f} {pct_text}')


def write_csv(path: str, rows: List[Dict]) -> str:
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    fields = list(rows[0].keys())
    with open(path, 'w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    return path


def report_wraparound(traj: Trajectory) -> List[str]:
    """Avisa de ejes fuera de [-180, 180] SIN modificar nada."""
    notes = []
    for j, axis in enumerate(AXES):
        col = traj.q_deg[:, j]
        if col.min() < -180.0 or col.max() > 180.0:
            notes.append(
                f'  {traj.label} / {axis}: rango [{col.min():.3f}, '
                f'{col.max():.3f}]° excede ±180°. Los valores se CONSERVAN '
                'tal cual: normalizarlos alteraria la configuracion '
                'articular almacenada.')
    return notes


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=('Compara la evolucion articular de trayectorias '
                     'PLANIFICADAS por MoveIt2. Offline, solo lectura.\n'
                     'Dos modos:\n'
                     '  2 condiciones : --preliminary + --tuned\n'
                     '  3 condiciones : --raw + --restricted + --tuned'),
        formatter_class=argparse.RawDescriptionHelpFormatter)
    # ── modo de DOS condiciones (historico) ─────────────────────────────
    ap.add_argument('--preliminary', default=None,
                    help='MODO 2: trayectoria anterior al afinamiento '
                         '(.json o .csv)')
    ap.add_argument('--label-preliminary', default='PRELIMINAR')
    # ── modo de TRES condiciones ────────────────────────────────────────
    ap.add_argument('--raw', default=None,
                    help='MODO 3: MoveIt2 Base RAW, sin las restricciones de '
                         'ejecutabilidad (.json o .csv)')
    ap.add_argument('--restricted', default=None,
                    help='MODO 3: MoveIt2 Base restringido, todavia condicion '
                         'base, ya con las restricciones de ejecutabilidad')
    ap.add_argument('--label-raw', default='MoveIt2 Base RAW')
    ap.add_argument('--label-restricted', default='MoveIt2 Base restringido')
    # ── comun a los dos modos ───────────────────────────────────────────
    ap.add_argument('--tuned', required=True,
                    help='Trayectoria afinada / final (.json o .csv)')
    ap.add_argument('--label-tuned', default='AFINADA')
    ap.add_argument('--output-dir', default='resultados_afinamiento',
                    help='Carpeta de salida (se crea si no existe)')
    ap.add_argument('--limits-source', default=None,
                    help='Xacro del que leer los limites. Por defecto el URDF '
                         'del proyecto.')
    ap.add_argument('--no-limits', action='store_true',
                    help='No calcular el margen respecto a los limites')
    ap.add_argument('--dpi', type=int, default=300)
    ap.add_argument('--prefix', default=None,
                    help='Prefijo de los archivos de salida. Por defecto '
                         '"comparacion_afinamiento" en modo de 2 condiciones '
                         'y "comparacion_global" en modo de 3.')
    ap.add_argument('--segments', default=None,
                    help='Restringir a ciertos segmentos, p. ej. "T1,T2,T3,T4". '
                         'Util para comparar SOLO el tramo comun a las '
                         'trayectorias. Solo aplica a los JSON del contrato.')
    args = ap.parse_args(argv)

    # ── Resolucion de modo. Ambiguo => se aborta, no se adivina ─────────
    has_two = args.preliminary is not None
    has_three_any = args.raw is not None or args.restricted is not None
    has_three_all = args.raw is not None and args.restricted is not None

    if has_two and has_three_any:
        print('ERROR: no se pueden mezclar los dos modos.\n'
              '  MODO 2 condiciones: --preliminary + --tuned\n'
              '  MODO 3 condiciones: --raw + --restricted + --tuned\n'
              'Se han recibido argumentos de los dos. Elige uno.')
        return 2
    if has_three_any and not has_three_all:
        missing = '--raw' if args.raw is None else '--restricted'
        print(f'ERROR: el modo de 3 condiciones necesita --raw, --restricted '
              f'y --tuned. Falta {missing}.')
        return 2
    if not has_two and not has_three_all:
        print('ERROR: indica una condicion de partida.\n'
              '  MODO 2 condiciones: --preliminary <archivo> --tuned <archivo>\n'
              '  MODO 3 condiciones: --raw <archivo> --restricted <archivo> '
              '--tuned <archivo>')
        return 2

    three = has_three_all
    prefix = args.prefix or (
        'comparacion_global' if three else 'comparacion_afinamiento')

    print('=' * 78)
    print('COMPARACION DE TRAYECTORIAS PLANIFICADAS POR MoveIt2')
    print(f'MODO: {"TRES" if three else "DOS"} condiciones')
    print('Datos articulares de trayectorias PLANIFICADAS. '
          'NO son telemetria fisica.')
    print('=' * 78)

    segments = None
    if args.segments:
        segments = [t.strip() for t in args.segments.split(',') if t.strip()]
        print(f'\nRestringido a los segmentos: {segments}')

    # ── Carga ───────────────────────────────────────────────────────────
    if three:
        wanted = [(args.raw, args.label_raw, 'raw'),
                  (args.restricted, args.label_restricted, 'restricted'),
                  (args.tuned, args.label_tuned, 'tuned3')]
    else:
        wanted = [(args.preliminary, args.label_preliminary, 'preliminary'),
                  (args.tuned, args.label_tuned, 'tuned')]
    try:
        loaded = [(load_trajectory(path, label, segments), style)
                  for path, label, style in wanted]
    except (FileNotFoundError, ValueError, RuntimeError,
            Exception) as exc:                        # noqa: BLE001
        if HAVE_CONTRACT_READER and isinstance(exc, JsonTrajectoryError):
            print(f'\nERROR de contrato: {exc}')
        else:
            print(f'\nERROR: {exc}')
        return 1

    for traj, _ in loaded:
        print(f'\n{traj.label}: {traj.path}')
        print(f'  formato   : {traj.source_format}')
        print(f'  unidades  : {traj.units_note}')
        print(f'  joints    : {traj.joint_names}')

    reference = loaded[0][0].joint_names
    for traj, _ in loaded[1:]:
        if traj.joint_names != reference:
            print('\n*** ABORTADO: las trayectorias no declaran el mismo '
                  'conjunto de articulaciones. No pertenecen al mismo robot '
                  'o al mismo grupo. ***')
            return 1
    print('\nMismo conjunto de articulaciones en todas: SI '
          '(mapeadas POR NOMBRE, no por posicion)')

    print()
    for traj, _ in loaded:
        print(f'N {traj.label}: {traj.n}')
    counts = {traj.n for traj, _ in loaded}
    if len(counts) > 1:
        print('\n  *** AVISO: las trayectorias tienen distinto numero de '
              'puntos. ***')
        print('  El maximo y el promedio de Δq DEPENDEN de la discretizacion.')
        print('  NO deben usarse por si solos como prueba de que una '
              'trayectoria es mejor: son indicadores COMPLEMENTARIOS.')
        print('  La evidencia principal esta en la evolucion de A1–A6, los '
              'extremos, el rango y el recorrido acumulado.')

    wrap = []
    for traj, _ in loaded:
        wrap += report_wraparound(traj)
    if wrap:
        print('\nAvisos de wrap-around (NO se modifico ningun valor):')
        for note in wrap:
            print(note)
    else:
        print('\nNingun eje excede ±180°: no hubo que tratar wrap-around.')

    # ── Limites ─────────────────────────────────────────────────────────
    limits = None
    limits_src = 'no utilizados (--no-limits)'
    if not args.no_limits:
        src = args.limits_source or os.path.join(_REPO, DEFAULT_LIMITS_SOURCE)
        try:
            limits, limits_src = read_joint_limits(src)
            print(f'\nLimites obtenidos desde: {limits_src}')
            for axis, (lo, hi) in zip(AXES, limits):
                print(f'  {axis}: [{lo:8.2f}, {hi:8.2f}] deg')
        except Exception as exc:                      # noqa: BLE001
            print(f'\nAVISO: no se pudieron leer los limites ({exc}). '
                  'Se omite el margen articular.')
            limits, limits_src = None, f'ERROR: {exc}'

    out_dir = os.path.abspath(args.output_dir)
    os.makedirs(out_dir, exist_ok=True)

    # ── Salidas ─────────────────────────────────────────────────────────
    per_traj = [metrics(traj, limits) for traj, _ in loaded]
    # Agrupado POR EJE: las condiciones de un mismo eje quedan contiguas, que
    # es como se lee la comparacion. per_traj[t][j] -> condicion t, eje j.
    summary = [rows[j] for j in range(len(AXES)) for rows in per_traj]
    csv_summary = write_csv(
        os.path.join(out_dir, f'{prefix}_resumen.csv'), summary)
    detail = [row for traj, _ in loaded for row in detail_rows(traj)]
    csv_detail = write_csv(
        os.path.join(out_dir, f'{prefix}_detalle.csv'), detail)

    if three:
        png_name = f'{prefix}_raw_restringido_afinado_articular.png'
        title = ('Configuraciones articulares planificadas por MoveIt2 en las '
                 'tres condiciones del estudio\n'
                 f'{loaded[0][0].label} · {loaded[1][0].label} · '
                 f'{loaded[2][0].label}')
    else:
        png_name = f'{prefix}_articular.png'
        title = None
    png = make_figure(loaded, limits, os.path.join(out_dir, png_name),
                      dpi=args.dpi, title=title)

    csv_evolution = None
    evo: List[Dict] = []
    if three:
        evo = evolution_rows(per_traj[0], per_traj[1], per_traj[2])
        if evo:
            csv_evolution = write_csv(
                os.path.join(out_dir, f'{prefix}_evolucion.csv'), evo)

    meta = {
        'mode': 'three_conditions' if three else 'two_conditions',
        'data_nature': ('Datos articulares de trayectorias PLANIFICADAS por '
                        'MoveIt2. No son telemetria fisica del robot.'),
        'joint_limits_source': limits_src,
        'joint_limits_deg': (
            {a: list(l) for a, l in zip(AXES, limits)} if limits else None),
    }
    keys = (['raw', 'restricted', 'tuned'] if three
            else ['preliminary', 'tuned'])
    for key, (traj, _) in zip(keys, loaded):
        meta[key] = {'path': traj.path, 'n_points': traj.n,
                     'label': traj.label, 'format': traj.source_format,
                     'units': traj.units_note, 'extra': traj.extra}
    if three:
        meta['stage_semantics'] = {
            'raw_to_restricted': ('aplicacion de restricciones de '
                                  'ejecutabilidad. NO es el afinamiento.'),
            'restricted_to_tuned': 'etapa de afinamiento.',
            'raw_to_tuned': 'evolucion global, separada de las dos etapas.',
        }
    meta['outputs'] = {'png': png, 'summary_csv': csv_summary,
                       'detail_csv': csv_detail,
                       'evolution_csv': csv_evolution}
    meta_path = os.path.join(out_dir, f'{prefix}_metadatos.json')
    with open(meta_path, 'w', encoding='utf-8') as handle:
        json.dump(meta, handle, indent=2, ensure_ascii=False, default=str)

    print(f'\nPNG generado: {png}')
    print(f'CSV resumen:  {csv_summary}')
    print(f'CSV detalle:  {csv_detail}')
    if csv_evolution:
        print(f'CSV evolucion:{csv_evolution}')
    print(f'Metadatos:    {meta_path}')

    # ── Tabla por consola ────────────────────────────────────────────
    print('\n' + '=' * 78)
    print('RESUMEN NUMERICO')
    print('=' * 78)
    width = 26 if three else 11
    head = (f'{"eje":4s} {"condicion":{width}s} {"N":>5s} {"q_min":>9s} '
            f'{"q_max":>9s} {"rango":>8s} {"max dq":>8s} {"RMS dq":>8s} '
            f'{"recorrido":>10s}')
    if limits:
        head += f' {"margen min":>11s}'
    print(head)
    print('-' * len(head))
    for j, axis in enumerate(AXES):
        for rows in per_traj:
            row = rows[j]
            line = (f'{axis:4s} {row["trajectory"][:width]:{width}s} '
                    f'{row["n_points"]:5d} {row["q_min_deg"]:9.3f} '
                    f'{row["q_max_deg"]:9.3f} {row["q_range_deg"]:8.3f} '
                    f'{row["max_delta_deg"]:8.3f} {row["rms_delta_deg"]:8.3f} '
                    f'{row["planned_accumulated_joint_path_deg"]:10.3f}')
            if limits:
                line += (
                    f' {row["min_joint_margin_to_configured_limits_deg"]:11.3f}')
            print(line)
        print()
    print('recorrido = recorrido articular acumulado planificado [deg]')
    print('margen min = margen articular minimo respecto a los limites '
          'configurados [deg]')
    print('max dq / RMS dq DEPENDEN de la discretizacion de cada trayectoria.')

    # ── Cambios entre etapas ─────────────────────────────────────────
    if three and evo:
        print('\n' + '=' * 78)
        print('CAMBIOS ENTRE ETAPAS')
        print('=' * 78)
        print('Los valores son lo que sale de los datos. Una metrica puede '
              'subir en una etapa\ny bajar en la siguiente: eso se muestra '
              'tal cual, sin seleccionar ni suavizar.')
        print_stage_comparison(
            evo, 'raw_to_restricted',
            'ETAPA 1 — RAW → RESTRINGIDO',
            'Efecto de aplicar las restricciones de ejecutabilidad. '
            'NO es el afinamiento.')
        print_stage_comparison(
            evo, 'restricted_to_tuned',
            'ETAPA 2 — RESTRINGIDO → AFINADO',
            'Efecto del afinamiento. Es la comparacion principal.')
        print_stage_comparison(
            evo, 'raw_to_tuned',
            'GLOBAL — RAW → AFINADO',
            'Evolucion global. Conceptualmente separada de las dos etapas '
            'anteriores: suma dos efectos distintos.')
        print('\n"cambio %" es un cambio porcentual, nada mas. Para el margen '
              'articular NO\nexpresa seguridad, ni mejora, ni factor de '
              'ningun tipo.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
