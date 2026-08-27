#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
continuity_metrics.py

Logica pura (sin ROS) del analizador de continuidad articular de la condicion
MOVEIT2 BASE.

Calcula, para una trayectoria ya planificada por MoveIt2:

  - delta articular entre waypoints consecutivos, por eje
  - delta maximo y en que par de waypoints ocurre
  - saltos de configuracion: cambio de signo o salto proximo a 180 deg en
    A4 y A6, y cambio abrupto en A5 asociado a inversion de muñeca
  - numero de waypoints
  - recorrido articular total acumulado por eje
  - distancia articular directa entre inicio y meta
  - relacion recorrido/distancia directa (indicador de desplazamiento
    innecesario)
  - tiempo total de la trayectoria
  - numero de condicion del Jacobiano en cada waypoint, con su maximo

NO planifica, NO ejecuta y NO modifica la trayectoria. Solo mide.

═══════════════════════════════════════════════════════════════════════════
PROCEDENCIA DE LA GEOMETRIA
═══════════════════════════════════════════════════════════════════════════
La cadena cinematica esta CONGELADA aqui, copiada de
    urdf/kr6r900sixx_macro.xacro   (lineas 115-156 de este paquete)
que a su vez es copia identica del URDF de kuka_kr6_support en el commit
0c2c022. Se replica en Python para no depender de KDL ni de un servidor de
parametros: el analizador debe poder correr sin move_group.

TIP DECLARADO: tool0, coherente con el <chain tip_link="tool0"> del SRDF.

Las dos transformadas fijas finales del URDF tienen TRASLACION NULA:
    joint_a6-flange : xyz="0 0 0"  rpy="0 0 0"              -> flange == link_6
    flange-tool0    : xyz="0 0 0"  rpy="0 ${radians(90)} 0" -> rotacion PURA

Consecuencia, verificada numericamente (README seccion 10.0):
  - link_6, flange y tool0 comparten EXACTAMENTE el mismo ORIGEN.
  - Por tanto el Jacobiano geometrico en base_link, cuya referencia es ese
    origen (Jv_i = a_i x (p_tip - o_i), Jw_i = a_i), es BIT A BIT IDENTICO
    para los tres: |J - J(link_6)|_max = 0.000e+00 exacto.
  - rank, sigma_min, cond(J) y todas las metricas de singularidad son
    INVARIANTES respecto al tip declarado.

Por eso forward_kinematics() calcula la cadena hasta el origen comun y
devuelve la ORIENTACION de link_6. Para obtener la pose completa en tool0
(necesaria para derivar objetivos cartesianos) usar forward_kinematics_tip().

Si el URDF de este paquete cambiara, esta tabla quedaria desactualizada.
Por diseño el paquete esta congelado (propiedad 1c del encargo), pero la
limitacion queda registrada en el README seccion 10.
"""

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

# ─────────────────────────────────────────────────────────────────────────────
# Constantes
# ─────────────────────────────────────────────────────────────────────────────

RAD_TO_DEG = 180.0 / math.pi

#: Etiquetas KUKA de los ejes, en el orden del grupo de planificacion.
AXES = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6']

#: Columnas de metadatos que se anexan al FINAL del CSV (J-D5).
#: `segment_id` y `global_time_s` los rellena csv_rows desde el analisis;
#: el resto salen del diccionario `metadata`.
METADATA_COLUMNS = ['segment_id', 'global_time_s', 'condition', 'source_file',
                    'source_md5', 'declared_tip']

#: Nombres ROS de los joints, en el mismo orden.
JOINT_NAMES = ['joint_a1', 'joint_a2', 'joint_a3',
               'joint_a4', 'joint_a5', 'joint_a6']

#: Cadena base_link -> link_6 (origen comun de link_6/flange/tool0):
#: (origen xyz del joint, eje de rotacion).
#: Copiado literalmente de urdf/kr6r900sixx_macro.xacro.
CHAIN: List[Tuple[Tuple[float, float, float], Tuple[float, float, float]]] = [
    ((0.000, 0.0, 0.400), (0.0, 0.0, -1.0)),   # joint_a1
    ((0.025, 0.0, 0.000), (0.0, 1.0, 0.0)),    # joint_a2
    ((0.455, 0.0, 0.000), (0.0, 1.0, 0.0)),    # joint_a3
    ((0.000, 0.0, 0.035), (-1.0, 0.0, 0.0)),   # joint_a4
    ((0.420, 0.0, 0.000), (0.0, 1.0, 0.0)),    # joint_a5
    ((0.080, 0.0, 0.000), (-1.0, 0.0, 0.0)),   # joint_a6
]

#: Transformadas fijas finales del URDF, copiadas de kr6r900sixx_macro.xacro.
#: Ambas con TRASLACION NULA -> los tres frames comparten origen.
#:   joint_a6-flange : xyz="0 0 0" rpy="0 0 0"
#:   flange-tool0    : xyz="0 0 0" rpy="0 ${radians(90)} 0"
FIXED_TIP_ROTATIONS: Dict[str, float] = {
    'link_6': 0.0,
    'flange': 0.0,              # identidad respecto a link_6
    'tool0': math.pi / 2.0,     # +90 deg alrededor de +Y
}

#: Tip declarado en el <chain> del SRDF del baseline.
DECLARED_TIP = 'tool0'


# ── Umbrales del DETECTOR (no son parametros del planificador) ──────────────
#: Salto en un solo paso a partir del cual A4/A6 se marca como candidato a
#: cambio de configuracion.
DEFAULT_FLIP_THRESHOLD_DEG = 90.0
#: Banda alrededor de 180 deg que se considera inversion de muñeca.
DEFAULT_NEAR_180_BAND_DEG = 30.0
#: Cambio de signo por debajo de esta magnitud se ignora (ruido alrededor de 0).
DEFAULT_SIGN_CHANGE_MIN_DEG = 10.0
#: Salto en A5 a partir del cual se marca cambio abrupto de muñeca.
DEFAULT_A5_ABRUPT_DEG = 45.0
#: sigma_min por debajo de la cual el waypoint se considera singular.
DEFAULT_SINGULARITY_SIGMA_MIN = 1e-6


# ─────────────────────────────────────────────────────────────────────────────
# Cinematica directa y Jacobiano
# ─────────────────────────────────────────────────────────────────────────────

def _rot_axis(axis: Sequence[float], angle: float) -> np.ndarray:
    a = np.array(axis, dtype=float)
    a /= np.linalg.norm(a)
    K = np.array([[0.0, -a[2], a[1]],
                  [a[2], 0.0, -a[0]],
                  [-a[1], a[0], 0.0]])
    return np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)


def forward_kinematics(q_rad: Sequence[float]):
    """
    FK de base_link a link_6.

    Devuelve (T_4x4, ejes_en_base, origenes_en_base), donde ejes[i] es el eje
    de rotacion del joint i expresado en base_link y origenes[i] su origen.
    """
    R = np.eye(3)
    p = np.zeros(3)
    axes = []
    origins = []
    for i, (xyz, axis) in enumerate(CHAIN):
        p = p + R @ np.array(xyz, dtype=float)
        unit = np.array(axis, dtype=float) / np.linalg.norm(axis)
        axes.append(R @ unit)
        origins.append(p.copy())
        R = R @ _rot_axis(axis, float(q_rad[i]))
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T, np.array(axes), np.array(origins)


def forward_kinematics_tip(q_rad: Sequence[float], tip: str = DECLARED_TIP):
    """
    FK de base_link al frame `tip` (link_6, flange o tool0).

    Los tres comparten origen; solo cambia la orientacion. Devuelve T_4x4.

    Esta es la funcion que debe usarse para DERIVAR objetivos cartesianos:
    la pose se calcula SIEMPRE desde los valores articulares con la cinematica
    de ESTE paquete, nunca se lee de una fuente externa.
    """
    if tip not in FIXED_TIP_ROTATIONS:
        raise ValueError(
            f'tip desconocido: {tip!r}. Validos: {sorted(FIXED_TIP_ROTATIONS)}')
    T, _, _ = forward_kinematics(q_rad)
    angle = FIXED_TIP_ROTATIONS[tip]
    if angle != 0.0:
        T = T.copy()
        T[:3, :3] = T[:3, :3] @ _rot_axis((0.0, 1.0, 0.0), angle)
    return T


def geometric_jacobian(q_rad: Sequence[float]) -> np.ndarray:
    """
    Jacobiano geometrico 6x6 expresado en base_link.

    INVARIANTE respecto al tip declarado: link_6, flange y tool0 comparten
    origen, asi que Jv es identico, y Jw no depende del frame del tip.
    Verificado: |J - J(link_6)|_max = 0.000e+00 exacto en los tres.
    """
    T, axes, origins = forward_kinematics(q_rad)
    p_end = T[:3, 3]
    J = np.zeros((6, 6))
    for i in range(6):
        J[:3, i] = np.cross(axes[i], p_end - origins[i])
        J[3:, i] = axes[i]
    return J


def jacobian_indices(q_rad: Sequence[float]) -> Dict[str, float]:
    """
    Indices de proximidad a singularidad en una configuracion.

    AVISO METODOLOGICO: el Jacobiano 6x6 mezcla metros y radianes, asi que
    cond(J) depende de las unidades y no debe citarse aislado. Los indicadores
    que NO dependen de unidades son sigma_min, el rango, y cond del bloque
    rotacional 3x3 (adimensional). Se devuelven los tres.
    """
    J = geometric_jacobian(q_rad)
    sv = np.linalg.svd(J, compute_uv=False)
    sigma_min = float(sv[-1])
    cond = float(sv[0] / sv[-1]) if sigma_min > 1e-15 else float('inf')

    Jw = J[3:, 3:]
    svw = np.linalg.svd(Jw, compute_uv=False)
    sigma_w = float(svw[-1])
    cond_w = float(svw[0] / svw[-1]) if sigma_w > 1e-15 else float('inf')

    _, axes, _ = forward_kinematics(q_rad)
    a4_a6 = float(abs(np.dot(axes[3], axes[5])))

    return {
        'sigma_min': sigma_min,
        'condition_number': cond,
        'rank': int(np.linalg.matrix_rank(J, tol=1e-9)),
        'wrist_condition_number': cond_w,
        'a4_a6_alignment': a4_a6,
        'is_singular': sigma_min < DEFAULT_SINGULARITY_SIGMA_MIN,
    }


# ─────────────────────────────────────────────────────────────────────────────
# Estructuras
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class Waypoint:
    """Un punto de la trayectoria, en el orden canonico de JOINT_NAMES."""

    index: int
    time_from_start_s: float
    positions_rad: List[float]
    velocities_rad_s: List[float] = field(default_factory=list)
    accelerations_rad_s2: List[float] = field(default_factory=list)
    #: Derivado, NO entregado por MoveIt. Lo rellena analyze(). Ver JERK_NOTE.
    jerk_rad_s3: List[float] = field(default_factory=list)

    @property
    def positions_deg(self) -> List[float]:
        return [v * RAD_TO_DEG for v in self.positions_rad]


@dataclass
class ConfigurationEvent:
    """Un salto de configuracion detectado entre dos waypoints."""

    kind: str          # 'sign_change' | 'near_180' | 'a5_abrupt' | 'singular'
    axis: str
    from_index: int
    to_index: int
    delta_deg: float
    detail: str = ''


@dataclass
class TrajectoryAnalysis:
    """Resultado completo del analisis de una trayectoria."""

    trajectory_id: int
    waypoint_count: int
    total_time_s: float
    waypoints: List[Waypoint]
    #: deltas[i][j] = |q[i+1][j] - q[i][j]| en grados
    deltas_deg: List[List[float]]
    max_delta_deg: float
    max_delta_axis: str
    max_delta_from: int
    max_delta_to: int
    #: recorrido acumulado por eje (grados)
    path_length_deg: List[float]
    #: |meta - inicio| por eje (grados)
    direct_distance_deg: List[float]
    #: recorrido / distancia directa, por eje (inf si la directa es 0)
    ratio_per_axis: List[float]
    total_path_deg: float
    total_direct_deg: float
    total_ratio: float
    events: List[ConfigurationEvent]
    #: indices del Jacobiano por waypoint
    jacobian: List[Dict[str, float]]
    max_condition_number: float
    max_condition_index: int
    min_sigma_min: float
    min_sigma_index: int
    singular_waypoints: List[int]
    #: Jerk maximo absoluto por eje [rad/s^3]. nan si no se pudo derivar.
    max_jerk_per_axis: List[float] = field(
        default_factory=lambda: [float('nan')] * 6)
    #: Jerk RMS por eje [rad/s^3]. nan si no se pudo derivar.
    rms_jerk_per_axis: List[float] = field(
        default_factory=lambda: [float('nan')] * 6)
    #: 'accelerations' | 'velocities' | 'unavailable'
    jerk_source: str = 'unavailable'
    #: Identificador del segmento (T1..TN) cuando la trayectoria es parte de
    #: una secuencia. Vacio para una planificacion suelta.
    segment_id: str = ''
    #: Desplazamiento del reloj GLOBAL de la secuencia. El tiempo de cada
    #: segmento REINICIA en 0 (asi lo produce MoveIt2 y asi viene en el JSON);
    #: global_time_s = time_from_start_s + global_time_offset_s.
    global_time_offset_s: float = 0.0
    #: Etiquetas de procedencia que viajan a cada fila del CSV.
    metadata: Dict[str, str] = field(default_factory=dict)


# ─────────────────────────────────────────────────────────────────────────────
# Analisis
# ─────────────────────────────────────────────────────────────────────────────

def reorder_to_canonical(joint_names: Sequence[str],
                         values: Sequence[float]) -> Optional[List[float]]:
    """
    Reordena un vector al orden de JOINT_NAMES.

    Devuelve None si el mensaje no trae los seis joints del grupo. No se
    inventa ningun valor.
    """
    if not values:
        return None
    names = list(joint_names)
    try:
        order = [names.index(n) for n in JOINT_NAMES]
    except ValueError:
        return None
    if max(order) >= len(values):
        return None
    return [float(values[i]) for i in order]


JERK_NOTE = """\
JERK — METRICA DERIVADA, NO ENTREGADA POR MoveIt2.

  MoveIt 2.5.9 NO publica jerk en ningun mensaje. La trayectoria de
  /move_action trae posiciones, velocidades y aceleraciones, nada mas. El jerk
  hay que derivarlo.

  METODO. Se deriva UNA VEZ desde las aceleraciones que entrega TOTG:
      j[i] = (a[i+1] - a[i-1]) / (t[i+1] - t[i-1])     (diferencia centrada)
      j[0] = (a[1] - a[0]) / (t[1] - t[0])             (adelantada)
      j[n-1] = (a[n-1] - a[n-2]) / (t[n-1] - t[n-2])   (atrasada)
  Si no hay aceleraciones se deriva DOS veces desde las velocidades, y se
  marca jerk_source = 'velocities'. Si tampoco hay velocidades, queda nan y
  jerk_source = 'unavailable'. NUNCA se rellena con ceros.

  LIMITACION, QUE DEBE CITARSE JUNTO A CUALQUIER CIFRA DE JERK.
  TOTG produce un perfil de aceleracion BANG-BANG: tramos de aceleracion
  constante saturada al limite, con conmutaciones instantaneas entre ellos. El
  jerk real de ese perfil es CERO dentro de cada tramo e INFINITO en cada
  conmutacion. Ademas TOTG remuestrea a resample_dt = 0.1 s, asi que la
  conmutacion casi nunca cae sobre un waypoint.

  Consecuencia: el jerk numerico NO mide una magnitud fisica del movimiento.
  Mide DONDE cayo la rejilla de muestreo respecto a las conmutaciones. Con
  |a| <= 1.0 rad/s^2, dt = 0.1 s y diferencia CENTRADA (denominador 2*dt),
  una conmutacion completa de -1.0 a +1.0 da como cota superior
      |j| = 2 * 1.0 / (2 * 0.1) = 10.0 rad/s^3
  que es un artefacto del muestreo, no del robot. Medido sobre la trayectoria
  grabada del sistema afinado: jerk maximo = 10.0000 rad/s^3 EXACTO, en A1 y
  en A5. La cota se alcanza, lo que confirma que el valor esta gobernado por
  la saturacion del limite y por dt, no por la geometria del movimiento.

  USO DEFENDIBLE: comparar el NUMERO y la MAGNITUD de las conmutaciones entre
  condiciones, siempre que ambas tengan el mismo resample_dt (lo tienen: es
  una constante interna de TOTG, no un parametro de la configuracion).
  USO NO DEFENDIBLE: citar el jerk como propiedad fisica del movimiento, o
  compararlo contra un limite de jerk de la hoja de datos del KUKA.
"""


def _derive(values: List[List[float]], times: List[float]) -> List[List[float]]:
    """Derivada temporal por diferencias centradas, extremos de un lado."""
    n = len(values)
    if n < 2:
        return [[float('nan')] * 6 for _ in range(n)]
    out: List[List[float]] = []
    for i in range(n):
        lo = max(0, i - 1)
        hi = min(n - 1, i + 1)
        dt = times[hi] - times[lo]
        if dt <= 0.0:
            out.append([float('nan')] * 6)
            continue
        out.append([(values[hi][j] - values[lo][j]) / dt for j in range(6)])
    return out


def compute_jerk(waypoints: List[Waypoint]) -> str:
    """
    Rellena waypoint.jerk_rad_s3 in situ. Devuelve la fuente usada.

    Ver JERK_NOTE para el metodo y sus limitaciones.
    """
    n = len(waypoints)
    if n < 2:
        for wp in waypoints:
            wp.jerk_rad_s3 = [float('nan')] * 6
        return 'unavailable'
    times = [wp.time_from_start_s for wp in waypoints]

    def complete(attr: str) -> bool:
        return all(len(getattr(wp, attr)) == 6 for wp in waypoints)

    if complete('accelerations_rad_s2'):
        jerk = _derive([wp.accelerations_rad_s2 for wp in waypoints], times)
        source = 'accelerations'
    elif complete('velocities_rad_s'):
        accel = _derive([wp.velocities_rad_s for wp in waypoints], times)
        jerk = _derive(accel, times)
        source = 'velocities'
    else:
        jerk = [[float('nan')] * 6 for _ in range(n)]
        source = 'unavailable'
    for wp, row in zip(waypoints, jerk):
        wp.jerk_rad_s3 = row
    return source


def analyze(waypoints: List[Waypoint],
            trajectory_id: int = 0,
            flip_threshold_deg: float = DEFAULT_FLIP_THRESHOLD_DEG,
            near_180_band_deg: float = DEFAULT_NEAR_180_BAND_DEG,
            sign_change_min_deg: float = DEFAULT_SIGN_CHANGE_MIN_DEG,
            a5_abrupt_deg: float = DEFAULT_A5_ABRUPT_DEG,
            segment_id: str = '',
            global_time_offset_s: float = 0.0,
            metadata: Optional[Dict[str, str]] = None,
            ) -> TrajectoryAnalysis:
    """Calcula todas las metricas de una trayectoria ya planificada."""
    n = len(waypoints)
    jerk_source = compute_jerk(waypoints)
    max_jerk = [float('nan')] * 6
    rms_jerk = [float('nan')] * 6
    if jerk_source != 'unavailable':
        for j in range(6):
            column = []
            for wp in waypoints:
                if len(wp.jerk_rad_s3) != 6:
                    continue
                value = wp.jerk_rad_s3[j]
                if not math.isnan(value):
                    column.append(abs(value))
            if column:
                max_jerk[j] = max(column)
                rms_jerk[j] = math.sqrt(
                    sum(v * v for v in column) / len(column))
    deltas: List[List[float]] = []
    path = [0.0] * 6
    events: List[ConfigurationEvent] = []

    for k in range(n - 1):
        a = waypoints[k].positions_deg
        b = waypoints[k + 1].positions_deg
        row = [abs(b[j] - a[j]) for j in range(6)]
        deltas.append(row)
        for j in range(6):
            path[j] += row[j]

        # ── Saltos de configuracion ────────────────────────────────────
        for j, axis in enumerate(AXES):
            signed = b[j] - a[j]
            mag = abs(signed)
            if axis in ('A4', 'A6'):
                if abs(mag - 180.0) <= near_180_band_deg:
                    events.append(ConfigurationEvent(
                        'near_180', axis, k, k + 1, signed,
                        f'salto de {mag:.2f} deg, proximo a 180 deg '
                        '(inversion de muñeca)'))
                elif mag >= flip_threshold_deg:
                    events.append(ConfigurationEvent(
                        'near_180', axis, k, k + 1, signed,
                        f'salto de {mag:.2f} deg en un solo paso'))
                crosses_zero = a[j] * b[j] < 0.0
                far_enough = min(abs(a[j]), abs(b[j])) >= sign_change_min_deg
                if crosses_zero and far_enough:
                    events.append(ConfigurationEvent(
                        'sign_change', axis, k, k + 1, signed,
                        f'{a[j]:+.2f} -> {b[j]:+.2f} deg'))
            if axis == 'A5' and mag >= a5_abrupt_deg:
                events.append(ConfigurationEvent(
                    'a5_abrupt', axis, k, k + 1, signed,
                    f'cambio abrupto de {mag:.2f} deg en A5'))

    # ── Delta maximo ───────────────────────────────────────────────────
    max_delta, max_axis, max_from = 0.0, AXES[0], 0
    for k, row in enumerate(deltas):
        for j, value in enumerate(row):
            if value > max_delta:
                max_delta, max_axis, max_from = value, AXES[j], k

    # ── Recorrido vs distancia directa ─────────────────────────────────
    if n >= 2:
        first = waypoints[0].positions_deg
        last = waypoints[-1].positions_deg
        direct = [abs(last[j] - first[j]) for j in range(6)]
    else:
        direct = [0.0] * 6
    ratio = [
        (path[j] / direct[j]) if direct[j] > 1e-9
        else (float('inf') if path[j] > 1e-9 else 1.0)
        for j in range(6)
    ]
    total_path = sum(path)
    total_direct = sum(direct)
    total_ratio = (total_path / total_direct if total_direct > 1e-9
                   else (float('inf') if total_path > 1e-9 else 1.0))

    # ── Jacobiano por waypoint ─────────────────────────────────────────
    jac = [jacobian_indices(w.positions_rad) for w in waypoints]
    max_cond, max_cond_i = 0.0, 0
    min_sigma, min_sigma_i = float('inf'), 0
    singular = []
    for i, d in enumerate(jac):
        c = d['condition_number']
        if c > max_cond or (math.isinf(c) and not math.isinf(max_cond)):
            max_cond, max_cond_i = c, i
        if d['sigma_min'] < min_sigma:
            min_sigma, min_sigma_i = d['sigma_min'], i
        if d['is_singular']:
            singular.append(i)
            events.append(ConfigurationEvent(
                'singular', 'A5', i, i, 0.0,
                f'waypoint {i} singular: sigma_min={d["sigma_min"]:.3e}, '
                f'rank(J)={d["rank"]}, |a4.a6|={d["a4_a6_alignment"]:.6f}'))

    return TrajectoryAnalysis(
        trajectory_id=trajectory_id,
        waypoint_count=n,
        total_time_s=(waypoints[-1].time_from_start_s if waypoints else 0.0),
        waypoints=waypoints,
        deltas_deg=deltas,
        max_delta_deg=max_delta,
        max_delta_axis=max_axis,
        max_delta_from=max_from,
        max_delta_to=max_from + 1,
        path_length_deg=path,
        direct_distance_deg=direct,
        ratio_per_axis=ratio,
        total_path_deg=total_path,
        total_direct_deg=total_direct,
        total_ratio=total_ratio,
        events=events,
        jacobian=jac,
        max_condition_number=max_cond,
        max_condition_index=max_cond_i,
        min_sigma_min=min_sigma,
        min_sigma_index=min_sigma_i,
        singular_waypoints=singular,
        max_jerk_per_axis=max_jerk,
        rms_jerk_per_axis=rms_jerk,
        jerk_source=jerk_source,
        segment_id=segment_id,
        global_time_offset_s=global_time_offset_s,
        metadata=dict(metadata or {}),
    )


# ─────────────────────────────────────────────────────────────────────────────
# Exportacion a CSV
# ─────────────────────────────────────────────────────────────────────────────
#
# Formato compatible con la seccion 3.3.11 de la propuesta: las columnas
# minimas exigidas (indice de waypoint, tiempo, y posicion de A1..A6) van
# primero, de modo que velocidad, aceleracion y jerk se pueden derivar
# numericamente con el mismo procedimiento que el resto del estudio.
#
# Las columnas de velocidad y aceleracion son las que entrega MoveIt2. Si el
# mensaje no las trae, se escribe 'nan'. NUNCA se escribe un cero inventado.

NAN = float('nan')


def csv_header() -> List[str]:
    """Cabecera del CSV. Las 8 primeras columnas son el minimo de 3.3.11."""
    header = ['trajectory_id', 'waypoint_index', 'time_from_start_s']
    header += [f'{a}_position_deg' for a in AXES]          # minimo 3.3.11
    header += [f'{a}_position_rad' for a in AXES]
    header += [f'{a}_velocity_rad_s' for a in AXES]
    header += [f'{a}_acceleration_rad_s2' for a in AXES]
    header += [f'{a}_jerk_rad_s3' for a in AXES]           # DERIVADO, ver JERK_NOTE
    header += [f'{a}_delta_deg' for a in AXES]             # respecto al previo
    header += [f'{a}_cumulative_deg' for a in AXES]
    header += ['jacobian_sigma_min', 'jacobian_condition_number',
               'jacobian_rank', 'wrist_condition_number', 'a4_a6_alignment',
               'is_singular']
    # ── Metadatos de procedencia (J-D5). Van al FINAL a proposito: las 8
    #    primeras columnas deben seguir siendo exactamente el minimo de 3.3.11.
    header += METADATA_COLUMNS
    return header


def _at(values: Sequence[float], index: int) -> float:
    if values is None or index >= len(values):
        return NAN
    return float(values[index])


def csv_rows(analysis: TrajectoryAnalysis) -> List[List]:
    """Una fila por waypoint, en el orden de la cabecera."""
    rows = []
    cumulative = [0.0] * 6
    for i, wp in enumerate(analysis.waypoints):
        if i == 0:
            delta = [0.0] * 6
        else:
            delta = analysis.deltas_deg[i - 1]
            cumulative = [cumulative[j] + delta[j] for j in range(6)]
        jac = analysis.jacobian[i]
        row: List = [analysis.trajectory_id, i, wp.time_from_start_s]
        row += list(wp.positions_deg)
        row += list(wp.positions_rad)
        row += [_at(wp.velocities_rad_s, j) for j in range(6)]
        row += [_at(wp.accelerations_rad_s2, j) for j in range(6)]
        row += [_at(wp.jerk_rad_s3, j) for j in range(6)]
        row += list(delta)
        row += list(cumulative)
        row += [jac['sigma_min'], jac['condition_number'], jac['rank'],
                jac['wrist_condition_number'], jac['a4_a6_alignment'],
                int(bool(jac['is_singular']))]
        row += [analysis.segment_id,
                wp.time_from_start_s + analysis.global_time_offset_s]
        row += [analysis.metadata.get(k, '') for k in METADATA_COLUMNS[2:]]
        rows.append(row)
    return rows


# ─────────────────────────────────────────────────────────────────────────────
# Resumen legible en consola
# ─────────────────────────────────────────────────────────────────────────────

def _fmt(value: float, width: int = 9, prec: int = 3) -> str:
    if math.isinf(value):
        return f'{"inf":>{width}}'
    if math.isnan(value):
        return f'{"nan":>{width}}'
    return f'{value:>{width}.{prec}f}'


def format_report(analysis: TrajectoryAnalysis, csv_path: str = '') -> str:
    """Bloque de texto que se imprime por terminal para cada trayectoria."""
    a = analysis
    out = []
    out.append('')
    out.append('=' * 78)
    out.append(f' TRAYECTORIA {a.trajectory_id} — CONDICION MOVEIT2 BASE')
    out.append('=' * 78)
    out.append(f'  waypoints           : {a.waypoint_count}')
    out.append(f'  tiempo total        : {a.total_time_s:.3f} s')
    out.append('')
    out.append('  ── Delta articular entre waypoints consecutivos (deg) ──')
    out.append('       eje        max     media    recorrido    directa   '
               'recorrido/directa')
    for j, axis in enumerate(AXES):
        col = [row[j] for row in a.deltas_deg] or [0.0]
        out.append(f'       {axis}  {_fmt(max(col))} {_fmt(sum(col)/len(col))}'
                   f'  {_fmt(a.path_length_deg[j], 10)}'
                   f' {_fmt(a.direct_distance_deg[j], 10)}'
                   f'  {_fmt(a.ratio_per_axis[j], 12)}')
    out.append('')
    out.append(f'  DELTA MAXIMO: {a.max_delta_deg:.3f} deg en {a.max_delta_axis}'
               f' entre los waypoints {a.max_delta_from} y {a.max_delta_to}')
    out.append('')
    out.append('  ── Desplazamiento innecesario ──')
    out.append(f'  recorrido articular total : {a.total_path_deg:.3f} deg')
    out.append(f'  distancia directa total   : {a.total_direct_deg:.3f} deg')
    out.append(f'  RELACION recorrido/directa: {_fmt(a.total_ratio, 8)}'
               '   (1.000 = sin desplazamiento sobrante)')
    out.append('')
    out.append('  ── Proximidad a singularidad (Jacobiano en base_link;'
               ' invariante al tip) ──')
    out.append(f'  sigma_min minimo   : {a.min_sigma_min:.6e}'
               f'  (waypoint {a.min_sigma_index})')
    out.append(f'  cond(J) maximo     : {_fmt(a.max_condition_number, 12, 4)}'
               f'  (waypoint {a.max_condition_index})')
    if a.singular_waypoints:
        out.append(f'  WAYPOINTS SINGULARES: {a.singular_waypoints}')
    else:
        out.append('  waypoints singulares: ninguno')
    out.append('  AVISO: cond(J) de la matriz 6x6 mezcla metros y radianes y '
               'depende de las unidades.')
    out.append('         sigma_min, el rango y cond(J_muñeca) no dependen de '
               'unidades.')
    out.append('')
    if a.events:
        out.append(f'  ── Saltos de configuracion detectados: {len(a.events)} ──')
        for e in a.events:
            if e.kind == 'singular':
                out.append(f'     [{e.kind:<11}] {e.detail}')
            else:
                out.append(f'     [{e.kind:<11}] {e.axis} waypoints '
                           f'{e.from_index}->{e.to_index}: {e.detail}')
    else:
        out.append('  ── Saltos de configuracion detectados: NINGUNO ──')
    if csv_path:
        out.append('')
        out.append(f'  CSV: {csv_path}')
    out.append('=' * 78)
    return '\n'.join(out)
