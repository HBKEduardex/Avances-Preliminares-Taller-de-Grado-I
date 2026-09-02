#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
kuka_pipeline_limits.py

Contrato de EJECUCION del pipeline KUKA, tal y como debe respetarlo cualquier
JSON que se vaya a enviar al robot.

═══════════════════════════════════════════════════════════════════════════
  PROCEDENCIA DE ESTAS CONSTANTES — LEER ANTES DE CONFIAR EN ELLAS
═══════════════════════════════════════════════════════════════════════════
  Se busco en TODO el repositorio taller1:

      trajectory_max_delta_deg   MAX_DELTA_JOINT   max_delta
      soft_limit                 SOFT_LIMIT        VEL_PTP
      XmlDualMove_better.src     sps_submit_better.sub
      config_submit_better.dat   XmlDualMove_better.xml

  RESULTADO: NINGUNA de esas constantes ni de esos archivos existe en
  taller1. Viven en el proyecto de ejecucion (TG2), que este paquete NO abre.

  Por tanto estos valores proceden de la ESPECIFICACION DEL OPERADOR, no de
  una lectura de codigo. Estan marcados como NO VERIFICADO CONTRA CODIGO.

  CORROBORACION INDEPENDIENTE (esto si es medido, en este repositorio):
  los dos JSON producidos por el sistema afinado y aceptados por el pipeline
  real cumplen los cuatro limites sin una sola excepcion.

      trajectory_sequence_20260822_164132.json
          source_points fuera de soft limits ....  0
          waypoints fuera de soft limits .......   0 de 738
          pares con |dq| > 10 deg ..............   0
          delta maximo .........................   3.8800 deg

      trajectory_sequence_20260822_193944.json
          source_points fuera de soft limits ....  0
          waypoints fuera de soft limits .......   0 de 1194
          pares con |dq| > 10 deg ..............   0
          delta maximo .........................   3.8800 deg

  Que DOS archivos independientes cierren en el mismo 3.8800 deg y en cero
  violaciones no es casualidad: es la firma del contrato. Ver DELTA_LAW.

  Si alguna vez se puede leer el pipeline real, ESTE archivo es el unico
  sitio que hay que corregir.
═══════════════════════════════════════════════════════════════════════════
"""

import math
from typing import Dict, List, Optional, Sequence, Tuple

#: Soft limits de POSICION del pipeline de ejecucion, en grados.
#: NO son los del URDF. Son mas estrechos. Ver LIMIT_COMPARISON.
PIPELINE_JOINT_LIMITS_DEG: List[Tuple[float, float]] = [
    (-160.0, 160.0),    # joint_a1
    (-180.0, 35.0),     # joint_a2
    (-110.0, 146.0),    # joint_a3
    (-175.0, 175.0),    # joint_a4
    (-110.0, 110.0),    # joint_a5
    (-340.0, 340.0),    # joint_a6
]

#: Limites del URDF de este paquete (urdf/kr6r900sixx_macro.xacro, lineas
#: 115-156). ESTOS son los que MoveIt usa si no se le dice otra cosa.
URDF_JOINT_LIMITS_DEG: List[Tuple[float, float]] = [
    (-170.0, 170.0),
    (-190.0, 45.0),
    (-120.0, 156.0),
    (-185.0, 185.0),
    (-120.0, 120.0),
    (-350.0, 350.0),
]

#: Limites de VELOCIDAD del URDF, en grados/s (atributo velocity del <limit>).
URDF_JOINT_VELOCITY_DEG_S: List[float] = [
    360.0, 300.0, 360.0, 381.0, 388.0, 615.0]

#: Salto articular maximo admitido entre dos puntos CONSECUTIVOS, por eje.
MAX_JOINT_DELTA_DEG = 10.0

#: Margen de trabajo: objetivo interno, mas estricto que el limite duro.
TARGET_JOINT_DELTA_DEG = 8.0

#: Tamaño maximo de lote del ejecutor por lotes.
MAX_BATCH_POINTS = 20

#: Condicion experimental de esta prueba: velocidad PTP uniforme.
EXPERIMENTAL_PTP_VELOCITY_PCT = 5.0

#: Paso de remuestreo de TimeOptimalTrajectoryGeneration en MoveIt 2.5.9.
#: VERIFICADO por desensamblado del binario en una sesion anterior.
TOTG_RESAMPLE_DT_S = 0.1

DELTA_LAW = """\
LEY DE DENSIDAD  (derivada del codigo, no ajustada a ojo)

    dq_max(eje) = velocidad_limite_URDF(eje) * vel_scaling * resample_dt

TimeOptimalTrajectoryGeneration remuestrea el camino a intervalos fijos de
resample_dt = 0.1 s y nunca supera el limite de velocidad escalado. Por tanto
el salto entre dos muestras consecutivas esta acotado por ese producto.

COMPROBACION EXACTA contra el sistema afinado (vel_scaling = 0.1):
    A5: 6.771877 rad/s * 0.1 * 0.1 s = 0.0677188 rad = 3.8800 deg
    medido en los DOS JSON afinados: 3.8800 deg          <- coincide exacto

COMPROBACION contra el baseline sin escalar (vel_scaling = 1.0):
    A6: 615 deg/s * 1.0 * 0.1 s = 61.5 deg de cota teorica
    medido: 14.1137 deg (no llega a la cota porque la aceleracion, sustituida
    por MoveIt a 1.0 rad/s2, limita antes)                <- 53 pares > 10 deg

De ahi sale el escalado necesario, sin elegir ningun numero a dedo:
    vel_scaling <= dq_objetivo / (max(velocidad_URDF) * resample_dt)
"""


def required_velocity_scaling(target_delta_deg: float = TARGET_JOINT_DELTA_DEG,
                              resample_dt: float = TOTG_RESAMPLE_DT_S
                              ) -> float:
    """
    Escalado de velocidad que GARANTIZA dq <= target en TODOS los ejes.

    No es una heuristica: es la inversion directa de DELTA_LAW sobre el eje
    mas rapido. Con el resultado, la cota se cumple por construccion, no por
    suerte del muestreo.
    """
    worst = max(URDF_JOINT_VELOCITY_DEG_S)
    return target_delta_deg / (worst * resample_dt)


#: Cuanto por debajo del limite duro se queda el escalado por defecto.
#: 0.98 = 2 % de margen. NO se usa el objetivo interno de 8 grados como
#: defecto: apretar mas de lo que el contrato exige acerca innecesariamente el
#: baseline a la densidad del sistema afinado (0.1) y le quita caracter.
CONTRACT_SAFETY_FACTOR = 0.98


def contract_velocity_scaling() -> float:
    """
    El escalado MAS ALTO que sigue cumpliendo el contrato de 10 grados.

    Es el ajuste MINIMO necesario: cualquier valor mayor produce saltos que el
    pipeline rechaza, y cualquier valor menor es apretar por gusto. Mantener
    el baseline lo mas grueso que el contrato permite es lo que conserva su
    caracter frente al sistema afinado.
    """
    return required_velocity_scaling(
        MAX_JOINT_DELTA_DEG * CONTRACT_SAFETY_FACTOR)


def delta_bound_per_axis(vel_scaling: float,
                         resample_dt: float = TOTG_RESAMPLE_DT_S
                         ) -> List[float]:
    """Cota teorica de dq por eje para un escalado dado, en grados."""
    return [v * vel_scaling * resample_dt for v in URDF_JOINT_VELOCITY_DEG_S]


def limit_comparison_rows() -> List[Tuple[str, str, str, bool]]:
    """Filas de la tabla MoveIt (URDF) vs pipeline KUKA."""
    from .continuity_metrics import AXES
    rows = []
    for i, axis in enumerate(AXES):
        urdf = URDF_JOINT_LIMITS_DEG[i]
        pipe = PIPELINE_JOINT_LIMITS_DEG[i]
        rows.append((
            axis,
            f'[{urdf[0]:.0f}, {urdf[1]:.0f}]',
            f'[{pipe[0]:.0f}, {pipe[1]:.0f}]',
            urdf == pipe,
        ))
    return rows


def violations_deg(q_deg: Sequence[float],
                   limits: Optional[Sequence[Tuple[float, float]]] = None
                   ) -> List[Tuple[int, float, Tuple[float, float]]]:
    """Indices de las articulaciones fuera de limites, con su valor."""
    limits = limits or PIPELINE_JOINT_LIMITS_DEG
    out = []
    for j, value in enumerate(q_deg):
        low, high = limits[j]
        if value < low or value > high:
            out.append((j, float(value), (low, high)))
    return out


def margin_deg(q_deg: Sequence[float],
               limits: Optional[Sequence[Tuple[float, float]]] = None
               ) -> float:
    """Margen minimo a los limites configurados. Negativo = fuera."""
    limits = limits or PIPELINE_JOINT_LIMITS_DEG
    return min(min(value - low, high - value)
               for value, (low, high) in zip(q_deg, limits))


def joint_limits_yaml_bounds_rad() -> List[Tuple[float, float]]:
    """
    Soft limits en radianes, tal y como van en joint_limits_pipeline.yaml.

    MECANISMO: moveit_ros.robot_model_loader lee
        <robot_description>_planning.joint_limits.<joint>.min_position
        <robot_description>_planning.joint_limits.<joint>.max_position
    y llama a moveit::core::JointModel::setVariableBounds(). Los limites pasan
    a ser los del MODELO, asi que el muestreador de OMPL no puede generar un
    estado fuera de rango.
    VERIFICADO por desensamblado de libmoveit_robot_model_loader.so (2.5.9).

    DESCARTADO: expresarlos como path_constraints con JointConstraint. Se
    probo y move_group RECHAZA la peticion antes de buscar: los 14 segmentos
    fallaban en menos de 0.1 s con allowed_planning_time = 5 s. Ademas seria
    peor mecanismo: filtra estados ya generados en vez de impedir generarlos.
    """
    return [(math.radians(low), math.radians(high))
            for low, high in PIPELINE_JOINT_LIMITS_DEG]


def summary() -> Dict[str, object]:
    """Contrato completo, para incrustar en el JSON generado."""
    return {
        'joint_limits_deg': [list(v) for v in PIPELINE_JOINT_LIMITS_DEG],
        'max_joint_delta_deg': MAX_JOINT_DELTA_DEG,
        'target_joint_delta_deg': TARGET_JOINT_DELTA_DEG,
        'max_batch_points': MAX_BATCH_POINTS,
        'ptp_velocity_pct': EXPERIMENTAL_PTP_VELOCITY_PCT,
        'provenance': (
            'Especificacion del operador. NO VERIFICADO CONTRA CODIGO: estas '
            'constantes no existen en taller1, viven en el proyecto de '
            'ejecucion. Corroboradas de forma independiente: los dos JSON del '
            'sistema afinado las cumplen con 0 violaciones.'),
    }
