# kuka_kr6_moveit_baseline

**Condición «MoveIt2 base» del estudio comparativo de Taller de Grado 2.**

Paquete ROS 2 (Humble) **autocontenido, aditivo y congelado**. No modifica
ningún archivo del repositorio: todo lo que necesita vive dentro de él.

---

## 1. Propósito y rol dentro del estudio

El estudio compara **tres condiciones**:

| # | Condición | Dónde vive |
|---|---|---|
| 1 | **KRL nativo** (referencia del fabricante) | Controlador KUKA |
| 2 | **MoveIt2 base** ← *este paquete* | `kuka_kr6_moveit_baseline` |
| 3 | **MoveIt2 + mejora** | `kuka_kr6_moveit_config` + resto del sistema actual |

Este paquete existe para **evidenciar, de forma reproducible y demostrable**,
los problemas de continuidad articular y de selección de configuración
articular que MoveIt2 produce con el KR6 R900 **sin ninguna** de las mejoras
implementadas después por el operador.

Cumple las tres propiedades exigidas:

| Propiedad | Cómo se cumple |
|---|---|
| **(a) Reproducible** | Todo parámetro está justificado con su fuente o marcado NO VERIFICADO (§2). Metadatos completos en `BASELINE_METADATA.yaml`. |
| **(b) No afinada** | Ningún valor procede de una decisión de ingeniería del operador (§3). |
| **(c) Congelada** | El modelo, las mallas y la configuración son **copias locales** del commit `0c2c022`, no referencias cruzadas (§4). Si la rama de mejora cambia, el baseline no. |

---

## 2. Qué significa «configuración base» aquí

**Base = lo que produciría el MoveIt Setup Assistant** para este URDF, sin
intervención posterior. No «lo que parezca simple».

Las plantillas realmente instaladas en el contenedor son solo nueve
(`moveit_setup_core_plugins` y `moveit_setup_srdf_plugins` **no traen carpeta
`templates/`**), así que varios valores no se pueden respaldar con un archivo
local. La regla aplicada sin excepción: **se escribe el valor, se marca
NO VERIFICADO, y se dice de dónde salió.**

### Tabla de parámetros y sus fuentes

| Parámetro | Valor | Origen | Estado |
|---|---|---|---|
| `planning_plugin` | `ompl_interface/OMPLPlanner` | `/opt/ros/humble/share/moveit_configs_utils/default_configs/ompl_planning.yaml` | ✅ VERIFICADO |
| `request_adapters` | los **6** por defecto | mismo archivo | ✅ VERIFICADO |
| `start_state_max_bounds_error` | `0.1` | mismo archivo | ✅ VERIFICADO |
| `jiggle_fraction` | `0.05` | mismo archivo | ✅ VERIFICADO |
| `planner_configs` | los **24** de la plantilla | `/opt/ros/humble/share/moveit_configs_utils/default_configs/ompl_defaults.yaml` | ✅ VERIFICADO |
| `RRTConnect.range` | `0.0` | `ompl_defaults.yaml` | ✅ VERIFICADO |
| `kinematics_solver` | `kdl_kinematics_plugin/KDLKinematicsPlugin` | `/opt/ros/humble/share/moveit_kinematics/kdl_kinematics_plugin_description.xml` | ✅ VERIFICADO (existe el plugin) |
| Límites de posición y velocidad | del URDF | `urdf/kr6r900sixx_macro.xacro` (copia) | ✅ VERIFICADO |
| `has_acceleration_limits` | `false` | el URDF **no declara** aceleración | ✅ VERIFICADO |
| Plantilla RViz | fork de la del SA | `/opt/ros/humble/share/moveit_setup_app_plugins/templates/config/moveit.rviz` | ✅ VERIFICADO |
| `default_planner_config` | `RRTConnect` | convención documentada del SA | ⚠️ **NO VERIFICADO** |
| `projection_evaluator` | `joints(joint_a1,joint_a2)` | convención del SA (dos primeros joints) | ⚠️ **NO VERIFICADO** |
| `longest_valid_segment_fraction` | `0.005` | documentación pública de MoveIt | ⚠️ **NO VERIFICADO** |
| `kinematics_solver_search_resolution` | `0.005` | documentación pública de MoveIt | ⚠️ **NO VERIFICADO** |
| `kinematics_solver_timeout` | `0.005` | documentación pública de MoveIt | ⚠️ **NO VERIFICADO** |
| `kinematics_solver_attempts` | *no se declara* | obsoleto en 2.5.9, sin fuente local | ⚠️ **NO VERIFICADO** |
| `allowed_planning_time` | `5.0 s` | defecto documentado de `MoveGroupInterface` | ⚠️ **NO VERIFICADO** |
| `num_planning_attempts` | `1` | defecto documentado de `MoveGroupInterface` | ⚠️ **NO VERIFICADO** |
| `max_velocity_scaling_factor` | `1.0` | defecto documentado de MoveIt | ⚠️ **NO VERIFICADO** |
| `max_acceleration_scaling_factor` | `1.0` | defecto documentado de MoveIt | ⚠️ **NO VERIFICADO** |
| `goal_joint_tolerance` | `1e-4 rad` | defecto documentado de `MoveGroupInterface` | ⚠️ **NO VERIFICADO** |
| `goal_position_tolerance` | `1e-4 m` | defecto documentado de `MoveGroupInterface` | ⚠️ **NO VERIFICADO** |
| `goal_orientation_tolerance` | `1e-3 rad` | defecto documentado de `MoveGroupInterface` | ⚠️ **NO VERIFICADO** |
| Matriz de colisiones del SRDF | copiada del SRDF afinado | equivalencia geométrica demostrada (§2.1) | ✅ **VERIFICADO POR EQUIVALENCIA GEOMÉTRICA** |
| Algoritmo de parametrización temporal | TOTG (`AddTimeOptimalParameterization`) | está entre los 6 `request_adapters` por defecto | ✅ VERIFICADO |
| Controlador simulado | **ninguno** | ver §9 | ✅ VERIFICADO (ausencia comprobada) |

### 2.1 Matriz de colisiones: verificada por equivalencia geométrica

La matriz `disable_collisions` del SRDF se **copió literalmente** del SRDF del
sistema afinado. No se marca NO VERIFICADO, y la razón no es la comodidad:

> La matriz de colisiones es una **función determinista de la geometría del
> modelo**. Las mallas del baseline son idénticas byte a byte a las del sistema
> afinado, y **ningún valor geométrico, `xyz` ni `rpy` fue modificado** (las 27
> reescrituras afectan solo a rutas `package://` y `$(find …)`, ver §4). Por
> tanto la matriz que el Setup Assistant generaría para el baseline **es la
> misma que ya existe**. No es un ajuste orientado a las métricas del estudio:
> es geometría.

Comprobaciones que respaldan la equivalencia (auditoría estática, §12):

| # | Comprobación | Resultado |
|---|---|---|
| D.1 | Todos los links citados en la matriz existen en el URDF del baseline | ✅ **11 de 11**, ninguno inventado |
| D.2 | Nº de entradas idéntico al del SRDF afinado, sin añadir/quitar/alterar | ✅ **17 = 17**, listas idénticas **incluido el orden** |
| D.3 | Conjunto de links con geometría de colisión idéntico entre ambos modelos | ✅ **11 = 11**, conjuntos iguales |

Los 4 links del URDF que la matriz **no** menciona (`base`, `flange`, `tool0`,
`world`) **no declaran `<collision>`**, así que el Setup Assistant tampoco
generaría entradas para ellos. La ausencia es coherente, no una omisión.

### Desviaciones del sistema afinado respecto al default instalado

Material directo para el capítulo de resultados:

| # | Elemento | Default instalado (MoveIt 2.5.9) | `kuka_kr6_moveit_config` (afinado) |
|---|---|---|---|
| 1 | `request_adapters` | 6, incluye `ResolveConstraintFrames` | 5, **omite** `ResolveConstraintFrames` |
| 2 | `planner_configs` | 24 planificadores | 2 (`RRTConnectkConfigDefault`, `RRTstarkConfigDefault`) |
| 3 | `start_state_max_bounds_error` | `0.1` | **ausente** |
| 4 | `jiggle_fraction` | `0.05` | **ausente** |

---

## 3. Qué se eliminó respecto al sistema afinado

Inventario del sistema afinado **en `taller1`**, que es el alcance de este
baseline (ver la nota sobre TG2 al final de la tabla).

| Ajuste encontrado | Archivo y línea en el sistema afinado | ¿En el baseline? | Justificación de la exclusión |
|---|---|---|---|
| **HOME con `A5 = 1.57 rad`** | `kuka_kr6_moveit_config/config/kuka_kr6.srdf:19` | **No** | Es una decisión de ingeniería posterior. El baseline conserva el HOME original `A5 = 0`, que resulta ser **singular** (§10). |
| **HOME con `A5 = 1.5708`** en el launch | `kuka_kr6_moveit_config/launch/demo.launch.py:133` | **No** | Mismo motivo. El launch del baseline arranca en `A5 = 0`. |
| `group_state` **`ready`** | `kuka_kr6_moveit_config/config/kuka_kr6.srdf:23` | **No** | Estado definido por el operador, no producido por el Setup Assistant. |
| **`request_adapters` recortados** (falta `ResolveConstraintFrames`) | `kuka_kr6_moveit_config/config/ompl_planning.yaml:2` | **No** | El baseline usa los 6 adaptadores por defecto. |
| **`planner_configs` reducidos a 2** | `kuka_kr6_moveit_config/config/ompl_planning.yaml:3-11` | **No** | El baseline expone los 24 de la plantilla. |
| **`kinematics_solver_timeout: 0.05`** (10× el defecto) | `kuka_kr6_moveit_config/config/kinematics.yaml:4` | **No** | El baseline usa `0.005`. |
| **`velocity_scaling: 0.1` / `acceleration_scaling: 0.1`** | `kuka_gui_moveit_bridge/config/kuka_bridge.yaml:29-30` | **No** | El baseline usa `1.0` (defecto de MoveIt). |
| **`planning_time: 10.0` / `planning_attempts: 10`** | `kuka_gui_moveit_bridge/config/kuka_bridge.yaml:27-28` | **No** | El baseline usa `5.0` y `1` (defectos de MoveIt). |
| **`joint_tolerance_deg: 0.2`** | `kuka_gui_moveit_bridge/config/kuka_bridge.yaml:37` | **No** | El baseline usa la tolerancia por defecto `1e-4 rad`. |
| **Tolerancias `ALREADY_AT_TARGET`** | `kuka_gui_moveit_bridge/config/kuka_bridge.yaml:44-46` | **No** | Lógica de conveniencia del operador; suprime planificaciones. |
| **IK con semilla del estado actual** vía `/compute_ik` | `kuka_gui_moveit_bridge/kuka_moveit_bridge_node.py:657-677` | **No** | Una semilla es un ajuste explícito de la §5 del encargo. El baseline envía restricciones de posición y orientación, que es la vía por defecto. |
| **`end_effector_link: tool0`** forzado en cada IK | `kuka_gui_moveit_bridge/config/kuka_bridge.yaml:8` | **No** | El baseline declara el tip en el SRDF y no lo fuerza por petición. |
| **`plan_only: false`** (el bridge ejecuta) | `kuka_gui_moveit_bridge/config/kuka_bridge.yaml:68` | **No** | El baseline es **solo planificación**, `plan_only` fijo en `true`. |
| **Transformación de TCP / punto efectivo del gripper** | — | **No existe en `taller1`** | **En `taller1` no hubo nada que eliminar**: `flange → tool0` es rotación pura (`xyz="0 0 0"`, `rpy="0 90° 0"`) y no hay ningún TCP declarado. **PERO existe evidencia numérica de que TG2 sí aplica una transformación de TCP de ≈ 149.77 mm**, inferida del campo `cartesian_diagnostic` del JSON de trayectoria. **Su origen NO está verificado**: TG2 queda fuera del alcance de este trabajo. Detalle completo en §10.3. |
| **`joint_limits.yaml` con límites afinados** | `kuka_kr6_moveit_config/config/joint_limits.yaml` | **No** | El baseline deriva los suyos **mecánicamente del URDF**. Ver el hallazgo asociado abajo. |
| **Post-procesado / filtrado / remuestreo de trayectoria** | — | **No existe** | No se encontró ninguno en `taller1`. |
| **Lógica de continuidad o desambiguación de muñeca** | — | **No existe** | No se encontró ninguna en `taller1`. Es precisamente lo que el estudio propone añadir. |

> **Hallazgo sobre el sistema afinado (no se corrige, es información del estudio):**
> `kuka_kr6_moveit_config/config/joint_limits.yaml` **existe pero
> `demo.launch.py` nunca lo carga**: `move_group` no recibe
> `robot_description_planning`. El sistema afinado planifica hoy con los
> límites del URDF y sin límites de aceleración, igual que el baseline.

> **Nota sobre los ajustes del entorno `TG2`.** El workspace `TG2` contiene
> otros ajustes del operador: el límite de ~10° por comando y los rangos
> internos por eje, los perfiles de velocidad PTP, y el bridge EKI. **Se
> consideraron y se descartaron con criterio, no por omisión**: todos actúan
> sobre el canal hacia el controlador KUKA, y el baseline no usa ese canal —
> no ejecuta movimiento físico. No hay nada que neutralizar porque nada de eso
> llega a intervenir en la planificación.

---

## 4. Qué se reutilizó del entorno y desde qué commit

Fork del entorno desde **`taller1` commit `0c2c022`**, árbol limpio.

| Origen | Destino | Copia |
|---|---|---|
| `kuka_kr6_support/urdf/kr6r900sixx.xacro` | `urdf/kr6r900sixx.xacro` | 2 rutas reescritas |
| `kuka_kr6_support/urdf/kr6r900sixx_macro.xacro` | `urdf/kr6r900sixx_macro.xacro` | 16 rutas reescritas |
| `kuka_kr6_support/urdf/environment.xacro` | `urdf/environment.xacro` | 8 rutas reescritas |
| `kuka_resources/urdf/common_materials.xacro` | `urdf/common_materials.xacro` | 1 ruta reescrita |
| `kuka_resources/urdf/common_constants.xacro` | `urdf/common_constants.xacro` | **idéntica** |
| `kuka_resources/urdf/common_colours.xacro` | `urdf/common_colours.xacro` | **idéntica** |
| `kuka_kr6_support/meshes/**` (18 mallas) | `meshes/**` | **idénticas byte a byte** |

**Lo único que se reescribió son 27 referencias de ruta** (`package://` y
`$(find …)`). Verificado con `diff`: en los seis xacro **no hay ni un solo
cambio en valores geométricos**, orígenes, `rpy`, `xyz`, límites de joint ni
masas. La tabla completa de las 27 reescrituras está en el reporte de entrega.

La celda se conserva completa: pedestal (`base_env_link`), mesa
(`mesa_env_link`), **gripper (`gripper_env_link`)** y cubo de 30 mm
(`cubo_env_link`). El gripper sigue siendo **visible y parte del modelo de
colisión**; lo que no existe —ni existía— es una transformación de TCP
**en `taller1`**. Sobre la evidencia de que TG2 sí aplica una, ver §10.3.

---

## 5. Requisitos previos y compilación

Requisitos: contenedor `kuka_ros2_humble_container` (Ubuntu 22.04.5, ROS 2
Humble, MoveIt 2.5.9). No hace falta ningún paquete adicional.

```bash
# Dentro del contenedor
cd /root/taller1/ros2_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select kuka_kr6_moveit_baseline

source install/setup.bash
```

Ningún otro paquete necesita recompilarse: este no depende de ninguno del
repositorio.

---

## 6. Cómo lanzar

**Un solo comando levanta todo.** Los valores por defecto dejan el sistema
listo para la demostración sin pasar ningún argumento.

```bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_LOCALHOST_ONLY=0
ros2 launch kuka_kr6_moveit_baseline baseline.launch.py
```

Levanta: `robot_state_publisher`, `joint_state_publisher` (en el HOME original
`A5=0`), `move_group`, RViz2, la GUI de pruebas y el analizador de
continuidad.

### Argumentos del launch

| Argumento | Defecto | Qué hace |
|---|---|---|
| `use_rviz` | `true` | Abre RViz2 con `rviz/baseline.rviz`. |
| `use_gui` | `true` | Abre la GUI de pruebas del baseline. |
| `use_analyzer` | `true` | Arranca el analizador de continuidad articular. |
| `write_csv` | `true` | El analizador escribe CSV en `analysis_output/`. |
| `output_directory` | `''` | Carpeta de los CSV. Vacío = `analysis_output/` dentro del paquete. |
| `use_sim_time` | `false` | Usar tiempo de simulación. |

**Argumentos del requisito J.** Los tres interruptores arrancan en `false`, de
modo que **el comportamiento por defecto del launch es idéntico al de antes de
añadirlos**:

| Argumento | Defecto | Qué hace |
|---|---|---|
| `analyze_json` | **`false`** | MODO ARCHIVO: analiza una secuencia grabada en JSON en vez de escuchar `/display_planned_path`. Desactiva el analizador en vivo mientras está activo. |
| `replan_json` | **`false`** | Replanifica con el baseline la tarea del JSON (variantes A y B) y genera la tabla comparativa de tres condiciones. |
| `input_json` | `''` | Ruta **absoluta** del JSON. Se abre en **SOLO LECTURA**. Necesario para los dos anteriores. |
| `replan_variant_joint` | `true` | Ejecutar la variante A (metas articulares). |
| `replan_variant_cartesian` | `true` | Ejecutar la variante B (metas cartesianas por FK). |

> ⚠️ El JSON es del **sistema afinado**, no del baseline. Ver la advertencia de
> procedencia del **Escenario 3** antes de interpretar cualquier métrica.

Ejemplos:

```bash
# Solo planificar y observar, sin escribir CSV
ros2 launch kuka_kr6_moveit_baseline baseline.launch.py write_csv:=false

# Sin GUI (enviar objetivos desde el panel MotionPlanning de RViz)
ros2 launch kuka_kr6_moveit_baseline baseline.launch.py use_gui:=false
```

---

## 7. GUÍA DE DEMOSTRACIÓN DEL ERROR DE CONTINUIDAD ARTICULAR

### Preparación

1. Lanza el sistema con el comando único de §6.
2. Espera a que RViz muestre el robot **en el HOME original**: brazo
   extendido hacia +X, muñeca recta. El estado articular de la GUI debe leer
   `A1=0.000  A2=-90.000  A3=90.000  A4=0.000  A5=0.000  A6=0.000`.
3. Comprueba en la terminal del analizador el banner
   `ANALIZADOR DE CONTINUIDAD — CONDICION MOVEIT2 BASE` y el mensaje
   `Esperando planificaciones de MoveIt2...`.
4. En la GUI, el estado cartesiano —rotulado **«de `tool0` en `base_link`»**—
   debe leer `X=525.00  Y=0.00  Z=890.00  A=0.00  B=90.00  C=0.00`.

> **`B = 90.00`, no `0.00`.** El tip declarado es `tool0`, girado +90° en Y
> respecto a `link_6` (§10.0, §10.2). La **posición** es idéntica en los tres
> frames; solo cambia la referencia de orientación.
>
> Esa lectura ya es evidencia: **en el HOME original la orientación de
> `link_6` es exactamente la identidad**, y esa configuración es singular
> (§10.1).

---

### 7.0 PRUEBA OBLIGATORIA — Validar la FK del analizador contra la TF real

**Haz esto antes que cualquier escenario.** Valida que la cinemática
reimplementada en `continuity_metrics.py` (§10.0.1) coincide con la que
`robot_state_publisher` publica realmente. Si esto falla, **todas las métricas
de Jacobiano y singularidad del estudio son inválidas** y no debes continuar.

> ### ⚠️ Por qué NO basta `tf2_echo`
>
> `ros2 run tf2_ros tf2_echo` imprime **tres decimales**, es decir resuelve
> **1 mm**. La tolerancia de aceptación es **1 × 10⁻⁶ m = 0.001 mm**, mil veces
> más fina. **Con `tf2_echo` no se puede verificar esa tolerancia**: sirve para
> descartar un error grosero, no para validar la cinemática.
>
> Por eso el paquete incluye `kr6_baseline_verify_fk`, que lee la TF por la
> API con la precisión completa del `float64` del mensaje.

**Paso 1.** Levanta el sistema (deja esta terminal abierta):

```bash
ros2 launch kuka_kr6_moveit_baseline baseline.launch.py
```

**Paso 2.** En otra terminal, ejecuta la verificación:

```bash
source /root/taller1/ros2_ws/install/setup.bash
ros2 run kuka_kr6_moveit_baseline kr6_baseline_verify_fk
```

El nodo lee `/joint_states`, consulta la TF de `base_link → {link_6, flange,
tool0}` y de `world → …`, calcula la FK del paquete para **esos mismos valores
articulares** y compara. Salida esperada:

```
╔══════════════════════════════════════════════════════════════════════════╗
║  §7.0 VERIFICACION DE LA CINEMATICA CONTRA LA TF REAL                    ║
╚══════════════════════════════════════════════════════════════════════════╝

ESTADO ARTICULAR LEIDO DE /joint_states:
  A1=+0.000000°  A2=-90.000000°  A3=+90.000000°  A4=+0.000000°  A5=+0.000000°  A6=+0.000000°

  A5 = 0 -> HOME BASELINE. Correcto para esta prueba.

referencia  tip        |d posicion| [m]   |d orient| [rad]  veredicto
────────────────────────────────────────────────────────────────────────
base_link   link_6    0.000000000000e+00 0.000000000000e+00  PASA
base_link   flange    0.000000000000e+00 0.000000000000e+00  PASA
base_link   tool0     ...e-16            ...e-16             PASA
world       link_6    ...                ...                 PASA
...

╔══════════════════════════════════════════════════════════════════════════╗
║  RESULTADO: PASA                                                         ║
╚══════════════════════════════════════════════════════════════════════════╝
```

Los residuos reales estarán en el orden de **10⁻¹⁶** (épsilon de máquina), no
en el de la tolerancia: la tolerancia de 10⁻⁶ es un margen amplio.

**Paso 3 (opcional, comprobación visual rápida).** `tf2_echo` sigue sirviendo
para ver de un vistazo que los frames son los esperados:

```bash
ros2 run tf2_ros tf2_echo base_link tool0    # [0.525, 0.000, 0.890]  rpy [0, 1.571, 0]
ros2 run tf2_ros tf2_echo link_6    tool0    # [0.000, 0.000, 0.000]  rpy [0, 1.571, 0]
```

El segundo confirma en runtime la afirmación de §10.0: **traslación nula,
rotación pura de +90° en Y**. Pero **el veredicto lo da el paso 2**, no éste.

#### Qué hacer si falla

**No continúes con ningún escenario.** Un desajuste sólo puede venir de tres
sitios, y hay que descartarlos en este orden:

| # | Causa | Cómo comprobarla | Cómo se ve |
|---|---|---|---|
| 1 | **El URDF cargado no es el del baseline** | `ros2 param get /robot_state_publisher robot_description \| head -c 400` | Debe citar `kuka_kr6_moveit_baseline` en las rutas `package://`. Si cita `kuka_kr6_support`, estás sobre el sistema afinado. |
| 2 | **El robot no está en HOME** | El propio nodo lo avisa: `A5 = … NO es el HOME baseline` | La FK seguiría siendo válida, pero no estarías midiendo la configuración singular del estudio. |
| 3 | **Error real en `continuity_metrics.py`** | Si 1 y 2 están bien y sigue fallando | **Es el caso grave.** Todas las cifras de Jacobiano del README quedan invalidadas. |

**Si es el caso 3, párate y repórtalo con:** la salida completa del nodo, el
estado articular que leyó, y la magnitud del desajuste. La pista más útil es
**qué tips fallan**: si fallan los tres por igual en posición, el error está
en la cadena `CHAIN`; si sólo falla `tool0` en orientación, está en
`FIXED_TIP_ROTATIONS`; si sólo falla `world`, está en `kuka_z`.

**Lo que NO debes hacer:** ajustar `continuity_metrics.py` para que cuadre.
El paquete está congelado y la discrepancia es el dato.

---

### Escenario 1 — Cambio de configuración articular por singularidad de muñeca

**Objetivo a enviar** — panel *OBJETIVO CARTESIANO de `tool0` en `base_link`*:

```
X = 485.0    Y = 0.0    Z = 959.28   (mm)
A = 0.0      B = 30.0   C = 0.0      (deg)
```

> **⚠️ `B = 30.0`, recalculado para el tip declarado `tool0`.** Con `link_6`
> este mismo objetivo sería `B = -60.0`; la diferencia es exactamente los +90°
> del joint `flange-tool0`. Verificado con la FK del paquete. La **posición**
> `X/Y/Z` es idéntica en ambos frames.

**Por qué este objetivo.** Está a solo 80 mm del HOME y corresponde
**exactamente** a la solución articular continua
`[0, -90, 90, 0, -60, 0]`. Pero esa **misma pose** del tip la produce también
`[0, -90, 90, 180, +60, 180]` — verificado numéricamente **en `tool0`**:
diferencia de posición `0.000000000 mm` y de orientación `0.000000000°`
(la equivalencia se mantiene exacta tras el cambio de tip). Son dos
configuraciones articulares distintas para la misma pose, separadas por
**180° en A4, 120° en A5 y 180° en A6**.

El baseline **no tiene ningún criterio** para preferir la solución continua:
parte de una configuración donde `|a4·a6| = 1.000000` (ejes exactamente
alineados) y `rank(J) = 5`, así que el reparto entre A4 y A6 está
indeterminado y KDL devuelve la rama que le toque.

**Qué observar en RViz:** con `Show Trail: true`, el rastro del efector.
Si aparece la inversión, la muñeca gira ~180° mientras el extremo apenas se
desplaza: el rastro se concentra en un punto mientras el robot se retuerce.

**Qué mirar en la GUI:** la barra de estado debe decir
`PLAN OK (cartesiano): N waypoints`.

**Qué esperar del analizador:**

```
DELTA MAXIMO: ~180 deg en A4 (o A6)
[near_180   ] A4 waypoints i->i+1: salto de ~180 deg, proximo a 180 deg (inversion de muñeca)
[near_180   ] A6 waypoints i->i+1: salto de ~180 deg, ...
[singular   ] waypoint 0 singular: sigma_min=7.850e-17, rank(J)=5, |a4.a6|=1.000000
```

El evento `singular` en el waypoint 0 aparecerá **siempre**, porque el
baseline arranca en el HOME singular. Es el hallazgo de §10.

> Si en una ejecución concreta MoveIt devuelve la rama continua, repite el
> envío: la elección no es determinista. Que **no** sea determinista es en sí
> mismo parte del resultado y conviene registrarlo.

---

### Escenario 2 — Recorrido articular innecesario

**Objetivo a enviar** — panel *OBJETIVO ARTICULAR (deg)*:

```
A1 = 120.0   A2 = -60.0   A3 = 60.0   A4 = 0.0   A5 = -60.0   A6 = 0.0
```

**Por qué este objetivo.** Lleva el tip de `[+525, 0, +890] mm` a
`[-356, -617, +898] mm`, es decir **al otro lado de la celda**, obligando al
brazo a rodear la mesa (`mesa_env_link`) y el pedestal (`base_env_link`), que
sí están en el modelo de colisión. La distancia articular directa es
`120 + 30 + 30 + 60 = 240°`, un valor conocido contra el que comparar.

RRTConnect es un planificador **por muestreo y no asintóticamente óptimo**, y
—verificado— **ninguno de los 6 `request_adapters` por defecto acorta,
suaviza ni re-muestrea la trayectoria**: solo hacen parametrización temporal
y corrección del estado inicial. Es decir, el camino que sale del muestreo es
el que se entrega.

**Qué observar en RViz:** el rastro del efector describiendo una curva mucho
más larga que el arco directo.

**Qué esperar del analizador:**

```
recorrido articular total : X deg
distancia directa total   : 240.000 deg
RELACION recorrido/directa: > 1.000
```

Cuanto más supere `1.000`, más desplazamiento sobrante. **Esa relación es la
cuantificación de «desplazamientos que no parecen necesarios»** de la §1.2.1
de la propuesta: convierte una observación cualitativa en un número.

Repite el mismo objetivo varias veces: RRTConnect da un camino distinto cada
vez, así que conviene reportar media y dispersión de la relación, no un valor
único.

---

### Escenario 3 — Trayectoria real grabada: análisis y replanificación

**Este es el escenario que produce la evidencia comparativa de §3.3.10.**

> ## ⚠️ ADVERTENCIA DE PROCEDENCIA — LEER ANTES DE INTERPRETAR NADA
>
> El archivo
> `/home/eduardex/Documents/taller1/trajectories/trajectory_sequence_20260822_193944.json`
> **lo generó el SISTEMA AFINADO en el entorno TG2**, no este paquete.
> Su `planner_metadata` lo demuestra: `velocity_scaling = 0.1`,
> `acceleration_scaling = 0.1`, `planning_attempts = 10`,
> `allowed_planning_time = 10.0 s`, `joint_goal_tolerance_rad = 0.0034906585`
> (= 0.2°). Todos ellos son ajustes que §3 lista como **eliminados** del
> baseline.
>
> **Cualquier métrica calculada sobre esa trayectoria caracteriza la condición
> AFINADA, no la base.** Por eso su CSV, el nombre del archivo y la consola
> llevan `condition = AFINADA/ORIGEN EXTERNO`.
>
> El archivo se abre en **SOLO LECTURA**. El paquete no lo modifica, no lo
> mueve, no lo renombra y no escribe junto a él.

#### Qué contiene el archivo

| | |
|---|---|
| `schema_version` | 1 |
| `source` | `kuka_gui_control` |
| Puntos enseñados | **15** (`P1`…`P15`) |
| Segmentos | **14** (`T1`…`T14`) |
| Waypoints | **199** |
| Duración total | **17.644533 s** |
| Unidades | **`positions_rad` y `positions_deg`, ambos presentes y coherentes a 0.000e+00** |
| Eventos de gripper | 4 (`close` P4, `open` P7, `close` P10, `open` P13) |
| Violaciones de límites del URDF baseline | **0 de 1194 componentes** |

**Unidades — decisión y verificación.** `positions_rad` es la **fuente** (es la
unidad nativa de ROS) y `positions_deg` es la **verificación cruzada**. No hay
conversión implícita en ningún sentido. El lector **ejecuta** la comprobación
sobre los 199 puntos y **aborta ruidosamente** si alguna vez difieren por
encima de `1e-9°`, en vez de elegir en silencio una de las dos
representaciones. El resultado sale por consola en el bloque de integridad.

**Tiempos.** `time_from_start_sec` **reinicia en 0 en cada segmento** — así lo
entrega MoveIt2 y así viene en el archivo. Se conservan **las dos formas**:
`time_from_start_s` (local del segmento, columna 3) y `global_time_s`
(acumulado de la secuencia, en los metadatos). El acumulado se construye
sumando duraciones; **no se interpola ni se reparametriza nada**.

**Velocidades y aceleraciones.** Son **reales** (1026 y 1004 componentes no
nulos de 1194) y se usan **tal cual**. No se recalculan. Si una condición no
las entrega, quedan como `nan`, nunca como ceros inventados.

**`from_point` / `to_point` vienen a `null`** en los 14 segmentos. Se resuelven
**por índice** según el contrato (`Tk` une `Pk` con `Pk+1`), y la resolución
está verificada: la desviación máxima entre los extremos reales y los
`source_points` es de **0.199726°**, que **es exactamente** la tolerancia de
0.2° del sistema afinado, no ruido.

---

#### USO 1 — Analizar la trayectoria grabada

Caracteriza la **condición afinada**. No necesita MoveIt ni el robot.

```bash
ros2 launch kuka_kr6_moveit_baseline baseline.launch.py \
    analyze_json:=true \
    input_json:=/home/eduardex/Documents/taller1/trajectories/trajectory_sequence_20260822_193944.json
```

**Qué observar en RViz.** Nada nuevo: este modo **no publica** trayectorias.
RViz sigue mostrando el robot en HOME. Es un análisis offline.

**Qué esperar del analizador.** Un banner de procedencia en morado con el
`md5`, el bloque de control de integridad (7 comprobaciones), la huella del
sistema afinado, el mapeo de segmentos, los eventos de gripper, y después un
informe por cada uno de los 14 segmentos. Termina con:

```
MODO ARCHIVO COMPLETADO — 14 segmentos, 199 waypoints.
RECORDATORIO: condition = AFINADA/ORIGEN EXTERNO.
```

**Dónde queda el CSV.**
`analysis_output/afinada_externa_continuity_<AAAAMMDD_HHMMSS>_trajectory_sequence_20260822_193944.csv`
— 199 filas × 51 columnas.

---

#### USO 1-bis — VER la trayectoria grabada moviéndose en RViz

**Esto es lo que sirve para la demostración visual.** El USO 1 mide y escribe
el CSV, pero **no publica nada**: no verás movimiento. Para *ver* las
configuraciones articulares, usa el reproductor:

```bash
ros2 launch kuka_kr6_moveit_baseline baseline.launch.py \
    preview_json:=true \
    preview_time_scale:=3.0 \
    input_json:=/root/taller1/trajectories/trajectory_sequence_20260822_193944.json
```

Publica **una sola** `RobotTrajectory` con los 199 waypoints de los 14
segmentos concatenados y un **reloj global continuo**, así que RViz la anima
de principio a fin **sin cortes ni reinicios** entre segmentos.

| Argumento | Defecto | Para qué |
|---|---|---|
| `preview_json` | `false` | Activa el reproductor |
| `preview_time_scale` | `1.0` | `3.0` lo pone 3× más lento. **Solo afecta a la animación**: ni a las posiciones ni al CSV |
| `preview_loop` | `true` | Repite en bucle |
| `preview_segment` | `-1` | `1` reproduce **solo T1**, la maniobra de escape de la singularidad |

**Aislar el segmento crítico:**

```bash
ros2 launch kuka_kr6_moveit_baseline baseline.launch.py \
    preview_json:=true preview_segment:=1 preview_time_scale:=5.0 \
    input_json:=<ruta>
```

**Qué mirar en RViz.** `Displays → MotionPlanning → Planned Path`; la plantilla
ya trae `Show Trail: true`, `Trail Step Size: 1` y `State Display Time: 0.05 s`.
El **rastro del efector** es lo que hace visible el problema: cuando la muñeca
gira sin que el TCP se desplace, el rastro se concentra en un punto mientras el
robot se retuerce.

> **El robot NO se mueve.** No hay controlador, ni cliente de acción, ni ningún
> tópico de ejecución. La animación de RViz es una visualización de una
> trayectoria ya calculada. El modelo «real» permanece en HOME.

**Los valores articulares no se tocan**: no se interpolan, no se remuestrean y
no se suavizan. Se publica exactamente lo que hay en el archivo. Lo único que
se construye es el reloj global, sumando las duraciones de los segmentos
previos al `time_from_start` local de cada punto.

**Comparación visual entre condiciones.** Reproduce primero el JSON grabado
(esto), y después lanza el USO 2 (`replan_json:=true`), que anima los 28 planes
del baseline. Las configuraciones que elige cada uno se ven una detrás de otra
en la misma escena.

#### USO 2 — Replanificar la misma tarea con el baseline

**Esto es lo que sirve para la comparación.** Necesita MoveIt levantado, así
que el launch arranca todo y además el nodo de replanificación:

```bash
ros2 launch kuka_kr6_moveit_baseline baseline.launch.py \
    replan_json:=true \
    input_json:=/home/eduardex/Documents/taller1/trajectories/trajectory_sequence_20260822_193944.json
```

Para una sola variante:

```bash
# solo la variante A (metas articulares)
ros2 launch kuka_kr6_moveit_baseline baseline.launch.py replan_json:=true \
    input_json:=<ruta> replan_variant_cartesian:=false
```

**Metodología aplicada** (decisiones J-D1…J-D5 del operador):

| # | Decisión | Por qué |
|---|---|---|
| **J-D1** | **Por segmento**, los 14 por separado. Nunca un replan global. | Con `P15 ≈ P1` un replan global degeneraría a **no moverse** y la tarea dejaría de existir. Además los eventos de gripper hacen obligatorios los puntos intermedios. |
| **J-D2** | Las metas son los **`source_points` P1…P15**, no los extremos alcanzados. | Los extremos se desvían hasta 0.199726°, y esa desviación **es** la tolerancia de 0.2° del sistema afinado. Heredarla contaminaría el baseline, que alcanza con **1e-4 rad = 0.0057°**. |
| **J-D3** | **Estado inicial encadenado**: T1 desde `P1`; T2…T14 desde el final **real** del plan del baseline anterior. | Replica el `segment_chaining = previous_trajectory_end` del sistema afinado y deja que la deriva sea la propia de cada condición. Se fija con `MotionPlanRequest.start_state`, explícito. |
| **J-D4** | **Ambas variantes, A y B.** | Juntas **separan la causa**: si A ya muestra recorrido excesivo, el problema es el **planificador**; si solo B lo muestra, es la **selección de configuración** (IK). |
| **J-D5** | El CSV lleva `md5` del JSON y `declared_tip`. | Trazabilidad. |

**Las dos variantes:**

| | **BASELINE-A** | **BASELINE-B** |
|---|---|---|
| Meta | los 6 valores articulares de `Pk` | la **pose** de `Pk`, derivada por FK |
| Qué mide | calidad del camino: recorrido, suavidad, proximidad a singularidad | lo anterior **+ selección de configuración articular** |
| Ambigüedad de configuración | ninguna: la meta la fija | **sí, y es el punto**: MoveIt resuelve IK y puede elegir otra rama |
| Evidencia de §1.2.1 | parcial | **completa** |

> **Condición dura sobre la variante B.** La pose objetivo se deriva
> **siempre** de los joints de `Pk` con
> `forward_kinematics_tip(q, 'tool0')` — **nunca** del campo
> `cartesian_diagnostic` del JSON, que arrastra el TCP de TG2 de ≈149.77 mm
> (§10.3). Así el offset queda eliminado **por construcción**.

**Qué observar en RViz.** Aquí **sí** hay actividad: cada segmento
replanificado se publica en `/display_planned_path` y RViz lo anima. Verás
28 planificaciones consecutivas (14 de A + 14 de B). **El robot no se mueve**:
`plan_only = True` es invariante.

**Qué esperar por consola.** Por cada segmento, una línea:

```
T1:   47 waypoints,   2.183 s, desviacion respecto a la meta 0.000412 deg
```

y si alguno falla:

```
T9: NO RESUELTO. Se continua con el siguiente.
```

**Un fallo es un resultado, no un error del paquete** — significa que la
condición base no resuelve una tarea que la afinada sí resolvió. Ver §10.5
sobre el margen de 20.11 mm entre gripper y mesa.

**Dónde quedan los archivos.** Todos en `analysis_output/`:

| Archivo | Contenido |
|---|---|
| `afinada_externa_continuity_<stamp>_<stem>.csv` | condición AFINADA (grabada) |
| `baseline_A_articular_continuity_<stamp>_<stem>.csv` | condición BASELINE-A |
| `baseline_B_cartesiana_continuity_<stamp>_<stem>.csv` | condición BASELINE-B |
| `comparacion_<stamp>.txt` | **la tabla comparativa completa** |

---

#### Cómo interpretar la tabla comparativa

La tabla lleva **tres filas por segmento** (una por condición) más un resumen
agregado, una tabla de delta máximo por eje y otra de recorrido acumulado por
eje. Va **siempre** encabezada por este aviso, que es parte del entregable:

> **COMPARACIÓN INDICATIVA, NO ENSAYO CONTROLADO.** La fila AFINADA es una
> trayectoria **grabada** por el sistema afinado en el entorno TG2 el
> 2026-08-22, con `velocity_scaling=0.1`, `acceleration_scaling=0.1`,
> `planning_attempts=10`, `allowed_planning_time=10.0 s` y tolerancia de meta
> de 0.2°. Las filas BASELINE son planes **generados ahora** por la condición
> base con sus propios defectos y tolerancia de 0.0057°. Difieren el entorno,
> la fecha, el planificador efectivo y la tolerancia. **No corrieron bajo el
> mismo procedimiento.**

**Cómo leerla:**

| Columna | Qué significa | Qué buscar |
|---|---|---|
| `wp` | número de waypoints | más waypoints para el mismo tramo = camino más troceado |
| `dmax[deg]` / `eje` | mayor salto articular entre waypoints consecutivos, y en qué eje | saltos grandes en A4/A6 sugieren cambio de configuración |
| `saltos` | eventos detectados (`near_180`, `sign_change`, `a5_abrupt`, `singular`) | **> 0 en BASELINE-B y 0 en BASELINE-A ⇒ el problema es la IK** |
| `recorrido` / `directa` / `ratio` | grados recorridos vs. distancia articular directa | `ratio` alto = desplazamiento sobrante |
| `condJ max` / `@wp` | peor número de condición del Jacobiano y dónde | valores enormes = paso cerca de singularidad |

**La lectura decisiva, y el motivo de correr las dos variantes:**

- **A y B ambas malas** → el problema está en el **planificador** (RRTConnect
  no es asintóticamente óptimo y ningún adaptador por defecto suaviza).
- **A bien, B mal** → el problema está en la **selección de configuración
  articular** por IK, que es exactamente §1.2.1.
- **A y B parecidas a la afinada** → el ajuste del operador no explica la
  diferencia y hay que buscarla en otro sitio.

#### ▶ Caso de interés: el segmento T1

**Éste es el ejemplo a presentar.** `T1` lleva de `P1` a `P2`, es decir del
HOME singular (`A5 ≈ 0`) a `A5 ≈ 90`. En la trayectoria grabada:

| | |
|---|---|
| Waypoints | **32** — el segmento **más largo de los 14** |
| Duración | **3.002417 s** — también la mayor |
| `cond(J)` en el waypoint 0 | **98 348.6** — el máximo de los 199 waypoints |
| σ_min en el waypoint 0 | 1.904 × 10⁻⁵ |
| \|a4·a6\| en el waypoint 0 | **0.999999998** |
| Δ máximo de A5 | 3.88°/waypoint |

> **El segmento que más tiempo y más puntos consume no hace trabajo útil: es
> la maniobra de escape de la singularidad del HOME.** Ningún segmento que
> realmente mueve una pieza (T3, T4, T6, T7…) pasa de 17 waypoints ni de
> 1.6 s.

---

## 8. Cómo interpretar el CSV

Un archivo por trayectoria, en `analysis_output/`:

```
baseline_continuity_YYYYMMDD_HHMMSS_trajNNN.csv
```

45 columnas. Las **8 primeras** son el mínimo exigido por la §3.3.11 de la
propuesta, para que velocidad, aceleración y jerk se deriven con el mismo
procedimiento que el resto del estudio:

| Columna | Contenido |
|---|---|
| `trajectory_id` | Número de trayectoria dentro de la sesión |
| `waypoint_index` | Índice del waypoint (0…N-1) |
| `time_from_start_s` | Tiempo del waypoint, en segundos |
| `A1_position_deg` … `A6_position_deg` | **Posición articular en grados** |
| `A1_position_rad` … `A6_position_rad` | Lo mismo en radianes (dato original) |
| `A1_velocity_rad_s` … | Velocidad **entregada por MoveIt2**; `nan` si no viene |
| `A1_acceleration_rad_s2` … | Aceleración entregada por MoveIt2; `nan` si no viene |
| `A1_delta_deg` … | Salto respecto al waypoint anterior |
| `A1_cumulative_deg` … | Recorrido acumulado hasta ese waypoint |
| `jacobian_sigma_min` | Menor valor singular del Jacobiano |
| `jacobian_condition_number` | Número de condición (`inf` si singular) |
| `jacobian_rank` | Rango del Jacobiano (6 = no singular) |
| `wrist_condition_number` | Condición del bloque rotacional 3×3 |
| `a4_a6_alignment` | \|a4·a6\|; **1.0 = ejes alineados = singularidad de muñeca** |
| `is_singular` | 1 si `sigma_min < 1e-6` |

### Métricas derivables

| Métrica | Cómo |
|---|---|
| Velocidad articular | Derivada numérica de `*_position_rad` sobre `time_from_start_s` |
| Aceleración y **jerk** | Segunda y tercera derivada, mismo procedimiento de §3.3.11 |
| Discontinuidad | `max(*_delta_deg)` y los eventos del resumen |
| Desplazamiento innecesario | `sum(*_cumulative_deg)` final ÷ distancia directa |
| Proximidad a singularidad | `min(jacobian_sigma_min)`, `max(jacobian_condition_number)` |

> **Aviso metodológico para el panel:** `jacobian_condition_number` proviene
> de una matriz 6×6 que **mezcla metros y radianes**, así que su valor
> absoluto depende de las unidades y no debe citarse aislado. Los indicadores
> que **no** dependen de unidades son `jacobian_sigma_min`, `jacobian_rank` y
> `wrist_condition_number` (adimensional). Los tres se exportan.

---

## 8.9 TABLA DE COMPARABILIDAD DE MÉTRICAS

> **Ésta es la tabla que decide qué se puede afirmar con cada número.** El
> aviso de cabecera de la tabla comparativa dice que las condiciones difieren;
> esta tabla dice **qué métricas quedan invalidadas por esas diferencias**.
> Sin ella, la comparación es una lista de números sin criterio.

### Las diferencias que causan todo

| Parámetro | AFINADA (grabada) | BASELINE | Factor |
|---|---:|---:|---:|
| `max_velocity_scaling_factor` | **0.1** | **1.0** | **10×** |
| `max_acceleration_scaling_factor` | 0.1 | 1.0 | *(irrelevante, ver abajo)* |
| Límite de velocidad efectivo (A1) | 0.628 rad/s | 6.283 rad/s | **10.01×** |
| **Límite de aceleración efectivo** | 10.0 × 0.1 = **1.0 rad/s²** | defecto interno = **1.0 rad/s²** | **1.00×, idénticos** |
| `joint_goal_tolerance` | 0.2° | 0.0057° | 35× más estricto |
| `resample_dt` de TOTG | 0.1 s | 0.1 s | **idéntico** (constante interna) |

### La tabla

| Métrica | ¿Comparable? | Qué la afecta | Qué se puede afirmar con ella |
|---|:---:|---|---|
| **Número de waypoints** | **NO** | TOTG remuestrea a `resample_dt = 0.1 s` **fijo**, así que `wp ≈ t/0.1`. Es un **proxy del tiempo**, no de la complejidad del camino, y hereda íntegro el problema del escalado de velocidad. **Verificado empíricamente: 171 de 185 intervalos de la trayectoria grabada miden exactamente 0.100000 s**; los 14 restantes son el resto final de cada segmento. | Sólo el coste de almacenamiento y la densidad de muestreo **dentro de una condición**. Nunca «el baseline trocea más el camino». |
| **Tiempo total `t[s]`** | **NO** | El escalado de velocidad 0.1 vs 1.0. **El factor no es lineal ni constante**: en tramos limitados por velocidad `T ∝ 1/v` (hasta 10×), pero en tramos cortos limitados por aceleración `T ∝ 1/√a` y, como el límite de aceleración efectivo es **idéntico** (1.0 rad/s²), esos tramos **no cambian nada**. El factor real varía entre **1× y 10×** según el segmento. | La duración **de esa condición con esos parámetros**. Es un dato descriptivo válido por fila. Leerla en horizontal produce conclusiones falsas. |
| **Delta articular máximo por eje** | **SÍ CON RESERVA** | Depende de la densidad de muestreo: con menos waypoints los saltos individuales son mayores aunque el camino sea idéntico. **Verificado: al diezmar la trayectoria de referencia a 1/2 y 1/4, `dmax` crece ×1.99 y ×3.58** — casi exactamente el factor de diezmado. | Detectar **inversiones de muñeca** (saltos de ~180°), que son de una magnitud completamente distinta al muestreo. **No** para comparar suavidad fina entre condiciones con densidades distintas. |
| **Recorrido acumulado por eje** | **SÍ** | **Invarianza verificada: 0.0000 % de variación** al diezmar a 1/2 y a 1/4, eje por eje y en total. Ver la demostración abajo. | Comparación directa. **Es la métrica más sólida de la tabla** para «¿cuánto se mueve de más el baseline?». |
| **Distancia directa** | **SÍ** | Sólo depende de los extremos de cada segmento, que son los mismos `Pk` en las tres condiciones (J-D2). Invariante por construcción. | Comparación directa. |
| **Relación recorrido/directa** | **SÍ** | Cociente de dos invariantes. **Verificado: 0.0000 % de variación.** | Comparación directa. Es la métrica adimensional de eficiencia del camino. |
| **Saltos de configuración** | **SÍ** | Los umbrales del detector son geométricos (90°, banda de 30° alrededor de 180°, 45° para A5) y muy superiores a cualquier efecto de muestreo. | Comparación directa. **Es la evidencia central de §1.2.1.** |
| **`cond(J)`, σ_min, rank** | **SÍ** | Se evalúan **por waypoint**, no entre waypoints: no dependen del muestreo. Y son **invariantes al tip declarado** (`\|ΔJ\| = 0.000e+00`, §10.0). | Comparación directa de proximidad a singularidad. Con la reserva de unidades: citar σ_min y `cond(J_muñeca)`, que son adimensionales, junto a `cond(J)`. |
| **Velocidad `A*_velocity_rad_s`** | **NO** | Escalada directamente por `max_velocity_scaling_factor`. La condición afinada está limitada a la décima parte. | El perfil de velocidad **de esa condición**. Para comparar habría que normalizar por el escalado, y aun así los tramos limitados por aceleración no escalan igual. |
| **Aceleración `A*_acceleration_rad_s2`** | **SÍ CON RESERVA** | Sorprendentemente, **el límite efectivo es el mismo en ambas condiciones (1.0 rad/s²)**, ver HALLAZGO 2. Pero el *perfil temporal* sí difiere, porque la duración difiere. | El **grado de saturación** (qué fracción del tiempo se está en `\|a\| = 1.0`) sí es comparable. El perfil instantáneo no. |
| **Jerk máximo y RMS** | **NO** | **Derivado, no entregado por MoveIt.** TOTG produce aceleración *bang-bang*: jerk real cero dentro de cada tramo e infinito en las conmutaciones. Con `dt = 0.1 s` y diferencia centrada, la cota del artefacto es `2×1.0/(2×0.1) = 10.0 rad/s³`, **y se alcanza exactamente**: el máximo medido sobre la trayectoria grabada es `10.0000` en A1 y A5. El número lo fija la rejilla de muestreo, no el movimiento. | El **número y la magnitud relativa de las conmutaciones** entre condiciones, ya que ambas comparten `resample_dt`. **Nunca** como propiedad física ni contra un límite de jerk del datasheet de KUKA. |

### P.2 — Demostración numérica de la invarianza a la densidad

Diezmado por índice conservando siempre el primer y el último waypoint de
cada segmento:

| | Original | 1/2 densidad | 1/4 densidad |
|---|---:|---:|---:|
| Waypoints | **199** | **110** | **66** |

**Recorrido acumulado [°] — invariante exacto:**

| Eje | Original | 1/2 | 1/4 | Dif. rel. 1/2 | Dif. rel. 1/4 |
|---|---:|---:|---:|---:|---:|
| A1 | 112.076285 | 112.076285 | 112.076285 | **0.0000 %** | **0.0000 %** |
| A2 | 127.415793 | 127.415793 | 127.415793 | **0.0000 %** | **0.0000 %** |
| A3 | 68.828070 | 68.828070 | 68.828070 | **0.0000 %** | **0.0000 %** |
| A4 | 1.754090 | 1.754090 | 1.754090 | **0.0000 %** | **0.0000 %** |
| A5 | 270.929468 | 270.929468 | 270.929468 | **0.0000 %** | **0.0000 %** |
| A6 | 68.054307 | 68.054307 | 68.054307 | **0.0000 %** | **0.0000 %** |
| **TOTAL** | **649.058012** | **649.058012** | **649.058012** | **0.0000 %** | **0.0000 %** |

**Relación recorrido/directa:** `1.000000` en los tres muestreos, `0.0000 %`.

**Delta máximo por eje [°] — NO invariante, crece con el diezmado:**

| Eje | Original | 1/2 | 1/4 | Factor 1/2 | Factor 1/4 |
|---|---:|---:|---:|---:|---:|
| A1 | 3.600000 | 7.177020 | 12.906598 | **1.994×** | **3.585×** |
| A2 | 3.000000 | 5.924432 | 10.486793 | 1.975× | 3.496× |
| A3 | 2.200954 | 4.362459 | 7.887890 | 1.982× | 3.584× |
| A4 | 0.061566 | 0.106365 | 0.145351 | 1.728× | 2.361× |
| A5 | 3.880000 | 7.760000 | 15.520000 | **2.000×** | **4.000×** |
| A6 | 3.562153 | 7.060462 | 12.766226 | 1.982× | 3.584× |

#### Por qué el recorrido es invariante aquí, y cuándo dejaría de serlo

**El razonamiento.** `Σ|Δq|` es la longitud **exacta** del camino
lineal-a-trozos que pasa por esos waypoints. Diezmar produce la longitud
exacta de un camino **distinto**, más grueso. Los dos coinciden **si y sólo si
ningún eje invierte el sentido entre dos muestras conservadas**: en un tramo
monótono, `|q_c − q_a| = |q_b − q_a| + |q_c − q_b|`, y la suma telescópica es
exacta.

**Comprobado en los datos:** de los **84 pares eje-segmento** (6 ejes × 14
segmentos) de la trayectoria de referencia, **0 presentan inversión de
sentido**. Por eso el recorrido coincide con la distancia directa eje a eje y
la razón vale exactamente 1.000000.

**Contraejemplo que fija el límite** — un eje que va y vuelve
(`A1: 0 → 30 → 0`, 21 waypoints):

| Muestreo | Recorrido A1 | % del real |
|---|---:|---:|
| 21 wp | 60.0000° | 100.00 % |
| 11 wp | 60.0000° | 100.00 % |
| 6 wp | 57.0634° | 95.11 % |
| **2 wp (sólo extremos)** | **0.0000°** | **0.00 %** |

> **Regla operativa.** El recorrido acumulado es **plenamente comparable
> mientras los caminos sean monótonos por eje dentro de cada segmento**, que
> es el caso de la trayectoria de referencia. Si un plan del baseline
> introdujera inversiones —y eso es precisamente lo que el estudio busca
> detectar— el recorrido medido sería un **límite inferior** del real, nunca
> una sobreestimación. **El sesgo, si aparece, juega en contra de la
> hipótesis del estudio, no a favor.** Eso lo hace defendible ante el panel.
>
> El analizador reporta las inversiones como eventos `sign_change`, así que
> el caso queda detectado y no pasa silencioso.

### Cómo aparece esto en la tabla comparativa

La tabla que genera el paquete marca las columnas en su propia cabecera, no
sólo en el aviso:

```
seg   condicion       wp!    t[s]! dmax[deg]~  eje  saltos  recorrido  ...  jerkMax!
```

| Marca | Significado |
|---|---|
| **`!`** | **NO COMPARABLE.** Valor descriptivo de su fila. Leer en horizontal produce conclusiones falsas. |
| **`~`** | **COMPARABLE CON RESERVA.** Depende de la densidad de muestreo. |
| *(sin marca)* | **COMPARABLE.** Invarianza verificada numéricamente. |

---

## 9. Qué NO hace este paquete

- **No se conecta al robot**: no hay sockets, ni EKI, ni TCP/IP, ni ningún
  tópico del bridge. Ni una sola línea de código puede alcanzar el
  controlador.
- **No ejecuta movimiento físico ni simulado**: `plan_only` es `true` fijo y
  no es configurable. No hay cliente de `/execute_trajectory` ni de ningún
  `follow_joint_trajectory`.
- **No sustituye al sistema de comunicación**: la ejecución pertenece al
  entorno `TG2`.
- **No arranca ningún controlador.** Esto **no es una carencia técnica: es el
  alcance declarado del proyecto.** El límite 4 de la §1.4.1 de la propuesta
  dice que «la validación preliminar se limitará a la visualización y
  evaluación cinemática de movimientos mediante ROS2, RViz2 y MoveIt2».
  A ello se suma que **`ros2_control` no está instalado en el contenedor**
  (comprobado: no existen `controller_manager`, `ros2_control`,
  `hardware_interface`, `mock_components`, `joint_trajectory_controller` ni
  `joint_state_broadcaster`), por lo que la sección de controladores del
  Setup Assistant **no es replicable aquí**. Por eso **no se escribe ningún
  `moveit_controllers.yaml`**: inventar uno sería inventar un default.
- **No escribe fuera del paquete**: los CSV van a `analysis_output/`.
- **No modifica nada del repositorio.**

---

## 10. Limitaciones y puntos no verificados

### 10.0 TABLA FK CONSOLIDADA — referencia única del paquete

**Esta tabla es la única referencia de cinemática directa del paquete.**
Cualquier cifra de FK que aparezca en otro sitio y la contradiga es errónea.

> ### ▶ TIP DECLARADO: **`tool0`**
> `config/kuka_kr6_baseline.srdf` → `<chain base_link="base_link" tip_link="tool0"/>`
> Las filas de `tool0` en la tabla son **las que gobiernan** los objetivos
> cartesianos de la GUI y de la variante B. Las de `link_6` y `flange` se
> conservan como referencia comparativa y porque son las que resuelve el SRDF
> **afinado**, que define el grupo por lista de joints.

Unidades: **posición en mm, RPY fijo `xyz` en grados** (convención URDF).
`world → base_link` es una traslación pura de **+500 mm en Z**
(`kuka_z = 0.5` en `urdf/environment.xacro`).

| Configuración | Frame | Referencia | X | Y | Z | R | P | Y |
|---|---|---|---:|---:|---:|---:|---:|---:|
| **Cero** `[0,0,0,0,0,0]` | `link_6` | `base_link` | 980.000 | 0.000 | 435.000 | 0.000 | 0.000 | 0.000 |
| | `link_6` | `world` | 980.000 | 0.000 | 935.000 | 0.000 | 0.000 | 0.000 |
| | `flange` | `base_link` | 980.000 | 0.000 | 435.000 | 0.000 | 0.000 | 0.000 |
| | `flange` | `world` | 980.000 | 0.000 | 935.000 | 0.000 | 0.000 | 0.000 |
| | **`tool0`** ◀ *declarado* | `base_link` | 980.000 | 0.000 | 435.000 | 0.000 | **90.000** | 0.000 |
| | `tool0` | `world` | 980.000 | 0.000 | 935.000 | 0.000 | **90.000** | 0.000 |
| **HOME BASELINE** `[0,-90,90,0,0,0]` | `link_6` | `base_link` | **525.000** | 0.000 | **890.000** | 0.000 | 0.000 | 0.000 |
| | `link_6` | `world` | 525.000 | 0.000 | 1390.000 | 0.000 | 0.000 | 0.000 |
| | `flange` | `base_link` | 525.000 | 0.000 | 890.000 | 0.000 | 0.000 | 0.000 |
| | `flange` | `world` | 525.000 | 0.000 | 1390.000 | 0.000 | 0.000 | 0.000 |
| | **`tool0`** ◀ *declarado* | `base_link` | 525.000 | 0.000 | 890.000 | 0.000 | **90.000** | 0.000 |
| | `tool0` | `world` | 525.000 | 0.000 | 1390.000 | 0.000 | **90.000** | 0.000 |
| **HOME AFINADO** `[0,-90,90,0,90,0]` | `link_6` | `base_link` | **445.000** | 0.000 | **810.000** | 0.000 | 90.000 | 0.000 |
| | `link_6` | `world` | 445.000 | 0.000 | 1310.000 | 0.000 | 90.000 | 0.000 |
| | `flange` | `base_link` | 445.000 | 0.000 | 810.000 | 0.000 | 90.000 | 0.000 |
| | `flange` | `world` | 445.000 | 0.000 | 1310.000 | 0.000 | 90.000 | 0.000 |
| | **`tool0`** ◀ *declarado* | `base_link` | 445.000 | 0.000 | 810.000 | **180.000** | 0.000 | **180.000** |
| | `tool0` | `world` | 445.000 | 0.000 | 1310.000 | **180.000** | 0.000 | **180.000** |

**Tres hechos que se leen directamente de la tabla:**

1. **`link_6`, `flange` y `tool0` comparten origen exactamente.** Los joints
   `joint_a6-flange` (`xyz="0 0 0" rpy="0 0 0"`) y `flange-tool0`
   (`xyz="0 0 0" rpy="0 ${radians(90)} 0"`) son **traslación nula**. La
   posición es idéntica en los tres; solo cambia la orientación.
2. **`tool0` = `link_6` girado +90° en Y.** Es la única diferencia entre los
   tres frames, y es una convención de referencia, no geometría.
3. En el **HOME BASELINE** la orientación de `link_6` es **exactamente
   `R=P=Y=0`** (identidad). En el **HOME AFINADO** es `P=+90°`.

### 10.0.1 Validación de la cinemática reimplementada

`continuity_metrics.py` reimplementa la cadena en Python para no depender de
KDL. Esa reimplementación **está validada**, no asumida:

| Prueba | Método | Resultado |
|---|---|---|
| **Validación cruzada** | FK del paquete vs. composición independiente de matrices 4×4 (`scipy.spatial.transform`) sobre transformadas **parseadas del propio xacro**, en 7 configuraciones | peor caso **2.5 × 10⁻¹³ mm** en posición y **2.0 × 10⁻¹⁴ °** en orientación |
| **Consistencia Jacobiano/FK** | Jacobiano analítico vs. diferencias finitas centradas (`h = 10⁻⁷`), 7 configuraciones | error máx. **6.8 × 10⁻¹⁰ m/rad** (lineal) y **2.8 × 10⁻⁹ rad/rad** (angular) |

Las 7 configuraciones incluyen la **cero**, el **HOME baseline**, el **HOME
afinado**, tres arbitrarias y una en los **límites extremos** de los seis ejes.

**Convenciones declaradas explícitamente:**

| Aspecto | Convención usada |
|---|---|
| Ejes de rotación | Tomados del atributo **`<axis xyz=…>` del URDF**, tal cual, **sin normalizar el signo**: `a1=(0,0,-1)`, `a2=a3=a5=(0,1,0)`, `a4=a6=(-1,0,0)` |
| Signo de rotación | **Regla de la mano derecha sobre el eje declarado**, idéntico al URDF. Los ejes negativos producen giro invertido, que es exactamente lo que declara el modelo |
| Rotación de joint | **Rodrigues** sobre el eje unitario (equivalente a `Rot.from_rotvec`) |
| `<origin rpy>` | **RPY fijo** (extrínseco `xyz`), compuesto como **Rz(yaw)·Ry(pitch)·Rx(roll)** |
| Orden de composición | Por joint: **`Trans(xyz) · RPY(rpy) · Rot(axis, q)`**; la cadena se compone por la **derecha** desde `base_link` |
| Frame resultante | El del **link hijo**, es decir el origen del joint tras aplicar su rotación — que es la definición del URDF |
| Jacobiano | **Geométrico, expresado en `base_link`**, punto de referencia en el origen del tip: `Jv_i = a_i × (p_tip − o_i)`, `Jw_i = a_i` |

### 10.2 Nota sobre el tip declarado y el recuerdo del operador

El grupo declara **`tool0`** (`<chain base_link="base_link" tip_link="tool0"/>`).
Eso cambia **solo** la referencia de orientación de los objetivos cartesianos:

> **Al teclear objetivos cartesianos, `A/B/C` se refieren a `tool0`, que está
> girado +90° en Y respecto a `link_6`. En el HOME baseline el tip muestra
> `A/B/C = [0, 90, 0]`, NO `[0, 0, 0]`.**

La GUI lo rotula explícitamente (los paneles dicen «de `tool0` en `base_link`»)
y lo avisa en el registro al arrancar.

**Consecuencia sobre el recuerdo del operador — que no cambia la decisión.**
Con `link_6` la orientación en HOME baseline era exactamente `[0, 0, 0]`, lo
que encajaba de forma sospechosa con lo recordado. **Con `tool0` deja de
encajar.** Ambas cosas quedan registradas para que la verificación en pantalla
sepa contra qué compara:

| Frame | Posición en HOME baseline [mm] | `A/B/C` [°] | ¿Encaja con «X y Z en cero»? |
|---|---|---|---|
| `link_6` / `flange` | `[525.00, 0.00, 890.00]` | `[0, 0, 0]` | posición **no**; orientación **sí, exacta** |
| **`tool0`** (declarado) | `[525.00, 0.00, 890.00]` | `[0, 90, 0]` | posición **no**; orientación **no** |

Esto **no es un argumento para no cambiar el tip**: el tip se eligió por
fidelidad al Setup Assistant y porque la cinemática es idéntica (§10.0). El
recuerdo sigue **NO CONFIRMADO** y se resuelve mirando la pantalla real.

### 10.3 TCP de TG2 — INFERIDO, NO VERIFICADO

**Hallazgo con trazabilidad completa.** El campo `cartesian_diagnostic` del
JSON de trayectoria del sistema afinado **no corresponde a ningún frame del
URDF de `taller1`**.

**Valor.** Para `P1`, el archivo declara:

```
X = 674.677551   Y = -2.331047   Z = 885.201538
A = -1e-06       B = -0.008562   C = 0.004937
```

**Cómo se obtuvo.** Se calculó la FK de este paquete para **exactamente esos
valores articulares** (`[0, -90, 90, -0.004938, -0.003916, 1e-06]°`) y se
comparó contra todos los frames candidatos:

| Frame | X [mm] | Y [mm] | Z [mm] | Δ X vs. diagnóstico |
|---|---:|---:|---:|---:|
| `link_6` | 525.0000 | −0.0000 | 890.0055 | **+149.6776** |
| `flange` | 525.0000 | −0.0000 | 890.0055 | **+149.6776** |
| `tool0` | 525.0000 | −0.0000 | 890.0055 | **+149.6776** |
| `gripper_env_link` | 525.0000 | −0.0000 | 890.0055 | +149.6776 |
| `cubo_env_link` | 544.9827 | −15.0017 | 910.0215 | +129.6948 |

**El offset completo:**

> **`[+149.678, −2.331, −4.804]` mm — módulo `149.773` mm**

**Por qué en HOME se lee directamente.** En el HOME baseline la orientación
del flange es **exactamente la identidad** (`R = P = Y = 0`, §10.0). Con
`R = I`, un offset expresado en el frame de la herramienta y otro expresado en
`base_link` **coinciden numéricamente**, así que la resta de coordenadas ya es
el vector de offset, sin cambio de base. En cualquier otra configuración
habría que rotarlo primero.

**Qué descarta y qué no.**

| | |
|---|---|
| **Descarta** | Que el diagnóstico corresponda a `link_6`, `flange`, `tool0`, `gripper_env_link` o `cubo_env_link` del URDF de `taller1`. Ninguno cuadra. |
| **Descarta** | Que sea un error de unidades o de convención: los tres frames candidatos coinciden entre sí a 0.0000 mm, y el desajuste es un desplazamiento limpio, no un factor de escala ni un giro. |
| **Sugiere** | Un TCP de herramienta. La malla `gripper.stl` mide **144.0 mm** en su eje longitudinal (AABB medido del STL), y los comentarios de `urdf/environment.xacro` sitúan la zona de agarre a 118 mm y las puntas a 144 mm del flange. **149.77 mm cae justo por delante de las puntas.** |
| **NO prueba** | De dónde sale exactamente. Podría ser un `$TOOL` del controlador KUKA, una transformada del entorno de TG2, o una constante de su GUI. |
| **NO se investigó** | TG2 está **fuera del alcance** por decisión del operador. |

**Estado: ⚠️ INFERIDO, NO VERIFICADO.**

**Prohibición operativa (#10.4), aplicada en el código.** El campo
`cartesian_diagnostic` **no se usa jamás** como objetivo, como referencia ni
para validar nada. La prohibición está escrita como comentario de bloque en
[`trajectory_json_reader.py`](kuka_kr6_moveit_baseline/trajectory_json_reader.py)
y en [`baseline_replan_node.py`](kuka_kr6_moveit_baseline/baseline_replan_node.py),
no solo aquí. El lector lo expone únicamente como
`cartesian_diagnostic_raw`, y **ninguna función del paquete lo consume**.
Cuando la variante B necesita una pose cartesiana de `Pk`, la **deriva** con
`forward_kinematics_tip(q, 'tool0')`. Eso elimina el offset por construcción.

### 10.4 HALLAZGO 3 — El sistema afinado tiene DOS HOME, y usa el singular

Los datos recogidos durante la auditoría revelan que **el sistema afinado no
tiene un HOME: tiene dos**, y el que gobierna la operación real **no** es el
que declara su configuración de MoveIt.

| # | Ubicación | Valor de A5 | Qué gobierna |
|---|---|---|---|
| 1 | `kuka_kr6_moveit_config/config/kuka_kr6.srdf:19` | **1.57 rad** (89.954°) | `group_state` de MoveIt |
| 2 | `kuka_kr6_moveit_config/launch/demo.launch.py:133` | **1.5708** (90.000°) | posición inicial al arrancar |
| 3 | `kuka_gui_moveit_bridge/config/kuka_test_gui.yaml:32` | **0.0** | botón HOME de la GUI |
| 4 | *default* del nodo `kuka_bridge_test_gui_node` | **0.0** | idem, sin YAML |
| 5 | `pick_place_points.yaml`, punto `home` (captura manual) | **0.0** | punto enseñado real |
| 6 | **JSON grabado por el sistema afinado, `P1` y `P15`** | **≈ 0** (−0.003916°) | **la tarea que se ejecutó** |

**La configuración de MoveIt apunta a uno y la operación real usa el otro.**

**Conclusión, explícita:**

> **La trayectoria de referencia del sistema afinado arranca y termina en la
> configuración de singularidad de muñeca.** `P1` y `P15` son
> `[0, −90, 90, −0.004938, −0.003916, 1e-06]°`, es decir el HOME con `A5 ≈ 0`.
> Medido sobre ese punto: `|a4·a6| = 0.999999998`, `σ_min = 1.904 × 10⁻⁵`,
> **`cond(J) = 98 348.6`** — el máximo de los 199 waypoints de toda la
> secuencia.
>
> Esto **refuerza** el hallazgo de singularidad del baseline en vez de
> contradecirlo: la condición singular no es un artefacto de haber congelado
> mal el baseline, es la configuración desde la que **realmente se operó**.

**Detalle adicional — 1.57 vs 1.5708.** Las dos ubicaciones que sí usan el
HOME afinado **tampoco coinciden entre sí**: `1.57` rad = 89.954°, `1.5708`
rad = 90.000°. La diferencia es de **0.046°**. Es irrelevante para la
cinemática, pero indica que el valor se tecleó dos veces por separado en vez
de compartirse desde una única fuente.

**Ninguno de esos archivos se corrige.** Son del sistema afinado y son datos
del estudio.

### 10.5 LIMITACIÓN DEL MODELO — El cubo viaja siempre con la pinza

En el URDF, `cubo_env_link` cuelga de `gripper_env_link`, que cuelga de
`flange`. **El cubo está rígidamente unido a la pinza en todas las
configuraciones.**

En la tarea real es un doble pick-and-place: el gripper cierra en `P4`, abre
en `P7`, cierra en `P10` y abre en `P13`. El cubo está **sobre la mesa** antes
de `P4` y después de `P7`. **En el modelo, siempre está en la mano.**

Esto **afecta a ambas condiciones por igual**: el baseline y el sistema
afinado comparten exactamente el mismo URDF, así que la simplificación no
introduce sesgo entre ellas. **El URDF no se modifica para «arreglarlo».**

**Verificación del riesgo de colisión (L.1–L.3):**

**L.1 — Qué deshabilita la matriz.** De los 55 pares posibles entre los 11
links con geometría de colisión, 17 están deshabilitados. Para el cubo:

| Par | Estado | `reason` |
|---|---|---|
| `cubo_env_link` ↔ `gripper_env_link` | ✅ **deshabilitado** | `Adjacent` |
| `cubo_env_link` ↔ `link_6` | ✅ **deshabilitado** | `Never` |
| `cubo_env_link` ↔ `link_5` | ✅ **deshabilitado** | `Never` |
| `cubo_env_link` ↔ `link_4`, `link_3`, `link_2`, `link_1`, `base_link` | ⚠️ **ACTIVO** | — |
| `cubo_env_link` ↔ `mesa_env_link`, `base_env_link` | ⚠️ **ACTIVO** | — |

Es decir: el cubo **no** puede chocar con la pinza ni con la muñeca, pero
**sí** con la mesa, el pedestal y el brazo bajo.

**L.2 — ¿Choca en algún punto?** Comprobación geométrica estática mediante
**AABB envolvente** (cotas de las mallas STL transformadas por la FK de este
paquete). La AABB **sobreestima** el volumen, así que **ausencia de solape es
prueba concluyente de ausencia de colisión**:

| Alcance | Solapes | Separación mínima |
|---|---|---|
| **Los 15 puntos enseñados `P1`…`P15`** | **0 de 15** | **104.02 mm** (cubo↔mesa, en `P4`) |
| **Los 199 waypoints de la trayectoria grabada** | **0 de 199** | ver abajo |

Separaciones mínimas sobre los 199 waypoints:

| Par | Separación mínima | Dónde |
|---|---:|---|
| `cubo` ↔ `base_env_link` | 395.56 mm | T9 wp10 |
| `cubo` ↔ `mesa_env_link` | 105.34 mm | T9 wp10 |
| `gripper` ↔ `base_env_link` | 310.34 mm | T9 wp10 |
| **`gripper` ↔ `mesa_env_link`** | **20.11 mm** | **T9 wp10** |

**L.3 — ¿Puede hacer fallar la replanificación?** **No en los extremos de los
segmentos**, que es lo que el baseline recibe como metas: los 15 puntos están
libres con holgura. **Pero el margen no es amplio en todo el recorrido:** el
gripper pasa a **20.11 mm** de la mesa en T9. Los waypoints *intermedios* de
un plan del baseline **no tienen por qué coincidir** con los de la trayectoria
grabada, así que podrían entrar en ese margen y hacer que un segmento —
plausiblemente T9, T10 o T11, que son los que bajan más — quede sin solución.

> **Consecuencia práctica:** si algún segmento falla, el nodo lo reporta como
> `— SEGMENTO NO RESUELTO —` en la tabla comparativa y **continúa con el
> siguiente**. No aborta la corrida. Un fallo aquí es **un resultado del
> estudio**, no un error del paquete: significa que la condición base no
> resuelve una tarea que la condición afinada sí resolvió.

### 10.6 El gripper está fuera de alcance

**El baseline no controla el gripper y no debe hacerlo.** La replanificación
reproduce únicamente la **geometría del movimiento**.

Los cuatro eventos del JSON (`close` en `P4`, `open` en `P7`, `close` en
`P10`, `open` en `P13`, con `initial_state: open`) **se leen, se reportan como
metadato de la tarea y no se actúan**. El analizador los imprime en el bloque
de procedencia.

**La comparación es de trayectorias articulares, no de ejecución de la tarea
completa.** No existe ninguna dependencia hacia
`kuka_pick_place_interfaces` ni ningún cliente de acción del gripper.

### HALLAZGO 1 — El HOME de la condición base es una singularidad de muñeca

El HOME original del robot es `A1=0, A2=-90, A3=90, A4=0, A5=0, A6=0`
(FK en §10.0). Verificado con el Jacobiano geométrico construido desde el URDF
y **validado** según §10.0.1:

| Indicador | HOME baseline (`A5=0`) | HOME afinado (`A5=90`) |
|---|---|---|
| Ángulo entre ejes A4–A6 | **0.000000°** (colineales) | 90.000000° |
| \|a4·a6\| | **1.000000000** | 0.000000000 |
| `rank(J)` | **5** (pérdida de 1 GDL) | 6 |
| σ_min | **7.85 × 10⁻¹⁷** | 0.238396 |
| `cond(J)` | **∞** | 7.592734 |
| `cond(J_muñeca)` | **∞** (rango 2 de 3) | **1.000** (isotrópico) |

**La condición MoveIt2 base parte de una configuración articular singular**, lo
que constituye una **causa candidata documentada** de las configuraciones
articulares poco convenientes y de las variaciones bruscas en velocidades
articulares que la propuesta identifica en §1.2.1, y se relaciona
directamente con la métrica de proximidad a singularidades definida en
§3.3.11.

> **El resultado NO depende del tip declarado.** Se calculó el Jacobiano con
> tip `link_6`, `flange` y `tool0`: como los tres comparten origen y solo se
> diferencian por una rotación, **`|J − J(link_6)|_max = 0.000 × 10⁰` exacto**
> en los tres casos. `rank`, `σ_min` y `cond(J)` son idénticos. Cambiar el tip
> del grupo **no invalidaría ninguna cifra de esta tabla**.

El HOME **no se corrige**: es precisamente la condición que el estudio debe
evidenciar.

### HALLAZGO 2 — El límite de aceleración del baseline es una constante oculta de MoveIt

> **⚠️ Este hallazgo CORRIGE la versión anterior de este apartado.** Antes decía
> que «el baseline opera sin límites de aceleración». **Es falso.** La
> verificación contra el binario de MoveIt 2.5.9 demuestra que TOTG *sí*
> aplica un límite: uno **interno, no documentado y no configurable**.

El URDF **no declara ningún límite de aceleración** (los seis joints traen
solo `effort="0"` y `velocity`). Derivando mecánicamente, `joint_limits.yaml`
queda con `has_acceleration_limits: false` en los seis ejes. Hasta ahí, el
hecho es correcto.

**Lo que pasa después no es que no haya límite.** Desensamblado de
`libmoveit_trajectory_processing.so.2.5.9`, función
`TimeOptimalTrajectoryGeneration::computeTimeStamps`:

```asm
22a2f:  movsd  0x8(%rsp),%xmm2      ; xmm2 = DEFECTO
22a49:  cmpb   $0x0,0x28(%rbx)      ; ¿velocity_bounded_ ?
22a4d:  movsd  %xmm2,(%rax)         ; max_velocity[j] = DEFECTO   ← SIEMPRE
22a51:  jne    22958                ;   si bounded → sobrescribe (y ×scaling)

2298e:  movsd  0x8(%rsp),%xmm3      ; xmm3 = DEFECTO
229a0:  cmpb   $0x0,0x40(%rbx)      ; ¿acceleration_bounded_ ?
229a4:  movsd  %xmm3,(%rax)         ; max_acceleration[j] = DEFECTO ← SIEMPRE
229a8:  je     22e00                ;   si NO bounded → SE QUEDA EL DEFECTO
229ae:  movsd  0x38(%rbx),%xmm1     ;   si bounded: max_acceleration_
229b7:  comisd → ja 23380           ;     si <0 → "Invalid max_acceleration"
229c1..229c6: fabs, minsd, mulsd 0x60(%rsp)   ;  ×acceleration_scaling
```

`0x8(%rsp)` se escribe **exactamente cuatro veces** en toda la función, las
cuatro con la constante de `.rodata` en `0x2a510`, cuyo valor leído del
binario es **`1.0`**.

**Tres consecuencias, todas verificadas:**

| # | Consecuencia |
|---|---|
| 1 | **TOTG no falla.** No existe en la biblioteca ningún mensaje de error por límite de aceleración ausente. El único mensaje «*Joint acceleration limits are not defined. Using the default*» pertenece a `ruckig_traj_smoothing.cpp`, y **Ruckig NO está en los 6 `request_adapters` por defecto**. |
| 2 | **El defecto es `1.0 rad/s²` para los seis ejes**, idéntico para A1 y para A6 pese a que sus capacidades mecánicas difieren en un factor 3 en velocidad. Es un número arbitrario respecto al robot. |
| 3 | **El factor de escalado NO se aplica al defecto.** El `mulsd` está *dentro* de la rama `bounded`. Poner `max_acceleration_scaling_factor` a 0.5 no cambia nada si el límite no está declarado. |

**Y el resultado numérico, que es lo relevante para el estudio:**

| | Declara | `max_acceleration` | `scaling` | **Límite EFECTIVO** |
|---|---|---:|---:|---:|
| **AFINADO** | sí | 10.0 | 0.1 | **1.0 rad/s²** |
| **BASELINE** | **no** | *(defecto interno)* 1.0 | *no se aplica* | **1.0 rad/s²** |

> **Las dos condiciones acaban planificando con el MISMO límite de aceleración
> efectivo, 1.0 rad/s², por dos caminos completamente distintos.** El sistema
> afinado llega ahí declarando 10.0 y escalando a 0.1; el baseline, por una
> constante interna de MoveIt que ni declara ni puede configurar.

**Confirmación empírica** en la trayectoria grabada del sistema afinado:
**153 de 1194** componentes de aceleración valen **exactamente 1.000000000**
— saturación *bang-bang* contra ese límite.

**Reformulación del hallazgo para el estudio.** El problema de la condición
base **no es la ausencia de límite**, sino que:

1. el límite es **invisible**: no aparece en ningún archivo de configuración;
2. es **arbitrario respecto al robot**: 1.0 rad/s² para los seis ejes;
3. es **inmune al escalado**, así que el operador no puede ajustarlo sin
   declarar primero límites reales;
4. y **coincide por casualidad** con el del sistema afinado, lo que significa
   que cualquier diferencia de jerk observada entre las dos condiciones
   **no** procede del límite de aceleración.

No se compensa ni se corrige: es el resultado.

### Puntos NO VERIFICADOS — lista completa y recuento

**Recuento: 13 parámetros de configuración + 4 puntos abiertos = 17 entradas.**

> **Corrección de recuentos anteriores.** En turnos previos dije «12» y luego
> «11». Ambos eran incorrectos: el «12» agrupaba las tres tolerancias de
> objetivo como una sola entrada, y el «11» restaba de ese 12 la matriz de
> colisiones. **La tabla de §2 tiene 13 filas marcadas NO VERIFICADO**, que es
> el número que vale. Contadas una por una a continuación.

#### A. Parámetros de configuración (13) — tabla de §2

| # | Parámetro | Valor en el baseline | Origen declarado |
|---|---|---|---|
| 1 | `default_planner_config` | `RRTConnect` | convención documentada del SA |
| 2 | `projection_evaluator` | `joints(joint_a1,joint_a2)` | convención del SA (dos primeros joints) |
| 3 | `longest_valid_segment_fraction` | `0.005` | documentación pública de MoveIt |
| 4 | `kinematics_solver_search_resolution` | `0.005` | documentación pública de MoveIt |
| 5 | `kinematics_solver_timeout` | `0.005` | documentación pública de MoveIt |
| 6 | `kinematics_solver_attempts` | *no se declara* | obsoleto en 2.5.9, sin fuente local |
| 7 | `allowed_planning_time` | `5.0 s` | defecto documentado de `MoveGroupInterface` |
| 8 | `num_planning_attempts` | `1` | defecto documentado de `MoveGroupInterface` |
| 9 | `max_velocity_scaling_factor` | `1.0` | defecto documentado de MoveIt |
| 10 | `max_acceleration_scaling_factor` | `1.0` | defecto documentado de MoveIt |
| 11 | `goal_joint_tolerance` | `1e-4 rad` | defecto documentado de `MoveGroupInterface` |
| 12 | `goal_position_tolerance` | `1e-4 m` | defecto documentado de `MoveGroupInterface` |
| 13 | `goal_orientation_tolerance` | `1e-3 rad` | defecto documentado de `MoveGroupInterface` |

**Causa común de los 13:** no existe plantilla local del Setup Assistant para
`kinematics.yaml`, `joint_limits.yaml` ni la sección de grupo de
`ompl_planning.yaml`. Se comprobó archivo por archivo en
`/opt/ros/humble/share/moveit_setup_*/templates/`.

#### B. Puntos abiertos (4)

| # | Punto | Estado | Cómo se cierra |
|---|---|---|---|
| 14 | **TCP de TG2 ≈ 149.77 mm** (§10.3) | ⚠️ **INFERIDO, NO VERIFICADO** | Lo verifica el operador. TG2 está fuera de alcance. **No bloquea nada**: la prohibición #10.4 impide que el valor se use. |
| 15 | **Validación en runtime de la cinemática** | ⚠️ **PENDIENTE** | Procedimiento **§7.0**, lo ejecuta el operador. La validación **estática** ya está cerrada (§10.0.1). |
| 16 | **Recuerdo del operador sobre X y Z en cero** | ⚠️ **NO CONFIRMADO** | Se mira la pantalla real. Ver la tabla de §10.2: con `tool0` **deja de encajar**; con `link_6` la orientación encajaba exactamente. |
| 17 | **Colisión real de los puntos del JSON** | ⚠️ **NO VERIFICABLE offline** | Requiere FCL en runtime. La comprobación AABB de §10.5 da **0 solapes en los 15 puntos y en los 199 waypoints**, y la AABB sobreestima, así que la ausencia de solape **sí** es concluyente para esos puntos. Lo que no cubre son los waypoints *intermedios* de un plan nuevo. |

#### C. Resueltos en revisiones posteriores — ya NO son puntos abiertos

| Punto | Estado final |
|---|---|
| **Matriz de colisiones del SRDF** | ✅ **VERIFICADO POR EQUIVALENCIA GEOMÉTRICA** (§2.1). Comprobaciones D.1–D.3 superadas: 11/11 links existen, 17 = 17 entradas idénticas incluido el orden, 11 = 11 links con geometría de colisión. |
| **Definición del grupo / tip declarado** | ✅ **RESUELTO**: `<chain base_link="base_link" tip_link="tool0"/>`. Se demostró que el Jacobiano es **bit a bit idéntico** (`\|ΔJ\| = 0.000e+00`) para `link_6`, `flange` y `tool0`, luego el cambio no altera ninguna métrica. |
| **Unidades del JSON externo** | ✅ **RESUELTO POR EVIDENCIA**: `positions_rad` y `positions_deg` ambos presentes y coherentes a `0.000e+00°` sobre los 199 puntos. El lector **ejecuta** la comprobación y aborta si alguna vez fallara. |
| **Cadena cinemática replicada en Python** | ✅ **VALIDADA ESTÁTICAMENTE** (§10.0.1): `2.5e-13 mm` frente a una composición independiente y `6.8e-10` de consistencia Jacobiano/FK. Queda solo la validación en runtime, que es el punto 15. |

#### D. Estado de la evidencia de singularidad

> ## ✅ SINGULARIDAD: **CONFIRMADA ESTÁTICAMENTE**
> — pendiente únicamente de la prueba en runtime **§7.0**, que ejecuta el operador.
>
> **Qué está cerrado:** validación cruzada de la FK contra una implementación
> independiente (peor caso `2.483e-13 mm`), consistencia Jacobiano/FK por
> diferencias finitas (`6.755e-10 m/rad`), convenciones documentadas una por
> una (§10.0.1), e **invariancia demostrada respecto al tip declarado**
> (`\|ΔJ\| = 0.000e+00` exacto en `link_6`, `flange` y `tool0`).
>
> **Qué falta:** comparar la FK contra la TF que publica `robot_state_publisher`
> con el sistema levantado. **Ese es el único paso que queda**, y hasta
> hacerlo la evidencia es estática, no empírica.

#### E. Alcance de la validación

**No se ha compilado ni lanzado nada.** Toda la verificación de este paquete
es **estática y numérica**: lectura de archivos, parseo de XML/YAML, cálculo
en Python y `lint`. Ninguna cifra de este README procede de una ejecución de
ROS 2.

---

## 11. Estructura del paquete

```text
kuka_kr6_moveit_baseline/
├── .gitignore                           ignora analysis_output/* salvo .gitkeep
├── package.xml                     # ament_python, sin dependencias del repo
├── setup.py / setup.cfg
├── README.md                       # este archivo
├── BASELINE_METADATA.yaml          # reproducibilidad (§12 del encargo)
├── config/
│   ├── kuka_kr6_baseline.srdf      # grupo por cadena a tool0, HOME A5=0
│   ├── kinematics.yaml             # KDL, defectos documentados
│   ├── ompl_planning.yaml          # default instalado, íntegro
│   └── joint_limits.yaml           # derivado del URDF, sin aceleración
├── launch/
│   └── baseline.launch.py          # LANZAMIENTO ÚNICO
├── rviz/
│   └── baseline.rviz               # fork de la plantilla del Setup Assistant
├── urdf/                           # copia congelada del modelo y la celda
├── meshes/                         # 18 mallas, idénticas byte a byte
├── analysis_output/                # CSV del analizador
└── kuka_kr6_moveit_baseline/
    ├── continuity_metrics.py       # métricas + FK + Jacobiano (sin ROS)
    ├── sequence_analysis.py        # secuencias multisegmento + tabla comparativa
    ├── trajectory_json_reader.py   # lector de JSON externo (sin ROS, propio)
    ├── continuity_analyzer_node.py # modo tópico + modo archivo
    ├── baseline_replan_node.py     # replanificación variantes A y B
    ├── verify_fk_node.py           # §7.0 FK vs TF real, precisión completa
    ├── json_preview_node.py        # reproduce en RViz una secuencia grabada
    └── baseline_test_gui_node.py   # fork de la GUI de pruebas
```
