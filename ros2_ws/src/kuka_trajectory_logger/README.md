# kuka_trajectory_logger

Paquete ROS 2 (Humble) que **registra las trayectorias generadas por MoveIt2**
para el robot KUKA KR6 R900.

Es un **observador pasivo**: se suscribe a un tópico que ya existe en el sistema,
no publica nada, no ejecuta nada y no modifica ningún nodo, launch, URDF, SRDF
ni configuración existente. Se puede iniciar y detener en cualquier momento sin
afectar a la GUI, al bridge, a MoveIt2, a RViz2 ni al robot real.

---

## 1. Objetivo

Cuando desde la GUI se ordena un movimiento (por ejemplo `+10°` en una
articulación, una posición articular objetivo, o un desplazamiento cartesiano en
X, Y o Z), **MoveIt2 planifica una trayectoria** desde el estado actual hasta el
objetivo:

```
P0 → P1 → P2 → P3 → ... → Pn
```

Este paquete guarda **esos puntos**, tal y como los produjo el planificador:

- numera cada plan automáticamente (`TRAYECTORIA 1`, `TRAYECTORIA 2`, ...);
- imprime sus puntos en terminal;
- escribe una fila por punto en un archivo CSV con fecha y hora.

> **No interpola, no deriva y no inventa valores.**
> Las velocidades y aceleraciones son las que entrega MoveIt2. Si un campo no
> viene en el mensaje, se guarda `nan`; nunca un cero inventado.

---

## 2. Fuente de datos

| | |
|---|---|
| **Tópico** | `/display_planned_path` |
| **Tipo de mensaje** | `moveit_msgs/msg/DisplayTrajectory` |
| **Nodo publicador** | `move_group` (`moveit_ros_move_group`), desde su `planning_pipeline` |
| **Contenido usado** | `trajectory[i].joint_trajectory` → `trajectory_msgs/msg/JointTrajectory` |
| **Frecuencia** | un mensaje por cada planificación resuelta con éxito |

### ¿Por qué esta interfaz y no otra?

El sistema actual planifica y ejecuta con la acción `/move_action`
(`moveit_msgs/action/MoveGroup`), enviada por `kuka_moveit_bridge_node`.

- La **trayectoria planificada** viaja dentro del *resultado* de esa acción
  (`result.planned_trajectory`) y, más tarde, dentro del *goal* de
  `/joint_trajectory_controller/follow_joint_trajectory`
  (`control_msgs/action/FollowJointTrajectory`).
- En ROS 2, **los goals y los results de una acción se transportan por
  servicios**, no por tópicos. Un tercer nodo **no puede observarlos** sin
  modificar el emisor o el receptor. Los únicos tópicos públicos de una acción
  (`.../_action/status` y `.../_action/feedback`) **no contienen los puntos** de
  la trayectoria.
- `/display_planned_path` **sí** es un tópico y **sí** contiene la trayectoria
  completa. Lo publica `planning_pipeline::PlanningPipeline` dentro de
  `move_group` cada vez que resuelve un plan
  (`displayComputedMotionPlans`, activo por defecto en MoveIt2).
- Ese mensaje se publica **después** de los *planning request adapters*
  configurados en `kuka_kr6_moveit_config/config/ompl_planning.yaml`, cuyo
  primer adapter es `default_planner_request_adapters/AddTimeOptimalParameterization`.
  Por eso el mensaje ya contiene `time_from_start`, `velocities` y
  `accelerations`: **es exactamente la misma trayectoria que después se envía al
  controlador**, no una versión previa sin temporizar.

Es decir: es el punto de intercepción **más fiable y el único realmente pasivo**
para obtener la trayectoria exacta generada por MoveIt2.

### Detección de una trayectoria nueva

La señal de "plan nuevo" es **la llegada de un mensaje nuevo en
`/display_planned_path`**: el pipeline publica una vez por planificación
resuelta (prioridad 1 de las opciones posibles; no se usa ninguna heurística
temporal para separar trayectorias).

Protecciones contra duplicados:

- **Suscripción `VOLATILE`**: si el middleware retuviera una publicación
  anterior, no se entrega al arrancar el logger. Nunca se registra un plan
  antiguo como nuevo.
- **Firma de contenido**: si llega un mensaje con exactamente los mismos joint
  names, los mismos tiempos y las mismas posiciones que el anterior dentro de
  `duplicate_window_sec` (0.5 s por defecto), se descarta como republicación.
  Dos planificaciones reales de RRTConnect nunca producen puntos y tiempos
  idénticos, por lo que este filtro no pierde trayectorias.
- **RViz2 no interfiere**: RViz sólo *se suscribe* a `/display_planned_path`
  para dibujar la trayectoria; no la republica.
- **La ejecución no genera un mensaje nuevo**: el controlador no publica en este
  tópico, así que una trayectoria se registra una sola vez aunque se ejecute.

---

## 3. Funcionamiento (arquitectura real del proyecto)

```
GUI dual  (kuka_bridge_test_gui_node  o  GUI externa TCP/IP)
  │
  │  /kuka_bridge/joint_command_deg      [std_msgs/Float64MultiArray]  (grados)
  │  /kuka_bridge/cartesian_command_deg  [std_msgs/Float64MultiArray]  (m + grados)
  ▼
kuka_moveit_bridge_node        (paquete kuka_gui_moveit_bridge)
  │   · valida límites y convierte grados → radianes
  │   · comandos cartesianos: /compute_ik  [moveit_msgs/srv/GetPositionIK]
  │   · construye JointConstraints + start_state
  ▼
/move_action                   [moveit_msgs/action/MoveGroup]
  ▼
move_group                     (grupo "manipulator", pipeline OMPL / RRTConnect,
  │                             adapter AddTimeOptimalParameterization)
  │
  ├─► /display_planned_path    [moveit_msgs/msg/DisplayTrajectory]   ◄── ESTE PAQUETE
  │        │
  │        ▼
  │   kuka_trajectory_logger_node
  │        │
  │        ├─► Terminal:  TRAYECTORIA n + puntos P0..Pn
  │        └─► CSV:       moveit_trajectories_AAAAMMDD_HHMMSS.csv
  │
  └─► /joint_trajectory_controller/follow_joint_trajectory
           [control_msgs/action/FollowJointTrajectory]
             ▼
      fake_trajectory_controller_node   (paquete kuka_pick_place_demo)
             ▼
      /fake_joint_states → joint_state_publisher (source_list) → /joint_states
             ▼
      robot_state_publisher → TF → RViz2
             ▼
      kuka_moveit_bridge_node lee /joint_states y TF y publica de vuelta:
        /kuka_bridge/status              [std_msgs/String]
        /kuka_bridge/joint_state_deg     [std_msgs/Float64MultiArray]
        /kuka_bridge/cartesian_state_deg [std_msgs/Float64MultiArray]
             ▼
           GUI dual  (sincronización)
```

El logger se conecta **sólo** a la rama marcada `◄── ESTE PAQUETE`.

### Qué movimientos de la GUI generan una trayectoria MoveIt2

| Acción en la GUI | ¿Pasa por MoveIt2? | Interfaz | ¿Se registra? |
|---|---|---|---|
| `+/− N°` en una articulación (A1..A6) y `ENVIAR A1-A6` | **Sí** | `/kuka_bridge/joint_command_deg` → `/move_action` | **Sí** |
| `HOME` / `READY` (tras pulsar ENVIAR) | **Sí** | `/kuka_bridge/joint_command_deg` → `/move_action` | **Sí** |
| `+/− X, Y, Z` y `+/− A, B, C` y `ENVIAR XYZABC` | **Sí** | `/compute_ik` (IK) y después `/move_action` | **Sí** |
| Movimiento con `SOLO PLANIFICAR` (`plan_only=true`) | **Sí** (planifica, no ejecuta) | `/move_action` | **Sí** |
| Comando que devuelve `ALREADY_AT_TARGET` | **No** (el bridge no envía goal) | — | **No** (no existe plan que registrar) |
| Comando con `IK_FAILED`, `GOAL_REJECTED` o `MOVEIT_ERROR` | Sí, pero sin solución | `/move_action` | **No** (sin plan válido no se publica nada) |
| `COPIAR JOINTS` / `COPIAR POSE` / cargar HOME sin enviar | No, sólo rellena campos | — | No |
| Sliders de `joint_state_publisher_gui` (`demo.launch.py` con `use_gui:=true`) | **No**, escriben directamente `/joint_states` | `/joint_states` | **No** |
| Botón *Plan* / *Plan & Execute* del panel MotionPlanning de RViz2 | **Sí** | `/move_action` | **Sí** |
| `pick_place_sequence_node` (paquete `kuka_pick_place_demo`) | **Sí**, un plan por punto de la secuencia | `/move_action` | **Sí** (una TRAYECTORIA por segmento) |

> **Importante sobre los movimientos cartesianos.**
> El bridge **no** usa `computeCartesianPath` / `/compute_cartesian_path`.
> Resuelve la pose con `/compute_ik` y después envía un objetivo **articular**
> a `/move_action`. Por tanto la trayectoria registrada para un `+X` es un plan
> **en espacio articular** hacia la solución IK, no una recta cartesiana
> interpolada. El logger registra lo que MoveIt2 realmente generó.

---

## 4. Datos almacenados

### Unidades originales de ROS 2 / MoveIt2

| Magnitud | Unidad original | Conversión adicional guardada |
|---|---|---|
| Posición articular | `rad` | `deg` |
| Velocidad articular | `rad/s` | `deg/s` |
| Aceleración articular | `rad/s²` | `deg/s²` |
| Esfuerzo (`effort`) | `N·m` (juntas rotacionales) | — |
| Tiempo | `s` | — |

Los valores originales en radianes **siempre** se conservan; las columnas en
grados son una comodidad para el análisis posterior. Todas las juntas del
KR6 R900 son rotacionales, por lo que la conversión rad→deg es válida para
todas ellas.

### Columnas del CSV

Columnas fijas:

| Columna | Significado |
|---|---|
| `trajectory_id` | Número de trayectoria (1, 2, 3, ... dentro de la sesión) |
| `trajectory_received_time_iso` | Fecha y hora local en que se recibió el plan |
| `trajectory_received_time_ros_s` | Mismo instante según el reloj ROS del nodo, en segundos |
| `segment_index` | Índice del segmento dentro del `DisplayTrajectory` (normalmente `0`) |
| `point_index` | Índice del punto dentro de la trayectoria (`0` = P0) |
| `point_count` | Número total de puntos de esa trayectoria |
| `time_from_start_s` | `time_from_start` del punto, tal cual lo entregó MoveIt2 |
| `trajectory_duration_s` | `time_from_start` del último punto (duración planificada) |

Columnas por articulación, **generadas dinámicamente con los joint names reales
recibidos en el mensaje** (para este robot: `joint_a1` … `joint_a6`):

```
joint_a1_position_rad        joint_a1_position_deg
joint_a1_velocity_rad_s      joint_a1_velocity_deg_s
joint_a1_acceleration_rad_s2 joint_a1_acceleration_deg_s2
joint_a2_position_rad        ...
```

- Las columnas de velocidad y aceleración pueden desactivarse con
  `save_velocities` / `save_accelerations`.
- Las columnas `<joint>_effort` sólo aparecen si `save_effort:=true`. MoveIt2
  normalmente **no** rellena `effort` en una trayectoria planificada; si el nodo
  detecta esfuerzos y el parámetro está desactivado, lo avisa por consola.
- Un campo ausente en el mensaje se escribe como `nan`
  (pandas, MATLAB y Excel lo interpretan como dato faltante).

---

## 5. Ejecución

El logger se ejecuta **en una terminal aparte**, con el sistema ya levantado.

```bash
# En el contenedor, con el workspace compilado y sourced:
source /opt/ros/humble/setup.bash
source ~/taller1/ros2_ws/install/setup.bash
```

Opción A — `ros2 run` (valores por defecto):

```bash
ros2 run kuka_trajectory_logger kuka_trajectory_logger_node
```

Opción B — launch del paquete (carga `config/kuka_trajectory_logger.yaml`):

```bash
ros2 launch kuka_trajectory_logger trajectory_logger.launch.py
```

Con argumentos:

```bash
ros2 launch kuka_trajectory_logger trajectory_logger.launch.py \
    output_directory:=/root/taller1/ros2_ws/trajectory_logs \
    print_points:=true
```

Con `ros2 run` los parámetros se pasan así:

```bash
ros2 run kuka_trajectory_logger kuka_trajectory_logger_node --ros-args \
    -p output_directory:=/root/taller1/ros2_ws/trajectory_logs \
    -p print_points:=false
```

### Parámetros

| Parámetro | Tipo | Por defecto | Descripción |
|---|---|---|---|
| `trajectory_topic` | string | `/display_planned_path` | Tópico `DisplayTrajectory` a observar |
| `output_directory` | string | `''` (automático) | Carpeta de los CSV |
| `print_points` | bool | `true` | Imprimir todos los puntos (`false` = sólo la cabecera) |
| `save_velocities` | bool | `true` | Guardar/mostrar velocidades entregadas por MoveIt2 |
| `save_accelerations` | bool | `true` | Guardar/mostrar aceleraciones entregadas por MoveIt2 |
| `save_effort` | bool | `false` | Guardar `effort` (sólo si MoveIt2 realmente lo entrega) |
| `duplicate_window_sec` | double | `0.5` | Ventana para descartar republicaciones idénticas |

---

## 6. Cierre

Para terminar la captura:

```
Ctrl+C
```

El nodo maneja `SIGINT` / `KeyboardInterrupt` / `ExternalShutdownException`,
cierra el CSV correctamente e imprime un resumen de la sesión.

Además, **el CSV se escribe de forma incremental**: después de cada trayectoria
se hace `flush()` + `os.fsync()`. Por eso el archivo nunca queda corrupto ni
incompleto, ni siquiera si el proceso terminara de forma anormal: como máximo
faltaría una trayectoria que aún no había llegado.

---

## 7. Ubicación de los CSV

Nombre del archivo (fecha y hora de arranque del nodo, nunca se sobrescribe):

```
moveit_trajectories_20260818_183500.csv
```

Directorio:

- Si se indica `output_directory`, se usa esa ruta.
- Si se deja vacío (por defecto):
  - `~/taller1/ros2_ws/trajectory_logs/` si ese workspace existe — que es el
    caso dentro del contenedor, donde el repositorio se monta en `/root/taller1`.
    Los CSV quedan así visibles **también desde el host**, en
    `Documents/taller1/ros2_ws/trajectory_logs/`;
  - en cualquier otro caso, `<directorio actual>/trajectory_logs/`.

La ruta exacta se imprime al arrancar el nodo, al crear el CSV y en el resumen
final. La carpeta se crea automáticamente si no existe.

> Si no desea versionar los CSV, añada `ros2_ws/trajectory_logs/` a su
> `.gitignore` (este paquete no modifica archivos existentes).

---

## 8. Ejemplo de salida

Terminal, al enviar `+10°` en A1 desde la GUI:

```
======================================================================
TRAYECTORIA 1
======================================================================

Fuente:             /display_planned_path [moveit_msgs/msg/DisplayTrajectory]
Recibida:           2026-08-18 18:35:12.417
Tiempo ROS:         1755541512.417233 s
Numero de puntos:   12
Duracion total:     3.457 s
Velocidades:        si
Aceleraciones:      si
Esfuerzos:          no (NaN)

Joint names:
  joint_a1
  joint_a2
  joint_a3
  joint_a4
  joint_a5
  joint_a6

Punto 0
t = 0.000 s
  A1   =     0.000000 rad =     0.0000 °   v =     0.000000 rad/s =     0.0000 °/s   a =     0.148000 rad/s² =     8.4798 °/s²
  A2   =    -1.570800 rad =   -90.0000 °   v =     0.000000 rad/s =     0.0000 °/s   a =     0.000000 rad/s² =     0.0000 °/s²
  ...

Punto 1
t = 0.412 s
  A1   =     0.012600 rad =     0.7220 °   v =     0.061000 rad/s =     3.4950 °/s   a =     0.148000 rad/s² =     8.4798 °/s²
  ...

Punto 11 (punto final)
t = 3.457 s
  A1   =     0.174533 rad =    10.0000 °   v =     0.000000 rad/s =     0.0000 °/s   a =    -0.148000 rad/s² =    -8.4798 °/s²
  ...

CSV: /root/taller1/ros2_ws/trajectory_logs/moveit_trajectories_20260818_183500.csv  (+12 filas)
======================================================================
```

Tras varios movimientos y `Ctrl+C`:

```
======================================================================
RESUMEN DE LA SESION
======================================================================

Trayectoria 1       12 puntos    duracion 3.457 s
Trayectoria 2       27 puntos    duracion 7.812 s
Trayectoria 3        8 puntos    duracion 2.104 s

Total: 3 trayectorias, 47 puntos.

CSV: /root/taller1/ros2_ws/trajectory_logs/moveit_trajectories_20260818_183500.csv
======================================================================
```

*(Los valores numéricos del ejemplo son ilustrativos; los reales dependen del
plan que genere RRTConnect en cada caso.)*

---

## 9. Waypoints planificados ≠ muestras del robot real

Esta distinción es fundamental para interpretar los datos:

- Una trayectoria de MoveIt2 contiene **N puntos planificados** (`P0 … Pn`)
  producidos por el planificador (RRTConnect) y temporizados por el adapter
  `AddTimeOptimalParameterization`. Su número depende del plan: puede ser 8, 12,
  27...
- Esos puntos son **la referencia enviada al controlador**, no la lista completa
  de posiciones físicas por las que pasó el robot. El controlador real del KUKA
  (o cualquier `joint_trajectory_controller`) realiza **interpolación interna**
  entre waypoints a su propia frecuencia de ciclo, y el robot añade además su
  propia dinámica y su error de seguimiento.
- Por tanto, `point_count` **no** es "el número de posiciones que recorrió el
  robot", sino "el número de puntos que produjo el planificador".

**Este paquete analiza la trayectoria generada por MoveIt2.** No registra
telemetría del KUKA real; para eso ya existe otro sistema en el proyecto. Si se
quisieran comparar plan y ejecución, habría que cruzar este CSV (plan) con el
registro de telemetría del robot (ejecución real) usando las marcas de tiempo.

---

## 10. Estructura del paquete

```
kuka_trajectory_logger/
├── package.xml
├── setup.py
├── setup.cfg
├── README.md
├── resource/
│   └── kuka_trajectory_logger
├── config/
│   └── kuka_trajectory_logger.yaml     ← parámetros por defecto
├── launch/
│   └── trajectory_logger.launch.py     ← lanza SOLO el logger
├── kuka_trajectory_logger/
│   ├── __init__.py
│   ├── trajectory_logger_node.py       ← nodo (suscripción + CSV + terminal)
│   └── trajectory_record.py            ← lógica pura (extracción, CSV, formato)
└── test/
    └── test_trajectory_record.py       ← tests de la lógica pura (sin ROS)
```

El launch de este paquete **no levanta** MoveIt2, RViz2, el bridge ni el
controlador: esos componentes ya los levanta
`kuka_gui_moveit_bridge/launch/kuka_bridge_system.launch.py`. El logger es un
observador que se conecta a un sistema ya en marcha, por lo que duplicar ese
launch sería innecesario y contrario al diseño pasivo del paquete.

---

## 11. Procedimiento de prueba

```bash
# Terminal 1 — sistema actual (sin cambios)
source /opt/ros/humble/setup.bash
source ~/taller1/ros2_ws/install/setup.bash
ros2 launch kuka_gui_moveit_bridge kuka_bridge_system.launch.py use_test_gui:=true

# Terminal 2 — logger
source /opt/ros/humble/setup.bash
source ~/taller1/ros2_ws/install/setup.bash
ros2 launch kuka_trajectory_logger trajectory_logger.launch.py
```

1. Espere a que la GUI muestre `⬤ LISTO` (estado `READY` del bridge).
2. En la GUI: pulse `+` en A1 hasta `+10°` y luego `ENVIAR A1-A6`.
   → En la Terminal 2 debe aparecer `TRAYECTORIA 1` con sus puntos.
3. En la GUI: pulse `+Z` y luego `ENVIAR XYZABC`.
   → Debe aparecer `TRAYECTORIA 2`.
4. Repita los movimientos que desee (`TRAYECTORIA 3`, `4`, ...).
5. Pulse `Ctrl+C` en la Terminal 2 → aparece el resumen y se cierra el CSV.
6. Verifique el archivo:

```bash
ls -l ~/taller1/ros2_ws/trajectory_logs/
head -3 ~/taller1/ros2_ws/trajectory_logs/moveit_trajectories_*.csv
```

Comprobaciones útiles si algo no aparece:

```bash
# ¿Existe el tópico y quién lo publica?
ros2 topic info /display_planned_path --verbose

# ¿Llega el mensaje al enviar un movimiento desde la GUI?
ros2 topic echo /display_planned_path --no-arr

# Estado del bridge (ALREADY_AT_TARGET no genera plan)
ros2 topic echo /kuka_bridge/status
```

---

## 12. Compilación

```bash
cd ~/taller1/ros2_ws
colcon build --packages-select kuka_trajectory_logger
source install/setup.bash
```

Tests de la lógica pura (opcional, no requieren ROS en ejecución):

```bash
cd ~/taller1/ros2_ws/src/kuka_trajectory_logger
python3 -m pytest test/ -q
```

---

## 13. Dependencias

| Paquete | Uso |
|---|---|
| `rclpy` | Nodo ROS 2 |
| `moveit_msgs` | `DisplayTrajectory` |
| `trajectory_msgs` | `JointTrajectory` / `JointTrajectoryPoint` (contenidos en el mensaje) |
| `builtin_interfaces` | `Duration` (`time_from_start`) |

Todas ya están presentes en el entorno del proyecto (`ros-humble-moveit`).
El CSV se genera con la biblioteca estándar de Python (`csv`): no se instala
nada nuevo.
