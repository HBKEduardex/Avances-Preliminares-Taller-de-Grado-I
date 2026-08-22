# kuka_moveit_trajectory_planner

Paquete ROS 2 (Humble) que **genera trayectorias con MoveIt2** a partir de una
lista de configuraciones articulares reales del KUKA KR6 R900, y que permite
**previsualizarlas en RViz2**.

Es un paquete **aislado**: no modifica ningún nodo, launch, URDF, SRDF, RViz ni
configuración existente del workspace. Se puede iniciar y detener en cualquier
momento sin afectar a MoveIt2, a RViz2, al bridge ni a nada más.

> [!IMPORTANT]
> Este contenedor **SOLO GENERA Y PREVISUALIZA**.
> - **No ejecuta** trayectorias: `plan_only` es siempre `true` y no hay
>   parámetro para desactivarlo.
> - **No controla el robot real**: no publica en `/joint_states`,
>   `/fake_joint_states`, `/kuka_bridge/*` ni en ningún `follow_joint_trajectory`.
>   No usa EnableMove, ni TCP/IP, ni EKI, ni el Submit Interpreter.
> - **No guarda archivos**: el resultado se publica en un tópico. **El archivo
>   final se guarda en el otro entorno**, donde están las GUIs y el TCP/IP.

---

## 1. Objetivo

La GUI externa captura del KUKA (por TCP/IP + Submit Interpreter) una lista de
configuraciones articulares reales, **en grados**:

```
P1 = [A1, A2, A3, A4, A5, A6]
P2 = [...]
P3 = [...]
...
PN = [...]
```

Este paquete convierte esa lista en **una trayectoria MoveIt2 por transición**:

```
T1 = P1 -> P2
T2 = P2 -> P3
T3 = P3 -> P4
...
T(N-1) = P(N-1) -> PN
```

Cada transición se mantiene como un **SEGMENTO INDEPENDIENTE**. Los segmentos
**no se unen** en una trayectoria nueva optimizada, **no se re-muestrean**, **no
se simplifican** y **no se suavizan**. Se guarda **exactamente** la trayectoria
base que MoveIt2 produce hoy con el pipeline ya configurado en el proyecto
(OMPL / RRTConnect), sin ningún "sistema de mejora".

---

## 2. Arquitectura

```
┌──────────────────────────────┐                    ┌───────────────────────────────┐
│   OTRO ENTORNO               │                    │  ESTE CONTENEDOR (ROS2/MoveIt2)│
│   GUI + TCP/IP + KRL/EKI     │                    │                                │
│                              │                    │                                │
│  captura P1..PN del KUKA     │                    │                                │
│                              │   request_json     │  ┌──────────────────────────┐  │
│  publica la secuencia  ──────┼───────────────────►│  │ kuka_trajectory_         │  │
│                              │                    │  │ generator_node           │  │
│                              │                    │  │  - deg -> rad            │  │
│                              │                    │  │  - start_state explícito │  │
│                              │                    │  │  - objetivo articular    │  │
│                              │                    │  │  - plan_only = TRUE      │  │
│                              │                    │  └────────────┬─────────────┘  │
│                              │                    │               │ /move_action   │
│                              │                    │               ▼                │
│                              │                    │        ┌─────────────┐         │
│                              │                    │        │  move_group │ (ya     │
│                              │                    │        │  OMPL/RRT   │ existía)│
│                              │                    │        └──────┬──────┘         │
│                              │   result_json      │               │                │
│  GUARDA EL ARCHIVO    ◄──────┼────────────────────┼───────────────┘                │
│  (aquí, no en el contenedor) │                    │                                │
│                              │                    │                                │
│                              │   preview/request  │  ┌──────────────────────────┐  │
│  reenvía una secuencia ──────┼───────────────────►│  │ kuka_trajectory_         │  │
│  guardada                    │                    │  │ preview_node             │  │
│                              │                    │  │  - DisplayTrajectory     │──┼─► RViz2
│                              │   preview/status   │  │  - DisplayRobotState     │  │
│  lee el estado        ◄──────┼────────────────────┼──┤  - respeta time_from_start│ │
│                              │                    │  └──────────────────────────┘  │
└──────────────────────────────┘                    └───────────────────────────────┘
```

El paquete **reutiliza la infraestructura MoveIt2 existente**: no levanta un
segundo `move_group`, no duplica el `robot_description` y no crea un pipeline
paralelo. Habla con el `move_group` que ya levanta
`kuka_kr6_moveit_config/demo.launch.py`.

### Nodos

| Nodo | Ejecutable | Función |
|---|---|---|
| Generador | `kuka_trajectory_generator_node` | Planifica un segmento por transición y publica el JSON de resultado. Solo `plan_only`. |
| Preview | `kuka_trajectory_preview_node` | Reproduce en RViz2 una secuencia ya guardada. Solo dibuja. |

### Relación con los paquetes existentes

| Paquete | Relación |
|---|---|
| `kuka_kr6_moveit_config` | **Se usa tal cual.** Aporta `move_group`, SRDF, OMPL/RRTConnect y el RViz con el display *Planned Path*. No se modifica. |
| `kuka_kr6_support` | **Se usa tal cual.** URDF/XACRO y mallas. No se modifica. |
| `kuka_gui_moveit_bridge` | **No se toca.** Es la vía de comandos en vivo de la GUI. Este paquete no publica en sus tópicos. |
| `kuka_trajectory_logger` | **Complementario.** Observa `/display_planned_path`, así que además registra en CSV cada segmento que se planifica aquí (ver §9). |
| `kuka_pick_place_demo` | **No se toca.** Su `pick_place_sequence_node` planifica desde YAML; este paquete lo hace desde JSON, por segmentos y sin ejecutar. |

---

## 3. Contrato ROS 2

Para no crear paquetes de interfaces duplicados entre los dos workspaces, todo
viaja como **`std_msgs/msg/String` con JSON**.

| # | Tópico | Tipo | Dirección |
|---|---|---|---|
| 1 | `/kuka_moveit/trajectory_generation/request_json` | `std_msgs/msg/String` | GUI/TCP-IP → MoveIt2 |
| 2 | `/kuka_moveit/trajectory_generation/result_json` | `std_msgs/msg/String` | MoveIt2 → GUI/TCP-IP |
| 3 | `/kuka_moveit/trajectory_preview/request_json` | `std_msgs/msg/String` | GUI/TCP-IP → MoveIt2 |
| 4 | `/kuka_moveit/trajectory_preview/status_json` | `std_msgs/msg/String` | MoveIt2 → GUI/TCP-IP |

**QoS de los cuatro tópicos** (publicadores y suscriptores):

| Parámetro | Valor |
|---|---|
| Reliability | `RELIABLE` |
| History | `KEEP_LAST` |
| Depth | `10` (parámetro `qos_depth`) |
| Durability | `VOLATILE` |

No se toca la configuración DDS existente (`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`,
`ROS_LOCALHOST_ONLY=0`); estos tópicos la usan tal cual.

### Tópicos internos de visualización

| Tópico | Tipo | Uso |
|---|---|---|
| `/display_planned_path` | `moveit_msgs/msg/DisplayTrajectory` | Mecanismo **nativo** MoveIt2/RViz2. Ya está en la config de RViz del proyecto. |
| `/kuka_moveit/trajectory_preview/robot_state` | `moveit_msgs/msg/DisplayRobotState` | Mecanismo **exclusivo de preview**, punto a punto con `time_from_start`. |

Ninguno de los dos llega al bridge TCP/IP ni a un controlador.

---

## 4. Flujo GUI → MoveIt → GUI

```
1. GUI          → publica request_json  (P1..PN en grados + eventos de garra)
2. Generador    → valida el contrato
3. Generador    → para cada Pi -> P(i+1):
                    - convierte grados a radianes
                    - fija el estado inicial EXPLÍCITO en el MotionPlanRequest
                    - fija P(i+1) como objetivo articular
                    - llama a /move_action con plan_only = true
                    - copia el JointTrajectory devuelto TAL CUAL
4. Generador    → publica result_json (status "ok" o "error")
5. GUI          → GUARDA EL ARCHIVO (el contenedor no guarda nada)
6. GUI          → más tarde publica preview/request_json con esa secuencia
7. Preview      → publica status "playing", dibuja en RViz2, publica "completed"
```

---

## 5. Formato de la solicitud de generación

```jsonc
{
  "schema_version": 1,
  "request_id": "uuid",
  "joint_names": ["joint_a1","joint_a2","joint_a3","joint_a4","joint_a5","joint_a6"],
  "points": [
    { "id": "P1", "joints_deg": [0, -90, 90, 0, 90, 0] },
    { "id": "P2", "joints_deg": [30, -80, 80, 0, 90, 0] },
    { "id": "P3", "joints_deg": [30, -60, 60, 0, 90, 0] }
  ],
  "gripper": {
    "initial_state": "open",
    "events": [
      { "at_point": "P2", "action": "close" },
      { "at_point": "P4", "action": "open"  }
    ]
  },
  "planner": { "mode": "moveit_base", "execute": false }
}
```

### Reglas de validación

| Campo | Regla |
|---|---|
| `schema_version` | Entero. Distinto de `1` ⇒ **aviso**, se procesa igualmente. |
| `request_id` | Cadena. Se devuelve tal cual en el resultado. Opcional (`""`). |
| `joint_names` | Lista no vacía y sin repetidos. Si falta, se usa `joint_a1..joint_a6`. |
| `points` | **Mínimo 2** (con 1 punto no hay ninguna transición). |
| `points[i].id` | Cadena no vacía. Si falta, se genera `P1`, `P2`, … Repetido ⇒ aviso. |
| `points[i].joints_deg` | Tantos números finitos como `joint_names`. **En grados.** |
| `gripper` | Opcional. Si falta ⇒ `initial_state: "open"` y `events: []`. |
| `gripper.events[i]` | Requiere `at_point` y `action`. `at_point` desconocido ⇒ **aviso**, no error. |
| `planner.mode` | Solo `"moveit_base"`. Cualquier otro valor ⇒ **error**. |
| `planner.execute` | Solo `false`. `true` ⇒ **error** (este entorno nunca ejecuta). |
| `planner.velocity_scaling` | Opcional, `(0, 1]`. Si falta se usa el valor del `config`. |
| `planner.acceleration_scaling` | Opcional, `(0, 1]`. Si falta se usa el valor del `config`. |

### Eventos de garra

- La garra se considera **inicialmente ABIERTA** (`initial_state: "open"`), y ese
  es el valor por defecto si el bloque no viene.
- Los eventos **NO afectan al cálculo de MoveIt**: no se generan segmentos extra,
  no se añaden paradas y no se modifica el tiempo.
- Se **conservan y se devuelven** íntegros en el resultado (incluidos campos
  adicionales que la GUI quiera añadir a cada evento) para que el otro entorno
  los almacene y los ejecute después, junto al punto correspondiente.
- Si un evento referencia un `at_point` que no existe en `points`, se añade un
  aviso en `warnings` pero el evento **no se borra ni se modifica**.

---

## 6. Generación de cada segmento

Para cada pareja consecutiva `Pi -> P(i+1)`:

1. `joints_deg` se convierte a **radianes**.
2. El **estado inicial** se envía EXPLÍCITAMENTE en `MotionPlanRequest.start_state`
   (`is_diff = false`), con los 6 joints. El nodo **no lee `/joint_states`**: la
   generación es independiente de dónde esté el robot real o el modelo de RViz.
3. El **objetivo** es un conjunto de `JointConstraint`, uno por joint, con la
   tolerancia `joint_goal_tolerance_deg` (0.2° por defecto).
4. Se usa el **pipeline y el planner BASE ya configurados** en el proyecto:
   `planning_pipeline_id` y `planner_id` van **vacíos**, por lo que `move_group`
   aplica su valor por defecto (`ompl` / `RRTConnectkConfigDefault`). Este
   paquete **no cambia la configuración de OMPL/RRTConnect**.
5. Se planifica con `planning_options.plan_only = true`.
6. **Nunca se ejecuta.**
7. Se extrae `result.planned_trajectory.joint_trajectory`
   (`trajectory_msgs/msg/JointTrajectory`).
8. Se conservan **todos** los puntos intermedios generados.
9. Se conserva `time_from_start` de cada punto.
10. Se conservan `positions`.
11. Se conservan `velocities` **si MoveIt2 las proporciona**.
12. Se conservan `accelerations` **si MoveIt2 las proporciona**.
13. Si un array llega vacío, se devuelve **lista vacía** (`[]`). **Nunca se
    inventa un cero.**
14. Se guardan las posiciones en **radianes** (dato original) y su conversión a
    **grados** (comodidad para la GUI).
15. El final de un segmento es el inicio del siguiente (ver más abajo).

### Encadenado de segmentos (`segment_chaining`)

| Valor | Estado inicial de `T(i+1)` |
|---|---|
| `previous_trajectory_end` *(por defecto)* | El **último punto real** de `Ti`. Garantiza continuidad exacta: el primer punto de `T(i+1)` es idéntico al último de `Ti`. |
| `source_point` | El punto nominal `P(i+1)` tal y como lo envió la GUI. |

Ambos coinciden salvo por la tolerancia del objetivo articular: MoveIt2 puede
terminar hasta `joint_goal_tolerance_deg` lejos de `P(i+1)`. Por eso el valor
por defecto es `previous_trajectory_end` (sin saltos entre segmentos).

### Orden de los joints

MoveIt2 puede devolver `joint_names` en un orden distinto al de la petición. El
resultado los reordena al **orden canónico de `joint_names` de la petición** —
es un cambio de orden de columnas, **los valores no se tocan** — y guarda el
orden original en `segments[i].moveit_joint_names`.

---

## 7. Formato del resultado

### `status: "ok"`

```jsonc
{
  "schema_version": 1,
  "request_id": "uuid",
  "status": "ok",

  "joint_names": ["joint_a1", "...", "joint_a6"],

  "source_points": [
    { "id": "P1", "joints_deg": [...], "joints_rad": [...] }
  ],

  "gripper": { "initial_state": "open", "events": [ ... ] },

  "planner_metadata": {
    "planning_pipeline": "",            // "" = pipeline por defecto (ompl)
    "planner_id": "",                   // "" = RRTConnectkConfigDefault
    "group": "manipulator",
    "plan_only": true,
    "velocity_scaling": 0.1,
    "acceleration_scaling": 0.1,
    "planning_attempts": 10,
    "allowed_planning_time_sec": 10.0,
    "joint_goal_tolerance_rad": 0.00349,
    "segment_chaining": "previous_trajectory_end"
  },

  "segments": [
    {
      "segment_id": "T1",
      "from": "P1",
      "to": "P2",
      "duration_sec": 1.234,
      "point_count": 42,
      "planning_time_sec": 0.031,
      "moveit_joint_names": ["..."],     // orden original devuelto por MoveIt2
      "trajectory_points": [
        {
          "index": 0,
          "time_from_start_sec": 0.0,
          "positions_rad": [...],
          "positions_deg": [...],
          "velocities_rad_s": [...],      // [] si MoveIt2 no las entregó
          "accelerations_rad_s2": [...]   // [] si MoveIt2 no las entregó
        }
      ]
    }
  ],

  "summary": {
    "source_point_count": 3,
    "segment_count": 2,
    "trajectory_point_count": 84,
    "total_duration_sec": 2.5
  },

  "warnings": ["..."]                    // solo si hubo avisos
}
```

### `status: "error"`

```jsonc
{
  "schema_version": 1,
  "request_id": "uuid",
  "status": "error",
  "message": "MoveIt2 fallo la planificacion: code=-1 (PLANNING_FAILED).",
  "failed_segment": "T2",
  "from_point": "P2",
  "to_point": "P3",
  "moveit_error_code": -1,
  "completed_segments": 1,
  "segment_count_expected": 3
}
```

### Manejo de errores

- Si un segmento falla, la generación se **detiene ahí**: no se continúa
  silenciosamente y **no se devuelve `status: "ok"`**.
- El resultado indica `failed_segment`, `from_point`, `to_point`, el mensaje y
  el código de error de MoveIt2.
- **La GUI NO debe considerar esa secuencia como ejecutable.**
- Los segmentos ya planificados **no se publican**: un resultado de error no
  trae `segments`, solo cuántos se habían completado.
- Errores posibles: JSON inválido, contrato inválido (`points` < 2, longitudes
  que no cuadran, `execute: true`, `mode` distinto de `moveit_base`),
  `/move_action` no disponible, objetivo rechazado, timeout, código de error de
  MoveIt2, trayectoria vacía o sin los joints pedidos.
- Si llega una petición mientras hay otra en curso, se responde
  `status: "error"` con `BUSY: ...` (nunca se mezclan dos secuencias).

### Avisos (`warnings`)

No detienen la generación. Se emiten, por ejemplo, cuando:

- un evento de garra referencia un punto inexistente;
- hay `id` de punto repetidos;
- `schema_version` no es 1;
- **el primer punto devuelto por MoveIt2 no coincide con el estado inicial
  pedido** (ocurre si un punto capturado del KUKA queda fuera de los límites del
  URDF: el adaptador `FixStartStateBounds` lo ajusta). El dato **no se corrige**,
  solo se avisa;
- **el último punto no coincide con `P(i+1)`** más allá de la tolerancia.

---

## 8. Previsualización en RViz2

El nodo de preview recibe una secuencia **ya guardada** (exactamente el mismo
JSON que produce la generación: se admite el resultado completo tal cual) y la
reproduce en RViz2 **sin mover el KUKA**.

### Secuencia visual continua `T1 → T2 → … → TN`

Los datos de MoveIt2 tienen dos propiedades correctas que, reproducidas
literalmente, se ven mal en RViz:

- la pose de cada frontera está **duplicada**: `Ti[-1] == T(i+1)[0]`;
- cada segmento **reinicia su `time_from_start_sec` en `0.0`**.

Por eso el nodo construye **solo en memoria** una única secuencia visual
continua (`build_preview_sequence()`):

1. `T1` aporta todos sus puntos. Para `T2..TN`, si la primera pose coincide con
   la última ya reproducida dentro de `boundary_tolerance_rad`, se omite **solo
   ese punto**. Si no coincide, no se omite nada.
2. Los tiempos se trasladan a un **reloj global monótono** que nunca vuelve a
   cero. Al omitir una frontera, el reloj se ancla en la pose omitida, de modo
   que el punto siguiente conserva **exactamente** su separación original.

Con la secuencia real de 8 segmentos y 123 puntos, la reproducción usa
`123 − 7 = 116` poses.

> [!IMPORTANT]
> Esto es **únicamente visual**. El JSON recibido, sus 8 segmentos, sus 123
> puntos y sus `positions_rad`, `positions_deg`, `velocities_rad_s`,
> `accelerations_rad_s2` y `time_from_start_sec` **no se modifican**. No se
> interpola ni se recalcula nada.

### Mecanismo 1 — nativo de MoveIt2/RViz2 (por defecto, sin configurar nada)

Se reconstruye **una sola** `moveit_msgs/msg/RobotTrajectory` continua a partir
de la secuencia anterior y se publica **una única vez** un
`moveit_msgs/msg/DisplayTrajectory` en **`/display_planned_path`**.

El display *Planned Path* del panel **MotionPlanning** del RViz del proyecto ya
está suscrito a ese tópico, así que **no hay que tocar la configuración de
RViz**.

> [!NOTE]
> Publicar un `DisplayTrajectory` **por segmento** hacía que RViz reiniciase su
> animación y su rastro en cada frontera: eso, junto con la pose duplicada y el
> reloj que volvía a cero, era el **parpadeo** que se veía al pulsar *PROBAR
> TRAYECTORIA*. Por eso ya no existe el parámetro `display_mode`.

> [!NOTE]
> La **velocidad de la animación** del display *Planned Path* la decide RViz con
> su ajuste **State Display Time** (hoy `3x` en `moveit.rviz`). Este paquete
> **no modifica el RViz existente**, así que esa animación no respeta
> exactamente `time_from_start`. Para la temporización exacta, use el mecanismo 2.

### Mecanismo 2 — reproducción punto a punto con `time_from_start`

El nodo publica un `moveit_msgs/msg/DisplayRobotState` **por cada pose de la
secuencia continua**, esperando su tiempo en el **reloj global** (dividido por
`playback_rate`), con un único cronómetro que arranca una sola vez y **no se
reinicia entre segmentos**, en el tópico **exclusivo de preview**:

```
/kuka_moveit/trajectory_preview/robot_state
```

Para verlo, añada **una vez** en RViz un display *RobotState*:

```
Displays → Add → moveit_rviz_plugin → RobotState
  Robot Description : robot_description
  Robot State Topic : /kuka_moveit/trajectory_preview/robot_state
```

Este display es **solo de dibujo**: no comanda nada. Se puede desactivar con el
parámetro `publish_robot_state: false`.

> [!WARNING]
> **Seguridad de la previsualización.** El nodo **no tiene** ningún cliente de
> acción ni de servicio: es incapaz de ejecutar nada. Además, los nombres de
> tópico configurados se validan al arrancar y el nodo **se niega a iniciarse**
> si contienen `kuka_bridge`, `joint_command`, `joint_states`,
> `follow_joint_trajectory`, `execute_trajectory`, `move_action`, `controller`
> o `enable_move`. Nunca se publica en `/kuka_bridge/joint_command_deg` ni en
> `/joint_states`, y nunca se activa EnableMove.

### Estado publicado en `/kuka_moveit/trajectory_preview/status_json`

Al empezar:

```json
{ "schema_version": 1, "request_id": "...", "status": "playing",
  "segment_count": 8, "trajectory_point_count": 123,
  "preview_point_count": 116, "dropped_boundary_points": 7,
  "total_duration_sec": 5.75, "playback_rate": 1.0,
  "display_trajectory_topic": "/display_planned_path", "warnings": [] }
```

`trajectory_point_count` son los puntos **originales** (no cambian);
`preview_point_count` son las poses realmente dibujadas tras omitir las
`dropped_boundary_points` fronteras duplicadas.

Al empezar cada segmento (si `publish_segment_status: true`):

```json
{ "schema_version": 1, "request_id": "...", "status": "playing",
  "segment_index": 0, "segment_id": "T1", "from": "P1", "to": "P2",
  "duration_sec": 1.4, "global_time_sec": 0.0 }
```

Estos mensajes son **solo diagnóstico**: no interrumpen ni alteran la
visualización, que sigue corriendo sobre el reloj global.

Al terminar:

```json
{ "schema_version": 1, "request_id": "...", "status": "completed",
  "segment_count": 3, "trajectory_point_count": 120 }
```

Si falla:

```json
{ "schema_version": 1, "request_id": "...", "status": "error",
  "message": "..." }
```

Si llega una petición nueva mientras se reproduce otra, la anterior se cancela:

```json
{ "schema_version": 1, "request_id": "...", "status": "cancelled",
  "message": "Reproduccion cancelada por una peticion nueva.",
  "segment_index": 1 }
```

### Formatos aceptados en `preview/request_json`

El nodo admite **dos formatos**, y en ambos **todos** los datos de la trayectoria
(`joint_names`, `segments`, `summary`, `schema_version`) se leen del **mismo
nivel**:

**a) Envuelto** — el que publica la GUI al releer un archivo guardado:

```jsonc
{
  "schema_version": 1,
  "preview_id": "...",          // identificador de esta previsualización
  "request_id": "...",          // identificador de la secuencia guardada
  "source_file": "...",         // informativo, no se usa
  "trajectory": {               // ← AQUÍ vive la trayectoria completa
    "schema_version": 1,
    "request_id": "...",
    "joint_names": [ ... ],
    "segments": [ ... ],
    "summary": { ... }
  }
}
```

**b) Directo** — el resultado de la generación tal cual:

```jsonc
{ "schema_version": 1, "request_id": "...", "joint_names": [...], "segments": [...] }
```

La identidad (`request_id`, `preview_id`) se toma del **sobre** y, si no viene
ahí, del objeto `trajectory`. `preview_id` se devuelve en el primer
`status_json` de la reproducción. Si `trajectory` existe pero no es un objeto,
se responde `status: "error"` diciéndolo explícitamente; si faltan los
`segments`, el mensaje de error indica que se buscaron **dentro de
`"trajectory"`**.

### Tolerancias del JSON de preview

- Si un punto no trae `positions_rad` pero sí `positions_deg`, se convierte.
- `time_from_start_sec` admite el alias `time_from_start`.
- Si una serie de velocidades/aceleraciones viene incompleta o con `null`, se
  **descarta entera** (con aviso) en vez de rellenarla: un
  `JointTrajectoryPoint` de ROS 2 no admite huecos.
- Se rechaza (error) si faltan `segments`, si un segmento no tiene puntos, si
  faltan posiciones o tiempos, o si `time_from_start_sec` decrece.

---

## 9. El contenedor NO guarda el archivo final

El generador **publica** el resultado y **no escribe nada en disco**. El archivo
final se guarda en el **otro entorno**, donde están las GUIs y el TCP/IP.

Como efecto secundario del flujo normal de MoveIt2, `move_group` publica cada
plan resuelto en `/display_planned_path`. Por eso:

- las trayectorias se ven en RViz2 mientras se generan, y
- si el paquete `kuka_trajectory_logger` está corriendo, registrará esos
  segmentos en su CSV (comportamiento propio de ese paquete, que no se ha
  modificado). Ese CSV es un registro de depuración, **no** el archivo final del
  sistema.

Lo mismo ocurre durante la previsualización: si el logger está activo, capturará
también los `DisplayTrajectory` publicados por la preview. Si eso molesta, hay
dos opciones **sin tocar nada existente**: no lanzar el logger, o cambiar el
parámetro `display_trajectory_topic` de la preview a un tópico propio (por
ejemplo `/kuka_moveit/trajectory_preview/display`) y añadir en RViz un display
*Trajectory* apuntando a él.

---

## 10. Estructura del paquete

```text
kuka_moveit_trajectory_planner/
├── package.xml
├── setup.py
├── setup.cfg
├── README.md
├── resource/
│   └── kuka_moveit_trajectory_planner
├── config/
│   └── kuka_moveit_trajectory_planner.yaml   # parámetros de los dos nodos
├── launch/
│   └── trajectory_planner.launch.py          # ADICIONAL y OPCIONAL
├── kuka_moveit_trajectory_planner/
│   ├── __init__.py
│   ├── trajectory_contract.py                # lógica pura del JSON (sin ROS)
│   ├── trajectory_generator_node.py          # generación (solo plan)
│   └── trajectory_preview_node.py            # previsualización (solo dibujo)
└── test/
    └── test_trajectory_contract.py           # tests del contrato (sin ROS)
```

---

## 11. Compilación

```bash
cd /root/taller1/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select kuka_moveit_trajectory_planner
source install/setup.bash
```

Ningún otro paquete necesita recompilarse: este paquete no modifica a ninguno.

---

## 12. Ejecución

Este launch **no levanta MoveIt2 ni RViz2**: se conecta al sistema que ya está
en marcha. No sustituye a ningún launch existente.

```bash
# Terminal 1 — el sistema de siempre, SIN CAMBIOS
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_LOCALHOST_ONLY=0
ros2 launch kuka_kr6_moveit_config demo.launch.py use_gui:=false use_rviz:=true
#   o bien:
# ros2 launch kuka_gui_moveit_bridge kuka_bridge_system.launch.py

# Terminal 2 — este paquete
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_LOCALHOST_ONLY=0
source /root/taller1/ros2_ws/install/setup.bash
ros2 launch kuka_moveit_trajectory_planner trajectory_planner.launch.py
```

Argumentos del launch:

| Argumento | Por defecto | Uso |
|---|---|---|
| `use_generator` | `true` | Lanzar el nodo de generación |
| `use_preview` | `true` | Lanzar el nodo de previsualización |
| `playback_rate` | `1.0` | Escala de tiempo de la preview (`0.5` = mitad de velocidad) |
| `log_points` | `false` | Imprimir en terminal todos los puntos generados |

También se puede lanzar cada nodo por separado:

```bash
ros2 run kuka_moveit_trajectory_planner kuka_trajectory_generator_node \
  --ros-args --params-file \
  /root/taller1/ros2_ws/install/kuka_moveit_trajectory_planner/share/kuka_moveit_trajectory_planner/config/kuka_moveit_trajectory_planner.yaml
```

### Prueba manual desde terminal

```bash
# Generar
ros2 topic pub --once /kuka_moveit/trajectory_generation/request_json \
  std_msgs/msg/String "{data: '{\"schema_version\":1,\"request_id\":\"demo-1\",\"joint_names\":[\"joint_a1\",\"joint_a2\",\"joint_a3\",\"joint_a4\",\"joint_a5\",\"joint_a6\"],\"points\":[{\"id\":\"P1\",\"joints_deg\":[0,-90,90,0,90,0]},{\"id\":\"P2\",\"joints_deg\":[20,-80,80,0,90,0]},{\"id\":\"P3\",\"joints_deg\":[20,-60,60,0,90,0]}],\"gripper\":{\"initial_state\":\"open\",\"events\":[{\"at_point\":\"P2\",\"action\":\"close\"}]},\"planner\":{\"mode\":\"moveit_base\",\"execute\":false}}'}"

# Ver el resultado (en otra terminal, ANTES de publicar la petición)
ros2 topic echo /kuka_moveit/trajectory_generation/result_json

# Estado de la preview
ros2 topic echo /kuka_moveit/trajectory_preview/status_json
```

---

## 13. Parámetros

### `kuka_trajectory_generator_node`

| Parámetro | Por defecto | Descripción |
|---|---|---|
| `request_topic` | `/kuka_moveit/trajectory_generation/request_json` | Entrada JSON |
| `result_topic` | `/kuka_moveit/trajectory_generation/result_json` | Salida JSON |
| `qos_depth` | `10` | Profundidad de la cola (RELIABLE) |
| `planning_group` | `manipulator` | Grupo del SRDF |
| `base_frame` | `base_link` | Frame del workspace del `MotionPlanRequest` |
| `move_action_name` | `/move_action` | Acción `moveit_msgs/action/MoveGroup` |
| `action_wait_timeout_sec` | `30.0` | Espera máxima a que aparezca `move_group` |
| `planning_pipeline_id` | `''` | Vacío = pipeline por defecto del proyecto |
| `planner_id` | `''` | Vacío = planner por defecto (RRTConnect) |
| `planning_time` | `10.0` | `allowed_planning_time` |
| `planning_attempts` | `10` | `num_planning_attempts` |
| `velocity_scaling` | `0.1` | Igual que el resto del proyecto |
| `acceleration_scaling` | `0.1` | Igual que el resto del proyecto |
| `joint_goal_tolerance_deg` | `0.2` | Tolerancia del objetivo articular |
| `endpoint_warning_tolerance_deg` | `0.5` | Umbral de aviso inicio/fin |
| `segment_chaining` | `previous_trajectory_end` | Ver §6 |
| `joint_names` | `joint_a1..joint_a6` | Orden canónico si la petición no lo trae |
| `log_points` | `false` | Imprimir cada punto en terminal |

### `kuka_trajectory_preview_node`

| Parámetro | Por defecto | Descripción |
|---|---|---|
| `request_topic` | `/kuka_moveit/trajectory_preview/request_json` | Entrada JSON |
| `status_topic` | `/kuka_moveit/trajectory_preview/status_json` | Salida de estado |
| `qos_depth` | `10` | Profundidad de la cola (RELIABLE) |
| `display_trajectory_topic` | `/display_planned_path` | Mecanismo nativo de RViz |
| `robot_state_topic` | `/kuka_moveit/trajectory_preview/robot_state` | Reproducción punto a punto |
| `publish_robot_state` | `true` | Desactiva el mecanismo 2 si es `false` |
| `playback_rate` | `1.0` | Escala de tiempo (no altera los datos) |
| `boundary_tolerance_rad` | `1e-6` | Tolerancia para dar por duplicada la pose de una frontera |
| `inter_segment_gap_sec` | `0.0` | Hueco en el reloj global, **solo** entre segmentos cuya frontera no estaba duplicada |
| `publish_segment_status` | `true` | Status adicional al empezar cada segmento |
| `robot_model_name` | `''` | `DisplayTrajectory.model_id` (vacío evita avisos) |
| `joint_names` | `joint_a1..joint_a6` | Orden canónico si la petición no lo trae |

---

## 14. Pruebas

La lógica del contrato JSON es pura (sin ROS) y se puede probar sin levantar
nada:

```bash
cd /root/taller1/ros2_ws/src/kuka_moveit_trajectory_planner
python3 -m pytest test/test_trajectory_contract.py -q
```

Cubre: formación de los segmentos consecutivos, conversión grados→radianes,
garra abierta por defecto y conservación literal de sus eventos, rechazo de
`execute: true` y de modos distintos de `moveit_base`, listas vacías cuando
MoveIt2 no entrega velocidades/aceleraciones, resumen del resultado, formato del
error con `failed_segment`, y el parseo del JSON de preview.

---

## 15. Este paquete NO modifica nada existente

| Elemento | Estado |
|---|---|
| TCP/IP, EKI, XML de KUKA, SPS, Submit Interpreter, KRL | **No se toca** |
| Configuración DDS / FastDDS / UDP | **No se toca** |
| URDF / XACRO / mallas | **No se toca** |
| SRDF, `kinematics.yaml`, `joint_limits.yaml` | **No se toca** |
| `ompl_planning.yaml` (OMPL/RRTConnect) | **No se toca** |
| `demo.launch.py`, `kuka_bridge_system.launch.py`, resto de launches | **No se sustituyen**; este launch es adicional |
| `moveit.rviz` | **No se toca** (el display *RobotState* del mecanismo 2 se añade a mano si se quiere) |
| `kuka_gui_moveit_bridge`, `kuka_pick_place_demo`, `kuka_trajectory_logger` | **No se tocan** |
| Cartesianas, HOME, límites articulares | **No se tocan** |
| Pilz, LIN, CIRC, SCIRC, detección de figuras | **No se añaden** |
