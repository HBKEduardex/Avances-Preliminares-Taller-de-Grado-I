# kuka_gui_moveit_bridge

Paquete ROS 2 que actúa como **puente entre una GUI externa y MoveIt2/RViz**
para el robot KUKA KR6 (modelo kr6r900sixx).
Incluye también una GUI de prueba interna (Tkinter).

---

> **⚠️ IMPORTANTE**
>
> La interfaz externa trabaja **exclusivamente en grados**.
> El nodo convierte internamente a radianes para MoveIt.
> **No se deben publicar radianes en los tópicos terminados en `_deg`.**

---

## 1. Objetivo

Permitir que una GUI externa envíe comandos de movimiento al robot simulado
en RViz **usando grados** (nunca radianes), sin necesidad de conocer los
detalles internos de MoveIt2.

El paquete se encarga de:

- Recibir comandos articulares y cartesianos en grados desde la GUI.
- Convertir internamente grados ↔ radianes.
- Calcular IK internamente usando el estado actual como semilla para comandos cartesianos.
- Evitar movimientos si el robot ya está en el objetivo (ALREADY_AT_TARGET).
- Enviar los objetivos a MoveIt2 mediante `/move_action`.
- Publicar el estado articular y cartesiano (vía TF2) en grados para la GUI.
- Publicar mensajes de estado comprensibles.

---

## 2. Requisitos del sistema

Ademas de ROS 2 Humble/Iron/Rolling y MoveIt2, se requiere el paquete `python3-tk` para la GUI de prueba.

```bash
sudo apt update
sudo apt install python3-tk
```

---

## 3. Arquitectura

```
GUI externa o GUI de prueba
    │
    ├── /kuka_bridge/joint_command_deg
    │   std_msgs/Float64MultiArray
    │   data: [A1, A2, A3, A4, A5, A6]  ← en GRADOS
    │
    └── /kuka_bridge/cartesian_command_deg
        std_msgs/Float64MultiArray
        data: [X(m), Y(m), Z(m), A(°), B(°), C(°)]  ← A,B,C en GRADOS
              │
              ▼
    ┌─────────────────────────────────────────┐
    │       kuka_moveit_bridge_node           │
    │                                         │
    │  1. Validación de límites (URDF)        │
    │  2. Conversión grados ↔ radianes        │
    │  3. Verificación ALREADY_AT_TARGET      │
    │  4. Cálculo de IK con semilla actual    │
    │  5. Creación de MoveGroup Goal          │
    │  6. Lectura TF2 (base_link → link_6)    │
    └─────────────┬───────────────────────────┘
                  │
                  ▼
           /move_action
           [moveit_msgs/action/MoveGroup]
                  │
                  ▼
               MoveIt2
                  │
                  ▼
    fake_trajectory_controller_node
    /joint_trajectory_controller/follow_joint_trajectory
                  │
                  ▼
    /fake_joint_states → joint_state_publisher → /joint_states
                  │
                  ▼
               RViz
                  │
    ┌─────────────┴───────────────────────────┐
    │  kuka_moveit_bridge_node (salidas)      │
    │                                         │
    │  /kuka_bridge/status           → GUI    │
    │  /kuka_bridge/joint_state_deg  → GUI    │
    │  /kuka_bridge/cartesian_state_deg → GUI │
    └─────────────────────────────────────────┘
```

---

## 4. Estructura del paquete

```
kuka_gui_moveit_bridge/
├── config/
│   ├── kuka_bridge.yaml              ← Parámetros configurables del puente
│   └── kuka_test_gui.yaml            ← Parámetros configurables de la GUI
├── launch/
│   └── kuka_bridge_system.launch.py  ← Launch único del sistema completo
├── kuka_gui_moveit_bridge/
│   ├── __init__.py
│   ├── kuka_moveit_bridge_node.py    ← Nodo puente principal
│   ├── kuka_bridge_test_gui_node.py  ← GUI de prueba (Tkinter)
│   └── transform_utils.py            ← Lógica pura (conversiones, límites, validaciones)
├── resource/
│   └── kuka_gui_moveit_bridge        ← Marker de ament
├── test/
│   ├── test_angle_conversion.py      ← Tests grados↔radianes
│   ├── test_quaternion_conversion.py ← Tests ABC↔cuaternión
│   ├── test_cartesian_state.py       ← Tests conversión estado cartesiano
│   ├── test_target_comparison.py     ← Tests ALREADY_AT_TARGET
│   └── test_gui_validation.py        ← Tests de límites articulares
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

---

## 5. Compilación

```bash
cd ~/Documents/taller1/ros2_ws
colcon build --packages-select kuka_gui_moveit_bridge --symlink-install
source install/setup.bash
```

---

## 6. Ejecución — Un único comando

El launch principal levanta todo: MoveIt, RViz, controlador simulado, nodo puente y GUI de prueba.

```bash
cd ~/Documents/taller1/ros2_ws
source install/setup.bash
ros2 launch kuka_gui_moveit_bridge kuka_bridge_system.launch.py
```

Si no deseas abrir la GUI de prueba (por ejemplo, si vas a conectar tu propia GUI externa):
```bash
ros2 launch kuka_gui_moveit_bridge kuka_bridge_system.launch.py use_test_gui:=false
```

---

## 7. Tópicos

### Entrada (GUI → Bridge)

| Tópico | Tipo | Formato |
|---|---|---|
| `/kuka_bridge/joint_command_deg` | `std_msgs/Float64MultiArray` | `[A1, A2, A3, A4, A5, A6]` en **grados** |
| `/kuka_bridge/cartesian_command_deg` | `std_msgs/Float64MultiArray` | `[X(m), Y(m), Z(m), A(°), B(°), C(°)]` |

### Salida (Bridge → GUI)

| Tópico | Tipo | Formato |
|---|---|---|
| `/kuka_bridge/status` | `std_msgs/String` | Mensajes de estado en texto |
| `/kuka_bridge/joint_state_deg` | `std_msgs/Float64MultiArray` | `[A1, A2, A3, A4, A5, A6]` en **grados** |
| `/kuka_bridge/cartesian_state_deg` | `std_msgs/Float64MultiArray` | `[X(m), Y(m), Z(m), A(°), B(°), C(°)]` en **grados**, extraído desde TF2 |

---

## 8. Funcionalidades Destacadas

### Lógica ALREADY_AT_TARGET
Antes de enviar un objetivo a MoveIt, el puente valida si el robot ya se encuentra en esa posición.
- Para comandos articulares, se comparan los grados directamente.
- Para comandos cartesianos, primero se calcula la IK. Si la IK es exitosa, se verifica si la posición articular calculada es igual a la actual. Alternativamente, se verifica la distancia posicional y angular de cuaterniones antes de la IK.
- Si está en el objetivo, el nodo publica `ALREADY_AT_TARGET` y **no envía un goal a MoveIt**.

### Uso de `/compute_ik` con Semilla
Para los comandos cartesianos (XYZABC), en lugar de usar un `PositionConstraint` y `OrientationConstraint` (que fallan con `error_code=99999` por ser demasiado estrictos), el puente primero calcula la Cinemática Inversa usando el servicio `/compute_ik`.
Crucialmente, envía el `joint_states` actual como `robot_state` (semilla) para favorecer la solución articular más cercana, evitando que el robot de saltos locos (cambios de configuración).

### GUI de Prueba (Tkinter)
Una interfaz nativa de ROS 2:
- Utiliza **MultiThreadedExecutor** para no bloquear la interfaz gráfica.
- **No se inventa estados**: Copia el estado inicial directamente desde los tópicos `/kuka_bridge/joint_state_deg` y `/kuka_bridge/cartesian_state_deg` la primera vez que se reciben.
- **Segura por defecto**: No envía NINGÚN comando al robot automáticamente cuando se abre.
- Soporta el cambio en caliente del parámetro `plan_only` (botón en la interfaz) para evaluar movimientos sin ejecutarlos realmente.
- Permite movimientos incrementales y probar valores extremos mediante validación interna (revisando los límites del URDF).

---

## 9. Pruebas unitarias completas

Para ejecutar todas las pruebas unitarias:
```bash
cd ~/Documents/taller1/ros2_ws
source install/setup.bash
pytest src/kuka_gui_moveit_bridge/test/ -v
```

Pruebas implementadas:
1. 0 grados -> 0 radianes.
2. 180 grados -> pi radianes.
3. -90 grados -> -pi/2 radianes.
4. pi radianes -> 180 grados.
5. ABC 0,0,0 -> cuaternión identidad.
6. Cuaternión identidad -> ABC 0,0,0.
7. Normalización del cuaternión.
8. Diferencia angular entre cuaterniones iguales igual a cero.
9. Rechazo de NaN e Inf en validaciones.
10. Rechazo de arreglos de tamaño distinto de 6.
11. Reordenamiento de /joint_states por nombre.
12. Detección ALREADY_AT_TARGET (articular y cartesiana).
13. Conversión de estado cartesiano a grados.
14. Validación de límites articulares.

---

## 10. Límites articulares reales (en GRADOS)

| Joint | Nombre ROS | Mínimo (°) | Máximo (°) |
|---|---|---|---|
| A1 | `joint_a1` | −170.0 | +170.0 |
| A2 | `joint_a2` | −190.0 | +45.0 |
| A3 | `joint_a3` | −120.0 | +156.0 |
| A4 | `joint_a4` | −185.0 | +185.0 |
| A5 | `joint_a5` | −120.0 | +120.0 |
| A6 | `joint_a6` | −350.0 | +350.0 |

---

## 11. Este paquete NO modifica paquetes existentes

Los paquetes existentes se utilizan tal cual:
- `kuka_kr6_moveit_config`: su `demo.launch.py` se incluye mediante `IncludeLaunchDescription`, sin modificaciones.
- `kuka_pick_place_demo`: su `fake_trajectory_controller_node` se lanza como nodo normal, sin modificaciones.
