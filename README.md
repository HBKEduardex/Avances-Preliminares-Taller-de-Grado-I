# Sistema preliminar de visualización KUKA KR6 R900 en ROS2 Humble + Docker

## Descripción
Este proyecto contiene los paquetes necesarios para visualizar el robot KUKA KR6 R900 en RViz2 usando ROS2 Humble, empaquetado en un entorno Docker para asegurar la portabilidad y facilidad de ejecución sin necesidad de instalar dependencias pesadas en el sistema local.

## Estructura del proyecto
- `docker/`: Archivos para construir la imagen de Docker (Dockerfile).
- `scripts/`: Scripts utilitarios para simplificar el uso de Docker y ROS2.
- `ros2_ws/src/`: Código fuente de los paquetes ROS2 (kuka_kr6_support y kuka_resources).
- `README.md`: Instrucciones del proyecto.

## Requisitos
- Linux con Docker instalado
- Soporte para GUI (servidor X11)

## Primer uso
Ejecuta los siguientes comandos en tu terminal local para inicializar el proyecto:

```bash
chmod +x scripts/*.sh
./scripts/build_image.sh
./scripts/create_container.sh
docker start kuka_ros2_humble_container
docker attach kuka_ros2_humble_container
```

Una vez **dentro del contenedor**, ejecuta:

```bash
cd /root/taller1/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch kuka_kr6_support display.launch.py
```

## Uso diario
Para volver a iniciar y usar el entorno en el día a día, ejecuta:

```bash
docker start kuka_ros2_humble_container
docker attach kuka_ros2_humble_container
```

Y una vez **dentro del contenedor**:

```bash
./scripts/launch_rviz.sh
```

## Entrar y salir del contenedor
- **Entrar:** Se utiliza `docker attach kuka_ros2_humble_container` porque el contenedor es persistente y te permite conectarte directamente a la terminal principal con la que fue creado. Esto es útil para mantener el flujo de trabajo en el mismo lugar.
- **Salir deteniendo el contenedor:** Si escribes `exit` dentro del contenedor, este se detendrá y volverás a la terminal del host.
- **Salir sin detener el contenedor:** Para salir y dejar el contenedor corriendo en segundo plano, presiona `Ctrl + P` y luego `Ctrl + Q`.

## Detener el contenedor
Si saliste usando `Ctrl + P` y `Ctrl + Q` y quieres detener el contenedor de forma manual desde el host:

```bash
docker stop kuka_ros2_humble_container
```

## Ver contenedores
Para listar todos tus contenedores y verificar si el tuyo está en ejecución (Up) o detenido (Exited):

```bash
docker ps -a
```

## Lanzar RViz2
Para lanzar la visualización desde dentro del contenedor (asumiendo que ya compilaste):

```bash
./scripts/launch_rviz.sh
```

## Compilar workspace
Si modificas archivos del paquete (por ejemplo, el `.xacro`), dentro del contenedor ejecuta:

```bash
cd /root/taller1/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## Limpiar build/install/log
Si necesitas reconstruir todo de cero, dentro del contenedor puedes ejecutar:

```bash
./scripts/clean_ros2_build.sh
```

## Tópicos esperados
Con el visualizador en ejecución, si abres otra terminal dentro del contenedor y ejecutas:

```bash
ros2 topic list
```

Deberían aparecer los siguientes tópicos:
- `/joint_states`
- `/robot_description`
- `/tf`
- `/tf_static`

## Problemas comunes
- **Si RViz2 abre sin robot:** agregar `RobotModel` y poner `Description Topic` en `/robot_description`.
- **Si aparece error de Fixed Frame:** cambiar `Fixed Frame` a `base_link`.
- **Si no abre interfaz gráfica:** ejecutar `xhost +local:docker` en el host antes de iniciar el contenedor.
- **Si el contenedor no existe:** ejecutar `./scripts/create_container.sh`.
- **Si se modifican dependencias del Dockerfile:** volver a ejecutar `./scripts/build_image.sh`.

## Nota sobre Docker persistente
- `docker build` crea o actualiza la imagen sin borrar los archivos del repositorio.
- Los cambios hechos dentro del contenedor se pierden si el contenedor se elimina, **excepto los cambios en `/root/taller1`** porque es un volumen montado desde el host.
- Por eso se usa un contenedor persistente llamado `kuka_ros2_humble_container`.
- Si alguna vez se elimina el contenedor por accidente, los archivos locales siguen intactos y se puede crear nuevamente con `./scripts/create_container.sh`.

## Nota sobre ROS2 Humble y futura integración con MoveIt2
Este paquete de visualización es un paso preliminar. La estructura actual en Humble (usando `xacro`, `robot_state_publisher`, etc.) está lista para ser integrada a futuro con MoveIt2 para planificación de trayectorias.

## MoveIt2 mínimo para KUKA KR6 R900
- El documento del proyecto menciona MoveIt, pero en ROS2 Humble corresponde implementar MoveIt2.
- Esta etapa solo valida planificación articular básica usando OMPL + RRTConnect.
- No incluye mesa, gripper, cubo, entorno, tool changer ni pick and place.
- El paquete MoveIt2 reutiliza el modelo existente de kuka_kr6_support.
- No se duplica el URDF/XACRO.
- La configuración está pensada como base previa para agregar después el entorno del laboratorio y pruebas de pick and place.

## Uso de MoveIt2

> **Postura inicial configurada:**
> Al lanzar con `use_gui:=true`, la ventana `joint_state_publisher_gui` inicia con una postura articular segura preconfigurada (no en ceros), lo que evita el error *"Skipping invalid start state"* que ocurría cuando todos los joints estaban en 0.0.
> Si aún se presenta el error, en RViz2 ve a *MotionPlanning* → *Planning*, en *Select Start State* elige `<ready>` o `<home>`, y presiona *Update*.

Iniciar contenedor:
```bash
docker start kuka_ros2_humble_container
```

Entrar al contenedor:
```bash
docker attach kuka_ros2_humble_container
```

Dentro del contenedor:
```bash
cd /root/taller1/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch kuka_kr6_moveit_config demo.launch.py
```

Ejemplo con argumentos:
```bash
ros2 launch kuka_kr6_moveit_config demo.launch.py use_rviz:=true use_gui:=true use_sim_time:=false fixed_frame:=base_link planning_group:=manipulator robot_model:=kr6r900sixx
```

Verificar tópicos:
```bash
ros2 topic list
```

Inspeccionar parámetros:
```bash
ros2 param list
ros2 param list /move_group
ros2 param list /robot_state_publisher
```

Mencionar que también se puede usar rqt para inspección y ajustes visuales o de parámetros:
```bash
rqt
```

## Parámetros configurables del launch
- `use_rviz`: abre o no RViz2.
- `use_gui`: abre o no joint_state_publisher_gui.
- `use_sim_time`: activa o desactiva tiempo simulado.
- `fixed_frame`: frame base usado para visualización.
- `planning_group`: grupo de planificación usado por MoveIt2.
- `robot_model`: variante del modelo del robot.
- `rviz_config`: archivo RViz usado, si aplica.

## Abrir otra terminal dentro del mismo contenedor
NO se debe usar `docker attach` en dos terminales al mismo tiempo, porque `docker attach` se conecta a la misma sesión principal del contenedor y por eso lo que se escribe o se ve en una terminal puede reflejarse en la otra.

Para abrir una segunda terminal independiente dentro del mismo contenedor, usar:

```bash
docker exec -it kuka_ros2_humble_container bash
```

Dentro de esa nueva terminal ejecutar:

```bash
cd /root/taller1/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Ejemplos de uso para monitorear tópicos:

```bash
ros2 topic list
ros2 node list
ros2 topic echo /joint_states
ros2 topic echo /tf --once
```

**Debe quedar claro:**
- `docker attach kuka_ros2_humble_container` entra a la sesión principal del contenedor.
- `docker exec -it kuka_ros2_humble_container bash` abre una terminal nueva e independiente dentro del mismo contenedor.
- Si se necesita lanzar RViz en una terminal y revisar tópicos en otra, usar `docker exec`, no `docker attach`.
- Para salir de una terminal abierta con `docker exec`, se puede usar `exit` sin apagar el contenedor.
- Para salir de `docker attach` sin detener el contenedor, usar `Ctrl + P` y luego `Ctrl + Q`.
## Seteo visual de puntos para pick and place

### Descripción
Esta herramienta permite **setear visualmente** las posiciones del robot usando los sliders de `joint_state_publisher_gui` (o la interfaz interactiva de RViz/MoveIt2) y luego **guardar cada posición** como un punto nombrado en un archivo YAML. Posteriormente, se puede **validar la planificación** o **ejecutar** toda la secuencia de pick and place.

### Paquetes nuevos
- **`kuka_pick_place_interfaces`**: Paquete CMake con la definición del servicio `SavePoint.srv`.
- **`kuka_pick_place_demo`**: Paquete Python con los nodos `point_recorder_node` y `pick_place_sequence_node`.

### Flujo de trabajo

#### Paso 1: Lanzar MoveIt2 con GUI (Terminal 1)
```bash
docker start kuka_ros2_humble_container
docker attach kuka_ros2_humble_container
```

Dentro del contenedor:
```bash
cd /root/taller1/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch kuka_kr6_moveit_config demo.launch.py use_gui:=true use_rviz:=true
```

Esto abre RViz2 con MoveIt2 y la ventana de `joint_state_publisher_gui` con sliders para mover el robot.

#### Paso 2: Lanzar el grabador de puntos (Terminal 2)
```bash
docker exec -it kuka_ros2_humble_container bash
cd /root/taller1/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch kuka_pick_place_demo point_recorder.launch.py
```

#### Paso 3: Setear y guardar puntos (Terminal 3)
```bash
docker exec -it kuka_ros2_humble_container bash
cd /root/taller1/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

El flujo es:
1. Mover los sliders de `joint_state_publisher_gui` hasta la pose deseada.
2. Observar la posición del robot en RViz2.
3. Cuando esté en la posición deseada, guardar el punto con:

```bash
# Guardar posición home
ros2 service call /save_pick_place_point kuka_pick_place_interfaces/srv/SavePoint "{name: 'home', overwrite: true}"

# Mover sliders → guardar approach_pick
ros2 service call /save_pick_place_point kuka_pick_place_interfaces/srv/SavePoint "{name: 'approach_pick', overwrite: true}"

# Mover sliders → guardar pick
ros2 service call /save_pick_place_point kuka_pick_place_interfaces/srv/SavePoint "{name: 'pick', overwrite: true}"

# Mover sliders → guardar lift
ros2 service call /save_pick_place_point kuka_pick_place_interfaces/srv/SavePoint "{name: 'lift', overwrite: true}"

# Mover sliders → guardar approach_place
ros2 service call /save_pick_place_point kuka_pick_place_interfaces/srv/SavePoint "{name: 'approach_place', overwrite: true}"

# Mover sliders → guardar place
ros2 service call /save_pick_place_point kuka_pick_place_interfaces/srv/SavePoint "{name: 'place', overwrite: true}"

# Mover sliders → guardar retreat
ros2 service call /save_pick_place_point kuka_pick_place_interfaces/srv/SavePoint "{name: 'retreat', overwrite: true}"
```

Los puntos se guardan en `config/pick_place_points.yaml` dentro del share del paquete.

#### Paso 4: Validar planificación de la secuencia

El action server de MoveIt2 verificado es `/move_action` (verificable con `ros2 action list`).

```bash
ros2 launch kuka_pick_place_demo pick_place_sequence.launch.py execute:=false
```

Esto solo planifica la trayectoria para cada punto y reporta si la planificación fue exitosa, **sin ejecutar** movimiento alguno.

#### Paso 5: Ejecutar la secuencia (simulación visual)

Para probar la ejecución (que el robot se mueva en RViz) sin requerir `ros2_control`, se incluye un Action Server falso que publica la trayectoria a RViz.
Asegúrate de que MoveIt2 se haya lanzado **sin** la GUI de sliders (`use_gui:=false`) para que el controlador falso pueda publicar los joints:

```bash
ros2 launch kuka_pick_place_demo pick_place_sequence.launch.py execute:=true use_fake_controller:=true
```

> **Nota:** Cuando tengas hardware real o `ros2_control` funcional (con un Action Server en `/joint_trajectory_controller/follow_joint_trajectory`), puedes omitir `use_fake_controller:=true` y la trayectoria se enviará a tu controlador real.

### Notas importantes
- Si se usa `use_gui:=true`, la GUI publica `/joint_states`. Esto es útil para setear puntos visualmente.
- Para ejecución real simulada con controladores, puede ser necesario usar `use_gui:=false` y `ros2_control` activo para evitar conflicto con `joint_state_publisher_gui`.
- `execute:=false` solo muestra si la trayectoria es planificable.
- `execute:=true` requiere un controlador simulado o real activo.
- El pick and place por ahora **solo mueve el robot entre puntos**; no controla el cierre/apertura del gripper.
- El archivo YAML de puntos se puede editar manualmente si se desea ajustar valores específicos.

### Parámetros configurables del nodo de secuencia
| Parámetro | Default | Descripción |
|---|---|---|
| `planning_group` | `manipulator` | Grupo de planificación de MoveIt2 |
| `execute` | `false` | Si es true, planifica y ejecuta |
| `velocity_scaling` | `0.10` | Factor de velocidad (0.0-1.0) |
| `acceleration_scaling` | `0.10` | Factor de aceleración (0.0-1.0) |
| `points_file` | `config/pick_place_points.yaml` | Archivo YAML con puntos |
| `return_home` | `true` | Regresar a home al final |
| `wait_between_points` | `1.0` | Segundos entre puntos |
| `stop_on_failure` | `true` | Detener si un punto falla |
| `reference_frame` | `base_link` | Frame de referencia |
| `move_group_action_name` | `/move_action` | Action server de MoveIt2 |
| `use_fake_controller` | `false` | Lanza controlador falso si no hay hardware |

## Flujo de avances
1. Visualización del KUKA en ROS2/RViz2.
2. MoveIt2 mínimo solo con el robot.
3. Entorno preliminar del laboratorio.
4. Cubo de 30 mm y gripper preliminar.
5. Seteo visual de puntos y secuencia preliminar de pick and place.
6. Futuro: comparación KRL vs MoveIt/MoveIt2 y métricas de desempeño.
