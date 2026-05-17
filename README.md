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

> **Nota importante sobre el estado inicial ("invalid start state"):**
> Si al presionar *Plan* en MoveIt2 observas el error *"Skipping invalid start state (invalid state)"*, se debe a que la posición por defecto (todos los joints en 0.0) hace que partes del robot choquen entre sí o superen algún límite. 
> **Solución:** En RViz2 ve a la pestaña *MotionPlanning* -> *Planning*, en *Select Start State* elige `<ready>` o `<home>`, y luego dale al botón *Update*. Alternativamente, usa la ventana emergente de *joint_state_publisher_gui* para mover ligeramente las articulaciones fuera del cero antes de planificar.

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
## Flujo de avances
1. Visualización del KUKA en ROS2/RViz2.
2. MoveIt2 mínimo solo con el robot.
3. Entorno preliminar del laboratorio.
4. Cubo de 30 mm y gripper preliminar.
5. Prueba mínima de pick and place.
6. Futuro: comparación KRL vs MoveIt/MoveIt2 y métricas de desempeño.
