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
