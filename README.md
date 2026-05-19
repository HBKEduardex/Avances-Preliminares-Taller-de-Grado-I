# Avances Preliminares de Taller de Grado I: Simulación de Pick and Place con KUKA KR6 R900 en ROS2 Humble usando MoveIt2

## 1. Descripción del proyecto

Este repositorio corresponde a los avances preliminares de Taller de Grado I, enfocados en el desarrollo de una simulación para el manipulador industrial KUKA KR6 R900. El entorno se basa en ROS2 Humble, MoveIt2, RViz2 y Docker para asegurar la portabilidad y la replicabilidad del proyecto sin necesidad de instalar ROS2 directamente en el sistema host.

El enfoque actual es enteramente simulado. El objetivo principal es visualizar el robot, cargar la escena del entorno de laboratorio, planificar trayectorias libres de colisión y preparar la ejecución de una secuencia tipo *pick and place* (tomar y dejar). La conexión física TCP/IP con el controlador real del robot KUKA queda planteada para una etapa posterior del desarrollo.

Actualmente, el sistema permite mover el robot a través de una interfaz gráfica para guardar posiciones articulares de manera interactiva, conformando una secuencia, para luego validar su planificación de trayectoria y, opcionalmente, simular la ejecución de los movimientos usando MoveIt2.

## 2. Alcance actual

Actualmente el proyecto permite:
- Visualizar el KUKA KR6 R900 en RViz2.
- Cargar la escena del laboratorio, incluyendo modelos de base, mesa, gripper, cubo y otros elementos ya configurados.
- Usar MoveIt2 con el grupo de planificación definido como `manipulator`.
- Planificar trayectorias avanzadas usando OMPL/RRTConnect.
- Visualizar de forma previsualizada las trayectorias planificadas en RViz2.
- Mover el robot visualmente mediante `joint_state_publisher_gui` para posicionarlo y establecer puntos clave.
- Guardar puntos articulares (joint states) mediante la invocación del servicio `/save_pick_place_point`.
- Almacenar automáticamente la secuencia completa en un archivo YAML.
- Validar la planificación cinemática de la secuencia tipo *pick and place*.
- Ejecutar la secuencia planificada si existe un controlador simulado activo o un *fake hardware*.

**Aspectos a considerar:**
- Por ahora no se controla el cierre ni la apertura real del gripper de la herramienta.
- El cubo no se adjunta físicamente al gripper durante la toma; es solo simulación de movimiento articular.
- El *pick and place* actual se restringe a una simulación de movimiento por puntos predefinidos.
- La ejecución real asume que el controlador `joint_trajectory_controller` se encuentra activo en el ecosistema ROS2.

## 3. Requisitos

Para poder levantar y usar el entorno se requiere:
- Sistema operativo Linux.
- Docker instalado en el sistema host.
- Permisos de administrador o inclusión del usuario en el grupo para ejecutar Docker.
- Servidor gráfico X11 disponible y configurado en el host.
- Git instalado.
- Conexión a internet estable (solo necesaria para la primera construcción de la imagen de Docker).
- **Nota:** No es necesario tener instalado ROS2 de forma nativa en el host, ya que todo el stack de ROS2 se ejecuta directamente desde dentro del contenedor Docker.

## 4. Estructura general del repositorio

La estructura de las carpetas es la siguiente:

```text
Taller1/
├── docker/                 # Archivos de configuración para Docker (Dockerfile, scripts)
│   └── Dockerfile
├── scripts/                # Scripts de automatización para creación y acceso al contenedor
├── ros2_ws/                # Espacio de trabajo (Workspace) principal de ROS2
│   └── src/
│       ├── kuka_kr6_support/           # Modelos y geometría del robot y el laboratorio
│       ├── kuka_resources/             # Recursos compartidos y materiales
│       ├── kuka_kr6_moveit_config/     # Configuración de MoveIt2 y SRDF
│       ├── kuka_pick_place_interfaces/ # Definición de servicios personalizados (interfaces CMake)
│       └── kuka_pick_place_demo/       # Nodos Python para lógica de secuencia y grabación
└── README.md
```

## 5. Descripción de paquetes ROS2

### 5.1 kuka_kr6_support
- Contiene el modelo base cinemático del KUKA KR6 R900.
- Incluye los archivos URDF/XACRO, las mallas (meshes) visuales y de colisión, y la configuración base.
- Se usa como el paquete base universal de descripción del robot, por lo que no debe duplicarse esta geometría en otros paquetes.
- Es consumido de manera modular tanto por los *launch files* de visualización como por MoveIt2.
- Lanzador básico principal: `display.launch.py`. Este lanza `robot_state_publisher`, `joint_state_publisher_gui` y `RViz2` para una visualización primaria.
- **Nodos típicos lanzados:**
  - `robot_state_publisher`: pública la jerarquía de transformadas dinámicas (TF) del robot a partir del URDF y el tópico `/joint_states`.
  - `joint_state_publisher_gui`: publica en `/joint_states` los valores ajustados por el usuario mediante barras deslizantes (sliders).
  - `rviz2`: el visualizador 3D interactivo.

### 5.2 kuka_resources
- Contiene recursos comunes, constantes o de colorimetría requeridos por los URDF/XACRO de la familia KUKA.
- Es una dependencia esencial del modelo `kuka_kr6_support`.
- No suele usarse directamente por el usuario, pero es obligatorio compilarlo e instalarlo.

### 5.3 kuka_kr6_moveit_config
- Contiene la configuración necesaria de MoveIt2 generada para el KUKA KR6 R900.
- Define el grupo cinemático de planificación principal: `manipulator`.
- Utiliza la suite de planificadores geométricos de OMPL (default: RRTConnect).
- Carga el archivo de semántica robótica (SRDF), las matrices de cinemática (IK solvers), límites articulares, y los controladores base para la integración con MoveIt.
- Reutiliza inteligentemente el URDF de `kuka_kr6_support`.
- **Launch principal:** `demo.launch.py`.
  - Genera y carga dinámicamente `robot_description` invocando xacro en tiempo de ejecución.
  - Genera y carga el `robot_description_semantic` (SRDF).
  - Lanza el pipeline de `move_group`.
  - Inicia la interfaz de RViz2 con el plugin MotionPlanning habilitado.
  - Puede lanzar paralelamente el `joint_state_publisher_gui` si se le pasa el argumento `use_gui:=true`.
  - Si cuenta con la configuración, puede operar bajo hardware falso si se usa `use_fake_hardware:=true`.
- **Diferencia operativa clave:**
  - `use_gui:=true`: Utilizado primordialmente para setear los puntos y verlos de forma interactiva.
  - `use_gui:=false` y `use_fake_hardware:=true`: Utilizado para delegar la ejecución articular a un controlador simulado.
- **Nodos principales:**
  - `move_group`: el nodo monolítico central de MoveIt2 que orquesta la planificación de trayectorias, chequeo de colisiones y ejecución.
  - `robot_state_publisher` y `rviz2`.
  - `joint_state_publisher_gui`.
  - Componentes de ejecución hipotéticos: `ros2_control_node`, `joint_state_broadcaster`, y `joint_trajectory_controller`.

### 5.4 kuka_pick_place_interfaces
- Un paquete compilado puramente mediante CMake que incluye las interfaces personalizadas del sistema.
- Contiene el servicio principal `SavePoint.srv`. Se opta por separarlo en CMake para seguir las directrices correctas de ROS2 a la hora de procesar interfaces mediante `rosidl`.
- **Servicio `SavePoint.srv`:**
  - *Request*: `name` (string), `overwrite` (booleano).
  - *Response*: `success` (booleano), `message` (string).
- **Tópico/Servicio exportado:** `/save_pick_place_point`
- **Uso:** Permite interrogar a ROS2 por la posición actual de `/joint_states` para ser grabada con un nombre específico, construyendo así la secuencia paso a paso.

### 5.5 kuka_pick_place_demo
- Paquete principal en Python orientado a contener la lógica algorítmica y de validación de la demostración tipo *pick and place*.
- Lee y escribe interactivamente el archivo de puntos (YAML).
- No modifica estáticamente la escena en RViz, sino que manipula el modelo cinemático usando la librería *moveit_commander* o la Action API.
- **Launchs provistos:**
  1. `point_recorder.launch.py`
  2. `pick_place_sequence.launch.py`
- **Nodos provistos:**
  - `point_recorder_node`:
    - Permanece suscrito al tópico `/joint_states`.
    - Levanta el servicio `/save_pick_place_point`.
    - Al recibir una petición, persiste los valores radiantes de los 6 ejes principales directamente a un archivo YAML en el sistema.
  - `pick_place_sequence_node`:
    - Carga los puntos secuenciales desde el YAML en el arranque.
    - Se conecta como cliente de acciones al servidor principal de MoveIt2 (`/move_action`).
    - En el modo `execute:=false` únicamente valida que exista un camino factible sin colisiones para cada etapa.
    - En el modo `execute:=true`, si hay controladores activos, no solo planifica sino que solicita el movimiento real/simulado.

## 6. Configuración de Docker

Dado que instalar todo el sistema de ROS2 y MoveIt2 desde cero en cada computadora es ineficiente y problemático, el uso de Docker permite encapsular y replicar el ambiente operativo.

### 6.1 Primer uso

La primera vez que uses el proyecto, debes compilar la imagen base y luego levantar tu contenedor:

```bash
cd ~/Documents/taller1
chmod +x scripts/*.sh
./scripts/build_image.sh
./scripts/create_container.sh
```

Una vez ejecutado, el contenedor estará creado. No se requiere invocar `create_container.sh` nuevamente, salvo que hayas eliminado permanentemente la instancia en Docker.

### 6.2 Activar permisos gráficos X11

Antes de iniciar el contenedor o ejecutar RViz2, **siempre** debes permitir que el contenedor Docker retransmita datos gráficos hacia la interfaz X11 de la máquina host.

En la terminal del host (fuera de Docker):

```bash
xhost +local:docker
```

*Nota: Esto habilita temporalmente las interfaces visuales; si recibes el error "could not open display" en RViz2, es síntoma de que omitiste este comando o que necesitas configuraciones adicionales.*

### 6.3 Uso diario del contenedor

El flujo normal para iniciar tu trabajo con el robot cada día:

```bash
cd ~/Documents/taller1
xhost +local:docker
docker start kuka_ros2_humble_container
docker attach kuka_ros2_humble_container
```

- `docker start`: Despierta el contenedor de su estado detenido (conservando su caché y datos previos).
- `docker attach`: Vincula tu entrada por teclado de la terminal actual a la terminal primaria del contenedor activo (equivalente a "entrar" físicamente al mismo).
- Para salir del attach y volver al host **sin apagar el contenedor**, utiliza la combinación de teclado: `Ctrl + P`, luego `Ctrl + Q`.
- Si estando atachado escribes el comando `exit`, el contenedor principal se detendrá.

### 6.4 Abrir una segunda terminal sin duplicar la sesión

Si tienes el attach corriendo en una consola y necesitas lanzar nodos en paralelo, **no utilices** `docker attach` de nuevo, o la nueva consola simplemente clonará visualmente lo que hace la primera. 

Abre una nueva terminal en el host y ejecuta:

```bash
docker exec -it kuka_ros2_humble_container bash
```

Una vez dentro de este nuevo *shell*:

```bash
cd /root/taller1/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Usar `docker exec` es crucial para revisar tópicos con `ros2 topic list`, llamar servicios manuales o mantener nodos funcionales divididos. Salir con `exit` desde un `docker exec` no afectará ni detendrá tu contenedor persistente.

## 7. Compilación del workspace

Siempre que modifiques lógica de nodos, archivos XACRO, o CMake, debes compilar el entorno desde el interior del contenedor.

```bash
cd /root/taller1/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

La bandera `--symlink-install` evita que debas recompilar si sólo modificaste un archivo de Python, YAML, o Launch script, ya que usa enlaces simbólicos en lugar de copias duras.

## 8. Uso básico: visualizar KUKA y MoveIt2

### 8.1 Visualización con GUI para setear puntos

```bash
ros2 launch kuka_kr6_moveit_config demo.launch.py use_gui:=true use_rviz:=true
```

- Abre la visualización en RViz2 y carga toda la suite de MoveIt2.
- Levanta un panel adicional flotante llamado `joint_state_publisher_gui` para manipular físicamente en la simulación los grados de libertad.
- **Es el modo correcto y recomendado para inspeccionar visualmente la celda robótica y guardar puntos iniciales.**
- Sin embargo, como el GUI inyecta un estado continuo estático a los motores virtuales del robot, interfiere posteriormente si queremos pedirle a MoveIt que ejecute y altere el movimiento automáticamente.

### 8.2 Modo con fake hardware para ejecución

```bash
ros2 launch kuka_kr6_moveit_config demo.launch.py use_fake_hardware:=true use_gui:=false use_rviz:=true
```

- Desactiva el panel GUI que traba los ejes articulares.
- Habilita la utilización de hardware simulado (*fake controllers*) en la arquitectura base si estos se encuentran debidamente provistos o instanciados localmente (p.e. a través de parámetros o dependencias anexas del setup local).
- Delega el control de `joint_states` a quien esté proveyendo la acción de la trayectoria (o en su defecto al Broadcaster).
- **Es el modo correcto para usar en simultáneo con `execute:=true` en rutinas secuenciales.**

## 9. Tópicos, acciones y servicios importantes

### Tópicos clave
| Tópico | Uso principal |
|---|---|
| `/joint_states` | Posición instantánea articular del robot (rad). |
| `/tf` y `/tf_static` | Árbol de transformaciones cinemáticas relativas (Transformations). |
| `/robot_description` | URDF/XACRO serializado a string para visualización e información. |
| `/display_planned_path` | Publica la previsualización del camino planificado en RViz2. |
| `/planning_scene` | Mantiene actualizada la escena contra posibles colisiones internas. |
| `/motion_plan_request` | Recepción subyacente para validaciones de MoveIt2. |
| `/trajectory_execution_event`| Eventos y diagnósticos reportados tras ejecución en hardware. |

### Acciones base
| Acción | Finalidad |
|---|---|
| `/move_action` | Action server global manejado por `move_group`. Escucha objetivos de poses y coordina el pipeline de planificar y mover. |
| `/execute_trajectory` | Acción subyacente delegada una vez que una ruta es probada y validada. |
| `/joint_trajectory_controller/follow_joint_trajectory` | Interfaz entre el manejador de MoveIt y el controlador físico/simulado. |

### Servicios personalizados
| Servicio | Funcionalidad |
|---|---|
| `/save_pick_place_point` | Invocación asíncrona para congelar el `/joint_states` actual a YAML. |

Puedes verificar si están activos mediante:
```bash
ros2 topic list
ros2 action list
ros2 service list
ros2 node list
ros2 control list_controllers
```

## 10. Flujo completo de ejercicio: setear y ejecutar una secuencia pick and place

Esta sección explica de principio a fin el flujo operativo en dos casos principales. 

### 10.1 Caso A: ya existe un YAML con puntos guardados

Si ya has creado los puntos y el archivo `pick_place_points.yaml` no está vacío, no precisas repetir el proceso de configuración.

**1. Abre una Terminal primaria (dentro de Docker) y lanza MoveIt:**

Para una validación visual simple donde no moveremos el hardware:
```bash
ros2 launch kuka_kr6_moveit_config demo.launch.py use_gui:=true use_rviz:=true
```

Para una ejecución activa sobre los actuadores o controladores simulados:
```bash
ros2 launch kuka_kr6_moveit_config demo.launch.py use_fake_hardware:=true use_gui:=false use_rviz:=true
```

**2. Abre una Terminal 2 independiente y lanza el nodo secuencial:**

```bash
docker exec -it kuka_ros2_humble_container bash
cd /root/taller1/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Para validar únicamente la viabilidad del camino:
```bash
ros2 launch kuka_pick_place_demo pick_place_sequence.launch.py execute:=false
```

Para ejecutar el camino físicamente/simuladamente en controlador:
```bash
ros2 launch kuka_pick_place_demo pick_place_sequence.launch.py execute:=true
```
- *Aviso:* Ejecutar `execute:=true` asume la disponibilidad del controlador o bien un equivalente *fake_controller* provisto mediante `use_fake_controller:=true`.

### 10.2 Caso B: crear nuevos puntos desde cero

Si necesitas capturar posiciones específicas o los parámetros físicos de tu modelo han cambiado, se puede inicializar el almacenamiento nuevamente.

Se recomienda mantener tu configuración en un backup por seguridad:
```bash
cp ros2_ws/src/kuka_pick_place_demo/config/pick_place_points.yaml ros2_ws/src/kuka_pick_place_demo/config/pick_place_points.backup.yaml
```

Abre `ros2_ws/src/kuka_pick_place_demo/config/pick_place_points.yaml` (usando tu IDE base del host, o usando `nano` desde Docker) y elimina toda su estructura dejándola vacía:

```yaml
sequence: []
```

### 10.3 Paso 1: lanzar MoveIt2 con GUI

En tu Terminal 1:
```bash
cd /root/taller1/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch kuka_kr6_moveit_config demo.launch.py use_gui:=true use_rviz:=true
```
Aparecerá RViz2 acompañado del panel de controles manuales. Manipulando estas barras lograrás posicionar el brazo en el área que desees. El panel GUI transmitirá su posición en `/joint_states`.

### 10.4 Paso 2: lanzar grabador de puntos

Abre una Terminal 2 independiente con `docker exec -it`:
```bash
cd /root/taller1/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch kuka_pick_place_demo point_recorder.launch.py
```
Este nodo en Python quedará latente analizando los `/joint_states` permanentemente. Provee la disponibilidad del servicio de captura que usaremos desde otra consola.

### 10.5 Paso 3: guardar puntos usando el servicio

Abre una Terminal 3 independiente con `docker exec -it`, actualiza el entorno y realiza secuencialmente las grabaciones. Recuerda que **antes de lanzar cada comando**, debes ajustar los sliders a la pose deseada para ese evento.

```bash
# 1. Ajustar sliders a postura base inicial
ros2 service call /save_pick_place_point kuka_pick_place_interfaces/srv/SavePoint "{name: 'home', overwrite: true}"

# 2. Ajustar sliders encima del objeto objetivo
ros2 service call /save_pick_place_point kuka_pick_place_interfaces/srv/SavePoint "{name: 'approach_pick', overwrite: true}"

# 3. Ajustar sliders bajando a tomar el objeto
ros2 service call /save_pick_place_point kuka_pick_place_interfaces/srv/SavePoint "{name: 'pick', overwrite: true}"

# 4. Ajustar sliders subiendo con el objeto (cierre no implementado aún)
ros2 service call /save_pick_place_point kuka_pick_place_interfaces/srv/SavePoint "{name: 'lift', overwrite: true}"

# 5. Ajustar sliders hacia la zona de destino superior
ros2 service call /save_pick_place_point kuka_pick_place_interfaces/srv/SavePoint "{name: 'approach_place', overwrite: true}"

# 6. Ajustar sliders al momento de soltar
ros2 service call /save_pick_place_point kuka_pick_place_interfaces/srv/SavePoint "{name: 'place', overwrite: true}"

# 7. Ajustar sliders alejándose (retirada segura)
ros2 service call /save_pick_place_point kuka_pick_place_interfaces/srv/SavePoint "{name: 'retreat', overwrite: true}"
```
Con el parámetro `overwrite: true`, si te equivocas de nombre o posición puedes simplemente invocar nuevamente la línea para machacar esa coordenada en el archivo YAML, el cual registrará un histórico válido.

### 10.6 Significado de cada punto

- `home`: Pose base para esperar instrucciones; típicamente un estado seguro plegado.
- `approach_pick`: Acercamiento superior del extremo del robot sobre la pieza a capturar.
- `pick`: Bajada milimétrica al contacto de aprensión del objeto.
- `lift`: Elevación tras la sujeción, regresando a la zona de seguridad.
- `approach_place`: Trayectoria aérea hacia las coordenadas cenitales de descenso finales.
- `place`: Bajada hasta la plancha o mesa de posicionamiento del cubo.
- `retreat`: Regreso a plano aéreo tras la expulsión del objeto (apertura de pinzas). 

### 10.7 Paso 4: validar planificación de la secuencia

Desde la consola que ocupaste para setear comandos (Terminal 3):

```bash
ros2 launch kuka_pick_place_demo pick_place_sequence.launch.py execute:=false
```
Este proceso procesa el array de YAML iterativamente contra el API de `move_action`. Evaluará factibilidades trigonométricas y previsualizará (animará fantasmagóricamente) la ruta sin tocar realmente los estados finales del actuador.

### 10.8 Paso 5: ejecutar secuencia

Cierra todos los nodos y consolas GUI usando `Ctrl + C`. El robot requiere ahora que soltemos el candado del GUI.

Vuelve a levantar la base bajo *fake_hardware* sin forzar la UI (Terminal 1):
```bash
ros2 launch kuka_kr6_moveit_config demo.launch.py use_fake_hardware:=true use_gui:=false use_rviz:=true
```

Si el sistema fue correctamente integrado con hardware emulado o base ros2_control, se pueden verificar los administradores.
```bash
ros2 control list_controllers
ros2 action list
```
Debe constatarse que aparezcan activos tópicos similares a:
- `joint_state_broadcaster active`
- `joint_trajectory_controller active`
- `/joint_trajectory_controller/follow_joint_trajectory`

Finalmente (Terminal 2):
```bash
ros2 launch kuka_pick_place_demo pick_place_sequence.launch.py execute:=true
```
Verás que el nodo solicitará activamente acatar el plan y enviar la señal final. Si la tolerancia está fuera de límites, valida que parta visualmente desde un punto anexo al solicitado.

## 11. Diferencia entre Plan, Execute y Plan & Execute

Es importante comprender la terminología subyacente que opera nuestro nodo tras bambalinas:
- **Plan:** Únicamente evalúa las matemáticas OMPL/RRT y proyecta hologramas mediante `/display_planned_path`.
- **Execute:** MoveIt2 recibe un flujo paramétrico ya procesado y busca un ActionServer vinculado (generalmente dependiente de la arquitectura `ros2_control`) enviándole un array denso de valores y latencias. Si no hay Controlador, falla abruptamente.
- **Plan & Execute:** Coordina lógicamente un *Plan* exitoso y automáticamente empuja la cascada de *Execute*.
- *Nota final:* Si activas GUI, el slider constantemente sobre-escribe `/joint_states`. Esto es una batalla directa con la instrucción programada si intentaras ejecutar trayectorias reales.

## 12. Problemas comunes

| Problema | Causa probable | Solución rápida |
|---|---|---|
| **RViz2 no abre de inmediato.** | Falta de variables gráficas delegadas. | Escribir `xhost +local:docker` en la terminal base host. |
| **Segunda terminal visualiza output duplicado.** | Se usó un `docker attach` simultáneo en consolas distintas. | Usa `docker exec -it kuka_ros2_humble_container bash` para abrir nuevos threads limpios. |
| **No aparece geometría del robot.** | Errores TF `Fixed Frame` a robot o descripción de URDF extraviada. | Ajusta `Fixed Frame` a `base_link` y revisa advertencias en Model. |
| **MoveIt solo muestra perfil CHOMP.** | Ausencia de inyección de parámetros OMPL en Launch. | Revisa dependencias `ompl_planning.yaml` en `move_group`. |
| **Falla sistemática de _Plan_.** | Posición en colisión con el entorno desde el inicio. | Modificar `home` a un sitio alejado, o verificar tolerancias de self-collision. |
| **Falla inmediata de _Execute_.** | No se encontraron instancias Action activas. | Falla de inicialización de controladores simulados. |
| **Nodo Secuencia trabado esperando.** | Está esperando a un Action string incorrecto. | Debe configurarse a `/move_action` explícitamente en el nodo y parámetros. |
| **Servicio no existente o no devuelve respuesta.** | El nodo subyacente detuvo su loop de vida. | Asegúrate que `point_recorder.launch.py` figure corriendo activamente. |
| **YAML estancado.** | Permisos del archivo o colisión de lectura/escritura symlink. | Verifica escritura del file crudo o restaura permisos con `chmod`. |
| **En modo GUI robot retrocede.** | El loop de slider estático es rey contra el display de ejecución. | El GUI nunca debe usarse en ejecuciones de rutas finales. |
| **_Execute:=true_ no se mueve.** | Ausencia de un manejador de ActionServer real de seguimiento. | Deben estar correctamente provisionados y acoplados los _controllers_. |

## 13. Criterios de validación de avances preliminares

Para considerar este avance de Taller de Grado I como sólido, se consolidaron los siguientes hitos operativos:
- El entorno Docker aísla exitosamente el middleware de ROS.
- El KUKA KR6 R900 es cinemáticamente visualizable desde RViz2 de forma nativa.
- MoveIt2 fue instrumentado correctamente para orquestar el grupo `manipulator`.
- OMPL/RRTConnect entrega validación probabilística de obstáculos con fiabilidad.
- La escena estática y geométrica se ha fusionado al ecosistema RViz de MoveIt.
- El panel de control visual logra interactuar y emitir estados al manipulador.
- La lógica *ad-hoc* es capaz de serializar y estampar puntos al sistema de archivos local (YAML).
- Un nodo central orquesta la lectura, envío y validación remota secuencial al servidor de acción.
- El framework sienta el terreno idóneo para condicionar mecánicamente las variables a controladores y ejecución física.

## 14. Próximos pasos

La siguiente etapa de escalamiento de la simulación hacia la implementación real contemplará las siguientes vías de desarrollo:
- Consolidar los puentes entre *fake hardware* / `ros2_control` y MoveIt2 para rutinas de automatización continua offline.
- Realizar microajustes de métricas precisas a los 7 puntos de pick and place dentro del espacio del cubo.
- Integrar la dinámica algorítmica y visual para comandar el cierre y apertura paramétrico del *gripper*.
- Modificar el estado del cubo (de un elemento de colisión fijo a un objeto manipulable acoplado mediante TF dinámica al *end effector*).
- Depurar las mediciones de tiempos precisos de fase de planificación vs fase de ejecución in situ.
- Realizar trazabilidad y análisis del vector de perfilería de velocidad (Jerk) del componente robótico frente al modelo.
- Diagramar los esquemas lógicos previos a la comparación entre KRL propietario (KUKA Robot Language) vs el stack de C++ de MoveIt2.
- Evaluar formalmente el enlace y handshake TCP/IP al gabinete del controlador de producción real del sistema físico.
