# trajectories/

Carpeta de salida de la funcionalidad **Secuencias de Trayectorias** de
`kuka_gui_control`.

Aquí es donde **la GUI** guarda el archivo JSON de cada secuencia después de
que el contenedor MoveIt2 devuelve una trayectoria válida. El contenedor NO
escribe nada aquí.

## Ruta

Ruta por defecto (resuelta en runtime):

```
<raíz del repositorio>/trajectories
```

que en este checkout es:

```
/home/eduardex/Documents/TG2/trajectories
```

Nunca se escribe dentro de `install/`. La ruta se puede cambiar por dos vías:

| Vía | Cómo | Afecta a |
|---|---|---|
| YAML | `trajectories_dir: "/ruta/…"` en `config/gui_dual_kuka_rviz.yaml` | GUI dual |
| Entorno | `export KUKA_TRAJECTORIES_DIR=/ruta/…` | Ambas GUIs |

El YAML tiene prioridad sobre la variable de entorno, y ambas sobre la
resolución automática.

## Nombre de archivo

```
trajectory_sequence_YYYYMMDD_HHMMSS.json
```

Un archivo completo por secuencia. JSON, no YAML.

## Contenido

`schema_version`, `request_id`, fecha/hora, `source_points`, `joint_names`,
`gripper.initial_state`, `gripper.events`, `planner_metadata`, `segments` con
sus `trajectory_points` (`positions_rad`, `positions_deg`,
`velocities_rad_s`, `accelerations_rad_s2`, `time_from_start_sec`),
`duration_sec` por segmento y `summary`.

La trayectoria se guarda **tal cual llega**: no se eliminan puntos, no se
reinterpola, no se modifican tiempos y no se suaviza nada.

## Uso

- **PROBAR TRAYECTORIA** → reproduce un archivo de esta carpeta en RViz.
  Solo RViz: nunca alcanza al KUKA real.
- **ENVIAR TRAYECTORIA** → ejecuta un archivo de esta carpeta en el KUKA real
  por el bridge TCP/IP existente, respetando `safe_mode`,
  `allow_motion_commands` y el checkbox `ENABLE MOVE`.
