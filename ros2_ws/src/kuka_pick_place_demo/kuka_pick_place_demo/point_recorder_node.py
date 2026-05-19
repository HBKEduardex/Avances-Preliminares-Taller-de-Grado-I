#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
point_recorder_node.py

Nodo ROS2 que se suscribe a /joint_states, mantiene la última posición
articular en memoria y expone el servicio /save_pick_place_point para
guardar la posición actual como un punto nombrado en un archivo YAML.

Uso típico:
  1. Lanzar MoveIt2 demo con use_gui:=true en Terminal 1.
  2. Lanzar este nodo en Terminal 2.
  3. Mover sliders en joint_state_publisher_gui.
  4. Llamar al servicio desde Terminal 3:
     ros2 service call /save_pick_place_point \
       kuka_pick_place_interfaces/srv/SavePoint \
       "{name: 'home', overwrite: true}"
"""

import os
import copy
import shutil
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Importar el servicio personalizado desde el paquete de interfaces
from kuka_pick_place_interfaces.srv import SavePoint

import yaml


class PointRecorderNode(Node):
    """Nodo para grabar posiciones articulares desde /joint_states."""

    def __init__(self):
        super().__init__('point_recorder_node')

        # ── Parámetros ──────────────────────────────────────────
        self.declare_parameter(
            'points_file',
            'config/pick_place_points.yaml'
        )
        self.declare_parameter(
            'joint_names',
            ['joint_a1', 'joint_a2', 'joint_a3',
             'joint_a4', 'joint_a5', 'joint_a6']
        )
        self.declare_parameter('autosave_backup', True)

        self._points_file = self.get_parameter('points_file') \
            .get_parameter_value().string_value
        self._joint_names = self.get_parameter('joint_names') \
            .get_parameter_value().string_array_value
        self._autosave_backup = self.get_parameter('autosave_backup') \
            .get_parameter_value().bool_value

        # Resolver ruta absoluta si es relativa
        if not os.path.isabs(self._points_file):
            # Buscar primero en el share del paquete instalado
            try:
                from ament_index_python.packages import \
                    get_package_share_directory
                pkg_share = get_package_share_directory(
                    'kuka_pick_place_demo')
                self._points_file = os.path.join(
                    pkg_share, self._points_file)
            except Exception:
                # Fallback: ruta relativa al directorio de trabajo
                self._points_file = os.path.abspath(self._points_file)

        self.get_logger().info(
            f'Archivo de puntos: {self._points_file}')
        self.get_logger().info(
            f'Joints monitoreados: {self._joint_names}')

        # ── Estado interno ──────────────────────────────────────
        self._last_joint_state = None  # type: JointState | None

        # ── Suscripción a /joint_states ─────────────────────────
        self._sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_states_cb,
            10
        )

        # ── Servicio /save_pick_place_point ─────────────────────
        self._srv = self.create_service(
            SavePoint,
            '/save_pick_place_point',
            self._save_point_cb
        )

        self.get_logger().info(
            'point_recorder_node listo. '
            'Servicio /save_pick_place_point disponible.')

    # ─── Callbacks ──────────────────────────────────────────────

    def _joint_states_cb(self, msg: JointState):
        """Almacenar la última lectura de /joint_states."""
        self._last_joint_state = msg

    def _save_point_cb(self, request, response):
        """Manejar solicitud del servicio SavePoint."""
        name = request.name.strip()
        overwrite = request.overwrite

        # Validar nombre
        if not name:
            response.success = False
            response.message = 'El nombre del punto no puede estar vacío.'
            self.get_logger().warn(response.message)
            return response

        # Validar que hay datos de joints
        if self._last_joint_state is None:
            response.success = False
            response.message = (
                'No se ha recibido ningún mensaje de /joint_states. '
                'Asegúrate de que la demo de MoveIt2 está corriendo '
                'con use_gui:=true.')
            self.get_logger().warn(response.message)
            return response

        # Extraer posiciones de los joints deseados
        joints_dict = self._extract_joints(self._last_joint_state)
        if not joints_dict:
            response.success = False
            response.message = (
                'No se encontraron los joints esperados en '
                '/joint_states. Joints recibidos: '
                f'{list(self._last_joint_state.name)}')
            self.get_logger().warn(response.message)
            return response

        # Cargar archivo YAML existente
        data = self._load_yaml()

        # Buscar si el punto ya existe
        existing_idx = None
        for i, pt in enumerate(data.get('sequence', [])):
            if pt.get('name') == name:
                existing_idx = i
                break

        if existing_idx is not None and not overwrite:
            response.success = False
            response.message = (
                f"El punto '{name}' ya existe. "
                f"Usa overwrite:=true para reemplazarlo.")
            self.get_logger().warn(response.message)
            return response

        # Construir entrada del punto
        point_entry = {
            'name': name,
            'type': 'joint_state',
            'joints': joints_dict,
        }

        # Insertar o reemplazar
        if 'sequence' not in data or data['sequence'] is None:
            data['sequence'] = []

        if existing_idx is not None:
            data['sequence'][existing_idx] = point_entry
            action = 'reemplazado'
        else:
            data['sequence'].append(point_entry)
            action = 'guardado'

        # Backup
        if self._autosave_backup:
            self._create_backup()

        # Guardar archivo
        self._save_yaml(data)

        response.success = True
        response.message = (
            f"Punto '{name}' {action} correctamente con "
            f"{len(joints_dict)} joints.")
        self.get_logger().info(response.message)
        self.get_logger().info(
            f'  Valores: {joints_dict}')
        return response

    # ─── Helpers ────────────────────────────────────────────────

    def _extract_joints(self, msg: JointState):
        """Extraer valores de los joints deseados del mensaje."""
        joints_dict = {}
        for jname in self._joint_names:
            if jname in msg.name:
                idx = list(msg.name).index(jname)
                # Redondear a 6 decimales para legibilidad
                joints_dict[jname] = round(msg.position[idx], 6)
        return joints_dict

    def _load_yaml(self):
        """Cargar el archivo YAML de puntos."""
        if os.path.isfile(self._points_file):
            try:
                with open(self._points_file, 'r') as f:
                    data = yaml.safe_load(f)
                    if data is None:
                        data = {'sequence': []}
                    return data
            except Exception as e:
                self.get_logger().error(
                    f'Error leyendo YAML: {e}')
                return {'sequence': []}
        return {'sequence': []}

    def _save_yaml(self, data):
        """Guardar datos en el archivo YAML."""
        try:
            # Asegurar que el directorio existe
            dirpath = os.path.dirname(self._points_file)
            if dirpath and not os.path.isdir(dirpath):
                os.makedirs(dirpath, exist_ok=True)

            with open(self._points_file, 'w') as f:
                yaml.dump(
                    data, f,
                    default_flow_style=False,
                    sort_keys=False,
                    allow_unicode=True
                )
            self.get_logger().info(
                f'Archivo guardado: {self._points_file}')
        except Exception as e:
            self.get_logger().error(
                f'Error escribiendo YAML: {e}')

    def _create_backup(self):
        """Crear backup del archivo YAML antes de modificarlo."""
        if os.path.isfile(self._points_file):
            try:
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                backup_path = f'{self._points_file}.bak.{timestamp}'
                shutil.copy2(self._points_file, backup_path)
                self.get_logger().debug(
                    f'Backup creado: {backup_path}')
            except Exception as e:
                self.get_logger().warn(
                    f'No se pudo crear backup: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PointRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
