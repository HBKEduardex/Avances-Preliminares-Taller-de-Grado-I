#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
apply_environment_poses.py
──────────────────────────
Lee  config/environment_poses.yaml  y sobreescribe los valores
default= de las propiedades xacro en  urdf/environment.xacro.

Uso:
    python3 scripts/apply_environment_poses.py

No requiere ROS2 ni colcon; trabaja con rutas relativas al paquete.
"""

import os
import re
import sys
import yaml


def find_package_root():
    """Devuelve la raíz del paquete kuka_kr6_support."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # scripts/ está dentro de la raíz del paquete
    pkg_root = os.path.dirname(script_dir)
    if not os.path.isfile(os.path.join(pkg_root, "package.xml")):
        sys.exit(f"ERROR: No se encontró package.xml en {pkg_root}")
    return pkg_root


# ── Mapeo: clave YAML → nombre de propiedad xacro ──────────────────
PROP_MAP = {
    "kuka":    {"x": "kuka_x",     "y": "kuka_y",     "z": "kuka_z",
                "roll": "kuka_R",   "pitch": "kuka_P",   "yaw": "kuka_Y"},
    "base":    {"x": "base_env_x", "y": "base_env_y", "z": "base_env_z",
                "roll": "base_env_R", "pitch": "base_env_P", "yaw": "base_env_Y"},
    "mesa":    {"x": "mesa_x",     "y": "mesa_y",     "z": "mesa_z",
                "roll": "mesa_R",   "pitch": "mesa_P",   "yaw": "mesa_Y"},
    "gripper": {"x": "gripper_x",  "y": "gripper_y",  "z": "gripper_z",
                "roll": "gripper_R","pitch": "gripper_P","yaw": "gripper_Y"},
    "cubo":    {"x": "cubo_x",     "y": "cubo_y",     "z": "cubo_z",
                "roll": "cubo_R",   "pitch": "cubo_P",   "yaw": "cubo_Y"},
}


def load_poses(yaml_path):
    """Carga el YAML y devuelve el dict."""
    with open(yaml_path, "r") as f:
        return yaml.safe_load(f)


def apply_to_xacro(xacro_path, poses):
    """
    Reemplaza los default="..." de cada <xacro:property> en el xacro
    usando los valores del dict de poses.
    """
    with open(xacro_path, "r") as f:
        content = f.read()

    for piece, axes in PROP_MAP.items():
        piece_data = poses.get(piece, {})
        for axis_key, prop_name in axes.items():
            value = piece_data.get(axis_key, 0.0)
            # Formatear: quitar ceros innecesarios pero mantener al menos un decimal
            formatted = f"{float(value):.6g}"

            # Patrón:  <xacro:property name="prop_name"  default="..."/>
            pattern = (
                r'(<xacro:property\s+name="'
                + re.escape(prop_name)
                + r'"\s+default=")[^"]*(")'
            )
            replacement = rf"\g<1>{formatted}\2"
            content, count = re.subn(pattern, replacement, content)
            if count == 0:
                print(f"  ADVERTENCIA: propiedad '{prop_name}' no encontrada en el xacro.")

    with open(xacro_path, "w") as f:
        f.write(content)


def main():
    pkg_root = find_package_root()
    yaml_path  = os.path.join(pkg_root, "config", "environment_poses.yaml")
    xacro_path = os.path.join(pkg_root, "urdf",   "environment.xacro")

    if not os.path.isfile(yaml_path):
        sys.exit(f"ERROR: No se encontró {yaml_path}")
    if not os.path.isfile(xacro_path):
        sys.exit(f"ERROR: No se encontró {xacro_path}")

    poses = load_poses(yaml_path)
    apply_to_xacro(xacro_path, poses)

    print("✔ environment.xacro actualizado con los valores de environment_poses.yaml")
    print("  Para ver los cambios en RViz, relanza:")
    print("    colcon build --packages-select kuka_kr6_support && source install/setup.bash")
    print("    ros2 launch kuka_kr6_support display.launch.py")


if __name__ == "__main__":
    main()
