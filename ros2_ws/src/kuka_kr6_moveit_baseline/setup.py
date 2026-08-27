import os
from glob import glob
from setuptools import setup

package_name = 'kuka_kr6_moveit_baseline'


def config_files():
    """YAML y SRDF de la configuracion base."""
    return (glob(os.path.join('config', '*.yaml'))
            + glob(os.path.join('config', '*.srdf')))  # noqa: W503


def mesh_files():
    """Instala las mallas conservando la estructura de subcarpetas."""
    entries = []
    for root, _dirs, files in os.walk('meshes'):
        if not files:
            continue
        entries.append((os.path.join('share', package_name, root),
                        [os.path.join(root, f) for f in files]))
    return entries


setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md',
                                   'BASELINE_METADATA.yaml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), config_files()),
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*.xacro'))),
        (os.path.join('share', package_name, 'rviz'),
            glob(os.path.join('rviz', '*.rviz'))),
    ] + mesh_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eduardex',
    maintainer_email='adrian.vargas@ucb.edu.bo',
    description=(
        'Condicion MoveIt2 base del estudio comparativo del Taller de Grado 2. '
        'Solo planifica y visualiza; no ejecuta movimiento fisico.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kr6_baseline_test_gui = '
            'kuka_kr6_moveit_baseline.baseline_test_gui_node:main',
            'kr6_baseline_continuity_analyzer = '
            'kuka_kr6_moveit_baseline.continuity_analyzer_node:main',
            'kr6_baseline_replan = '
            'kuka_kr6_moveit_baseline.baseline_replan_node:main',
            'kr6_baseline_verify_fk = '
            'kuka_kr6_moveit_baseline.verify_fk_node:main',
            'kr6_baseline_json_preview = '
            'kuka_kr6_moveit_baseline.json_preview_node:main',
        ],
    },
)
