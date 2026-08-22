import os
from glob import glob
from setuptools import setup

package_name = 'kuka_moveit_trajectory_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eduardex',
    maintainer_email='eduardex@todo.com',
    description=(
        'Generacion (solo plan) y previsualizacion en RViz2 de trayectorias '
        'MoveIt2 para el KUKA KR6 R900, con contrato JSON sobre std_msgs/String.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kuka_trajectory_generator_node = '
            'kuka_moveit_trajectory_planner.trajectory_generator_node:main',
            'kuka_trajectory_preview_node = '
            'kuka_moveit_trajectory_planner.trajectory_preview_node:main',
        ],
    },
)
