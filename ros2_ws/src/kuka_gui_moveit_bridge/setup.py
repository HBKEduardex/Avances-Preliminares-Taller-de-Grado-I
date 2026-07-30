import os
from glob import glob
from setuptools import setup

package_name = 'kuka_gui_moveit_bridge'

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
        'Puente entre GUI externa y MoveIt/RViz para KUKA KR6. '
        'Incluye una GUI interna de prueba en Tkinter.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kuka_moveit_bridge_node = '
            'kuka_gui_moveit_bridge.kuka_moveit_bridge_node:main',
            'kuka_bridge_test_gui_node = '
            'kuka_gui_moveit_bridge.kuka_bridge_test_gui_node:main',
        ],
    },
)
