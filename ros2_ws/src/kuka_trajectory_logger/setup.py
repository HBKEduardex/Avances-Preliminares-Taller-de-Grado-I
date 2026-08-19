import os
from glob import glob
from setuptools import setup

package_name = 'kuka_trajectory_logger'

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
        'Registrador pasivo de las trayectorias planificadas por MoveIt2 '
        'para el KUKA KR6 R900. Terminal + CSV.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kuka_trajectory_logger_node = '
            'kuka_trajectory_logger.trajectory_logger_node:main',
        ],
    },
)
