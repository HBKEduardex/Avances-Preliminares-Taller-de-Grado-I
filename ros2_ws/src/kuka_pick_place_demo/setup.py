import os
from glob import glob
from setuptools import setup

package_name = 'kuka_pick_place_demo'

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
    description='Pick and place point recorder and sequence executor for KUKA KR6 R900',
    license='MIT',
    entry_points={
        'console_scripts': [
            'point_recorder_node = kuka_pick_place_demo.point_recorder_node:main',
            'pick_place_sequence_node = kuka_pick_place_demo.pick_place_sequence_node:main',
            'fake_trajectory_controller_node = kuka_pick_place_demo.fake_trajectory_controller_node:main',
        ],
    },
)
