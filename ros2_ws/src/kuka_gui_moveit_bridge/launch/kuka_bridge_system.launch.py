import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # ── Rutas de paquetes ────────────────────────────────────────────────────
    kuka_bridge_dir = get_package_share_directory('kuka_gui_moveit_bridge')
    kuka_moveit_dir = get_package_share_directory('kuka_kr6_moveit_config')

    bridge_config_file = os.path.join(
        kuka_bridge_dir, 'config', 'kuka_bridge.yaml')
    gui_config_file = os.path.join(
        kuka_bridge_dir, 'config', 'kuka_test_gui.yaml')

    demo_launch_file = os.path.join(
        kuka_moveit_dir, 'launch', 'demo.launch.py')

    # ── Argumentos del launch ────────────────────────────────────────────────
    # Argumento para habilitar o deshabilitar la GUI interna de pruebas
    use_test_gui_arg = DeclareLaunchArgument(
        'use_test_gui',
        default_value='true',
        description='Inicia la GUI interna de pruebas (Tkinter)'
    )

    # ── Nodos y Launch files ─────────────────────────────────────────────────

    # 1. kuka_kr6_moveit_config/demo.launch.py
    # RViz=true, GUI=false (deshabilita el joint_state_publisher_gui)
    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(demo_launch_file),
        launch_arguments={
            'use_gui': 'false',
            'use_rviz': 'true'
        }.items()
    )

    # 2. Fake trajectory controller
    fake_controller_node = Node(
        package='kuka_pick_place_demo',
        executable='fake_trajectory_controller_node',
        name='fake_trajectory_controller_node',
        output='both'
    )

    # 3. Nuestro puente (Bridge Node)
    bridge_node = Node(
        package='kuka_gui_moveit_bridge',
        executable='kuka_moveit_bridge_node',
        name='kuka_moveit_bridge_node',
        output='both',
        parameters=[bridge_config_file]
    )

    # 4. GUI de pruebas opcional (Test GUI Node)
    test_gui_node = Node(
        package='kuka_gui_moveit_bridge',
        executable='kuka_bridge_test_gui_node',
        name='kuka_bridge_test_gui_node',
        output='both',
        parameters=[gui_config_file],
        condition=IfCondition(LaunchConfiguration('use_test_gui'))
    )

    return LaunchDescription([
        use_test_gui_arg,
        moveit_rviz_launch,
        fake_controller_node,
        bridge_node,
        test_gui_node
    ])
