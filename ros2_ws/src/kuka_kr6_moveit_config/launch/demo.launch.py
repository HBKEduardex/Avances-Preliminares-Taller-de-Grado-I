import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    # Arguments
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="true", description="Open RViz2"
    )
    use_gui_arg = DeclareLaunchArgument(
        "use_gui", default_value="true", description="Open joint_state_publisher_gui"
    )
    use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware", default_value="false", description="Use fake hardware"
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation time"
    )
    fixed_frame_arg = DeclareLaunchArgument(
        "fixed_frame", default_value="world", description="RViz fixed frame"
    )
    planning_group_arg = DeclareLaunchArgument(
        "planning_group", default_value="manipulator", description="MoveIt planning group"
    )
    robot_model_arg = DeclareLaunchArgument(
        "robot_model", default_value="kr6r900sixx", description="Robot model"
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config", 
        default_value=PathJoinSubstitution([FindPackageShare("kuka_kr6_moveit_config"), "rviz", "moveit.rviz"]),
        description="RViz configuration file"
    )

    # Configurations
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Robot description from kuka_kr6_support
    xacro_file = PathJoinSubstitution([
        FindPackageShare("kuka_kr6_support"),
        "urdf",
        "kr6r900sixx.xacro"
    ])
    robot_description_content = ParameterValue(
        Command(["xacro ", xacro_file]), value_type=str
    )
    robot_description = {"robot_description": robot_description_content}

    # Semantic description (SRDF)
    srdf_file = PathJoinSubstitution([
        FindPackageShare("kuka_kr6_moveit_config"),
        "config",
        "kuka_kr6.srdf"
    ])
    robot_description_semantic_content = ParameterValue(
        Command(["cat ", srdf_file]), value_type=str
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    # Kinematics
    robot_description_kinematics = {"robot_description_kinematics": load_yaml("kuka_kr6_moveit_config", "config/kinematics.yaml")}
    
    # Planning
    ompl_planning_pipeline_config = load_yaml("kuka_kr6_moveit_config", "config/ompl_planning.yaml") or {}
    planning_pipeline = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": ompl_planning_pipeline_config,
    }

    # Trajectory Execution
    moveit_controllers = load_yaml("kuka_kr6_moveit_config", "config/moveit_controllers.yaml") or {}
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    # Planning Scene
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Nodes
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_pipeline,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
        ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # Initial joint positions for joint_state_publisher_gui
    initial_joint_positions = {
        "joint_a1": 0.080,
        "joint_a2": -1.609,
        "joint_a3": 1.603,
        "joint_a4": 0.0,
        "joint_a5": 0.0,
        "joint_a6": 0.0,
    }

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[{
            "zeros": initial_joint_positions,
            "use_sim_time": use_sim_time,
        }],
        condition=IfCondition(LaunchConfiguration("use_gui")),
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{
            "zeros": initial_joint_positions,
            "use_sim_time": use_sim_time,
            "source_list": ["/fake_joint_states"],
        }],
        condition=UnlessCondition(LaunchConfiguration("use_gui")),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_pipeline,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription([
        use_rviz_arg,
        use_gui_arg,
        use_fake_hardware_arg,
        use_sim_time_arg,
        fixed_frame_arg,
        planning_group_arg,
        robot_model_arg,
        rviz_config_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        move_group_node,
        rviz_node,
    ])
