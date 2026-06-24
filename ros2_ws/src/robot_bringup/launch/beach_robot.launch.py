import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_bringup')
    params_file = os.path.join(pkg_share, 'config', 'robot_params.yaml')

    # --- Launch arguments ---
    _repo_root = str(Path(__file__).resolve().parents[4])
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=os.path.join(_repo_root, 'ml', 'models', 'trash_v3_best.onnx'),
        description='Absolute path to the YOLO ONNX model file',
    )
    # mission_fsm PICKUP log — one row per flagged item.
    fsm_log_path_arg = DeclareLaunchArgument(
        'fsm_log_path',
        default_value='trash_detections_log.csv',
        description='CSV path for mission_fsm PICKUP events',
    )
    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='false',
        description='Set true when running with fake_hardware.py — disables STUCK watchdog',
    )
    # --- Nodes ---
    trash_detector_node = Node(
        package='perception',
        executable='trash_detector',
        name='trash_detector',
        parameters=[
            params_file,
            {'model_path': LaunchConfiguration('model_path')},
        ],
        output='screen',
    )

    terrain_monitor_node = Node(
        package='perception',
        executable='terrain_monitor',
        name='terrain_monitor',
        parameters=[params_file],
        output='screen',
    )

    mission_fsm_node = Node(
        package='v1_navigator',
        executable='mission_fsm',
        name='mission_fsm',
        parameters=[
            params_file,
            {'csv_log_path': LaunchConfiguration('fsm_log_path')},
            {'sim_mode': LaunchConfiguration('sim_mode')},
        ],
        output='screen',
    )

    coordinator_node = Node(
        package='mission_control',
        executable='coordinator',
        name='coordinator',
        output='screen',
    )

    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        parameters=[{'image_size': [640, 480]}],
        output='screen',
    )

    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyUSB0', '-b', '115200'],
        output='screen',
    )

    return LaunchDescription([
        model_path_arg,
        fsm_log_path_arg,
        sim_mode_arg,
        trash_detector_node,
        terrain_monitor_node,
        mission_fsm_node,
        coordinator_node,   
        camera_node,
        micro_ros_agent_node,
    ])
