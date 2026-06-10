import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_bringup')
    params_file = os.path.join(pkg_share, 'config', 'robot_params.yaml')

    # --- Launch arguments ---
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/ttkan/AutonomousBeachRobot/ml/models/trash_v1_best.onnx',
        description='Absolute path to the YOLO ONNX model file',
    )
    # mission_fsm PICKUP log — one row per flagged item.
    fsm_log_path_arg = DeclareLaunchArgument(
        'fsm_log_path',
        default_value='trash_detections_log.csv',
        description='CSV path for mission_fsm PICKUP events',
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
        ],
        output='screen',
    )

    coordinator_node = Node(
        package='mission_control',
        executable='coordinator',
        name='coordinator',
        output='screen',
    )

    return LaunchDescription([
        model_path_arg,
        fsm_log_path_arg,
        trash_detector_node,
        terrain_monitor_node,
        mission_fsm_node,
        coordinator_node,
    ])
