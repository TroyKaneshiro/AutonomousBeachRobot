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
    # model_path differs per machine; make it easy to override without editing
    # source files or the params YAML.
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/ttkan/AutonomousBeachRobot/ml/models/trash_v1_best.onnx',
        description='Absolute path to the YOLO ONNX model file',
    )
    # CSV log written by mission_fsm on each PICKUP event.
    log_path_arg = DeclareLaunchArgument(
        'log_path',
        default_value='trash_detections_log.csv',
        description='Path for the CSV detection log',
    )

    # --- Nodes ---
    trash_detector_node = Node(
        package='perception',
        executable='trash_detector',
        name='trash_detector',
        parameters=[
            params_file,
            # Override model_path from the launch arg so it can be set at
            # the command line without touching the params YAML.
            {'model_path': LaunchConfiguration('model_path')},
        ],
        output='screen',
    )

    mission_fsm_node = Node(
        package='v1_navigator',
        executable='mission_fsm',
        name='mission_fsm',
        parameters=[
            params_file,
            {'csv_log_path': LaunchConfiguration('log_path')},
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
        log_path_arg,
        trash_detector_node,
        mission_fsm_node,
        coordinator_node,
    ])
