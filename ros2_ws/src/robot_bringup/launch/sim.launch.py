import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

REPO_ROOT = str(Path(__file__).resolve().parents[4])
TOOLS_DIR = os.path.join(REPO_ROOT, 'tools')


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_bringup')

    image_path_arg = DeclareLaunchArgument(
        'image_path',
        default_value=os.path.join(TOOLS_DIR, 'test_trash.jpg'),
        description='Image file for fake_camera — published as /camera/image_raw at 30 fps',
    )
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=os.path.join(REPO_ROOT, 'ml', 'models', 'trash_v3_best.onnx'),
        description='YOLO ONNX model path',
    )

    fake_hardware = ExecuteProcess(
        cmd=['python3', os.path.join(TOOLS_DIR, 'fake_hardware.py')],
        output='screen',
        name='fake_hardware',
    )

    fake_camera = ExecuteProcess(
        cmd=['python3', os.path.join(TOOLS_DIR, 'fake_camera.py'),
             LaunchConfiguration('image_path')],
        output='screen',
        name='fake_camera',
    )

    robot_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'beach_robot.launch.py')
        ),
        launch_arguments={
            'sim_mode': 'true',
            'model_path': LaunchConfiguration('model_path'),
        }.items(),
    )

    return LaunchDescription([
        image_path_arg,
        model_path_arg,
        fake_hardware,
        fake_camera,
        robot_nodes,
    ])
