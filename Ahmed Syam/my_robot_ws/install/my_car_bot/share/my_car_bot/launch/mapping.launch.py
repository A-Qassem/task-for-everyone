import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'my_car_bot'
    pkg_path = get_package_share_directory(pkg_name)
    
    # 1. Start Simulation (Sim, Robot, Bridge, XTerm)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'sim.launch.py')
        )
    )

    # 2. SLAM Toolbox (using standard launch file to handle lifecycle)
    slam_params_file = os.path.join(pkg_path, 'config', 'mapper_params_online_async.yaml')
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'slam_params_file': slam_params_file,
                          'use_sim_time': 'True'}.items()
    )

    # 3. RViz
    # Create simple rviz config if not exists, or just launch empty
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        sim_launch,
        slam_launch,
        rviz
    ])
