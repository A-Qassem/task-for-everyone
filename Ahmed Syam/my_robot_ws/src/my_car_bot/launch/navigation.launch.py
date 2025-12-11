import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'my_car_bot'
    pkg_path = get_package_share_directory(pkg_name)
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file = LaunchConfiguration('map', default=os.path.join(pkg_path, 'maps', 'my_map.yaml'))
    nav2_params_file = LaunchConfiguration('params_file', default=os.path.join(pkg_path, 'config', 'nav2_params.yaml'))
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_map_file = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_path, 'maps', 'my_map.yaml'),
        description='Full path to map yaml file to load'
    )
    
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_path, 'config', 'nav2_params.yaml'),
        description='Full path to the Nav2 parameters file'
    )

    # 1. Start Simulation
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'sim.launch.py')
        )
    )

    # 2. Nav2 Bringup with remapping to publish directly to /cmd_vel
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file,
            'params_file': nav2_params_file,
            'autostart': 'true'
        }.items()
    )

    # 3. RViz with Nav2 config
    rviz_config_file = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz'
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 4. Simple cmd_vel relay node (subscribes to /cmd_vel_nav, publishes to /cmd_vel)
    cmd_vel_relay = Node(
        package='my_car_bot',
        executable='cmd_vel_relay',
        name='cmd_vel_relay',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map_file,
        declare_params_file,
        sim_launch,
        nav2_bringup_launch,
        rviz,
        cmd_vel_relay
    ])
