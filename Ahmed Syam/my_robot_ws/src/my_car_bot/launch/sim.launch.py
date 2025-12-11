import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'my_car_bot'
    pkg_path = get_package_share_directory(pkg_name)

    # Process Xacro
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # Gazebo Sim (ros_gz_sim)
    world_file = os.path.join(pkg_path, 'worlds', 'road_with_obstacles.sdf')
    gaz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r "{world_file}"'}.items(),
    )

    # Spawn Entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-entity', 'my_car_bot',
                   '-z', '0.5'],
        output='screen'
    )

    # ROS-GZ Bridge
    # Bridge cmd_vel, odom, scan, and clock topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Robot Localization (EKF) to publish odom -> base_link transform
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'frequency': 30.0,
            'sensor_timeout': 0.1,
            'two_d_mode': True,
            'publish_tf': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_link_frame': 'base_link',
            'world_frame': 'odom',
            'odom0': '/odom',
            # odom0 configuration: X, Y, Z, R, P, Yaw, VX, VY, VZ, Vroll, Vpitch, Vyaw, AX, AY, AZ
            # Using X, Y, Yaw, VX, VY, Vyaw
            'odom0_config': [True, True, False,
                             False, False, True,
                             True, True, False,
                             False, False, True,
                             False, False, False],
        }]
    )

    # Lidar Frame Static Transform (Gazebo uses scoped name, ROS uses URDF name)
    # Parent: laser_frame (from URDF), Child: tank_bot/base_link/lidar (Gazebo frame_id in scan msg)
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'laser_frame', '--child-frame-id', 'tank_bot/base_link/lidar'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )


    # Teleop Term
    teleop_term = ExecuteProcess(
        cmd=['xterm', '-e', 'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard'],
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gaz_sim,
        spawn_entity,
        bridge,
        ekf_node,
        lidar_tf,
        teleop_term
    ])
