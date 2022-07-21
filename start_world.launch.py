import os
import random
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_prefix
from glob import glob


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_rmod3_gazebo = get_package_share_directory('rmod3_gazebo_pkg')

    description_package_name = 'rmod3_description_pkg'
    install_dir = get_package_prefix(description_package_name)

    gazebo_models_path = os.path.join(pkg_rmod3_gazebo, 'models')

    position = [0.0, 0.0, 0.2]  # X, Y and Z
    orientation = [0.0, 0.0, 0.0]   # roll, pitch, yaw
    robot_base_name = "rmod3"

    entity_name = robot_base_name+"_"+str(int(random.random()*100000))

    nav2_yaml = os.path.join(
        pkg_rmod3_gazebo,
        'config',
        'amcl_config.yaml'
    )

    map_file = os.path.join(
        pkg_rmod3_gazebo,
        'config',
        'map_world.yaml'
    )
    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    # Spawn robot
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_entity",
        output="screen",
        arguments=[
            '-entity',
            entity_name,
            '-x', str(position[0]),
            '-y', str(position[1]),
            '-z', str(position[2]),
            '-R', str(orientation[0]),
            '-P', str(orientation[1]),
            '-Y', str(orientation[2]),
            '-topic', '/robot_description'
        ]
    )

    """
    ROS2 uses AMCL (Adaptive Monte-Carlo Localization) algorithm for localization.
    """
    # map_server provides the map to the localization algorithm
    
    rmod3_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        parameters=[{
            'use_sim_time': True},
            {'yaml_filename': map_file}
            ]
    ),

    # localization algorithm
    rmod3_amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        output="screen",
        parameters=[nav2_yaml]
    ),

    # this node manages the lifecycle of the nodes involved in the navigation
    rmod3_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names':['map_server', 'amcl']}
        ]
    )

    
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(pkg_rmod3_gazebo, 'worlds', 'rmod3.world'), ''],
            description='SDF world file'
        ),

        gazebo,
        spawn_robot,
    ])

