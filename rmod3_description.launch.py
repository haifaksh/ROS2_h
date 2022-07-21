import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command




def generate_launch_description():
    # Get the URDF file absolute path
    urdf_file = "range.urdf"
    package_description = "rmod3_description_pkg"
    robot_des_path = os.path.join(
        get_package_share_directory(package_description),
        "urdf",
        urdf_file
    )

    rviz_config_dir = os.path.join(
        get_package_share_directory(package_description),
        'rviz',
        'urdf_vis.rviz'
    )

    rmod3_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="rmod3_state_publisher",
        emulate_tty=True,
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command(['xacro ', robot_des_path])
        }],
        output="screen"
    )

    rmod3_joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="rmod3_joint_state_publisher",
        arguments=[robot_des_path]
    )

    rmod3_joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="rmod3_joint_state_publisher_gui",
        arguments=[robot_des_path]
    )

    rmod3_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir]
    )

    return LaunchDescription([
        rmod3_robot_state_publisher,
        rmod3_joint_state_publisher,
        #rmod3_joint_state_publisher_gui,
        #rmod3_rviz
    ])