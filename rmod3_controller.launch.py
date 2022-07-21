from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rmod3_follow_wall = Node(
        package="rmod3_controller_pkg",
        executable="RmodControl.py",
        name="RmodControl",
        output="screen"
    )

    rmod3_turn = Node(
        package="rmod3_controller_pkg",
        executable="turn_robot.py",
        name="turn_robot",
        output="screen"
    )

    return LaunchDescription([
        rmod3_follow_wall,
        #rmod3_turn
    ])