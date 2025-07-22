import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """Generated launch file from monitored ROS2 commands"""
    
    # Launch arguments
    declared_arguments = []
    
    # Nodes and processes
    nodes_and_processes = []

    # Node from PID 12345 - 2025-07-23 14:30:15
    nodes_and_processes.append(
        Node(
            package="turtlesim",
            executable="turtlesim_node",
            name="turtlesim_node",
        parameters=[{
            "use_sim_time": true,            "background_r": 255
        }],
            output="screen"
        )
    )
    # Node from PID 12346 - 2025-07-23 14:30:20
    nodes_and_processes.append(
        Node(
            package="turtlesim",
            executable="turtle_teleop_key",
            name="turtle_teleop_key",
        parameters=[{
            "scale_linear": 2.0
        }],
        remappings=[
            ("turtle1/cmd_vel", "cmd_vel")
        ],
        arguments=["--ros-args"],
            output="screen"
        )
    )
    # Launch file from PID 12347 - 2025-07-23 14:30:25
    nodes_and_processes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("nav2_bringup"),
                    "launch",
                    "tb3_simulation.launch.py"
                ])
            ]),
            launch_arguments={
                "world": "turtlebot3_world",                "x_pose": "0.0",                "y_pose": "0.0"
            },
        )
    )
    # Launch file from PID 12348 - 2025-07-23 14:30:30
    nodes_and_processes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("joystick_controller"),
                    "launch",
                    "joystick_teleop.launch.py"
                ])
            ]),
            launch_arguments={
                "device": "/dev/input/js0",                "deadzone": "0.1"
            },
        )
    )

    return LaunchDescription(declared_arguments + nodes_and_processes)
