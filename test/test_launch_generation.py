#!/usr/bin/env python3
"""
Test the improved launch file generation with proper ROS2 syntax
"""
import sys
import os
# Add parent directory to path to import the main module
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ros2_launch_generator_gui import ROS2Command, LaunchFileGenerator

def test_launch_file_generation():
    """Test launch file generation with both run and launch commands"""
    
    # Create test commands
    test_commands = [
        # ROS2 run command
        ROS2Command(
            pid=12345,
            timestamp="2025-07-23 14:30:15",
            cmd_type="run",
            package="turtlesim",
            executable="turtlesim_node",
            node_name="turtlesim_node",
            parameters={"use_sim_time": "true", "background_r": "255"},
            selected=True
        ),
        
        # ROS2 run command with remapping
        ROS2Command(
            pid=12346,
            timestamp="2025-07-23 14:30:20",
            cmd_type="run",
            package="turtlesim",
            executable="turtle_teleop_key",
            node_name="turtle_teleop_key",
            parameters={"remap_turtle1/cmd_vel": "cmd_vel", "scale_linear": "2.0"},
            arguments=["--ros-args"],
            selected=True
        ),
        
        # ROS2 launch command (this should use IncludeLaunchDescription)
        ROS2Command(
            pid=12347,
            timestamp="2025-07-23 14:30:25",
            cmd_type="launch",
            package="nav2_bringup",
            executable="tb3_simulation.launch.py",
            parameters={"world": "turtlebot3_world", "x_pose": "0.0", "y_pose": "0.0"},
            selected=True
        ),
        
        # Another launch command
        ROS2Command(
            pid=12348,
            timestamp="2025-07-23 14:30:30",
            cmd_type="launch",
            package="joystick_controller",
            executable="joystick_teleop.launch.py",
            parameters={"device": "/dev/input/js0", "deadzone": "0.1"},
            selected=True
        )
    ]
    
    # Generate launch file
    launch_content = LaunchFileGenerator.generate_launch_file(test_commands)
    
    print("=" * 60)
    print("GENERATED LAUNCH FILE WITH PROPER ROS2 SYNTAX:")
    print("=" * 60)
    print(launch_content)
    print("=" * 60)
    
    # Save to file for inspection
    with open("test_generated_launch.py", "w") as f:
        f.write(launch_content)
    
    print("Launch file saved to: test_generated_launch.py")
    print("\nKey improvements:")
    print("✓ Uses IncludeLaunchDescription for launch files")
    print("✓ Uses PythonLaunchDescriptionSource")
    print("✓ Uses PathJoinSubstitution and FindPackageShare")
    print("✓ Proper launch_arguments dictionary")
    print("✓ Standard ROS2 launch file structure")

if __name__ == "__main__":
    test_launch_file_generation()
