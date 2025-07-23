#!/usr/bin/env python3
"""
Quick Selection Test for ROS2 Launch Generator GUI

This script provides a quick way to test the selection functionality
by creating some mock commands and launching the GUI.
"""

import sys
import time
import threading
from ros2_launch_generator_gui import ROS2Command, ROS2LaunchGeneratorGUI
import tkinter as tk

def create_test_commands():
    """Create some test commands for demonstration"""
    test_commands = [
        ROS2Command(
            pid=12345,
            timestamp="2025-07-23 14:30:15",
            cmd_type="run",
            package="turtlesim",
            executable="turtlesim_node",
            node_name="turtlesim_node",
            selected=True
        ),
        ROS2Command(
            pid=12346,
            timestamp="2025-07-23 14:30:20",
            cmd_type="run",
            package="turtlesim",
            executable="turtle_teleop_key",
            node_name="turtle_teleop_key",
            parameters={"use_sim_time": "true"},
            selected=False
        ),
        ROS2Command(
            pid=12347,
            timestamp="2025-07-23 14:30:25",
            cmd_type="launch",
            package="nav2_bringup",
            executable="tb3_simulation.launch.py",
            parameters={"world": "turtlebot3_world", "x_pose": "0.0"},
            arguments=["--verbose"],
            selected=True
        ),
        ROS2Command(
            pid=12348,
            timestamp="2025-07-23 14:30:30",
            cmd_type="run",
            package="rviz2",
            executable="rviz2",
            node_name="rviz2",
            parameters={"config": "/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz"},
            selected=False
        )
    ]
    
    return test_commands

class TestGUI(ROS2LaunchGeneratorGUI):
    """Test version of the GUI with mock commands"""
    
    def __init__(self, root):
        super().__init__(root)
        
        # Add test commands
        test_commands = create_test_commands()
        for cmd in test_commands:
            self.monitor.commands[cmd.pid] = cmd
        
        # Force initial refresh
        self.force_refresh()

def main():
    """Main function for testing"""
    print("Starting ROS2 Launch Generator GUI with test commands...")
    print("This includes sample commands to test selection functionality.")
    print("\nTest Commands:")
    print("1. turtlesim_node (selected)")
    print("2. turtle_teleop_key (not selected)")
    print("3. nav2 launch file (selected)")
    print("4. rviz2 (not selected)")
    print("\nTry the following:")
    print("- Double-click on commands to toggle selection")
    print("- Use Select All / Deselect All buttons")
    print("- Use keyboard Enter/Space to toggle selection")
    print("- Generate launch file to see the result")
    print()
    
    root = tk.Tk()
    app = TestGUI(root)
    
    # Handle window closing
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        app.on_closing()

if __name__ == "__main__":
    main()
