#!/usr/bin/env python3
"""
ROS2 Launch File Generator - Usage Examples and Demo

This script demonstrates how to use the ROS2 Launch File Generator tool.
"""

import os
import sys

def print_banner():
    print("=" * 60)
    print("  ROS2 Launch File Generator GUI Tool")
    print("=" * 60)
    print()

def print_usage():
    print("USAGE:")
    print("------")
    print("1. Start the GUI tool:")
    print("   ./launch_gui.sh")
    print("   or")
    print("   python3 ros2_launch_generator_gui.py")
    print()
    print("2. Test monitoring functionality:")
    print("   python3 test_monitor.py")
    print()
    print("3. Use original command monitor:")
    print("   python3 command_monitor.py [--follow]")
    print()

def print_features():
    print("FEATURES:")
    print("---------")
    print("✓ Real-time monitoring of ROS2 run and launch commands")
    print("✓ Automatic extraction of PID, package, executable, parameters")
    print("✓ Interactive GUI with command selection")
    print("✓ Live updates - removes terminated processes automatically")
    print("✓ Generate proper ROS2 launch files from selected commands")
    print("✓ Preview and save generated launch files")
    print("✓ Support for parameters, arguments, and remappings")
    print()

def print_workflow():
    print("WORKFLOW:")
    print("---------")
    print("1. Start the GUI tool")
    print("2. Run your ROS2 commands in any terminal:")
    print("   ros2 run turtlesim turtlesim_node")
    print("   ros2 run turtlesim turtle_teleop_key")
    print("   ros2 launch nav2_bringup tb3_simulation.launch.py")
    print("3. Commands appear automatically in the GUI")
    print("4. Select/deselect commands as needed")
    print("5. Click 'Generate Launch File'")
    print("6. Preview and save the generated launch file")
    print()

def print_requirements():
    print("REQUIREMENTS:")
    print("-------------")
    print("✓ Python 3.6+")
    print("✓ psutil package (pip install psutil)")
    print("✓ tkinter (usually included, or: sudo apt-get install python3-tk)")
    print("✓ ROS2 environment (for testing)")
    print()

def check_dependencies():
    print("DEPENDENCY CHECK:")
    print("-----------------")
    
    # Check Python version
    version = sys.version_info
    if version.major >= 3 and version.minor >= 6:
        print("✓ Python version:", f"{version.major}.{version.minor}.{version.micro}")
    else:
        print("✗ Python version too old:", f"{version.major}.{version.minor}.{version.micro}")
    
    # Check psutil
    try:
        import psutil
        print("✓ psutil:", psutil.__version__)
    except ImportError:
        print("✗ psutil not found - install with: pip install psutil")
    
    # Check tkinter
    try:
        import tkinter
        print("✓ tkinter available")
    except ImportError:
        print("✗ tkinter not found - install with: sudo apt-get install python3-tk")
    
    # Check file permissions
    current_dir = os.path.dirname(os.path.abspath(__file__))
    gui_file = os.path.join(current_dir, "ros2_launch_generator_gui.py")
    if os.path.exists(gui_file):
        print("✓ GUI file found:", gui_file)
    else:
        print("✗ GUI file not found:", gui_file)
    
    print()

def main():
    print_banner()
    print_features()
    check_dependencies()
    print_requirements()
    print_usage()
    print_workflow()
    
    print("FILES CREATED:")
    print("--------------")
    current_dir = os.path.dirname(os.path.abspath(__file__))
    files = [
        "ros2_launch_generator_gui.py",
        "command_monitor.py", 
        "test_monitor.py",
        "launch_gui.sh",
        "requirements.txt",
        "README.md"
    ]
    
    for file in files:
        file_path = os.path.join(current_dir, file)
        if os.path.exists(file_path):
            print(f"✓ {file}")
        else:
            print(f"✗ {file}")
    
    print()
    print("Ready to use! Start with: ./launch_gui.sh")

if __name__ == "__main__":
    main()
