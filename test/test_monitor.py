#!/usr/bin/env python3
"""
Test script for ROS2 command monitoring functionality
Run this to verify that the monitoring logic works correctly
"""

import sys
import os
# Add parent directory to path to import the main module
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import time
import threading
from ros2_launch_generator_gui import ROS2CommandMonitor

def test_monitor():
    """Test the ROS2 command monitoring"""
    print("Testing ROS2 Command Monitor...")
    print("This will monitor for ROS2 commands for 30 seconds.")
    print("In another terminal, try running:")
    print("  ros2 run turtlesim turtlesim_node")
    print("  ros2 launch nav2_bringup tb3_simulation.launch.py")
    print("Press Ctrl+C to stop early.\n")
    
    monitor = ROS2CommandMonitor()
    monitor.start_monitoring()
    
    try:
        for i in range(30):
            commands = monitor.get_commands()
            if commands:
                print(f"\n=== Detected Commands ({len(commands)}) ===")
                for cmd in commands:
                    print(f"PID: {cmd.pid}")
                    print(f"  Type: {cmd.cmd_type}")
                    print(f"  Package: {cmd.package}")
                    print(f"  Executable: {cmd.executable}")
                    print(f"  Parameters: {cmd.parameters}")
                    print(f"  Arguments: {cmd.arguments}")
                    print(f"  Timestamp: {cmd.timestamp}")
                    print()
            else:
                print(f"Monitoring... ({30-i}s remaining, {len(commands)} commands detected)")
            
            time.sleep(1)
    
    except KeyboardInterrupt:
        print("\nStopping monitor...")
    
    finally:
        monitor.stop_monitoring()
        print("Monitor stopped.")

if __name__ == "__main__":
    test_monitor()
