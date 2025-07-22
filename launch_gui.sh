#!/bin/bash

# ROS2 Launch Generator GUI Launcher Script

# Check if tkinter is available
python3 -c "import tkinter" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "Error: tkinter is not available."
    echo "Please install it with: sudo apt-get install python3-tk"
    exit 1
fi

# Check if psutil is available
python3 -c "import psutil" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "Installing psutil..."
    pip3 install psutil
fi

# Get the directory of this script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

echo "Starting ROS2 Launch File Generator GUI..."
echo "Monitor will detect 'ros2 run' and 'ros2 launch' commands automatically."
echo "Press Ctrl+C to exit."

# Run the GUI application
python3 "$DIR/ros2_launch_generator_gui.py"
