# ROS2 Launch File Generator GUI

A GUI-based tool for automatically generating ROS2 launch files by monitoring running ROS2 commands.

## Features

- **Real-time Monitoring**: Continuously monitors all running and new terminals for `ros2 run` and `ros2 launch` commands
- **Command Extraction**: Automatically extracts PID, command type, package, executable, node name, arguments, and parameters
- **Interactive GUI**: Organized display of detected commands with selection capabilities
- **Live Updates**: Commands are constantly refreshed and updated, removing entries for processes that no longer exist
- **Launch File Generation**: Generate proper ROS2 launch files from selected commands
- **Preview and Save**: Preview generated launch files before saving to disk

## 🎥 Video Demo

[![ROS2 Launch File Generator Demo](https://img.youtube.com/vi/u5tqTKkaqgo/0.jpg)](https://youtu.be/u5tqTKkaqgo)

**[Watch the Demo on YouTube](https://youtu.be/u5tqTKkaqgo)**

See the tool in action! This video demonstrates:
- Real-time monitoring of ROS2 commands
- Interactive GUI for selecting commands
- Launch file generation with proper ROS2 syntax
- Complete workflow from monitoring to launch file creation

## Installation

1. Ensure you have Python 3.6+ installed
2. Install required dependencies:
   ```bash
   pip install -r requirements.txt
   ```
3. If you encounter tkinter issues on Linux:
   ```bash
   sudo apt-get install python3-tk
   ```

## Usage

### Starting the GUI Tool

```bash
python3 ros2_launch_generator_gui.py
```

### Testing the Command Monitoring

To test the command monitoring functionality. This should output ROS2 commands which you executed on a seperate terminal:

```bash
python3 test/command_monitor.py
```

This will launch the GUI with sample commands to test the selection interface.

### Testing Launch File Generation

To test the launch file generation with proper ROS2 syntax:

```bash
python3 test/test_launch_generation.py
```

This will generate a sample launch file demonstrating the correct syntax for both Node and IncludeLaunchDescription actions.

### Using the Tool

1. **Start Monitoring**: The tool automatically starts monitoring ROS2 commands when launched
2. **Run ROS2 Commands**: Execute your ROS2 commands in any terminal:
   ```bash
   ros2 run turtlesim turtlesim_node
   ros2 run turtlesim turtle_teleop_key
   ros2 launch nav2_bringup tb3_simulation.launch.py
   ```
3. **View Detected Commands**: Commands appear automatically in the GUI table
4. **Select/Deselect Commands**: 
   - **Double-click** any row to toggle selection
   - **Click** a row and press **Enter** or **Space** to toggle
   - Use **Select All** or **Deselect All** buttons
   - Selected commands show ✓, unselected show ○
5. **Generate Launch File**: Click "Generate Launch File" to create a launch file from selected commands
6. **Preview and Save**: Review the generated launch file in the preview pane and save to disk

### GUI Controls

- **Clear All**: Remove all monitored commands
- **Select All**: Select all commands for launch file generation
- **Deselect All**: Deselect all commands
- **Manual Refresh**: Manually refresh the command list (auto-refreshes every 2 seconds)
- **Generate Launch File**: Create launch file from selected commands
- **Save Launch File**: Save the previewed launch file to disk

### Selection Methods

1. **Double-click**: Double-click any row in the table to toggle its selection
2. **Keyboard**: Click a row to select it, then press Enter or Space to toggle selection
3. **Batch operations**: Use Select All/Deselect All buttons for bulk operations
4. **Visual feedback**: Selected commands show ✓, unselected show ○ in the first column
5. **Status display**: Bottom of control panel shows total and selected command counts

### Command Information Displayed

- **PID**: Process ID of the ROS2 command
- **Time**: When the command was detected
- **Type**: Command type (`run` or `launch`)
- **Package**: ROS2 package name
- **Executable**: Executable or launch file name
- **Parameters**: Arguments and parameters passed to the command

## Generated Launch File Format

The tool generates standard ROS2 launch files with proper syntax:

### For `ros2 run` commands:
- **Node declarations** with proper package, executable, and name
- **Parameter dictionaries** with correct data types (bool, int, string)
- **Remappings** extracted from command line arguments
- **Arguments** passed to the executable
- **Output redirection** to screen

### For `ros2 launch` commands:
- **IncludeLaunchDescription** actions (not ExecuteProcess)
- **PythonLaunchDescriptionSource** for .py launch files
- **PathJoinSubstitution** with FindPackageShare for proper path resolution
- **launch_arguments** dictionary for passing parameters
- **Standard ROS2 launch composition** following best practices

### Key Features:
- ✅ **Proper ROS2 syntax** - Uses IncludeLaunchDescription instead of ExecuteProcess
- ✅ **Parameter type detection** - Automatically detects bool, int, and string types
- ✅ **Remapping support** - Extracts and applies topic/service remappings
- ✅ **Package path resolution** - Uses FindPackageShare for robust package finding
- ✅ **Launch arguments** - Properly passes parameters to included launch files
- ✅ **Comments with metadata** - Includes source PID and timestamp for traceability

### Example Generated Launch File

```python
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
                "use_sim_time": true,
                "background_r": 255
            }],
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
                "world": "turtlebot3_world",
                "x_pose": "0.0",
                "y_pose": "0.0"
            },
        )
    )

    return LaunchDescription(declared_arguments + nodes_and_processes)
```

## Technical Details

### Command Detection

The tool uses `psutil` to monitor system processes and detects ROS2 commands by:
1. Scanning process command lines for `ros2` executable
2. Identifying `run` or `launch` subcommands
3. Parsing package names, executables, and arguments

### Parameter Parsing

Supports various ROS2 parameter formats:
- `-p parameter_name:=value`
- `--param parameter_name:=value`
- `parameter_name:=value`
- `-r old_name:=new_name` (remappings)
- `--remap old_name:=new_name`

### Process Management

- Continuously monitors for new processes
- Automatically removes entries for terminated processes
- Maintains process state across GUI updates

## File Structure

```
ros2_launch_file_generator/
├── ros2_launch_generator_gui.py    # Main GUI application
├── launch_gui.sh                   # Launcher script with dependency checks
├── requirements.txt                # Python dependencies
├── README.md                      # This documentation file
├── LICENSE                        # License file
├── test/                          # Test files and utilities
│   ├── command_monitor.py         # Original command monitoring script
│   ├── test_monitor.py            # Test monitoring functionality
│   ├── test_selection.py          # Test GUI selection with mock data
│   ├── test_launch_generation.py  # Test launch file generation
│   ├── demo.py                    # Demo and setup verification script
│   └── test_generated_launch.py   # Example generated launch file

```

## Troubleshooting

### Common Issues

1. **tkinter not found**: Install with `sudo apt-get install python3-tk`
2. **Permission errors**: Ensure the user has permission to read process information
3. **No commands detected**: Verify ROS2 commands are running and contain the expected format

### Debug Mode

You can monitor the console output for debugging information when commands are detected or removed.

## 📝 TODO

- [ ] **Node name detection improvement** - Better extraction of custom node names from command line arguments
- [ ] **XML launch file support** - Add support for detecting and including .xml launch files
- [ ] **Launch argument validation** - Validate that detected parameters match the target launch file's declared arguments
- [ ] **Namespace detection** - Better handling of ROS2 namespaces in monitored commands
- [ ] **Conditional launch logic** - Generate launch files with conditional node starting
- [ ] **Parameter file generation** - Generate separate YAML parameter files
- [ ] **Launch file validation** - Validate generated launch files against ROS2 launch syntax
- [ ] **Real-time parameter editing** - Allow editing parameters before generating launch files

## Contributing

Feel free to submit issues, feature requests, or pull requests to improve the tool.

## License

This project is open source. Please check the license file for details.
