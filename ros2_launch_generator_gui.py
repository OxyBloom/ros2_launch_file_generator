#!/usr/bin/env python3
"""
ROS2 Launch File Generator GUI Tool

This tool monitors running ROS2 commands and provides a GUI interface
for generating launch files based on the detected commands.
"""

import sys
import psutil
import time
import threading
from datetime import datetime
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Set
from pathlib import Path

try:
    import tkinter as tk
    from tkinter import ttk, messagebox, filedialog, scrolledtext
except ImportError:
    print("Error: tkinter is not available. Please install python3-tk")
    sys.exit(1)


@dataclass
class ROS2Command:
    """Data class to store ROS2 command information"""
    pid: int
    timestamp: str
    cmd_type: str  # 'run' or 'launch'
    package: str
    executable: str = ""
    node_name: str = ""
    arguments: List[str] = field(default_factory=list)
    parameters: Dict[str, str] = field(default_factory=dict)
    namespace: str = ""
    selected: bool = True
    
    def __str__(self):
        return f"PID:{self.pid} {self.cmd_type} {self.package}/{self.executable}"


class ROS2CommandMonitor:
    """Monitor for ROS2 commands running on the system"""
    
    def __init__(self):
        self.seen_pids: Set[int] = set()
        self.commands: Dict[int, ROS2Command] = {}
        self.monitoring = False
        self.monitor_thread = None
        
    def is_ros2_command(self, cmdline: List[str]) -> bool:
        """Check if command line is a ROS2 command"""
        if len(cmdline) < 3:
            return False
            
        # Remove empty strings and get relevant parts
        format_cmdline = [part for part in cmdline if part]
        if len(format_cmdline) < 3:
            return False
            
        # Look for ros2 in the command line
        for i, entry in enumerate(format_cmdline):
            if 'ros2' in entry:
                # Check if next elements contain 'run' or 'launch'
                if i + 1 < len(format_cmdline):
                    next_cmd = format_cmdline[i + 1]
                    if next_cmd in ['run', 'launch']:
                        return True
        return False
    
    def parse_ros2_command(self, pid: int, cmdline: List[str]) -> Optional[ROS2Command]:
        """Parse ROS2 command line into structured data"""
        try:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            
            # Find ros2 position and extract command parts
            ros2_idx = -1
            for i, part in enumerate(cmdline):
                if 'ros2' in part:
                    ros2_idx = i
                    break
            
            if ros2_idx == -1 or ros2_idx + 3 >= len(cmdline):
                return None
                
            cmd_type = cmdline[ros2_idx + 1]  # 'run' or 'launch'
            package = cmdline[ros2_idx + 2]
            
            command = ROS2Command(
                pid=pid,
                timestamp=timestamp,
                cmd_type=cmd_type,
                package=package
            )
            
            if cmd_type == 'run':
                if ros2_idx + 3 < len(cmdline):
                    command.executable = cmdline[ros2_idx + 3]
                
                # Parse additional arguments and parameters
                remaining_args = cmdline[ros2_idx + 4:]
                command.arguments, command.parameters, command.namespace = self.parse_arguments(remaining_args)
                
                # If no node name specified, use executable name
                if not command.node_name:
                    command.node_name = command.executable
                    
            elif cmd_type == 'launch':
                if ros2_idx + 3 < len(cmdline):
                    command.executable = cmdline[ros2_idx + 3]  # launch file name
                
                # Parse launch arguments
                remaining_args = cmdline[ros2_idx + 4:]
                command.arguments, command.parameters, command.namespace = self.parse_arguments(remaining_args)
            
            return command
            
        except Exception as e:
            print(f"Error parsing command for PID {pid}: {e}")
            return None
    
    def parse_arguments(self, args: List[str]) -> tuple:
        """Parse command arguments into arguments, parameters, and namespace"""
        arguments = []
        parameters = {}
        namespace = ""
        
        i = 0
        while i < len(args):
            arg = args[i]
            
            if arg == '--ros-args':
                i += 1
                continue
            elif arg == '-p' or arg == '--param':
                # Parameter: -p param_name:=value
                if i + 1 < len(args):
                    param_str = args[i + 1]
                    if ':=' in param_str:
                        key, value = param_str.split(':=', 1)
                        parameters[key] = value
                    i += 1
            elif arg == '-r' or arg == '--remap':
                # Remapping: -r old_name:=new_name
                if i + 1 < len(args):
                    # Store as parameter for now
                    remap_str = args[i + 1]
                    if ':=' in remap_str:
                        key, value = remap_str.split(':=', 1)
                        parameters[f"remap_{key}"] = value
                    i += 1
            elif arg.startswith('--'):
                # Long argument
                if ':=' in arg:
                    key, value = arg[2:].split(':=', 1)
                    parameters[key] = value
                else:
                    arguments.append(arg)
            elif arg.startswith('-'):
                # Short argument
                arguments.append(arg)
            elif ':=' in arg:
                # Direct parameter assignment
                key, value = arg.split(':=', 1)
                parameters[key] = value
            else:
                # Regular argument
                arguments.append(arg)
            
            i += 1
        
        return arguments, parameters, namespace
    
    def start_monitoring(self):
        """Start monitoring ROS2 commands"""
        if not self.monitoring:
            self.monitoring = True
            self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
            self.monitor_thread.start()
    
    def stop_monitoring(self):
        """Stop monitoring ROS2 commands"""
        self.monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=1)
    
    def _monitor_loop(self):
        """Main monitoring loop"""
        while self.monitoring:
            try:
                # Get current processes
                current_pids = set()
                
                for proc in psutil.process_iter(['pid', 'cmdline']):
                    try:
                        pid = proc.info['pid']
                        current_pids.add(pid)
                        
                        # Check new processes
                        if pid not in self.seen_pids:
                            self.seen_pids.add(pid)
                            cmdline = proc.info['cmdline']
                            
                            if cmdline and self.is_ros2_command(cmdline):
                                command = self.parse_ros2_command(pid, cmdline)
                                if command:
                                    self.commands[pid] = command
                                    print(f"New ROS2 command detected: {command}")
                    
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        continue
                
                # Remove commands for processes that no longer exist
                dead_pids = []
                for pid in self.commands:
                    if pid not in current_pids:
                        dead_pids.append(pid)
                
                for pid in dead_pids:
                    print(f"Removing dead process: PID {pid}")
                    del self.commands[pid]
                    self.seen_pids.discard(pid)
                
            except Exception as e:
                print(f"Error in monitoring loop: {e}")
            
            time.sleep(1)  # Check every second
    
    def get_commands(self) -> List[ROS2Command]:
        """Get current list of monitored commands"""
        return list(self.commands.values())
    
    def clear_commands(self):
        """Clear all monitored commands"""
        self.commands.clear()
        self.seen_pids.clear()


class LaunchFileGenerator:
    """Generate ROS2 launch files from monitored commands"""
    
    @staticmethod
    def generate_launch_file(commands: List[ROS2Command], package_name: str = "generated_launch") -> str:
        """Generate launch file content from ROS2 commands"""
        
        launch_content = f'''import os
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
'''
        
        # Group commands by type
        run_commands = [cmd for cmd in commands if cmd.cmd_type == 'run' and cmd.selected]
        launch_commands = [cmd for cmd in commands if cmd.cmd_type == 'launch' and cmd.selected]
        
        # Add nodes for 'ros2 run' commands
        for cmd in run_commands:
            node_name = cmd.node_name or cmd.executable
            
            # Build parameters dictionary
            params_str = ""
            if cmd.parameters:
                params_list = []
                for key, value in cmd.parameters.items():
                    if key.startswith('remap_'):
                        continue  # Handle remappings separately
                    # Try to determine parameter type
                    if value.lower() in ['true', 'false']:
                        params_list.append(f'"{key}": {value.lower()}')
                    elif value.replace('.', '').replace('-', '').isdigit():
                        params_list.append(f'"{key}": {value}')
                    else:
                        params_list.append(f'"{key}": "{value}"')
                
                if params_list:
                    params_str = f'''
        parameters=[{{
{','.join(f"            {param}" for param in params_list)}
        }}],'''
            
            # Build remappings
            remappings_str = ""
            remappings = []
            for key, value in cmd.parameters.items():
                if key.startswith('remap_'):
                    original_key = key[6:]  # Remove 'remap_' prefix
                    remappings.append(f'("{original_key}", "{value}")')
            
            if remappings:
                remappings_str = f'''
        remappings=[
{','.join(f"            {remap}" for remap in remappings)}
        ],'''
            
            # Build arguments
            arguments_str = ""
            if cmd.arguments:
                args_list = [f'"{arg}"' for arg in cmd.arguments]
                arguments_str = f'''
        arguments=[{', '.join(args_list)}],'''
            
            node_code = f'''
    # Node from PID {cmd.pid} - {cmd.timestamp}
    nodes_and_processes.append(
        Node(
            package="{cmd.package}",
            executable="{cmd.executable}",
            name="{node_name}",{params_str}{remappings_str}{arguments_str}
            output="screen"
        )
    )'''
            
            launch_content += node_code
        
        # Add launch files for 'ros2 launch' commands
        for cmd in launch_commands:
            # Build launch arguments dictionary
            launch_args = {}
            if cmd.parameters:
                for key, value in cmd.parameters.items():
                    launch_args[key] = value
            
            # Build launch arguments string
            args_str = ""
            if launch_args:
                args_list = []
                for key, value in launch_args.items():
                    args_list.append(f'                "{key}": "{value}"')
                args_str = f'''
            launch_arguments={{
{','.join(args_list)}
            }},'''
            
            launch_code = f'''
    # Launch file from PID {cmd.pid} - {cmd.timestamp}
    nodes_and_processes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("{cmd.package}"),
                    "launch",
                    "{cmd.executable}"
                ])
            ]),{args_str}
        )
    )'''
            
            launch_content += launch_code
        
        # Complete the launch file
        launch_content += '''

    return LaunchDescription(declared_arguments + nodes_and_processes)
'''
        
        return launch_content


class ROS2LaunchGeneratorGUI:
    """Main GUI application for ROS2 launch file generator"""
    
    def __init__(self, root):
        self.root = root
        self.root.title("ROS2 Launch File Generator")
        self.root.geometry("1200x800")
        
        # Initialize monitor
        self.monitor = ROS2CommandMonitor()
        
        # GUI refresh rate (slower to allow for easier interaction)
        self.refresh_rate = 2000  # ms (changed from 1000 to 2000)
        self.last_command_count = 0
        
        self.setup_gui()
        self.start_monitoring()
        self.refresh_commands()
    
    def setup_gui(self):
        """Setup the GUI elements"""
        
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(1, weight=1)
        
        # Title
        title_label = ttk.Label(main_frame, text="ROS2 Launch File Generator", 
                               font=("Arial", 16, "bold"))
        title_label.grid(row=0, column=0, columnspan=3, pady=(0, 10))
        
        # Control panel
        control_frame = ttk.LabelFrame(main_frame, text="Controls", padding="5")
        control_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 10))
        
        # Monitoring status
        self.status_var = tk.StringVar(value="Monitoring: Active")
        status_label = ttk.Label(control_frame, textvariable=self.status_var, 
                                font=("Arial", 10, "bold"))
        status_label.grid(row=0, column=0, columnspan=2, pady=(0, 5))
        
        # Selection status
        self.selection_var = tk.StringVar(value="Commands: 0 | Selected: 0")
        selection_label = ttk.Label(control_frame, textvariable=self.selection_var, 
                                   font=("Arial", 9))
        selection_label.grid(row=1, column=0, columnspan=2, pady=(0, 10))
        
        # Control buttons
        ttk.Button(control_frame, text="Clear All", 
                  command=self.clear_commands).grid(row=2, column=0, sticky=tk.W+tk.E, padx=(0, 5))
        ttk.Button(control_frame, text="Select All", 
                  command=self.select_all).grid(row=2, column=1, sticky=tk.W+tk.E, padx=(5, 0))
        
        ttk.Button(control_frame, text="Deselect All", 
                  command=self.deselect_all).grid(row=3, column=0, sticky=tk.W+tk.E, padx=(0, 5), pady=(5, 0))
        ttk.Button(control_frame, text="Manual Refresh", 
                  command=self.force_refresh).grid(row=3, column=1, sticky=tk.W+tk.E, padx=(5, 0), pady=(5, 0))
        
        # Generate button
        ttk.Button(control_frame, text="Generate Launch File", 
                  command=self.generate_launch_file,
                  style="Accent.TButton").grid(row=4, column=0, columnspan=2, 
                                              sticky=tk.W+tk.E, pady=(10, 0))
        
        # Instructions
        instructions_text = "Instructions:\n• Double-click to toggle selection\n• Enter/Space to toggle selected row\n• Monitor detects ROS2 commands automatically"
        ttk.Label(control_frame, text=instructions_text, 
                 font=("Arial", 8), foreground="gray").grid(row=5, column=0, columnspan=2, 
                                                           pady=(10, 0), sticky=tk.W)
        
        # Commands list frame
        commands_frame = ttk.LabelFrame(main_frame, text="Monitored ROS2 Commands", padding="5")
        commands_frame.grid(row=1, column=1, sticky=(tk.W, tk.E, tk.N, tk.S))
        commands_frame.columnconfigure(0, weight=1)
        commands_frame.rowconfigure(0, weight=1)
        
        # Treeview for commands
        columns = ("Select", "PID", "Time", "Type", "Package", "Executable", "Parameters")
        self.tree = ttk.Treeview(commands_frame, columns=columns, show="headings", height=15)
        
        # Configure columns
        self.tree.heading("Select", text="✓")
        self.tree.heading("PID", text="PID")
        self.tree.heading("Time", text="Time")
        self.tree.heading("Type", text="Type")
        self.tree.heading("Package", text="Package")
        self.tree.heading("Executable", text="Executable")
        self.tree.heading("Parameters", text="Parameters")
        
        self.tree.column("Select", width=40, minwidth=40)
        self.tree.column("PID", width=60, minwidth=60)
        self.tree.column("Time", width=150, minwidth=100)
        self.tree.column("Type", width=70, minwidth=70)
        self.tree.column("Package", width=120, minwidth=100)
        self.tree.column("Executable", width=120, minwidth=100)
        self.tree.column("Parameters", width=300, minwidth=200)
        
        # Scrollbars for treeview
        v_scrollbar = ttk.Scrollbar(commands_frame, orient=tk.VERTICAL, command=self.tree.yview)
        h_scrollbar = ttk.Scrollbar(commands_frame, orient=tk.HORIZONTAL, command=self.tree.xview)
        self.tree.configure(yscrollcommand=v_scrollbar.set, xscrollcommand=h_scrollbar.set)
        
        # Grid treeview and scrollbars
        self.tree.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        v_scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        h_scrollbar.grid(row=1, column=0, sticky=(tk.W, tk.E))
        
        # Bind click events to toggle selection
        self.tree.bind("<Double-1>", self.toggle_selection)
        self.tree.bind("<Button-1>", self.on_tree_click)
        self.tree.bind("<Return>", self.toggle_selection_keyboard)
        self.tree.bind("<space>", self.toggle_selection_keyboard)
        
        # Preview frame
        preview_frame = ttk.LabelFrame(main_frame, text="Launch File Preview", padding="5")
        preview_frame.grid(row=1, column=2, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(10, 0))
        preview_frame.columnconfigure(0, weight=1)
        preview_frame.rowconfigure(0, weight=1)
        
        # Text widget for preview
        self.preview_text = scrolledtext.ScrolledText(preview_frame, width=50, height=20,
                                                     font=("Courier", 9))
        self.preview_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Save button for preview
        ttk.Button(preview_frame, text="Save Launch File", 
                  command=self.save_launch_file).grid(row=1, column=0, pady=(5, 0))
    
    def start_monitoring(self):
        """Start the ROS2 command monitoring"""
        self.monitor.start_monitoring()
        self.status_var.set("Monitoring: Active")
    
    def stop_monitoring(self):
        """Stop the ROS2 command monitoring"""
        self.monitor.stop_monitoring()
        self.status_var.set("Monitoring: Stopped")
    
    def refresh_commands(self):
        """Refresh the commands display"""
        # Get current commands
        commands = self.monitor.get_commands()
        
        # Only refresh if the command count changed or if forced
        if len(commands) == self.last_command_count and hasattr(self, '_last_refresh_forced'):
            if not self._last_refresh_forced:
                # Schedule next refresh without updating display
                self.root.after(self.refresh_rate, self.refresh_commands)
                return
        
        self.last_command_count = len(commands)
        self._last_refresh_forced = False
        
        # Store current selection to preserve it
        selected_items = self.tree.selection()
        selected_pids = []
        for item in selected_items:
            try:
                pid = int(self.tree.item(item, "values")[1])
                selected_pids.append(pid)
            except (IndexError, ValueError):
                pass
        
        # Clear existing items
        for item in self.tree.get_children():
            self.tree.delete(item)
        
        # Sort commands by timestamp
        commands.sort(key=lambda x: x.timestamp)
        
        # Add commands to tree
        items_to_select = []
        for cmd in commands:
            # Format parameters string
            params_str = ""
            if cmd.parameters:
                param_items = []
                for key, value in cmd.parameters.items():
                    param_items.append(f"{key}:={value}")
                params_str = ", ".join(param_items[:3])  # Show first 3 parameters
                if len(cmd.parameters) > 3:
                    params_str += f" ... (+{len(cmd.parameters) - 3} more)"
            
            # Add arguments to parameters string
            if cmd.arguments:
                if params_str:
                    params_str += " | "
                params_str += f"args: {' '.join(cmd.arguments[:2])}"
                if len(cmd.arguments) > 2:
                    params_str += f" ... (+{len(cmd.arguments) - 2} more)"
            
            # Insert into tree
            item = self.tree.insert("", "end", values=(
                "✓" if cmd.selected else "○",
                cmd.pid,
                cmd.timestamp.split()[1],  # Show only time, not date
                cmd.cmd_type,
                cmd.package,
                cmd.executable,
                params_str
            ))
            
            # Restore selection if this item was previously selected
            if cmd.pid in selected_pids:
                items_to_select.append(item)
        
        # Restore tree selection
        if items_to_select:
            for item in items_to_select:
                self.tree.selection_add(item)
        
        # Update status display
        total_commands = len(commands)
        selected_commands = len([cmd for cmd in commands if cmd.selected])
        self.selection_var.set(f"Commands: {total_commands} | Selected: {selected_commands}")
        
        # Update preview
        self.update_preview()
        
        # Schedule next refresh
        self.root.after(self.refresh_rate, self.refresh_commands)
    
    def toggle_selection_keyboard(self, event):
        """Toggle command selection with keyboard (Enter or Space)"""
        selected_items = self.tree.selection()
        if selected_items:
            item = selected_items[0]
            try:
                values = self.tree.item(item, "values")
                if len(values) > 1:
                    pid = int(values[1])
                    if pid in self.monitor.commands:
                        cmd = self.monitor.commands[pid]
                        cmd.selected = not cmd.selected
                        # Force refresh to show selection change
                        self._last_refresh_forced = True
                        self.refresh_commands()
                        print(f"Toggled selection for PID {pid}: {'Selected' if cmd.selected else 'Deselected'}")
            except (IndexError, ValueError) as e:
                print(f"Error toggling selection: {e}")
    
    def on_tree_click(self, event):
        """Handle single click on tree - select item for interaction"""
        item = self.tree.identify_row(event.y)
        if item:
            # Clear previous selection and select clicked item
            self.tree.selection_set(item)
    
    def toggle_selection(self, event):
        """Toggle command selection on double-click"""
        item = self.tree.identify_row(event.y)
        if item:
            try:
                values = self.tree.item(item, "values")
                if len(values) > 1:
                    pid = int(values[1])
                    if pid in self.monitor.commands:
                        cmd = self.monitor.commands[pid]
                        cmd.selected = not cmd.selected
                        # Force refresh to show selection change
                        self._last_refresh_forced = True
                        self.refresh_commands()
                        print(f"Toggled selection for PID {pid}: {'Selected' if cmd.selected else 'Deselected'}")
            except (IndexError, ValueError) as e:
                print(f"Error toggling selection: {e}")
    
    def select_all(self):
        """Select all commands"""
        for cmd in self.monitor.commands.values():
            cmd.selected = True
        self._last_refresh_forced = True
        self.refresh_commands()
        print("All commands selected")
    
    def deselect_all(self):
        """Deselect all commands"""
        for cmd in self.monitor.commands.values():
            cmd.selected = False
        self._last_refresh_forced = True
        self.refresh_commands()
        print("All commands deselected")
    
    def clear_commands(self):
        """Clear all monitored commands"""
        self.monitor.clear_commands()
        self._last_refresh_forced = True
        self.refresh_commands()
        print("All commands cleared")
    
    def force_refresh(self):
        """Force a manual refresh of the commands display"""
        self._last_refresh_forced = True
        self.refresh_commands()
        print("Manual refresh completed")
    
    def update_preview(self):
        """Update the launch file preview"""
        selected_commands = [cmd for cmd in self.monitor.get_commands() if cmd.selected]
        
        if selected_commands:
            preview_content = LaunchFileGenerator.generate_launch_file(selected_commands)
        else:
            preview_content = "# No commands selected\n# Select ROS2 commands from the list to generate a launch file"
        
        self.preview_text.delete(1.0, tk.END)
        self.preview_text.insert(1.0, preview_content)
    
    def generate_launch_file(self):
        """Generate and display launch file"""
        selected_commands = [cmd for cmd in self.monitor.get_commands() if cmd.selected]
        
        if not selected_commands:
            messagebox.showwarning("No Selection", "Please select at least one ROS2 command to generate a launch file.")
            return
        
        # Generate launch file
        launch_content = LaunchFileGenerator.generate_launch_file(selected_commands)
        
        # Update preview
        self.preview_text.delete(1.0, tk.END)
        self.preview_text.insert(1.0, launch_content)
        
        messagebox.showinfo("Success", f"Launch file generated with {len(selected_commands)} command(s)!")
    
    def save_launch_file(self):
        """Save the current launch file preview to disk"""
        content = self.preview_text.get(1.0, tk.END)
        
        if not content.strip() or content.strip().startswith("# No commands"):
            messagebox.showwarning("No Content", "No launch file content to save.")
            return
        
        # Ask for file location
        file_path = filedialog.asksaveasfilename(
            defaultextension=".py",
            filetypes=[("Python files", "*.py"), ("All files", "*.*")],
            title="Save Launch File"
        )
        
        if file_path:
            try:
                with open(file_path, 'w') as f:
                    f.write(content)
                messagebox.showinfo("Success", f"Launch file saved to:\n{file_path}")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to save file:\n{str(e)}")
    
    def on_closing(self):
        """Handle application closing"""
        self.stop_monitoring()
        self.root.destroy()


def main():
    """Main function"""
    root = tk.Tk()
    app = ROS2LaunchGeneratorGUI(root)
    
    # Handle window closing
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        app.on_closing()


if __name__ == "__main__":
    main()
