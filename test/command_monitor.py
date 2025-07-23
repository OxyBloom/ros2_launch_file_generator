import psutil
import time
import argparse
from datetime import datetime

CHECK_INTERVAL = 1  # seconds

# Keep track of already seen process IDs
seen_pids = set()

def is_ros2_command(cmdline):
    if len(cmdline) >= 3:
        # Check if the command is a ROS2 command 
        # e.g., "['/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'run', 'turtlesim', 'turtlesim_node']"
        format_cmdline = [part for part in cmdline if part]  # Remove empty strings
        #remove the first part which is usually the python interpreter
        if format_cmdline and len(format_cmdline) > 3:
            format_cmdline = format_cmdline[1:]
        # for entry in format_cmdline:
        for entry in format_cmdline:
            if entry == "run" or entry == "launch":
                return True
    return False

def log_command(pid, cmdline, follow):
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    # extract command: cmd(run/launch), package, and node
    if len(cmdline) >= 3:
        cmdline = cmdline[1:]
    else:
        return  # Not enough information to log
    
    # Ensure we have enough elements before accessing them
    if len(cmdline) < 4:
        return
    
    #extract run/launch from second element: 'run'
    cmd = cmdline[1]
    pkg = cmdline[2]
    node = cmdline[3]
    param = cmdline[4]
    cmd_list = [pid, cmd, pkg, node]
    print(f"[{timestamp}] PID={pid} -> {cmd_list}")

def monitor_ros2_commands(follow=False):
    print("Monitoring for 'ros2 run' and 'ros2 launch' commands...")
    while True:
        try:
            for proc in psutil.process_iter(['pid', 'cmdline']):
                pid = proc.info['pid']
                if pid not in seen_pids:
                    seen_pids.add(pid)
                    cmdline = proc.info['cmdline']
                    # print(f"Checking PID={pid}, Command Line: {cmdline}")
                    if is_ros2_command(cmdline):
                        log_command(pid, cmdline, follow)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass
        time.sleep(CHECK_INTERVAL)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Monitor ROS2 commands.")
    parser.add_argument('--follow', action='store_true', help="Print to terminal")
    args = parser.parse_args()
    monitor_ros2_commands(follow=args.follow)