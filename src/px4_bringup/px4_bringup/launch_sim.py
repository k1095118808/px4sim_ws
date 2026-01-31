#!/usr/bin/env python3
import shutil
import subprocess
import os
import sys

def main():
    # Check if kitty is installed
    has_kitty = shutil.which('kitty') is not None
    
    workspace_dir = os.path.abspath(os.path.join(os.getcwd()))
    # Ensure checking relative to where it is run, or expect user to run from ws root?
    # Better to find the workspace root.
    # Assumption: User runs this from workspace root or similar? 
    # Actually, if we launch `ros2 run`, CWD is usually where the user ran it.
    
    # Let's assume the user runs it from the workspace root for the session config to work 
    # (since session conf has `cd /home/kxd/ws/px4sim_ws`).
    # If we want to be robust, we should generate the conf dynamically or just use the fixed path since this is a specific user workspace.
    
    conf_path = os.path.join(workspace_dir, 'launch_session.conf')
    
    if has_kitty and os.path.exists(conf_path):
        print(f"Kitty detected. Launching split session from {conf_path}...")
        try:
            subprocess.run(['kitty', '--session', conf_path], check=True)
        except subprocess.CalledProcessError as e:
            print(f"Error launching kitty: {e}")
            sys.exit(1)
    else:
        if not has_kitty:
            print("Kitty terminal not found.")
        if not os.path.exists(conf_path):
            print(f"Configuration file not found at {conf_path}")
            
        print("Falling back to standard ros2 launch...")
        # Fallback: Launch SITL (NOTE: This won't launch offboard control automatically)
        cmd = ['ros2', 'launch', 'px4_bringup', 'sitl.launch.py']
        try:
            subprocess.run(cmd, check=True)
        except KeyboardInterrupt:
            pass

if __name__ == '__main__':
    main()
