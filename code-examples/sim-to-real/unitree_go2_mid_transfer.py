# code-examples/sim-to-real/unitree_go2_mid_transfer.py

# This script is a conceptual guide and needs to be adapted to a specific Unitree Go2 SDK version
# and your trained object tracking policy. It outlines the general steps for sim-to-real transfer
# for an object tracking policy on a Unitree Go2 robot.

import os
import subprocess
import time

def print_section_header(title):
    print(f"\n{'='*50}\n{title}\n{'='*50}\n")

def run_command(command, description):
    print(f"Executing: {description} -> {command}")
    try:
        process = subprocess.run(command, shell=True, check=True, capture_output=True, text=True)
        print(f"STDOUT:\n{process.stdout}")
        if process.stderr:
            print(f"STDERR:\n{process.stderr}")
        return True
    except subprocess.CalledProcessError as e:
        print(f"ERROR: {description} failed.")
        print(f"STDOUT:\n{e.stdout}")
        print(f"STDERR:\n{e.stderr}")
        return False
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return False

def main():
    print_section_header("Sim-to-Real Transfer Guide for Unitree Go2 Mid Tier")
    print("This guide assumes you have:")
    print("1. An object tracking policy trained in Isaac Sim (e.g., using Isaac ROS perception components and Reinforcement Learning).")
    print("2. A Unitree Go2 robot with its SDK configured.")
    print("3. ROS 2 Jazzy installed on both the host (for training) and the Go2's onboard computer.")
    print("4. Network connectivity between your host machine and the Go2.")

    policy_model_path = "/path/to/your/go2_tracking_policy.pth" # Placeholder for your trained policy
    go2_ip = "192.168.123.10" # Default IP for Go2's onboard computer (check your configuration)
    go2_user = "unitree" # Default user for Unitree's onboard computer

    if not os.path.exists(policy_model_path):
        print(f"WARNING: Policy model not found at {policy_model_path}. Please update 'policy_model_path' to a valid path.")
        # return

    print_section_header("Step 1: Export Trained Policy from Host")
    print("Similar to the Jetson guide, convert your policy to an optimized format like ONNX or TensorRT.")
    # if not run_command(f"python3 /path/to/export_script.py --model_path {policy_model_path} --output_format ONNX --output_path ./go2_tracking_policy.onnx", "Exporting policy for Go2"):
    #     return

    print_section_header("Step 2: Transfer Policy to Unitree Go2 Onboard Computer")
    print(f"Using SCP to copy the exported policy to the Go2's onboard computer at {go2_ip}.")
    if not run_command(f"scp ./go2_tracking_policy.onnx {go2_user}@{go2_ip}:~/unitree_ws/src/go2_tracking/policies/", "Copying policy to Go2"):
        print("Failed to transfer policy. Ensure SSH is configured and Go2 IP/user are correct.")
        # return

    print_section_header("Step 3: Build & Deploy ROS 2 Tracking Stack on Go2")
    print("This assumes you have a ROS 2 workspace on your Go2 with an object tracking package.")
    print("This package should contain the ROS 2 node that loads your transferred policy and interfaces with the Go2 SDK.")
    print("Connecting to Go2 via SSH to build the workspace...")
    ssh_command_build = (
        f"ssh {go2_user}@{go2_ip} \" "
        "source /opt/ros/jazzy/setup.bash && "
        "cd ~/unitree_ws && "
        "colcon build --packages-select go2_tracking --symlink-install && "
        "source install/setup.bash "
        ""
    )
    if not run_command(ssh_command_build, "Building ROS 2 tracking package on Go2"):
        print("Failed to build tracking stack on Go2.")
        # return

    print_section_header("Step 4: Launch Object Tracking Node on Go2")
    print("Launching the ROS 2 object tracking node on the Go2 that uses the policy.")
    print("This node will subscribe to the Go2's camera topic, run inference, and publish Go2 motor commands.")
    launch_command = (
        f"ssh {go2_user}@{go2_ip} \" "
        "nohup bash -c ' "
        "source /opt/ros/jazzy/setup.bash && "
        "source ~/unitree_ws/install/setup.bash && "
        "export ROS_DOMAIN_ID=42 && "
        "ros2 launch go2_tracking object_tracker.launch.py ' > ~/go2_tracking_launch.log 2>&1 &"
        ""
    )
    print(f"Conceptual launch command for Go2:\n{launch_command}")
    print("Remember to use `nohup` or `screen`/`tmux` for persistent launches.")
    # if not run_command(launch_command, "Launching object tracking node on Go2"):
    #     return
    print("Waiting a few seconds for nodes to come up...")
    time.sleep(5)

    print_section_header("Step 5: Monitor Go2 and Policy Performance (Host Machine)")
    print("From your host machine, monitor the Go2's status via ROS 2 topics.")
    print(f"Ensure your host and Go2 have the same ROS_DOMAIN_ID (e.g., export ROS_DOMAIN_ID=42).")
    print("Example monitoring commands (run on host):")
    run_command("ros2 topic list", "Listing active ROS 2 topics")
    run_command("ros2 node list", "Listing active ROS 2 nodes")
    run_command("ros2 topic echo /camera/image_raw", "Echoing Go2's camera feed")
    run_command("ros2 topic echo /go2_tracking/target_pose", "Echoing detected target pose")
    run_command("ros2 run rviz2 rviz2", "Launching Rviz2 to visualize Go2 state and target")

    print("\nSim-to-Real transfer guide complete for Unitree Go2. Observe robot performance in the real world.")
    print("Troubleshooting: Check go2_tracking_launch.log on the Go2 for errors.")
    print("Ensure correct camera topic and frame transformations are set up in your Go2 tracking package.")

if __name__ == "__main__":
    main()
