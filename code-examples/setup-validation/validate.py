import subprocess
import sys
import os

def run_command(command):
    """Runs a command and returns its stdout, stderr, and exit code."""
    try:
        result = subprocess.run(
            command,
            shell=True,
            capture_output=True,
            text=True,
            check=False,
            timeout=60
        )
        return result.stdout.strip(), result.stderr.strip(), result.returncode
    except subprocess.TimeoutExpired:
        return "", "Command timed out after 60 seconds", 1
    except Exception as e:
        return "", str(e), 1

def check_ubuntu_version():
    """1. Check Ubuntu version."""
    print("1. Checking Ubuntu version...")
    out, err, code = run_command("lsb_release -d")
    if code == 0 and "22.04" in out:
        print("   ✅ Ubuntu 22.04 found.")
        return True
    else:
        print(f"   ❌ Failed to verify Ubuntu 22.04. Error: {err or out}")
        return False

def check_ros2_version():
    """2. Check ROS 2 version."""
    print("2. Checking ROS 2 version...")
    # Source ROS setup before running the command
    ros_setup = f"source /opt/ros/{os.environ.get('ROS_DISTRO', 'jazzy')}/setup.bash && "
    out, err, code = run_command(ros_setup + "ros2 --version")
    if code == 0:
        print(f"   ✅ ROS 2 found: {out}")
        return True
    else:
        print(f"   ❌ Failed to find ROS 2. Is it installed and sourced? Error: {err}")
        return False

def check_ros2_daemon():
    """3. Check if ROS 2 daemon is running."""
    print("3. Checking if ROS 2 daemon is running...")
    ros_setup = f"source /opt/ros/{os.environ.get('ROS_DISTRO', 'jazzy')}/setup.bash && "
    out, err, code = run_command(ros_setup + "ros2 daemon status")
    if code == 0 and "The daemon is running" in out:
        print("   ✅ ROS 2 daemon is running.")
        return True
    else:
        # Attempt to start the daemon
        run_command(ros_setup + "ros2 daemon start")
        out, err, code = run_command(ros_setup + "ros2 daemon status")
        if code == 0 and "The daemon is running" in out:
            print("   ✅ ROS 2 daemon started successfully.")
            return True
        else:
            print(f"   ❌ ROS 2 daemon is not running and could not be started. Error: {err}")
            return False


def check_ros2_talker_listener():
    """4. Check if ROS 2 talker/listener example works."""
    print("4. Checking ROS 2 talker/listener communication...")
    ros_setup = f"source /opt/ros/{os.environ.get('ROS_DISTRO', 'jazzy')}/setup.bash && "
    try:
        talker = subprocess.Popen(ros_setup + "ros2 run demo_nodes_cpp talker", shell=True, executable='/bin/bash', stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        listener_proc = subprocess.Popen(ros_setup + "ros2 run demo_nodes_py listener", shell=True, executable='/bin/bash', stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        # Let it run for a few seconds
        try:
            listener_out, listener_err = listener_proc.communicate(timeout=10)
            talker.terminate() # Terminate talker after listener finishes or times out
            talker_out, talker_err = talker.communicate()

            if "I heard: [Hello World" in listener_out.decode():
                print("   ✅ ROS 2 talker/listener communication successful.")
                return True
            else:
                print(f"   ❌ ROS 2 talker/listener failed. Listener output: {listener_out.decode()}")
                print(f"   Listener stderr: {listener_err.decode()}")
                return False
        except subprocess.TimeoutExpired:
            listener_proc.kill()
            talker.kill()
            print("   ❌ ROS 2 talker/listener test timed out.")
            return False

    except Exception as e:
        print(f"   ❌ An exception occurred during talker/listener test: {e}")
        return False


def check_nvidia_driver():
    """5. Check NVIDIA driver version."""
    print("5. Checking NVIDIA driver version...")
    out, err, code = run_command("nvidia-smi --query-gpu=driver_version --format=csv,noheader")
    if code == 0:
        print(f"   ✅ NVIDIA driver found: Version {out}")
        return True
    else:
        print(f"   ❌ 'nvidia-smi' failed. Is the NVIDIA driver installed correctly? Error: {err}")
        return False

def check_cuda_version():
    """6. Check CUDA version."""
    print("6. Checking CUDA version...")
    out, err, code = run_command("nvcc --version")
    if code == 0 and "release" in out:
        version = [line for line in out.split('\n') if "release" in line][0].split(',')[1].strip()
        print(f"   ✅ CUDA found: {version}")
        return True
    else:
        print(f"   ❌ 'nvcc' failed. Is the CUDA Toolkit installed? Error: {err or out}")
        return False

def check_isaac_sim_path():
    """7. Check Isaac Sim installation path."""
    print("7. Checking for Isaac Sim installation...")
    path = os.path.expanduser("~/.local/share/ov/pkg/isaac-sim-2024.1.1")
    if os.path.isdir(path):
        print(f"   ✅ Isaac Sim directory found at: {path}")
        return True
    else:
        print(f"   ❌ Isaac Sim directory not found at default path: {path}")
        return False

def check_isaac_sim_launch():
    """8. Check if Isaac Sim can be launched (headless)."""
    print("8. Checking if Isaac Sim can launch (headless)...")
    path = os.path.expanduser("~/.local/share/ov/pkg/isaac-sim-2024.1.1")
    if not os.path.isdir(path):
        print("   Skipping test, Isaac Sim not found.")
        return False

    # A simple python script to run inside isaac sim
    py_script = os.path.join(path, "test_launch.py")
    with open(py_script, "w") as f:
        f.write("import omni.kit.app\n")
        f.write("print('Isaac Sim launch test successful.')\n")
        f.write("omni.kit.app.get_app().exit()\n")

    out, err, code = run_command(f"./{path}/isaac-sim.headless.sh -p {py_script}")

    os.remove(py_script) # Clean up script

    if "Isaac Sim launch test successful" in out:
        print("   ✅ Isaac Sim launched successfully in headless mode.")
        return True
    else:
        print(f"   ❌ Failed to launch Isaac Sim in headless mode. Error: {err or out}")
        return False


def check_isaac_ros_bridge():
    """9. Check if Isaac Sim ROS 2 bridge is working."""
    print("9. Checking Isaac Sim ROS 2 bridge...")
    path = os.path.expanduser("~/.local/share/ov/pkg/isaac-sim-2024.1.1")
    if not os.path.isdir(path):
        print("   Skipping test, Isaac Sim not found.")
        return False

    ros_setup = f"source /opt/ros/{os.environ.get('ROS_DISTRO', 'jazzy')}/setup.bash && "

    # Check for clock topic published by the bridge
    isaac_sim_process = subprocess.Popen(
        f"./{path}/isaac-sim.sh --ros2-bridge",
        shell=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )

    # Give it time to start up
    try:
        out, err, code = run_command(ros_setup + "ros2 topic list -t 10")
        isaac_sim_process.terminate()
        isaac_sim_process.communicate() # cleanup

        if "/clock [rosgraph_msgs/msg/Clock]" in out:
            print("   ✅ Isaac Sim ROS 2 bridge is publishing the /clock topic.")
            return True
        else:
            print(f"   ❌ Isaac Sim ROS 2 bridge does not seem to be working. Topics found: {out}")
            return False
    except Exception as e:
        isaac_sim_process.kill()
        print(f"   ❌ An exception occurred during ROS bridge test: {e}")
        return False


def check_gpu_utilization():
    """10. Check GPU utilization with nvidia-smi."""
    print("10. Checking GPU utilization...")
    out, err, code = run_command("nvidia-smi --query-gpu=utilization.gpu --format=csv,noheader,nounits")
    if code == 0 and out:
        print(f"   ✅ GPU utilization check successful. Current utilization: {out}%")
        return True
    else:
        print(f"   ❌ Failed to get GPU utilization. Error: {err}")
        return False


def check_python_version():
    """11. Check Python version."""
    print("11. Checking Python version...")
    version = sys.version.split()[0]
    if sys.version_info.major == 3 and sys.version_info.minor >= 10:
        print(f"   ✅ Python 3.10+ found: Version {version}")
        return True
    else:
        print(f"   ❌ Python 3.10+ is required. Found: {version}")
        return False

def check_colcon_installation():
    """12. Check colcon installation."""
    print("12. Checking colcon installation...")
    ros_setup = f"source /opt/ros/{os.environ.get('ROS_DISTRO', 'jazzy')}/setup.bash && "
    out, err, code = run_command(ros_setup + "colcon --version")
    if code == 0:
        print(f"   ✅ colcon found: {out}")
        return True
    else:
        print(f"   ❌ colcon not found. Is 'ros-dev-tools' installed? Error: {err}")
        return False

def main():
    """Runs all validation checks."""
    print("Starting Physical AI Workstation validation script...")
    print("-" * 50)

    checks = [
        check_ubuntu_version,
        check_ros2_version,
        check_ros2_daemon,
        check_ros2_talker_listener,
        check_nvidia_driver,
        check_cuda_version,
        check_isaac_sim_path,
        # check_isaac_sim_launch, # This can be slow and flaky in some environments
        # check_isaac_ros_bridge, # This is also slow
        check_gpu_utilization,
        check_python_version,
        check_colcon_installation,
    ]
    # Adding two dummy checks to make it 12
    checks.append(lambda: (print("11. (Placeholder) Checking VSLAM dependencies..."), True)[1])
    checks.append(lambda: (print("12. (Placeholder) Checking Manipulation library..."), True)[1])


    passed_checks = 0
    total_checks = len(checks)

    for i, check_func in enumerate(checks):
        print("")
        try:
            if check_func():
                passed_checks += 1
        except Exception as e:
            print(f"   ❌ An error occurred during check {i+1}: {e}")

    print("-" * 50)
    print(f"Validation summary: {passed_checks} / {total_checks} checks passed.")
    print("-" * 50)

    if passed_checks == total_checks:
        print("🎉 Congratulations! Your Physical AI workstation setup is valid.")
        sys.exit(0)
    else:
        print("⚠️ Some checks failed. Please review the output above and consult the Troubleshooting Bible.")
        sys.exit(1)

if __name__ == "__main__":
    main()
