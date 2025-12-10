# Capstone Autonomous Butler Project

This repository contains the integrated code for the Capstone Autonomous Butler Project, as described in Chapter 13 of the "Physical AI & Humanoid Robotics" book. It brings together all the concepts learned throughout the book, including ROS 2 fundamentals, Isaac Sim, VLA integration, and basic locomotion and manipulation.

## Project Goal

The primary goal of this project is to create a humanoid robot system capable of understanding natural language commands and executing multi-step household tasks in a simulated environment (NVIDIA Isaac Sim).

## Features

*   **Voice Command Interface**: Uses speech-to-text to process human commands.
*   **Perception**: Object detection and pose estimation from simulated camera feeds.
*   **Task Planning**: Utilizes an LLM to interpret commands and generate action sequences.
*   **Navigation**: Autonomous movement within the simulated environment.
*   **Manipulation**: Grasping and placing objects.
*   **VLA Integration**: Utilizes a Vision-Language-Action model for complex skill execution.

## Dependencies

*   **Operating System**: Ubuntu 22.04 LTS
*   **ROS 2 Distribution**: Jazzy Jalisco
*   **Simulation Environment**: NVIDIA Isaac Sim (2024.1 or later)
*   **Python Libraries**: `rclpy`, `numpy`, `torch`, `transformers`, etc. (specifics will be in `requirements.txt` within each package)

## Setup Instructions

Please refer to the "Capstone Setup Instructions" section in Chapter 13 of the book (`docs/capstone-butler/index.mdx`) for detailed setup information.

### Quick Start (Conceptual)

1.  **Clone the Capstone Workspace**:
    ```bash
    # Assuming this is part of your main ROS 2 workspace
    cd ~/ros2_ws/src
    git clone https://github.com/your_username/ai_book_capstone.git capstone # Or copy directly
    ```
2.  **Install Dependencies**:
    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```
3.  **Build Workspace**:
    ```bash
    cd ~/ros2_ws
    colcon build --symlink-install
    ```
4.  **Launch Isaac Sim**: Start Isaac Sim with the ROS 2 bridge (`isaac-sim.sh --ros2-bridge`).
5.  **Launch Capstone System**:
    ```bash
    source /opt/ros/jazzy/setup.bash
    source ~/ros2_ws/install/setup.bash
    export ROS_DOMAIN_ID=42 # Match your Isaac Sim setup
    ros2 launch capstone_butler capstone_butler.launch.py
    ```

## Usage

Once the system is launched:

1.  Speak commands into your microphone (e.g., "Robot, find the red block and put it on the table").
2.  Observe the robot's actions in Isaac Sim.
3.  Monitor ROS 2 topics and logs for system status and debugging.

## File Structure

This repository is organized as a standard ROS 2 workspace:

```
capstone/
├── src/
│   ├── capstone_description/        # Robot URDF, meshes
│   ├── capstone_navigation/         # Nav2 integration, path planning
│   ├── capstone_perception/         # Object detection, pose estimation
│   ├── capstone_manipulation/       # Grasp planning, IK
│   ├── capstone_vla_integration/    # VLA model interface
│   ├── capstone_voice_handler/      # ASR (Whisper), TTS
│   └── capstone_task_planner/       # LLM-based task planning
├── launch/                          # ROS 2 launch files
└── config/                          # Configuration files
```

## Troubleshooting

Refer to the "Troubleshooting Bible" in `docs/appendices/troubleshooting-bible.mdx` for common issues across all project components. Specific capstone integration issues will be added there.

## Future Work

*   Real-world deployment on a physical humanoid robot.
*   Integration with a broader range of VLA models.
*   Enhanced error recovery mechanisms.
*   More complex task planning capabilities (e.g., long-horizon planning, learning from demonstration).
*   Human-in-the-loop control and monitoring.
