# Test Plan for VLA & Voice Pipeline Integration

**Objective**: Verify the functionality, performance, and accuracy of the VLA (Vision-Language-Action) models and the end-to-end voice pipeline.

**Test Environment**:
*   **Operating System**: Ubuntu 22.04 LTS (as per book's setup guide).
*   **Hardware**: An NVIDIA GPU with at least 12GB VRAM (e.g., RTX 4070 Ti, RTX 3080, or better).
*   **Software**:
    *   ROS 2 Jazzy.
    *   NVIDIA Isaac Sim (configured with ROS 2 bridge).
    *   All Python dependencies for VLA models (e.g., `transformers`, `torch`, `sentencepiece`, `bitsandbytes`).
    *   All Python dependencies for voice pipeline (e.g., `transformers` for Whisper, `openai` or similar for LLM).
    *   A functional microphone connected to the system.

## Test Case 1: OpenVLA-7B Inference (T066)

**Purpose**: Test OpenVLA-7B inference to verify latency and memory requirements on target hardware.

1.  **Setup**:
    *   Ensure OpenVLA-7B is correctly set up as described in Chapter 10.
    *   Use the provided `code-examples/vla/openvla_inference.py` script.
2.  **Execution**:
    *   Run the script with a sample image and a command like "pick up the red block".
    *   Monitor GPU memory usage during inference using `nvidia-smi`.
    *   Measure inference time using Python's `time` module around the model inference call.
    ```bash
    # Example command (adjust as per actual script usage)
    python3 code-examples/vla/openvla_inference.py --image_path <path_to_image> --command "pick up the red block"
    ```
3.  **Expected Result**:
    *   The model should load successfully.
    *   Peak VRAM usage should be approximately 8-10 GB (for FP16) or less (for quantized versions), confirming it runs on 12GB GPUs.
    *   Inference latency for a single image-command pair should be < 500 ms.
    *   The script should output a plausible action (e.g., joint angles or gripper command).

## Test Case 2: VLA Inference Example (T079)

**Purpose**: Test the full VLA inference pipeline with 20 diverse test commands and document success rate and latency.

1.  **Setup**:
    *   Ensure the `code-examples/vla/openvla_inference.py` script (or the final integration script for VLA) is ready.
    *   Prepare 20 diverse test commands covering various manipulation tasks (e.g., "pick up the blue sphere", "place the mug on the shelf", "push the box", "open the drawer").
    *   Prepare corresponding visual inputs (images or Isaac Sim scene states).
2.  **Execution**:
    *   For each of the 20 commands:
        *   Provide the visual input and the text command to the VLA inference script.
        *   Record whether the generated action is correct and effective (i.e., would achieve the goal in simulation).
        *   Measure the end-to-end latency for each command (from input to action generation).
3.  **Expected Result**:
    *   Success rate should be at least 80% (16 out of 20 commands result in correct actions).
    *   Average latency per command should be within acceptable limits (e.g., < 1 second).
    *   Document the success rate, average latency, and any common failure modes.

## Test Case 3: End-to-End Voice Pipeline (T085)

**Purpose**: Verify the end-to-end latency of the voice pipeline with 20 spoken commands and overall functionality.

1.  **Setup**:
    *   Ensure the full voice pipeline (`code-examples/voice-pipeline/voice_robot.py` or equivalent) is running. This includes:
        *   Microphone input.
        *   Whisper speech-to-text.
        *   LLM for command parsing.
        *   VLA model for action generation.
        *   ROS 2 interface to a simulated robot in Isaac Sim.
    *   Prepare 20 diverse spoken commands covering various manipulation tasks.
    *   Ensure the Isaac Sim environment is set up with a robot capable of executing these commands.
2.  **Execution**:
    *   For each of the 20 commands:
        *   Speak the command clearly into the microphone.
        *   Start a timer when speaking and stop it when the robot in Isaac Sim *begins* its action.
        *   Observe if the robot correctly interprets and attempts to execute the command.
        *   Record the latency for each command.
3.  **Expected Result**:
    *   End-to-end latency for each spoken command should be < 3 seconds on average.
    *   The robot in Isaac Sim should correctly interpret and initiate the action for at least 80% of the commands (16 out of 20).
    *   Document average latency, success rate, and any observed issues (e.g., speech recognition errors, LLM misinterpretations).

## Test Automation

*   These test cases require significant manual interaction and observation due to the nature of voice input, visual assessment of actions, and performance measurement.
*   Some aspects (e.g., latency measurement, success rate calculation) can be aided by scripting, but the overall assessment needs human judgment.
*   This plan is intended for execution by a beta tester or developer during the "STOP and VALIDATE" phase of the MVP development.
