# Test Plan for Locomotion Controller (Phase 6)

**Objective**: Verify the functionality and stability of the locomotion controller for humanoid robots in simulation, and document its performance under various conditions.

**Test Environment**:
*   **Operating System**: Ubuntu 22.04 LTS.
*   **Hardware**: System with NVIDIA GPU (e.g., RTX 4070 Ti or better).
*   **Software**:
    *   ROS 2 Jazzy.
    *   NVIDIA Isaac Sim (configured with ROS 2 bridge).
    *   Humanoid URDF model (e.g., `simple_humanoid.urdf` from Chapter 04).
    *   Locomotion controller code (T097) fully integrated into the robot's ROS 2 stack.

## Test Case 1: Locomotion Controller Code (T097 - Implementation Verification)

**Purpose**: Verify the basic functionality of the locomotion controller, ensuring it can generate gait parameters and send commands to the simulated robot.

1.  **Setup**:
    *   Ensure Isaac Sim is running with the humanoid robot loaded and the ROS 2 bridge active.
    *   Ensure the locomotion controller node (from T097) is launched and connected to the robot's joint state interfaces.
2.  **Execution**:
    *   Send a simple walk command (e.g., forward 0.5 m/s) to the controller via a ROS 2 topic or service.
    *   Observe the robot's behavior in Isaac Sim.
    *   Monitor ROS 2 topics related to joint commands and robot state.
3.  **Expected Result**:
    *   The robot's legs should exhibit a recognizable walking gait.
    *   Joint commands should be published by the controller.
    *   No critical errors should appear in the controller's logs.

## Test Case 2: Humanoid Walking Performance (T098)

**Purpose**: Verify that the humanoid can walk at a specified speed on flat terrain without falling.

1.  **Setup**:
    *   Isaac Sim running with a flat ground plane and the humanoid robot.
    *   Locomotion controller launched (T097).
2.  **Execution**:
    *   Command the robot to walk forward at 0.5 m/s for a duration of at least 30 seconds.
    *   Observe the robot's stability and movement in Isaac Sim.
    *   Record any instances of falling or significant instability.
3.  **Expected Result**:
    *   The humanoid robot should walk forward at approximately 0.5 m/s.
    *   The robot should maintain balance and not fall during the 30-second test.
    *   The trajectory should be relatively straight (minimal drift).

## Test Case 3: Balance Perturbation Test (T099)

**Purpose**: Verify the robot's ability to recover balance after an external perturbation.

1.  **Setup**:
    *   Isaac Sim running with the humanoid robot standing in a stable pose.
    *   Locomotion controller launched (T097), ideally with a dedicated balance recovery module enabled.
    *   A mechanism to apply a brief external force (e.g., a simple ROS 2 service that applies a force to the robot's torso in Isaac Sim).
2.  **Execution**:
    *   Apply a sudden, brief external force to the robot's torso (e.g., a 50N force for 0.1 seconds, from the side).
    *   Measure the time taken for the robot to return to a stable upright pose after the perturbation.
    *   Repeat the test 5 times from different directions and magnitudes (within reasonable limits).
3.  **Expected Result**:
    *   The robot should recover its balance and return to a stable pose within 1.5 seconds for at least 4 out of 5 perturbation tests.
    *   The robot should not fall over.

## Test Automation

*   These tests require the Isaac Sim environment and interaction with a simulated robot.
*   While some aspects can be automated (e.g., commanding speeds, applying forces via ROS 2 services), the visual assessment of "falling" or "stability" often requires human observation.
*   This plan is intended for execution by a beta tester or developer. Further automation would require advanced Isaac Sim scripting and ROS 2 integration for automated metrics collection.
