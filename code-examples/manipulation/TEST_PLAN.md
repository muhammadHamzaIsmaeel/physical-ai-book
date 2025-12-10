# Test Plan for Manipulation Controller (Phase 6)

**Objective**: Verify the effectiveness of the dexterous manipulation controller and its integration with locomotion for combined tasks.

**Test Environment**:
*   **Operating System**: Ubuntu 22.04 LTS.
*   **Hardware**: System with NVIDIA GPU (e.g., RTX 4070 Ti or better).
*   **Software**:
    *   ROS 2 Jazzy.
    *   NVIDIA Isaac Sim (configured with ROS 2 bridge).
    *   Humanoid URDF model with dexterous hands/grippers.
    *   Manipulation controller code (T104) fully integrated into the robot's ROS 2 stack.
    *   Locomotion controller (T097) for integrated tests.

## Test Case 1: Manipulation Controller Grasping (T105)

**Purpose**: Verify the manipulation controller's ability to grasp a variety of household objects.

1.  **Setup**:
    *   Isaac Sim running with the humanoid robot positioned in front of a flat surface (e.g., a table).
    *   Place 15 diverse household objects (e.g., mug, block, sphere, bottle, pen, small box, screwdriver, banana, etc.) on the table within the robot's reach.
    *   Manipulation controller node (from T104) launched and ready to receive grasping commands (e.g., target object name, pose).
2.  **Execution**:
    *   For each of the 15 objects:
        *   Command the robot to grasp the object (e.g., via a ROS 2 service call with the object's name or pose).
        *   Observe the robot's grasping attempt and whether it successfully picks up and holds the object.
        *   Record the success or failure for each object.
3.  **Expected Result**:
    *   The robot should successfully grasp and hold at least 12 out of 15 household objects (80% success rate).
    *   The grasping action should be stable and the object should not drop immediately after grasping.

## Test Case 2: Integrated Locomotion + Manipulation (T106)

**Purpose**: Verify the robot's ability to perform a combined task involving both locomotion and manipulation.

1.  **Setup**:
    *   Isaac Sim running with the humanoid robot and a target object (e.g., a block) placed some distance away from the robot (e.g., 2-3 meters).
    *   A target location (e.g., a designated drop-off zone) is also defined in the scene.
    *   Both locomotion (T097) and manipulation (T104) controllers are launched and integrated to receive high-level task commands.
2.  **Execution**:
    *   Command the robot to perform a task like "Walk to the block, pick it up, and place it in the drop-off zone."
    *   Observe the robot's entire sequence of actions:
        *   Locomotion towards the object.
        *   Manipulation to grasp the object.
        *   Locomotion towards the drop-off zone.
        *   Manipulation to release the object.
    *   Repeat the test 4 times.
3.  **Expected Result**:
    *   The robot should successfully complete the entire locomotion-then-manipulation task in at least 3 out of 4 trials (75% success rate).
    *   The transition between locomotion and manipulation phases should be smooth and coordinated.

## Test Automation

*   These tests involve complex robotic behaviors and interactions within a simulated environment.
*   While high-level commands can be scripted via ROS 2, the assessment of "successful grasp" or "completed task" often requires human observation and judgment.
*   The setup of diverse objects for grasping and dynamically positioned targets for combined tasks also benefits from manual configuration or semi-automated tools.
*   This plan is intended for execution by a beta tester or developer.
