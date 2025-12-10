# Test Plan for Capstone Autonomous Butler Project (Phase 8)

**Objective**: Verify the end-to-end functionality, integration, and capabilities of the Capstone Autonomous Butler system.

**Test Environment**:
*   **Operating System**: Ubuntu 22.04 LTS (as per book's setup guide).
*   **Hardware**: System with NVIDIA GPU (e.g., RTX 4070 Ti or better).
*   **Software**:
    *   ROS 2 Jazzy.
    *   NVIDIA Isaac Sim (configured with ROS 2 bridge).
    *   All capstone modules (navigation, perception, manipulation, VLA, voice, task planner) built and launched.
    *   A functional microphone.

## Test Case 1: Capstone System Capabilities (T133)

**Purpose**: Verify that the integrated capstone system can correctly perform at least 5 out of 7 defined butler capabilities.

1.  **Setup**:
    *   Launch Isaac Sim with the humanoid robot and a simulated household environment (e.g., living room with various objects like mugs, blocks, books, bins, tables).
    *   Launch the full capstone butler system using `ros2 launch capstone_butler capstone_butler.launch.py`.
    *   Ensure all ROS 2 nodes are active (`ros2 node list`).
2.  **Execution**:
    *   Perform a series of 7 tests, each targeting one of the butler capabilities:
        1.  **Object Identification & Localization**: Speak "Find the blue mug." -> Verify robot highlights/reports blue mug's location.
        2.  **Navigation to Target**: Speak "Go to the kitchen counter." -> Verify robot navigates to the counter avoiding obstacles.
        3.  **Pick & Place**: Speak "Pick up the block and put it on the table." -> Verify robot executes pick and place.
        4.  **Spatial Reasoning**: Speak "Place the book next to the laptop." -> Verify robot places correctly.
        5.  **Multi-Step Task Execution**: Speak "Clean up the living room." -> Verify robot executes a sequence of pick/place tasks.
        6.  **Human-Robot Interaction**: Speak a command, then ask "What are you doing?" -> Verify spoken response.
        7.  **Error Recovery**: Intentionally place an object out of reach, then command "Pick up the unreachable object." -> Verify robot reports failure or attempts simple recovery.
    *   Record success/failure for each capability.
3.  **Expected Result**:
    *   At least 5 out of 7 butler capabilities should be successfully demonstrated.
    *   The system should respond to voice commands, plan, and execute actions with reasonable latency.

## Test Case 2: Video Demonstrations (T134 - Manual Task)

**Purpose**: Record clear video demonstrations of the butler capabilities for Chapter 13.

1.  **Setup**: Ensure the capstone system is running in Isaac Sim as per Test Case 1. Prepare screen recording software.
2.  **Execution**: Systematically record clear, high-quality video clips showcasing each of the 7 butler capabilities identified in Test Case 1. Focus on smooth robot movements and clear task completion.
3.  **Expected Result**: 7 distinct video clips (or a single compilation) demonstrating the core functionalities of the autonomous butler.

## Test Case 3: Jetson Deployment Package (T135 - Placeholder/Conceptual)

**Purpose**: Create a conceptual deployment package for a Jetson-based system.

1.  **Setup**: Placeholder. This task involves creating a script/instructions for deploying the capstone ROS 2 workspace to a Jetson.
2.  **Execution**: Create the `code-examples/capstone/deploy/jetson/deploy.sh` script (or a similar set of instructions).
3.  **Expected Result**: A script or documentation outlining the process to package and deploy the capstone project onto a Jetson platform.

## Test Case 4: Capstone Setup Instructions Verification (T136 - Documentation Review)

**Purpose**: Verify that the capstone setup instructions in Chapter 13 are clear, concise, and allow for completion in under 10 minutes.

1.  **Setup**: Access to `docs/capstone-butler/index.mdx`.
2.  **Execution**:
    *   Review the "Capstone Setup Instructions" section in Chapter 13.
    *   (Optional but Recommended): Attempt to follow these instructions on a clean Ubuntu 22.04 environment (virtual machine or physical hardware) and time the process.
3.  **Expected Result**:
    *   The instructions are step-by-step and easy to understand.
    *   All prerequisites and commands are clearly listed.
    *   (If timed) The setup process can be completed in under 10 minutes.

## Test Case 5: Chapter 13 Diagrams (T137 - Documentation Review)

**Purpose**: Verify the presence and correctness of 6-8 Mermaid diagrams in Chapter 13.

1.  **Setup**: Access to `docs/capstone-butler/index.mdx`.
2.  **Execution**: Review the content for embedded Mermaid diagrams.
3.  **Expected Result**: At least 6-8 Mermaid diagrams are present and render correctly (visually or via placeholders in the text), illustrating system architecture, component integration, and data flow.

## Test Case 6: Embedded Quiz for Chapter 13 (T138 - Documentation Review)

**Purpose**: Verify the presence and quality of the embedded quiz for Chapter 13.

1.  **Setup**: Access to `docs/capstone-butler/index.mdx` (or a referenced `quiz.mdx` file).
2.  **Execution**: Review the content for the embedded quiz.
3.  **Expected Result**: An embedded quiz with 10-15 questions covering integration concepts is present or referenced, with questions that are relevant and test comprehensive understanding.

## Test Case 7: Capstone Project README.md (T139 - Documentation Review)

**Purpose**: Verify the presence and completeness of the `README.md` in `code-examples/capstone/`.

1.  **Setup**: Access to `code-examples/capstone/README.md`.
2.  **Execution**: Review the content of the `README.md`.
3.  **Expected Result**: The `README.md` includes project goal, features, dependencies, setup instructions, usage, file structure, troubleshooting, and future work sections.

## Test Automation

*   These test cases are primarily conceptual, require human review, or involve interaction with physical hardware.
*   The system-level integration makes full automation challenging without dedicated hardware and advanced testing frameworks.
