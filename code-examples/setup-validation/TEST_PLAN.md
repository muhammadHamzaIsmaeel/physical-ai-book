# Test Plan for `validate.py` Script

**Objective**: Verify that the `validate.py` script correctly checks a user's environment for the required Physical AI book dependencies.

**Test Environment**:
*   **Operating System**: Clean installation of Ubuntu 22.04 LTS.
*   **Hardware**: A machine that meets the "Economy" tier requirements from the book, including an NVIDIA GPU (RTX 3060 or better recommended).
*   **Software**: No pre-installed ROS, Isaac Sim, or NVIDIA drivers other than what comes with the base OS.

## Test Cases

### Test Case 1: Clean Install (Expect Failure)

1.  **Setup**: Use the clean Ubuntu 22.04 VM.
2.  **Execution**:
    ```bash
    git clone <repository_url>
    cd ai_book/code-examples/setup-validation
    python3 validate.py
    ```
3.  **Expected Result**:
    *   The script should execute and run through all checks.
    *   It should fail on most checks (ROS, NVIDIA drivers, CUDA, Isaac Sim).
    *   The final summary should report a low number of passed checks (e.g., 2/12, for Python and Ubuntu version).
    *   The script must exit with a non-zero status code.

### Test Case 2: Partial Install (Expect Partial Success)

1.  **Setup**:
    *   On the clean VM, install NVIDIA drivers and the CUDA toolkit according to the book's instructions.
    *   Do **not** install ROS 2 or Isaac Sim.
2.  **Execution**: Run `python3 validate.py`.
3.  **Expected Result**:
    *   The script should pass the checks for Ubuntu version, Python version, NVIDIA drivers, and CUDA.
    *   It should fail the checks for ROS 2 and Isaac Sim.
    *   The final summary should reflect a partial success.
    *   The script must exit with a non-zero status code.

### Test Case 3: Full Install (Expect Success)

1.  **Setup**:
    *   On the clean VM, follow the book's instructions to install all required software:
        *   NVIDIA Drivers
        *   CUDA Toolkit
        *   ROS 2 Jazzy
        *   Isaac Sim
        *   All required Python packages and development tools (`ros-dev-tools`, `colcon`).
2.  **Execution**: Run `python3 validate.py`.
3.  **Expected Result**:
    *   The script should run through all 12 checks.
    *   All 12 checks should pass with a "✅" indicator.
    *   The final summary should report "12 / 12 checks passed."
    *   The script should print the final success message.
    *   The script must exit with a zero status code.

### Test Case 4: Non-LTS Ubuntu (Expect Failure)

1.  **Setup**: Install a non-LTS version of Ubuntu (e.g., 23.10). Install all other dependencies.
2.  **Execution**: Run `python3 validate.py`.
3.  **Expected Result**:
    *   The "Check Ubuntu version" step should fail.
    *   The script should report a failure and exit with a non-zero status code.

### Test Case 5: Incorrect Python Version (Expect Failure)

1.  **Setup**: Use a clean Ubuntu 22.04 VM, but create a virtual environment with an older Python version (e.g., 3.8).
2.  **Execution**: Run `python3 validate.py` from within the environment.
3.  **Expected Result**:
    *   The "Check Python version" step should fail.
    *   The script should report a failure and exit with a non-zero status code.

## Test Automation

*   This test plan is designed for manual execution by a beta tester or developer.
*   Automating this test would require a dedicated CI/CD pipeline with GPU-enabled runners and the ability to provision fresh VMs for each run, which is beyond the current scope of the project. The script itself serves as the "test" for a user's machine.
