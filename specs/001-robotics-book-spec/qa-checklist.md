# QA Checklist and Acceptance Test Plan: Physical AI & Humanoid Robotics Book

**Feature Branch**: `001-robotics-book-spec` | **Date**: 2025-12-07 | **Plan**: specs/001-robotics-book-spec/plan.md
**Spec**: specs/001-robotics-book-spec/spec.md
**Tasks**: specs/001-robotics-book-spec/tasks.md

This document outlines the QA checklist and acceptance test plan for the "Physical AI & Humanoid Robotics" book, ensuring content quality, code reproducibility, and overall system functionality.

## 1. Per-Chapter Acceptance Criteria

For each chapter in the book, the following criteria must be met:

-   [ ] **Content Accuracy**: All technical claims, concepts, and definitions are accurate and align with primary official sources.
-   [ ] **Clarity and Readability**: Content is written in clear, concise technical English, suitable for beginner-to-intermediate learners.
-   [ ] **Learning Objectives Met**: The chapter effectively addresses all stated learning objectives.
-   [ ] **Lab Task Reproducibility**: All step-by-step lab tasks are fully reproducible on the specified development environment (Ubuntu 22.04, ROS 2 Humble).
-   [ ] **Code Snippet Correctness**: All provided code snippets compile/run without errors and demonstrate the intended functionality.
-   [ ] **Diagram Accuracy**: All diagrams are clear, correctly labeled, and accurately represent the described concepts.
-   [ ] **APA Citation Adherence**: All external references and in-text citations follow APA 7th edition style.
-   [ ] **Word Count Compliance**: Chapter word count is within the target range (800-2500 words).
-   [ ] **Docusaurus Compatibility**: Chapter content is compatible with Markdown/MDX and renders correctly within Docusaurus.

## 2. End-to-End Capstone Reproducibility Test

This test verifies the full capstone project pipeline from start to finish.

### Test Scenario: "Robot, please clear the table. Start with the red cup."

1.  **Environment Setup**:
    -   [ ] Verify Ubuntu 22.04 with ROS 2 Humble, Isaac Sim (if applicable for simulation) installed and configured.
    -   [ ] Ensure all necessary Python packages and dependencies for VLA integration are installed.
2.  **Launch VLA System**:
    -   [ ] Execute the ROS 2 launch file for the VLA system:
        ```bash
        ros2 launch vla_system_launch vla_system.launch.py
        ```
    -   [ ] Verify all VLA nodes (ASR, NLU, Task Planner, Robot Control) are running using `ros2 node list`.
3.  **Provide Verbal Command**:
    -   [ ] Use a microphone to verbally command the robot: "Robot, please clear the table. Start with the red cup."
    -   [ ] Verify that the ASR (Whisper) component correctly transcribes the command (e.g., by monitoring a transcription topic).
4.  **NLU Interpretation**:
    -   [ ] Verify that the NLU component correctly parses the command into structured intent and entities (e.g., `{'intent': 'clear_table', 'object': 'red cup', 'order': 'first'}`).
    -   [ ] Monitor the structured command topic.
5.  **Action Planning and Execution**:
    -   [ ] Verify that the Task Planner generates a logical sequence of robot actions (e.g., navigate to table, detect red cup, pick red cup, place in bin).
    -   [ ] Observe the simulated robot (in Gazebo/Isaac Sim) executing the planned actions.
    -   [ ] Verify visual and verbal feedback from the robot (e.g., "Okay, I will clear the table.").
6.  **Error Handling (Negative Test)**:
    -   [ ] Provide an ambiguous or unsupported command (e.g., "Do something complicated.").
    -   [ ] Verify the robot provides appropriate feedback indicating it could not understand or execute the command.
7.  **Completion**:
    -   [ ] Verify the robot successfully completes the task of clearing the red cup from the table.

## 3. URDF Unit Tests

For each custom URDF model (e.g., in `book/src/gazebo_sims/my_robot.urdf`):

-   [ ] **Syntax Validation**: Run `check_urdf <urdf_file_path>` and ensure no errors are reported.
-   [ ] **Visualization in RViz2**:
    -   [ ] Launch `rviz2` with the URDF model:
        ```bash
        ros2 launch your_pkg display_my_robot.launch.py
        ```
    -   [ ] Verify the robot model is displayed correctly with all links and joints.
    -   [ ] Verify joint limits and transformations are accurate (e.g., by manipulating joints in GUI).
-   [ ] **Spawning in Gazebo**:
    -   [ ] Spawn the URDF model in Gazebo:
        ```bash
        ros2 launch your_pkg spawn_my_robot_in_gazebo.launch.py
        ```
    -   [ ] Verify the robot model appears in Gazebo without errors and exhibits basic physics (e.g., falls due to gravity, interacts with environment).

## 4. Isaac Sim Checks

For each Isaac Sim scene/script (e.g., in `book/src/isaac_sims/`):

-   [ ] **Scene Load/Script Execution**:
    -   [ ] Launch Isaac Sim and load the scene or execute the script.
    -   [ ] Verify the scene loads without errors in the Isaac Sim console.
-   [ ] **Log Inspection**: Inspect Isaac Sim console/logs for critical warnings or errors during simulation.
-   [ ] **Frame Inspection**: Verify visual output in Isaac Sim viewport (e.g., correct object placement, lighting, robot pose).
-   [ ] **Sensor Data Verification**: For simulated sensors, verify the data output (e.g., camera images, lidar point clouds) is realistic and as expected.
-   [ ] **Physics Behavior**: Verify physics interactions (e.g., collisions, object manipulation) are physically plausible.

## 5. Whisper + LLM Pipeline Test

For the VLA pipeline's language components:

-   [ ] **Sample Utterance**: "Robot, move to the kitchen and fetch the apple."
    -   **Expected ASR Output**: "robot move to the kitchen and fetch the apple" (or very similar text).
    -   **Expected NLU Breakdown**:
        ```json
        {
          "intent": "fetch_object",
          "location": "kitchen",
          "object": "apple",
          "sequence": ["navigate(kitchen)", "fetch(apple, kitchen)"]
        }
        ```
-   [ ] **Sample Utterance**: "Turn left. Then go forward two meters."
    -   **Expected ASR Output**: "turn left then go forward two meters"
    -   **Expected NLU Breakdown**:
        ```json
        [
          {"intent": "turn", "direction": "left"},
          {"intent": "move", "direction": "forward", "distance": "2 meters"}
        ]
        ```
-   [ ] **Sample Utterance**: "What is your battery level?"
    -   **Expected ASR Output**: "what is your battery level"
    -   **Expected NLU Breakdown**:
        ```json
        {"intent": "query_status", "attribute": "battery_level"}
        ```
-   [ ] **Ambiguous Utterance**: "Go get that thing."
    -   **Expected NLU Response**: Indicate ambiguity or request clarification (e.g., `{"intent": "clarification_needed", "reason": "unspecified_object"}`).

## 6. Docusaurus Build & Link Checks

-   [ ] **Successful Build**: `npm run build` command in `book/` directory completes with 0 errors.
-   [ ] **Broken Links**: Run Docusaurus link checker (if configured) or a third-party tool to verify no broken internal or external links.
-   [ ] **Deployment Verification**: Verify the deployed site on GitHub Pages is accessible and fully functional.
