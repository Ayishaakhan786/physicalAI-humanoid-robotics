# Data Model: Physical AI & Humanoid Robotics Book Content

**Feature Branch**: `001-robotics-book-spec` | **Date**: 2025-12-07 | **Spec**: specs/001-robotics-book-spec/spec.md
**Plan**: specs/001-robotics-book-spec/plan.md

## Content Entities and Relationships

This section describes the logical structure of the book's content and associated artifacts. Given that the project is a Docusaurus-based book, the "data model" refers to the organization and attributes of the educational materials rather than a traditional database schema.

### 1. Book Module

-   **Description**: Represents a distinct, self-contained section of the book. Each module focuses on a specific core topic within Physical AI & Humanoid Robotics.
-   **Fields**:
    -   `id`: Unique identifier (e.g., "ros2-module", "digital-twin-module").
    -   `title`: Human-readable title (e.g., "ROS 2 Fundamentals").
    -   `content_path`: Filesystem path to the module's Markdown/MDX content (e.g., `book/docs/ros2-module/`).
    -   `learning_objectives`: Key takeaways for the user.
    -   `prerequisites`: Required prior knowledge or setup.
    -   `labs`: List of associated lab exercises.
    -   `code_examples`: References to relevant example code.
    -   `checkpoints`: Self-assessment questions or tasks.
-   **Relationships**:
    -   Contains many `Chapter` (Markdown/MDX files).
    -   References many `Example Code` instances.
    -   References many `Asset` instances.
    -   Part of the overall `Book`.
-   **Validation**: Each module must contain at least an introduction, theory, labs, code examples, checkpoints, and a summary.

### 2. Chapter / Content Page

-   **Description**: An individual Markdown/MDX file contributing to a `Book Module`.
-   **Fields**:
    -   `file_name`: Name of the Markdown/MDX file.
    -   `title`: Title of the chapter/page.
    -   `path`: Full filesystem path.
    -   `word_count`: Number of words in the chapter (validation rule: 800-2500 words).
    -   `citations`: List of APA-formatted citations.
-   **Relationships**:
    -   Belongs to one `Book Module`.
    -   Contains references to `Asset` and `Example Code`.
-   **Validation**: Must adhere to Docusaurus-compatible Markdown/MDX. Must follow APA citation style for all external references.

### 3. Asset

-   **Description**: Supplementary files such as images, 3D models (URDFs, USDs), configuration files, etc., used within the book's content.
-   **Fields**:
    -   `file_name`: Name of the asset file.
    -   `file_type`: Type of asset (e.g., "image/png", "model/urdf", "model/usd").
    -   `path`: Relative path within the `/assets` directory (e.g., `book/docs/assets/`).
    -   `description`: Brief description of the asset's purpose.
-   **Relationships**:
    -   Referenced by `Chapter` and `Example Code`.
-   **Validation**: Must be organized within the `/assets` folder.

### 4. Example Code

-   **Description**: Source code, scripts, or ROS 2 packages provided for hands-on learning and demonstrating concepts.
-   **Fields**:
    -   `name`: Name of the code example (e.g., "simple_publisher_cpp", "robot_description.urdf").
    -   `language`: Programming language (e.g., Python, C++, XML).
    -   `path`: Filesystem path (e.g., `book/src/ros2_examples/`).
    -   `dependencies`: Required software or libraries for execution.
    -   `description`: Explanation of what the code does.
-   **Relationships**:
    -   Referenced by `Chapter` and `Book Module`.
    -   May utilize `Asset` files.
-   **Validation**: Must be reproducible on Ubuntu 22.04 with ROS 2 Humble. ROS 2 packages must be buildable with `colcon build`.

### 5. User

-   **Description**: Represents the target audience of the book.
-   **Fields**:
    -   `role`: "Student" or "Instructor".
    -   `experience_level`: "Beginner" to "Intermediate" in Physical AI.
    -   `goals`: Learning objectives for using the book.
-   **Validation**: The book's content must cater to this audience profile.

### 6. Development Environment

-   **Description**: The required software and hardware setup for following the tutorials and executing code examples.
-   **Fields**:
    -   `os`: Ubuntu 22.04.
    -   `ros2_distro`: Humble (LTS).
    -   `simulation_tools`: Gazebo, NVIDIA Isaac Sim.
    -   `other_software`: Docusaurus build tools (Node.js/npm), Python 3.x, C++ toolchain.
    -   `hardware_notes`: Details for platforms like Jetson boards.
-   **Validation**: All tutorials and code examples must be reproducible within this defined environment.
