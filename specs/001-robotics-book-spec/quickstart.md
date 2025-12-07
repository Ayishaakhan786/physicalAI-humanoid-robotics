# Quickstart Guide: Physical AI & Humanoid Robotics Book Development

**Feature Branch**: `001-robotics-book-spec` | **Date**: 2025-12-07 | **Spec**: specs/001-robotics-book-spec/spec.md
**Plan**: specs/001-robotics-book-spec/plan.md

This guide provides quick steps to set up your development environment and get the Docusaurus-based book running locally.

## 1. Prerequisites

Ensure you have the following installed on your **Ubuntu 22.04** system:

-   **Git**: For cloning the repository.
-   **Node.js (LTS version, e.g., 18.x or 20.x)**: Required for Docusaurus.
-   **npm**: Node.js package manager (comes with Node.js).
-   **Python 3.x**: For ROS 2 and other Python-based examples.
-   **ROS 2 Humble**: The chosen ROS 2 distribution. Follow official installation guides.
-   **VS Code (Recommended)**: For development and editing.

## 2. Clone the Repository

Open a terminal and clone the book repository:

```bash
git clone https://github.com/Ayishaakhan786/physicalAI-Humanoid-robotics.git
cd physicalAI-Humanoid-robotics
```

## 3. Set Up Docusaurus

Navigate to the `book/` directory within the cloned repository and install its dependencies:

```bash
cd book
npm install
```

## 4. Start the Docusaurus Development Server

Once dependencies are installed, you can start the local development server:

```bash
npm start
```

This command will open your browser to `http://localhost:3000` (or another port if 3000 is in use), where you can see the book content. Any changes you make to the Markdown/MDX files will automatically refresh the browser.

## 5. Build the Docusaurus Site

To generate a static build of the book, which can be deployed to GitHub Pages:

```bash
npm run build
```

The static files will be generated in the `book/build/` directory.

## 6. ROS 2 Environment Setup (Example)

For running ROS 2 examples, ensure your ROS 2 Humble environment is sourced. If you installed ROS 2 following the official guide, you might source it like this:

```bash
source /opt/ros/humble/setup.bash
# If you have a ROS 2 workspace, source its setup.bash after the main ROS 2 setup
# source ~/ros2_ws/install/setup.bash
```

Then navigate to a ROS 2 example directory (e.g., `book/src/ros2_examples/my_package`) and build it:

```bash
cd book/src/ros2_examples/my_package
colcon build
source install/setup.bash
ros2 run my_package my_node
```
