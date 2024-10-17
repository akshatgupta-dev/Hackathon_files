# Acroba Hackathon

This repository contains the setup instructions, environment configurations, and scripts developed during the Acroba Hackathon. The project simulates a real industrial manufacturing process in a virtual environment, where participants designed and tested algorithms to control a robotic system.

## Environment Setup

### Requirements

- **Operating System**: Windows 10
- **Software**:
  - [Visual Studio Code (VSCode)](https://code.visualstudio.com/download)
  - [Docker for Windows](https://docs.docker.com/desktop/install/windows-install/) with WSL2 backend

### Installation Steps

1. **Install VSCode**:
   - Download and install from [Visual Studio Code Download Page](https://code.visualstudio.com/download).

2. **Install Docker**:
   - Follow the instructions at [Docker for Windows Installation Guide](https://docs.docker.com/desktop/install/windows-install/).

3. **Set up Docker Image**:
   - Open a terminal and pull the Docker image with:
     ```bash
     docker pull jameslinxiao/acroba_hackathon:latest
     ```
   - Tag the image:
     ```bash
     docker tag image_id acroba_hackathon:latest
     ```

4. **Download and Extract Hackathon Package**:
   - Download the package, extract it, and open it in VSCode.

5. **Install Dev Container Extension in VSCode**:
   - Go to Extensions and search for "Dev Containers". Install it.

6. **Open Hackathon Package in Dev Container**:
   - Reopen the folder in the dev container.

7. **Optional - Install Xming for GUI Display**:
   - If the dev container requires GUI display, install [Xming](http://www.straightrunning.com/XmingNotes/).

### Starting the Environment

1. **Retrieve Dev Container IP Address**:
   - In VSCode terminal, run:
     ```bash
     ifconfig
     ```

2. **Build Catkin Workspace**:
   - Navigate to `/ros-workspaces/ros1-noetic` and build the workspace:
     ```bash
     catkin build
     ```

3. **Launch Required ROS Packages**:
   - Execute the launch file with:
     ```bash
     roslaunch acroba_hackathon_case hackathon_cell.launch endpoint_tcp_ip:=your_container_ip_address
     ```

4. **Launch Virtual Environment**:
   - Open `generic_cell.exe` from the `hackathon_scene_win` folder to view the virtual scene. 

5. **Run Python Scripts**:
   - In the dev container, install the Python extension in VSCode to debug and run scripts. Sample scripts are available in the `acroba_gym` package.

## Hackathon Challenges

Participants were tasked with addressing one or more of the following challenges:

1. Localizing pieces cluttered in a container.
2. Designing a grasping pose based on object location.
3. Ranking grasping order for multiple detected objects.
4. Estimating piece orientation for re-grasping.
5. Controlling robot movements to place pieces accurately into a jig.

## Virtual Gym Overview

The simulation environment uses ROS1 for communication through topics, services, and action servers. Python interfaces are available for:

- **Jig Control** (`/jig_state`, `/move_jig`)
- **Object Management** (`/model_states`, `/remove_model`, `/set_model_states`)
- **Gripper Control** (`/move_tool_joints`, `/tool_joint_states`)
- **Robot Control** (`/set_joints`, `/robot_collision`, `/joint_states`, `/robot_pose`)

### Key Action Servers

- **Camera Control** (`/unity_camera/`)
- **Robot Trajectory Control** (`/unity_robot/`)

## Project Structure

- **launch/**: Contains launch files for initializing the environment.
- **meshes/**: Holds mesh models for the simulation.
- **urdf/**: URDF files for the virtual scene.
- **scripts/**: Sample scripts with detailed documentation.
- **video/**: Demonstration video of the process.

For more details, please refer to the individual package documentation and sample scripts.
