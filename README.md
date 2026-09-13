# 🦾 6-DoF Robotic Manipulator (FANUC LR Mate 200iC)

![ROS2](https://img.shields.io/badge/ROS2-Humble-34a853?style=flat&logo=ros)

![Python](https://img.shields.io/badge/Python-3.10%2B-blue?style=flat&logo=python)

![RViz2](https://img.shields.io/badge/Visualization-RViz2-orange?style=flat)

A comprehensive implementation of a 6-Degree-of-Freedom (6-DoF) robotic arm based on the **FANUC LR Mate 200iC**.

This project features a complete kinematic pipeline, including:

- Forward Kinematics (FK)
- Analytical Inverse Kinematics (IK)
- Joint-space trajectory generation
- Task-space trajectory generation
- Linear, cubic, and quintic interpolation
- ROS 2 (Humble) integration
- RViz2 visualization
- End-effector path tracing
- Arm and wrist singularity analysis

---

## 📑 Table of Contents

- [✨ Features](#-features)
- [🛠️ Tech Stack](#️-tech-stack)
- [🚀 Installation & Quick Start](#-installation--quick-start)
- [📂 Project Structure](#-project-structure)
- [📐 Kinematic Model](#-kinematic-model)
- [🖥️ RViz Visualization](#️-rviz-visualization)
- [📊 Trajectory Profiles](#-trajectory-profiles)
- [⚠️ Singularity Analysis](#️-singularity-analysis)
- [▶️ How to Run](#️-how-to-run)

---

## ✨ Features

- **URDF Modeling:** Accurate representation of the 6-DoF FANUC LR Mate 200iC manipulator.

- **Analytical Inverse Kinematics:** Closed-form mathematical IK solver for end-effector positioning.

- **Advanced Trajectory Generation:**
  - **Joint-Space:** Linear, Cubic, and Quintic polynomial interpolation.
  - **Task-Space:** Linear, Cubic, and Quintic interpolation for Cartesian motion.

- **Forward Kinematics:** Computation of end-effector pose from joint configurations.

- **Multiple IK Solutions:** Calculation and analysis of multiple valid joint configurations.

- **Live Visualization:** Real-time visualization of the manipulator in RViz2.

- **Path Tracing:** End-effector trajectory visualization for task-space motion.

- **Singularity Analysis:** Visualization and analysis of arm and wrist singular configurations.

---

## 🛠️ Tech Stack

- **Framework:** ROS 2 Humble
- **Language:** Python 3
- **ROS 2 Python Client:** `rclpy`
- **Visualization:** RViz2
- **Mathematics:** NumPy
- **Plotting:** Matplotlib
- **Robot Description:** URDF
- **Mesh Format:** STL

---

## 🚀 Installation & Quick Start

Ensure that **ROS 2 Humble** is installed on an Ubuntu 22.04 system.

ROS 2 Humble installation guide:

https://docs.ros.org/en/humble/Installation.html

Clone the repository and navigate to the ROS 2 workspace:

```bash
cd 6DoF-Robotic-Arm/6dof_arm
```

Build the workspace:

```bash
colcon build
```

Source the workspace:

```bash
source install/setup.bash
```

Launch the robot:

```bash
ros2 launch robot_arm launch.py
```

---

## 📂 Project Structure

```text
6DoF_Robotic_Arm/
│
├── 6dof_arm/
│   │
│   └── src/
│       │
│       └── robot_arm/
│           │
│           ├── URDF/
│           │   └── LR-Mate-200iC.urdf
│           │
│           ├── launch/
│           │   └── launch.py
│           │
│           ├── meshes/
│           │   ├── Link1.STL
│           │   ├── Link2.STL
│           │   ├── Link3.STL
│           │   ├── Link4.STL
│           │   ├── Link5.STL
│           │   ├── Link6.STL
│           │   └── base_link.STL
│           │
│           ├── resource/
│           │   └── robot_arm
│           │
│           ├── robot_arm/
│           │   ├── __init__.py
│           │   ├── Joint_Space_Cubic_Trajectory.py
│           │   └── Task_Space_Cubic_Trajectory.py
│           │
│           └── test/
│               ├── test_copyright.py
│               ├── test_flake8.py
│               └── test_pep257.py
│
├── Python_Kinematics/
│   ├── Articulated_FK.py
│   ├── Desired_Trajectory.py
│   ├── Joint_Space_Cubic_Trajectory.py
│   ├── Joint_Space_Linear_Trajectory.py
│   ├── Joint_Space_Quintic_Trajectory.py
│   ├── Multiple_Solution_IK.py
│   ├── Single_Solution_IK.py
│   ├── Task_Space_Cubic_Trajectory.py
│   ├── Task_Space_Linear_Trajectory.py
│   └── Task_Space_Quintic_Trajectory.py
│
├── .gitignore
├── LR Mate 200iC Series_10.pdf
└── Notes.txt
```

### Directory Overview

| Directory / File | Description |
|---|---|
| `6dof_arm/` | ROS 2 workspace containing the robot package |
| `URDF/` | Robot URDF description |
| `launch/` | ROS 2 launch files |
| `meshes/` | STL models of the robot links |
| `resource/` | ROS 2 package resource files |
| `robot_arm/` | ROS 2 Python trajectory nodes |
| `test/` | ROS 2 package tests |
| `Python_Kinematics/` | Standalone Python kinematics and trajectory implementations |
| `LR Mate 200iC Series_10.pdf` | Robot reference/documentation |
| `Notes.txt` | Project notes |

---

## 📐 Kinematic Model

The manipulator is modeled as a **6-DoF serial robotic arm** consisting of six revolute joints.

The kinematic pipeline includes:

```text
Joint Configuration
        │
        ▼
Forward Kinematics
        │
        ▼
End-Effector Pose
```

For inverse kinematics:

```text
Desired End-Effector Pose
          │
          ▼
   Wrist Centre Calculation
          │
          ▼
     Position IK
      (J1-J3)
          │
          ▼
 Orientation IK
      (J4-J6)
          │
          ▼
  Joint Configuration
```

### Kinematic Decoupling

The robot uses a spherical wrist, allowing the inverse kinematics problem to be separated into:

1. **Position of the wrist centre**
2. **Orientation of the end-effector**

The wrist centre is represented by **K** in the manipulator diagrams below.

---

# MANIPULATOR IMAGES

> Cylinders represent the revolute joint axes.

## Side View

**K is the wrist centre.**

<img width="587" height="769" alt="Side view" src="https://github.com/user-attachments/assets/a6eb9cf8-ca96-4b52-b1f5-40a3e7377cb2">

<img width="700" height="728" alt="Side view" src="https://github.com/user-attachments/assets/b2efaef3-2f5d-4aa3-8a36-5104abec168e">

---

## Top View

<img width="953" height="609" alt="Top view" src="https://github.com/user-attachments/assets/08abc235-e3a9-4acb-8964-2bdb2c007c36">

---

## Revolute Joints and Links

<img width="1133" height="762" alt="Manipulator" src="https://github.com/user-attachments/assets/c510933a-966d-4a82-91ca-4fab930709ca">

---

# 🖥️ RViz Visualization

The robot is integrated with **ROS 2 Humble** and visualized in **RViz2**.

The RViz model is generated from the URDF and corresponding STL meshes.

## RViz View 1

<img width="744" height="656" alt="RViz Arm 3" src="https://github.com/user-attachments/assets/1ca8d97f-2b14-4166-85f7-158ecd5a32ce">

## RViz View 2

<img width="742" height="616" alt="RViz Arm 2" src="https://github.com/user-attachments/assets/a932673c-c756-4f34-9a03-6f23a53c1cea">

## RViz View 3

<img width="713" height="671" alt="RViz Arm 1" src="https://github.com/user-attachments/assets/f121e1b9-20a0-433b-a3c1-812a0faa923d">

---

# 📊 Trajectory Profiles

The project implements trajectory generation in both **joint space** and **task space**.

The implemented interpolation methods are:

```text
Linear
Cubic
Quintic
```

---

## Joint-Space Linear Interpolation

### Joint Velocity Profiles

<img width="1108" height="735" alt="Joint Linear Velocity" src="https://github.com/user-attachments/assets/fd5e2c43-ab90-445a-b4af-bbd592cdb41b">

---

## Joint-Space Cubic Interpolation

### Joint Velocity Profiles

<img width="1102" height="702" alt="Joint Cubic Velocity" src="https://github.com/user-attachments/assets/f37ffeef-5311-4332-aeac-bf2cbbe1faf1">

### Joint Acceleration Profiles

<img width="1075" height="702" alt="Joint Cubic Acceleration" src="https://github.com/user-attachments/assets/fd2f1369-313b-42b5-88dc-9c2be984dfd9">

---

## Joint-Space Quintic Interpolation

### Joint Velocity Profiles

<img width="1102" height="717" alt="Joint Quintic Velocity" src="https://github.com/user-attachments/assets/46d877e6-22c0-48de-8f29-111e94664fbc">

### Joint Acceleration Profiles

<img width="1027" height="702" alt="Joint Quintic Acceleration" src="https://github.com/user-attachments/assets/9d651e33-3ad6-4f74-8fcb-d3d5b419a0e3">

### Joint Jerk Profiles

<img width="1018" height="690" alt="Joint Quintic Jerk" src="https://github.com/user-attachments/assets/7069a849-d4ce-4209-bed6-87a692152640">

---

# 🎯 Task-Space Trajectory Profiles

Task-space trajectories describe the motion of the end-effector in Cartesian space.

---

## Task-Space Linear Interpolation

### End-Effector Velocity Profiles

<img width="1144" height="733" alt="Task Linear Velocity" src="https://github.com/user-attachments/assets/c08e6719-a012-4de6-8d62-a1b382d05abd">

---

## Task-Space Cubic Interpolation

### End-Effector Velocity Profiles

<img width="1166" height="685" alt="Task Cubic Velocity" src="https://github.com/user-attachments/assets/eef33399-e3eb-4a20-91fa-fb4a3eca5ade">

### End-Effector Acceleration Profiles

<img width="1163" height="720" alt="Task Cubic Acceleration" src="https://github.com/user-attachments/assets/7ba6d334-9528-4a7c-9a6a-986ba4638bf3">

---

## Task-Space Quintic Interpolation

### End-Effector Velocity Profiles

<img width="1141" height="742" alt="Task Quintic Velocity" src="https://github.com/user-attachments/assets/7d0b21c5-639a-4c16-a5b7-932291d9c0b1">

### End-Effector Acceleration Profiles

<img width="1156" height="730" alt="Task Quintic Acceleration" src="https://github.com/user-attachments/assets/6c3db528-aafc-4a97-8034-ded107b702a4">

### End-Effector Jerk Profiles

<img width="1147" height="729" alt="Task Quintic Jerk" src="https://github.com/user-attachments/assets/045ce610-38e4-4133-9275-246b0aa12f12">

---

# ⚠️ Singularity Analysis

Singularities are configurations where the manipulator loses one or more independent directions of motion.

The project includes visualization and analysis of both:

- Arm singularities
- Wrist singularities

---

## Arm Singularity

<img width="928" height="921" alt="Arm Singularity" src="https://github.com/user-attachments/assets/fa044ff7-08d3-4fae-aef0-0df3667b03df">

---

## Wrist Singularity

### View 1

<img width="908" height="909" alt="Wrist Singularity View 1" src="https://github.com/user-attachments/assets/7309d374-3521-42e7-bbc8-bbe27da21feb">

### View 2

<img width="895" height="914" alt="Wrist Singularity View 2" src="https://github.com/user-attachments/assets/d61399b3-aa92-4f8e-a240-43f6a9297a3a">

---

# ▶️ How to Run

Navigate to the ROS 2 workspace:

```bash
cd 6DoF-Robotic-Arm/6dof_arm
```

Build the package:

```bash
colcon build
```

Source the workspace:

```bash
source install/setup.bash
```

Launch the robot:

```bash
ros2 launch robot_arm launch.py
```

---

# 📚 Reference

The project includes the FANUC LR Mate 200iC reference document:

```text
LR Mate 200iC Series_10.pdf
```

Additional project notes are available in:

```text
Notes.txt
```

---


**ROS 2 Humble + Python + NumPy + Matplotlib + RViz2**

---
