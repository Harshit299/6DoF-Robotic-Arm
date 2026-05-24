🦾 6DoF Robotic Manipulator (FANUC LR Mate 200iC)

This project implements a 6-DOF robotic arm with:
1. Forward Kinematics (FK)
2. Inverse Kinematics (IK)
3. Task-space trajectory generation (linear, cubic & quintic)
4. Joint-space trajectory generation (linear, cubic & quintic)
5. Visualization in RViz (ROS2)


🚀 FEATURES:
1. URDF model of 6DOF Real Arm
2. Analytical inverse kinematics
3. Task-space & joint-space trajectories
4. RViz visualization with live motion
5. End-effector path tracing


🛠️ TECH STACK:
1. ROS2 (Humble)
2. Python (rclpy)
3. RViz2
4. NumPy, Matplotlib


📂 PROJECT STRUCTURE:
6DoF_Robotic_Arm/
│
├── 6dof_arm/
│   │
│   ├── src/
│   │   │
│   │   └── robot_arm/
│   │       ├── URDF/
│   │       │   ├── LR-Mate-200iC.urdf
│   │       │
│   │       ├── launch/
│   │       │   ├── launch.py
│   │       │
│   │       ├── meshes/
│   │       │   ├── Link1.STL
│   │       │   ├── Link2.STL
│   │       │   ├── Link3.STL
│   │       │   ├── Link4.STL
│   │       │   ├── Link5.STL
│   │       │   └── Link6.STL
│   │       │   ├── base_link.STL
│   │       │
│   │       ├── resource/
│   │       │   ├── robot_arm
│   │       │
│   │       └── robot_arm/
│   │       │   ├── __init__.py
│   │       │   ├── Joint_Space_Cubic_Trajectory.py
│   │       │   ├── Task_Space_Cubic_Trajectory.py
│   │       │
│   │       └── test/
│   │           ├── test_copyright.py
│   │           ├── test_flake8.py
│   │           ├── test_pep257.py
│   │
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
│   ├── Task_Space_Quintic_Trajectory.py
│
├─── .gitignore
├─── LR Mate 200iC Series_10.pdf
├─── Notes.txt


MANIPULATOR IMAGES: (Cylinders are representing revolute joints)

Side View:
<img width="587" height="769" alt="Side view" src="https://github.com/user-attachments/assets/a6eb9cf8-ca96-4b52-b1f5-40a3e7377cb2" /> <img width="700" height="728" alt="Screenshot 2026-05-24 213142" src="https://github.com/user-attachments/assets/b2efaef3-2f5d-4aa3-8a36-5104abec168e" />

Top View: 
<img width="731" height="552" alt="top view" src="https://github.com/user-attachments/assets/e8adce9c-648f-41de-bf34-d98466e26d84" />

Revolute Joints and Links:
<img width="1083" height="740" alt="manipulator" src="https://github.com/user-attachments/assets/7a51cae5-aac9-4c70-a114-744aff782f46" />


MANIPULATOR IMAGES IN RVIZ:
<img width="744" height="656" alt="arm3" src="https://github.com/user-attachments/assets/1ca8d97f-2b14-4166-85f7-158ecd5a32ce" />
<img width="742" height="616" alt="arm2" src="https://github.com/user-attachments/assets/a932673c-c756-4f34-9a03-6f23a53c1cea" />
<img width="713" height="671" alt="arm1" src="https://github.com/user-attachments/assets/f121e1b9-20a0-433b-a3c1-812a0faa923d" />


PLOTS:

Joint velocity profiles (Joint Space Linear Interpolation): 
<img width="1143" height="733" alt="JL_v" src="https://github.com/user-attachments/assets/5e6696b9-6d2c-4553-b13a-c0f5651572a2" />

Joint velocity profiles (Joint Space Cubic Interpolation):
<img width="1133" height="736" alt="JC_v" src="https://github.com/user-attachments/assets/ad645ab5-fdf8-44a5-b4a3-65498d6331fe" />

Joint acceleration profiles (Joint Space Cubic Interpolation):
<img width="1112" height="710" alt="JC_a" src="https://github.com/user-attachments/assets/0d832d71-e17d-4b87-99c3-b563777c780d" />

Joint velocity profiles (Joint Space Quintic Interpolation):
<img width="1164" height="734" alt="JQ_v" src="https://github.com/user-attachments/assets/9b803e01-66c5-4ca3-b2f4-081ad48d49fd" />

Joint acceleration profiles (Joint Space Quintic Interpolation):
<img width="1088" height="739" alt="JQ_a" src="https://github.com/user-attachments/assets/e4bd4ae5-190d-4314-93bf-a3ce1923559e" />

Joint jerk profiles (Joint Space Quintic Interpolation):
<img width="1139" height="761" alt="JQ_j" src="https://github.com/user-attachments/assets/08ed4154-c479-4cae-8782-11133bfb1467" />

EE velocity profiles (Task Space Linear Interpolation):
<img width="1144" height="733" alt="TL_v" src="https://github.com/user-attachments/assets/c08e6719-a012-4de6-8d62-a1b382d05abd" />

EE velocity profiles (Task Space Cubic Interpolation):
<img width="1166" height="685" alt="TC_v" src="https://github.com/user-attachments/assets/eef33399-e3eb-4a20-91fa-fb4a3eca5ade" />

EE acceleration profiles (Task Space Cubic Interpolation):
<img width="1163" height="720" alt="TC_a" src="https://github.com/user-attachments/assets/7ba6d334-9528-4a7c-9a6a-986ba4638bf3" />

EE velocity profiles (Task Space Quintic Interpolation):
<img width="1141" height="742" alt="TQ_v" src="https://github.com/user-attachments/assets/7d0b21c5-639a-4c16-a5b7-932291d9c0b1" />

EE acceleration profiles (Task Space Quintic Interpolation):
<img width="1156" height="730" alt="TQ_a" src="https://github.com/user-attachments/assets/6c3db528-aafc-4a97-8034-ded107b702a4" />

EE jerk profiles (Task Space Quintic Interpolation):
<img width="1147" height="729" alt="TQ_j" src="https://github.com/user-attachments/assets/045ce610-38e4-4133-9275-246b0aa12f12" />


▶️ HOW TO RUN?
1. cd 6DoF-Robotic-Arm/6dof_arm
2. colcon build
3. source install/setup.bash
4. ros2 launch robot_arm launch.py
