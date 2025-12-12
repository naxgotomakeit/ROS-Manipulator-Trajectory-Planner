# ROS-Manipulator-Trajectory-Planner

# KUKA YouBot Kinematics & Control System 🤖

![ROS](https://img.shields.io/badge/ROS-Noetic-black) ![Python](https://img.shields.io/badge/Python-3.8-blue) ![Robotics](https://img.shields.io/badge/Robotics-Kinematics-orange) ![KUKA](https://img.shields.io/badge/Hardware-KUKA_YouBot-red)

> A ROS-based kinematics solver and trajectory planner for the KUKA YouBot manipulator, implementing Forward/Inverse Kinematics (FK/IK), Jacobian-based control, and singularity detection.

---

## 📸 Demo

![Simulation Demo](https://via.placeholder.com/800x400?text=Please+Upload+Your+Simulation+GIF+Here)

---

## 🚀 Project Overview

This project implements a complete control pipeline for a 5-DOF (Degrees of Freedom) **KUKA YouBot** manipulator. It bridges the gap between theoretical kinematics and practical ROS implementation.

The system calculates the robot's state using **Jacobian matrices**, solves trajectory tracking problems using **Inverse Kinematics (IK)**, and broadcasts real-time transforms (TF) within the ROS ecosystem.

### Key Features:
* **Forward Kinematics (FK):** Calculated end-effector poses from joint angles using standard DH parameters.
* **Jacobian-based Control:** Implemented analytical Jacobian computation for velocity control and **Singularity Detection** to ensure safe operation.
* **Inverse Kinematics (IK):** Implemented a numerical IK solver using the **Pseudo-Inverse method** ($\dot{q} = J^{\dagger} v$) to handle robot singularities and ensure smooth joint velocities.
* **Trajectory Planning:** Capable of executing complex paths defined in data files (e.g., modifying target points and trajectory shapes).
* **ROS Integration:** Developed a custom ROS node to listen to trajectory topics and publish `tf` (transform) data for visualization in Rviz.
---

## 🛠️ Tech Stack

* **Framework:** ROS (Robot Operating System)
* **Language:** Python / C++
* **Math:** NumPy (Matrix operations, Linear Algebra)
* **Simulation/Viz:** Rviz, Gazebo
* **Hardware Target:** KUKA YouBot Mobile Manipulator

---

## 🧩 Implementation Details

### 1. Kinematics & Singularity Analysis
* Modeled the 5-DOF kinematic chain of the YouBot.
* Computed the **Jacobian Matrix** $J(q)$ to map joint velocities $\dot{q}$ to end-effector velocities $v$.
* Implemented a determinant check (`det(J)`) to identify and avoid singular configurations where the robot loses degrees of freedom.

### 2. Inverse Kinematics (IK) Solver
* Solved the equation $\dot{q} = J^{\#} v$ (where $J^{\#}$ is the pseudo-inverse) to drive the robot along a predefined path.
* Optimized for smooth joint transitions to prevent "jerk" motion.

### 3. ROS Node & TF Broadcasting
* Created a subscriber node to receive trajectory waypoints.
* Used `tf2_ros` to broadcast the dynamic transform of the end-effector frame, allowing for real-time visualization of the path tracking error.

---

## 💻 Usage

To run the simulation and control node:

```bash
# Clone this repository
git clone [https://github.com/YOUR-USERNAME/KUKA-YouBot-Kinematics-Control.git](https://github.com/YOUR-USERNAME/KUKA-YouBot-Kinematics-Control.git)

# Navigate to your workspace
cd catkin_ws/src

# Build the package
catkin_make

# Source the workspace
source devel/setup.bash

# Launch the node (Example)
roslaunch youbot_control main_controller.launch
