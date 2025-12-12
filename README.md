# KUKA YouBot Kinematics & Control System 🤖

![ROS](https://img.shields.io/badge/ROS-Noetic-black) ![Python](https://img.shields.io/badge/Python-3.8-blue) ![Robotics](https://img.shields.io/badge/Robotics-Kinematics-orange)

> A ROS-based kinematics solver and trajectory planner for the KUKA YouBot manipulator.

---

## 📸 Demo

![Simulation Demo](https://via.placeholder.com/800x400?text=Insert+Demo+GIF+Here)

---

## 🚀 Project Overview

This project implements a complete control pipeline for a 5-DOF **KUKA YouBot** manipulator. It calculates the robot's state using **Jacobian matrices**, solves trajectory tracking problems using **Inverse Kinematics (IK)**, and broadcasts real-time transforms (TF) within ROS.

### Key Features:
* **Forward Kinematics (FK):** Calculated end-effector poses from joint angles using standard DH parameters.
* **Jacobian-based Control:** Implemented analytical Jacobian computation for velocity control and **Singularity Detection** to ensure safe operation.
* **Inverse Kinematics (IK):** Implemented a numerical IK solver using the **Pseudo-Inverse method** (`q_dot = pinv(J) * v`) to handle robot singularities and ensure smooth joint velocities.
* **Trajectory Planning:** Capable of executing complex paths defined in data files.
* **ROS Integration:** Developed a custom ROS node to listen to trajectory topics and publish `tf` data for Rviz visualization.

---

## 🛠️ Tech Stack

* **Framework:** ROS (Robot Operating System)
* **Languages:** Python
* **Libraries:** NumPy, SciPy
* **Simulation:** Rviz, Gazebo

---

## 💻 Usage

```bash
# Clone this repository
git clone [https://github.com/YOUR-USERNAME/KUKA-YouBot-Kinematics.git](https://github.com/YOUR-USERNAME/KUKA-YouBot-Kinematics.git)

# Build the workspace
cd catkin_ws
catkin_make
source devel/setup.bash

# Run the controller
roslaunch youbot_control main_controller.launch
