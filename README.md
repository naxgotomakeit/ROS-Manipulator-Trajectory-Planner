# KUKA YouBot Kinematics & Control System 🤖

![ROS](https://img.shields.io/badge/ROS-Noetic-black) ![Python](https://img.shields.io/badge/Python-3.8-blue) ![Robotics](https://img.shields.io/badge/Robotics-Kinematics-orange) ![Algorithm](https://img.shields.io/badge/Algorithm-Jacobian-red)

> A complete Kinematics, Jacobian, and Path Planning solution for the KUKA YouBot manipulator.

---

## 📸 Demo

![Simulation Demo](https://via.placeholder.com/800x400?text=Please+Upload+Your+Simulation+GIF+Here)

---

## 🚀 Project Overview

This project implements a robust control pipeline for a 5-DOF **KUKA YouBot** manipulator within the ROS ecosystem. It addresses core robotic challenges including Forward Kinematics (FK), Inverse Kinematics (IK), and real-time singularity avoidance.

The system allows for dynamic trajectory modification via data files and ensures smooth motion execution by optimizing joint velocities.

### Key Features & Implementation Details:

* **Forward Kinematics (FK) & DH Parameters**
    * Derived and implemented standard **Denavit-Hartenberg (DH) parameters** for the KUKA YouBot's specific dimensions.
    * Computed the transformation matrix hierarchy from the base to the end-effector.

* **Jacobian & Singularity Detection**
    * Implemented an analytical Jacobian computation to map joint velocities to Cartesian velocities.
    * **Safety Feature:** Integrated a **determinant check (`det(J)`)** to identify and avoid singular configurations where the robot loses degrees of freedom.

* **Inverse Kinematics (IK) Solver**
    * Developed a numerical IK solver using the **Pseudo-Inverse method** (`q_dot = pinv(J) * v`).
    * **Motion Optimization:** Optimized the solver for smooth joint transitions to strictly **prevent "jerk" motion** during trajectory execution.

* **Trajectory & Path Planning**
    * Designed a flexible system that reads target points from external **data files**.
    * Allows users to easily modify trajectories by adding more target points or changing path shapes without recompiling the code.

* **ROS Node Integration**
    * Created a custom ROS node that listens to trajectory topics in real-time.
    * Continuously **publishes the transform** to the end-effector frame, ensuring the TF tree matches the planned trajectory for visualization.

---

## 🛠️ Tech Stack

* **Framework:** ROS (Robot Operating System)
* **Languages:** Python, C++
* **Math:** NumPy (Linear Algebra, Matrix Inversion)
* **Tools:** Rviz (Visualization), URDF (Robot Modeling)

---

## 💻 Usage

```bash
# Clone this repository
git clone [https://github.com/YOUR-USERNAME/KUKA-YouBot-Kinematics-Control.git](https://github.com/YOUR-USERNAME/KUKA-YouBot-Kinematics-Control.git)

# Navigate to your workspace
cd catkin_ws

# Build the package
catkin_make
source devel/setup.bash

# Launch the controller node
roslaunch youbot_control main_controller.launch
