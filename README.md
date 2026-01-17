# Robotic-Hand-Simulation-in-ROS2
A ROS-based simulation of a robotic hand featuring full finger articulation and wrist rotation, designed for digital twin visualization in RViz.





---


# 🤖 ROS 2 Robotic Hand Simulation (Digital Twin)

![ROS2](https://img.shields.io/badge/ROS2-Humble%20%7C%20Iron-blue)
![Build](https://img.shields.io/badge/build-colcon-orange)
![License](https://img.shields.io/badge/license-MIT-green)

## 📖 Overview
This package contains a high-fidelity **ROS 2 simulation of a robotic hand**, featuring full finger articulation and **wrist rotation**. 

It is designed to act as a **Digital Twin**, allowing you to visualize and control the hand's orientation in 3D Cartesian space using RViz2 and GUI sliders.

---

## ⚙️ Prerequisites
* **ROS 2** (Humble, Iron, or Foxy)
* **Colcon** build system
* **Python 3**

---

## 📥 Installation & Setup

### 1. Create a Workspace (if you haven't already)
```bash
mkdir -p ~/Hand_Sim/src
cd ~/Hand_Sim/src

```

### 2. Clone the Repository

```bash
git clone [https://github.com/paneendrakumar0/robotic-hand-ros2.git]

```

### 3. Install Dependencies

```bash
cd ~/Hand_Sim
rosdep install --from-paths src --ignore-src -r -y

```

### 4. Build the Package

```bash
cd ~/Hand_Sim
colcon build --symlink-install
source install/setup.bash

```

---

## 🚀 Usage

To launch the simulation with the advanced controller and RViz visualization:

```bash
ros2 launch dexhand_control advanced_control.launch.py

```

---

## 🎮 How to Use (Digital Twin Mode)

### 1. Controlling Joints

A GUI window (Joint State Publisher) will appear.

* Use the sliders to curl fingers individually.
* Use the **Wrist/Roll** sliders to rotate the hand.

### 2. Setting up the "World" Frame in RViz2

To visualize the hand rotating freely in space (like a real hand moving in the air relative to the ground):

1. In RViz2, go to the **Displays** panel (left side).
2. Find **Global Options** > **Fixed Frame**.
3. Change it from `base_link` to **`world`**.
* *If `world` is not listed, type it manually.*


4. Now, when you move the "Wrist" or "Arm" sliders, the entire hand will rotate relative to the grid.

---

## 📂 File Structure

* `urdf/` - Xacro/URDF model of the hand.
* `launch/` - Python launch files (`advanced_control.launch.py`).
* `config/` - RViz2 configuration files.

---

## 🤝 Contributing

Contributions are welcome! Please fork the repository and submit a pull request.

## 📞 Contact

* **Developer:** Paneendra Kumar
* **Email:** paneendra100@gmail.com

```

```
