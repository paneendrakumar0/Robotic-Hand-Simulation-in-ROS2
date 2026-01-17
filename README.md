Robotic-Hand-Simulation-in-ROS2

A ROS-based simulation of a robotic hand featuring full finger articulation and wrist rotation, designed for digital twin visualization in RViz.
🤖 ROS 2 Robotic Hand Simulation (Digital Twin)
📖 Overview

This package contains a high-fidelity ROS 2 simulation of the DexHandV2, a highly dexterous robotic hand. It features full finger articulation (thumb, index, middle, ring, and pinky) along with wrist rotation capabilities.

It is designed to act as a Digital Twin, allowing you to visualize and control the hand's orientation in 3D Cartesian space using RViz2 and GUI sliders.
⚙️ Prerequisites

    ROS 2 (Humble, Iron, or Foxy)

    Colcon build system

    Python 3

    Joint State Publisher GUI (sudo apt install ros-$ROS_DISTRO-joint-state-publisher-gui)

📥 Installation & Setup
1. Create a Workspace (if you haven't already)
Bash

mkdir -p ~/Hand_Sim/src
cd ~/Hand_Sim/src

2. Clone the Repositories

You need both the control logic and the DexHandV2 model files:
Bash

# Clone this controller repository
git clone https://github.com/paneendrakumar0/Robotic-Hand-Simulation-in-ROS2.git

# Clone the DexHandV2 model repository (Required for URDF/Meshes)
git clone https://github.com/osudrl/dexhandv2.git

3. Install Dependencies
Bash

cd ~/Hand_Sim
rosdep install --from-paths src --ignore-src -r -y

4. Build the Package
Bash

cd ~/Hand_Sim
colcon build --symlink-install
source install/setup.bash

🚀 Usage

To launch the simulation with the advanced controller and RViz visualization:
Bash

ros2 launch dexhand_control advanced_control.launch.py

🎮 How to Use (Digital Twin Mode)
1. Controlling Joints

A GUI window (Joint State Publisher) will appear.

    Use the sliders to curl fingers individually.

    Use the Wrist/Roll sliders to rotate the hand.

2. Setting up the "World" Frame in RViz2

To visualize the hand rotating freely in space (like a real hand moving in the air relative to the ground):

    In RViz2, go to the Displays panel (left side).

    Find Global Options > Fixed Frame.

    Change it from base_link to world.

        If world is not listed in the dropdown, click the text box and type it manually.

    Now, when you move the "Wrist" or "Arm" sliders, the entire hand will rotate relative to the grid.

📂 File Structure

    dexhand_control/ - Main Python logic for hand tracking and control.

    launch/ - Python launch files (advanced_control.launch.py).

    urdf/ - Xacro/URDF model of the DexHandV2.

    config/ - RViz2 configuration files.

🤝 Contributing

Contributions are welcome! Please fork the repository and submit a pull request.
📞 Contact

    Developer: Paneendra Kumar

    Email: paneendra100@gmail.com
