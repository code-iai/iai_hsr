# HSR Velocity Controller for ROS 2 Jazzy

This package provides a velocity controller for the HSR robot using ROS 2 Jazzy and `ros2_control`. The controller is implemented in C++ and can be used to command the robot's joints in real time.

---

## Features

- Velocity control of multiple joints
- Real-time command handling
- Publishes controller state for monitoring
- Compatible with ROS 2 Jazzy and `ros2_control`

---

## Installation

Clone the repository into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/code-iai/iai_hsr.git --recursive
```

---

## Build the workspace
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Visualise the robot

Use the provided launch file to start the robot:

```bash
ros2 launch iai_hsr_bringup hsr.launch.py
```

## Launching the controller

Use the provided launch file to start the robot and controller:

```bash
ros2 launch iai_hsr_bringup hsr.launch.py velocity_controller:=True
```
