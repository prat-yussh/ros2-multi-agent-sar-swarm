# ROS 2 Multi-Agent Search and Rescue (SAR) Swarm

An advanced, autonomous multi-agent robotics simulation designed for disaster response operations. This project features a decentralized **Hive-Mind communication network**, **Lawnmower Grid Coverage Path Planning**, and a **Leader-Follower hierarchical architecture** built using **ROS 2 Jazzy** and **Gazebo Harmonic**.

---

## 🚀 Key Features

* **Leader-Follower Hierarchy:** The lead agent (`robot_1`) performs a tactical breach through restrictive bottlenecks first, while follower agents (`robot_2`, `robot_3`) hold station in a staggered formation to prevent traffic jams.
* **Coverage Path Planning (Lawnmower Grid):** Robots execute calculated mathematical grid sweeps to systematically cover search zones without missing blind spots.
* **Decentralized Hive-Mind Communication:** Agents share a custom ROS 2 pub/sub network (`/rescue_alert`). Once an agent localizes a target, it broadcasts the signal instantly, causing the rest of the swarm to safely stand down.
* **Dynamic LiDAR Perception & QoS Configuration:** Leverages custom sensor data Quality of Service (QoS) profiles and dynamic field-of-view cones to filter out multi-agent interference and avoid structural obstacles safely at high speeds.
* **Odometry-to-World Coordinate Mapping:** Translates local robot odometry streams into global Gazebo world frames for precise distress beacon localization.

---

## 🛠️ Tech Stack & Requirements

* **Operating System:** Ubuntu 24.04 LTS
* **Middleware:** ROS 2 Jazzy Jalisco
* **Physics Simulator:** Gazebo Harmonic
* **Programming Language:** Python 3 (`rclpy`)
* **Core Messages:** `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `std_msgs`

---

## 📂 Workspace Architecture

```text
sar_swarm_ws/
├── src/
│   ├── sar_bringup/       # Launch files, spawn configurations, and world setups
│   │   └── launch/
│   │       ├── hospital_world.launch.py
│   │       ├── spawn_robot.launch.py
│   │       └── swarm_logic.launch.py
│   ├── sar_gazebo/        # Custom SDF worlds and robot URDF/Xacro models
│   │   ├── urdf/rescue_robot.xacro
│   │   └── worlds/disaster_hospital.sdf
│   └── sar_logic/         # Autonomous finite state machine and hive-mind controllers
│       └── sar_logic/
│           └── robot_controller.py
