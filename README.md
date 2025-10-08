# 🛰️ PX4-ROS2 Odyssey  
*A journey from simulation to autonomy with PX4, ROS 2 Humble, and Gazebo Harmonic.*

---

## 📖 Overview  
**PX4-ROS2 Odyssey** is a complete guide and documentation of my journey exploring the integration of **PX4 Autopilot** with **ROS 2 Humble** using **Gazebo Harmonic**.  
It walks through everything from the fundamentals of ROS 2 and PX4 to building, simulating, and deploying fully autonomous robotic systems — both aerial and ground-based.  

Whether you’re a **student**, **researcher**, or **enthusiast**, this repository will help you learn, practice, and master key robotics concepts with real-world applications.

---

## 🎯 Goals  
- Build a solid understanding of **ROS 2** architecture, topics, services, and actions  
- Learn to integrate **PX4 Autopilot** with **ROS 2** for UAV and UGV projects  
- Simulate and test robots in **Gazebo Harmonic**  
- Retrieve and tune **parameters**, manage **missions**, and analyze **telemetry**  
- Transition from **simulation** to **real-world** deployment on actual hardware  

---

## ⚙️ Prerequisites  
Before starting, ensure you have:  
- A Linux-based environment (Ubuntu 22.04 recommended)  
- Installed **ROS 2 Humble Hawksbill**  
- Installed **PX4 Autopilot** and **QGroundControl**  
- Installed **Gazebo Harmonic**  
- Basic understanding of Python or C++  
- Git, colcon, and VSCode (optional but recommended)

> 💡 Follow the setup instructions in [Module 1](modules/module-1-environment-setup.md).

---

## 🗺️ Roadmap at a Glance  

| Stage | Module | Focus | Status |
|--------|---------|--------|--------|
| 🧩 1 | **ROS 2 + PX4 Environment Setup** | Installation, workspace configuration | ⬜ |
| 🔌 2 | **ROS 2 Basics for PX4** | Topics, services, and nodes | ⬜ |
| 🚁 3 | **PX4 SITL with Gazebo Harmonic** | Simulation environment setup | ⬜ |
| 🧠 4 | **Parameter Management & Mission Upload** | Retrieve, set, and upload mission data | ⬜ |
| 🛰️ 5 | **ROS 2-PX4 Communication Layer** | Using `px4_ros_com` and `px4_msgs` | ⬜ |
| 🎮 6 | **Control Nodes & Flight Commands** | Arm, takeoff, waypoint missions | ⬜ |
| 📡 7 | **Sensor Integration** | IMU, GPS, and camera streams | ⬜ |
| 🧭 8 | **Autonomy & Computer Vision** | Perception and decision-making | ⬜ |
| 🧰 9 | **Real Hardware Deployment** | Connecting and testing on real UAVs | ⬜ |
| 🧾 10 | **Advanced Topics** | MAVROS bridge, custom control nodes | ⬜ |

---

## 🧩 Modules  

### **Module 1: Environment Setup**  
Setting up ROS 2 Humble, PX4 Autopilot, Gazebo Harmonic, and necessary dependencies.  

### **Module 2: ROS 2 Fundamentals**  
Understanding ROS 2 communication concepts and writing basic publisher-subscriber nodes.  

### **Module 3: PX4 SITL Simulation**  
Running PX4 in Software-in-the-Loop (SITL) mode with Gazebo Harmonic and connecting ROS 2 nodes.  

### **Module 4: Parameter Data and Mission Upload**  
- Retrieving and modifying PX4 parameters via ROS 2  
- Uploading mission data (waypoints, flight paths)  
- Executing and monitoring missions  

### **Module 5–10**  
Continue expanding each module with detailed walkthroughs, code examples, and simulation outputs.

---

## 🧠 Learning Outcomes  
By the end of this journey, you will be able to:  
- Understand ROS 2-PX4 communication flows  
- Simulate drones in Gazebo with realistic physics  
- Control UAVs autonomously through ROS 2 nodes  
- Transition from simulation to field tests confidently  

---

## 🧰 Tech Stack  
- **ROS 2 Humble Hawksbill**  
- **PX4 Autopilot**  
- **Gazebo Harmonic**  
- **MAVLink / MAVROS**  
- **Python / C++**  
- **QGroundControl**

---

## 🗂️ Repository Structure  
```bash
px4-ros2-odyssey/
├── modules/
│   ├── module-1-environment-setup.md
│   ├── module-2-ros2-basics.md
│   ├── module-3-sitl-simulation.md
│   ├── module-4-parameter-and-mission.md
│   └── ...
├── assets/             # Images, diagrams, videos
├── scripts/            # ROS 2 and PX4 scripts
├── launch/             # ROS 2 launch files
└── README.md
