# 🛰️ PX4-ROS2 Odyssey  
*A journey from simulation to autonomy with PX4, ROS 2 Humble, and Gazebo Harmonic.*  
*Created by [@Tinny-Robot](https://github.com/Tinny-Robot)*

---

## 📖 Overview  
**PX4-ROS2 Odyssey** is an ongoing collaborative project exploring the integration of **PX4 Autopilot** with **ROS 2 Humble** using **Gazebo Harmonic**, specifically focused on the **Holybro X500v2** drone frame.  
This guide documents our journey in setting up a complete development environment for drone simulation and control, starting with the fundamentals of ROS 2 and PX4.

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

## 🗺️ Current Focus  

We are currently working on:

| Stage | Module | Focus | Status |
|--------|---------|--------|--------|
| 🧩 1 | **ROS 2 + PX4 Environment Setup** | Installation, workspace configuration, X500v2 simulation | � In Progress |
| 🔌 2 | **ROS 2 Basics for PX4** | Topics, services, and nodes | ⬜ |
| 🚁 3 | **PX4 SITL with Gazebo Harmonic** | Simulation environment setup | ⬜ |
| 🧠 4 | **Parameter Management & Mission Upload** | Retrieve, set, and upload mission data | ⬜ |
| 🛰️ 5 | **ROS 2-PX4 Communication Layer** | Using `px4_ros_com` and `px4_msgs` | ⬜ |
| 🎮 6 | **Control Nodes & Flight Commands** | Arm, takeoff, waypoint missions | ⬜ |
| 📡 7 | **Sensor Integration** | IMU, GPS, and camera streams | ⬜ |
| 🧭 8 | **Autonomy & Computer Vision** | Perception and decision-making | ⬜ |
| 🧰 9 | **Real Hardware Deployment** | Connecting and testing on real UAVs | ⬜ |
| 🧾 10 | **Advanced Topics** | MAVROS bridge, custom control nodes | ⬜ |

## 🧩 Module 1: Environment Setup  

This module covers:
- Setting up ROS 2 Humble
- Installing PX4 Autopilot
- Configuring Gazebo Harmonic
- Setting up the Holybro X500v2 simulation model
- Creating a complete development environment

> 💡 We're actively developing this module together. Follow along in [Module 1](modules/module-1-environment-setup.md).

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
- **Holybro X500v2 Frame**  
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
````

---

## 📸 Media and Demonstrations

Screenshots, simulation GIFs, and videos of flight tests will be included in the `assets/` folder and embedded in each module.

---

## 📚 References

* [ROS 2 Documentation](https://docs.ros.org/en/humble/)
* [PX4 User Guide](https://docs.px4.io/main/en/)
* [Gazebo Harmonic Docs](https://gazebosim.org/docs/harmonic)
* [MAVROS Wiki](https://wiki.ros.org/mavros)

---

## 🧑‍💻 Author

**Nathaniel Handan (Tinny-Robot)**
Roboticist | AI & Drone Engineer | ROS 2 Developer
🌐 [LinkedIn](https://www.linkedin.com/in/nathanielhandan/)
🐙 [GitHub](https://github.com/Tinny-Robot)
✉️ [handanfoun@gmail.com](mailto:handanfoun@gmail.com)

---

## 🪶 License

This project is licensed under the **MIT License** — feel free to use, modify, and share it with attribution.

---

> *“The journey of mastering autonomy begins with one launch file.”* 🚀

