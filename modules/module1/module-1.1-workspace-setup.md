# Module 1.1: Workspace Setup 🗂️

This module guides you through creating a ROS 2 workspace tailored for PX4 development on the Holybro X500v2. We'll set up the workspace, clone px4_msgs and px4_ros_com, configure dependencies, and validate everything with quick smoke tests.

## Prerequisites
- Completed Module 1.0: Environment Setup
- ROS 2 Humble installed and sourced
- PX4-Autopilot cloned and SITL verified (gz_x500)

---

## 1. Create a Clean ROS 2 Workspace

```bash
# Create workspace structure
mkdir -p ~/px4_ros2_ws/src
cd ~/px4_ros2_ws

# Initialize empty workspace
colcon build

# Source local setup
source install/setup.bash
```

*Persist sourcing (optional)*
```bash

# Persist sourcing (optional)
echo 'source ~/px4_ros2_ws/install/setup.bash' >> ~/.bashrc
```

Verification:
```bash
# Should print colon-separated paths including px4_ros2_ws
echo $AMENT_PREFIX_PATH
```

---

## 2. Add PX4 ROS 2 Packages (px4_msgs, px4_ros_com)

📚 Official: https://docs.px4.io/main/en/ros/ros2_comm.html

```bash
cd ~/px4_ros2_ws/src

# Clone message definitions and bridge packages
git clone https://github.com/PX4/px4_msgs.git
# px4_ros_com uses RTPS bridge utilities
git clone https://github.com/PX4/px4_ros_com.git

```

---

## 3. Install Dependencies with rosdep

```bash
cd ~/px4_ros2_ws

# Update rosdep metadata (safe to re-run)
sudo rosdep init 2>/dev/null || true
rosdep update

# Install package dependencies (skip keys not provided via apt)
rosdep install --from-paths src --ignore-src -r -y \
  --rosdistro humble \
  --skip-keys "fastcdr fastrtps urdfdom_headers rti-connext-dds-6.0.1 cyclonedds"
```

Notes:
- We skip vendor DDS packages if not using them.
- Ensure Fast DDS (default in Humble) is present via ros-humble-desktop.

---

## 4. Build the Workspace

```bash
cd ~/px4_ros2_ws

# Clean previous build artifacts (optional)
rm -rf build/ install/ log/

# Build with colcon
colcon build --symlink-install --event-handlers console_direct+
```

##### Build Output Example

When the build completes successfully, you should see output similar to the following:

```plaintext
Finished <<< px4_ros_com [11min 45s]

Summary: 2 packages finished [14min 26s]

Tinny-Robot@Innov8:~/px4_ros2_ws$
```

If the build fails, review the error messages and ensure all dependencies are installed correctly. Refer to the [Common Issues](#7-common-issues) section for troubleshooting tips.

If you encounter DDS-related errors, try:
```bash
# Ensure Fast-DDS RMW is available
sudo apt install ros-humble-rmw-fastrtps-cpp
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

---

## 5. Source and Verify

```bash
# Source overlay on top of ROS 2
source /opt/ros/humble/setup.bash
source ~/px4_ros2_ws/install/setup.bash

# Verify packages are discoverable
ros2 pkg list | grep -E "px4_msgs|px4_ros_com" || true
```

---

## 6. Quick Smoke Tests

### 6.1. Start the Micro XRCE-DDS Agent (ROS Bridge)
PX4 uses uXRCE-DDS to communicate; run an agent to bridge PX4 to the ROS 2 DDS network.

Install one agent (choose ONE):
```bash
# Option A: eProsima Micro XRCE-DDS Agent (non-ROS)
sudo snap install micro-xrce-dds-agent

# Option B: micro-ROS Agent (ROS 2 package)
sudo apt install ros-humble-micro-ros-agent
```

Run the agent in its own terminal:
```bash
# eProsima agent (default PX4 UDP port 8888)
MicroXRCEAgent udp -p 8888

# OR micro-ROS agent (equivalent)
ros2 run micro_ros_agent micro_ros_agent udp -p 8888
```

Notes:
- Keep the agent running while using PX4.
- Ensure all terminals share the same ROS_DOMAIN_ID (default 0).

### 6.2. Run PX4 SITL (Gazebo Harmonic X500)
In a new terminal:
```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

### 6.3. Echo a PX4 Topic
With SITL running, in another terminal:
```bash
source /opt/ros/humble/setup.bash
source ~/px4_ros2_ws/install/setup.bash

# List topics and look for px4 prefixed topics
ros2 topic list | grep px4 || true

# Example: echo vehicle odometry if available
ros2 topic echo /fmu/out/vehicle_odometry --qos-reliability best_effort --qos-durability volatile
```

### 6.4. Minimal Publisher Test (Optional)
Create a simple demo package (optional):
```bash
cd ~/px4_ros2_ws/src
ros2 pkg create --build-type ament_cmake demo_px4_ws --dependencies rclcpp std_msgs

# Build and run a simple talker (follow ROS 2 tutorial)
cd ~/px4_ros2_ws
colcon build
source install/setup.bash
ros2 run demo_px4_ws demo_px4_ws_node  # if node created
```

---

## 7. Common Issues
- Missing DDS libs: ensure rmw_fastrtps is installed and exported
- No topics: verify SITL started successfully and the bridge is active
- Colcon build failures: clean build dirs and re-run rosdep

---

## 8. What’s Next
- Module 1.2: Launch files and convenience scripts
- Module 1.3: VS Code + ROS 2 tooling setup
- Continue to Module 2 once comfortable with builds and topics
