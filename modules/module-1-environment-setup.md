# Module 1: Environment Setup 🧩

## Overview
This module covers the complete setup of your development environment for PX4-ROS2 development, including all necessary software installations and configurations.

## Prerequisites
- Ubuntu 22.04 LTS (Jammy Jellyfish)
- At least 16GB RAM recommended
- At least 50GB free disk space
- Internet connection

## Installation Steps

### 1. ROS 2 Humble Installation

```bash
# Set locale
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup Sources
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-humble-ros-base
sudo apt install ros-dev-tools

# Environment setup
source /opt/ros/humble/setup.bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
```

### 2. PX4 Autopilot Installation

```bash
# Install dependencies
sudo apt-get update -y
sudo apt-get install git make cmake build-essential genromfs ninja-build exiftool astyle -y
sudo apt-get install python3-pip python3-dev python3-setuptools python3-wheel python3-venv -y
sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev -y
sudo apt-get install libprotobuf-dev libprotoc-dev protobuf-compiler -y

# Clone PX4-Autopilot
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# Install PX4 dependencies
bash Tools/setup/ubuntu.sh
```

### 3. Gazebo Harmonic Installation

```bash
# Add Gazebo package repository
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo Harmonic
sudo apt-get update
sudo apt-get install gz-harmonic
```

### 4. Additional Tools

```bash
# Install QGroundControl
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libqt5gui5 -y
sudo apt install libfuse2 -y

# Download and install QGroundControl AppImage
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
chmod +x ./QGroundControl.AppImage
```

## Workspace Setup

### Create ROS 2 Workspace

```bash
mkdir -p ~/px4_ros2_ws/src
cd ~/px4_ros2_ws
colcon build
```

### Install PX4-ROS 2 Bridge

```bash
cd ~/px4_ros2_ws/src
git clone https://github.com/PX4/px4_ros_com.git
git clone https://github.com/PX4/px4_msgs.git
cd ..
colcon build
```

## Verification Steps

1. Verify ROS 2 Installation:
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker  # Terminal 1
ros2 run demo_nodes_cpp listener  # Terminal 2
```

2. Verify PX4 SITL:
```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

3. Verify Gazebo:
```bash
gz sim
```

## Troubleshooting

Common issues and their solutions:

1. **ROS 2 sourcing issues**
   - Ensure your `.bashrc` has the correct source command
   - Try opening a new terminal

2. **PX4 build errors**
   - Run `make clean` before rebuilding
   - Check if all dependencies are installed

3. **Gazebo launching issues**
   - Check graphics drivers
   - Verify installation with `gz --version`

## Next Steps
Once your environment is set up, proceed to [Module 2: ROS 2 Basics](module-2-ros2-basics.md) to start learning about ROS 2 concepts and how they integrate with PX4.

## Additional Resources
- [ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- [PX4 Dev Guide](https://docs.px4.io/master/en/dev_setup/dev_env.html)
- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic)