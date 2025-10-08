# Module 1: Environment Setup 🧩

## Overview
This module provides a detailed, step-by-step setup of your PX4-ROS2 development environment for the Holybro X500v2 platform. We'll follow official documentation while providing specific commands for each step.

## Prerequisites
- Ubuntu 22.04 LTS (Jammy Jellyfish)
- At least 16GB RAM recommended
- At least 50GB free disk space
- Internet connection
- Basic command line knowledge

## Step-by-Step Installation Guide

### 1. ROS 2 Humble Installation

📚 **Official Documentation:**
- [ROS 2 Humble Ubuntu Development Setup](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html)
- [ROS 2 Humble Official Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

#### Step 1.1: Set Locale
```bash
# Check current locale
locale

# Set up locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

#### Step 1.2: Set up ROS 2 Repository
```bash
# Install necessary packages
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS 2 GPG key
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

#### Step 1.3: Install ROS 2 Packages
```bash
# Update package lists
sudo apt update

# Install development tools
sudo apt install -y \
  python3-pip \
  python3-rosdep \
  python3-vcstool \
  python3-colcon-common-extensions

# Install ROS 2 base
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
```
#### Step 1.4: Install dependencies using rosdep

```bash
# Initialize rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"
```

#### Step 1.5: Environment Setup
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Add sourcing to .bashrc
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc

# Create a new workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
```

💡 **Verification:**
```bash
# Open a new terminal or source .bashrc
source ~/.bashrc

# Test ROS 2 installation
ros2 run demo_nodes_cpp talker  # In terminal 1
ros2 run demo_nodes_cpp listener  # In terminal 2
```
---
### 2. PX4 Autopilot Development Environment

📚 **Official Documentation:**
- [PX4 Development Environment on Ubuntu](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html)
- [PX4 Build Instructions](https://docs.px4.io/main/en/dev_setup/building_px4.html)
- [X500v2 Airframe Reference](https://docs.px4.io/main/en/frames_multicopter/holybro_x500v2_pixhawk5x.html)

#### Step 2.1: Get PX4 Source Code
```bash
# Clone PX4-Autopilot repository
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# Update submodules
git submodule update --init --recursive
```

#### Step 2.2: Install PX4 Dependencies
```bash
# Run the ubuntu.sh script with no prompt
bash Tools/setup/ubuntu.sh --no-nuttx

# Install python packages
pip3 install --user -r Tools/setup/requirements.txt
```

#### Step 2.3: Configure for X500v2
```bash
# First clean build
make clean

# Build for X500v2 SITL
make px4_sitl gz_x500
```

💡 **Verification:**
```bash
# Test PX4 SITL
make px4_sitl gz_x500
```

### 3. Gazebo Harmonic Installation

📚 **Official Documentation:**
- [Gazebo Harmonic Installation Guide](https://gazebosim.org/docs/harmonic/install_ubuntu)


#### Step 3.1: Install Required Tools
```bash
# Update package lists and install required tools
sudo apt-get update
sudo apt-get install curl lsb-release gnupg
```

#### Step 3.2: Add Gazebo Package Repository
```bash
# Add Gazebo GPG key
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

# Add Gazebo repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
```

#### Step 3.3: Install Gazebo Harmonic
```bash
# Update package lists
sudo apt-get update

# Install Gazebo Harmonic
sudo apt-get install gz-harmonic
```

💡 **Warning:** If you have gazebo-classic (e.g., `gazebo11`) installed, you'll need special steps for side-by-side installation. See the [official documentation](https://gazebosim.org/docs/harmonic/install_ubuntu_src#installing-gazebo11-side-by-side-with-new-gazebo) for details.

💡 **Verification:**
```bash
# Test Gazebo installation
gz sim
```

### 4. QGroundControl Setup

📚 **Official Documentation:**
Follow official Documentation
- [QGroundControl INstallation Guide](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html)

or 

#### Step 4.1: Install Required Dependencies 
```bash
# Install GStreamer and other required packages
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y
```

#### Step 4.2: Install QGroundControl
```bash
# Download QGroundControl
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage

# Make it executable
chmod +x ./QGroundControl.AppImage

# Run QGroundControl
./QGroundControl.AppImage
```

💡 **Important Notes:**
- Store the AppImage in a permanent location
- Create a desktop shortcut for easy access
- Consider adding to system PATH
___
## Workspace Setup

### 5. ROS 2 Workspace Configuration

Follow the official workspace setup guide:
[Creating a ROS 2 Workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)

Key steps:
1. Create workspace directory structure
2. Initialize source space
3. Install dependencies
4. Build the workspace

### 6. PX4-ROS 2 Bridge Setup

Follow the official microRTPS bridge setup:
[PX4-ROS 2 Bridge Setup](https://docs.px4.io/main/en/ros/ros2_comm.html)

Key steps:
1. Clone required repositories
2. Install dependencies
3. Build the workspace
4. Configure message bridges

## Testing and Verification

### 7. Environment Testing

#### ROS 2 Verification
Follow: [First ROS 2 Program](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html)
- Test basic ROS 2 functionality
- Verify workspace setup
- Check environment variables

#### PX4 SITL Testing
Follow: [PX4 SITL Testing](https://docs.px4.io/main/en/simulation/gazebo.html)
- Test PX4 simulation
- Verify Gazebo integration
- Check X500v2 model loading

#### Bridge Communication Test
Follow: [Testing ROS 2 Communication](https://docs.px4.io/main/en/ros/ros2_comm.html#testing)
- Verify message passing
- Test command interfaces
- Check sensor data flow

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
   - Verify installation with `gz --versions`

## Next Steps
Once your environment is set up, proceed to [Module 2: ROS 2 Basics](module-2-ros2-basics.md) to start learning about ROS 2 concepts and how they integrate with PX4.

## Additional Resources

### Official Documentation
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/index.html)
- [PX4 Development Guide](https://docs.px4.io/main/en/development/development.html)
- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic)
- [QGroundControl User Guide](https://docs.qgroundcontrol.com/master/en/)
- [PX4 Development Kit - X500v2](https://docs.holybro.com/drone-development-kit/px4-development-kit-x500v2)

### Community Resources
- [ROS 2 Discourse](https://discourse.ros.org/)
- [PX4 Discuss](https://discuss.px4.io/)
- [Gazebo Community](https://gazebosim.org/community)

### Troubleshooting Guides
- [ROS 2 Troubleshooting](https://docs.ros.org/en/humble/How-To-Guides/Installation-Troubleshooting.html)
- [PX4 Troubleshooting](https://docs.px4.io/main/en/debug/troubleshooting.html)
- [Gazebo Known Issues](https://gazebosim.org/docs/harmonic/troubleshooting)