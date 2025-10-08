# Module 2.0: ROS 2 Basics for PX4 🔌

In this module we’ll cover the essentials of ROS 2 (Humble) with a PX4-centric lens: nodes, topics, services, actions, and QoS. You’ll build and run a minimal Python pub/sub and learn the QoS settings typically needed to interact with PX4 topics.

## Prerequisites
- Completed Module 1.0 and 1.1
- Working ROS 2 Humble environment
- Workspace at `~/px4_ros2_ws` built and sourced
- Basic Python knowledge

---

## 1. Quick ROS 2 CLI Tour

```bash
# Source environment
source /opt/ros/humble/setup.bash
source ~/px4_ros2_ws/install/setup.bash

# Discover
ros2 node list
ros2 topic list
ros2 service list
ros2 action list

# Inspect topic info
ros2 topic info /fmu/out/vehicle_odometry
ros2 interface show px4_msgs/msg/VehicleOdometry
```

---

## 2. Create a Minimal Python Package

```bash
cd ~/px4_ros2_ws/src
ros2 pkg create --build-type ament_python px4_ros2_basics --dependencies rclpy std_msgs px4_msgs
```

Directory layout created:
```
px4_ros2_basics/
  package.xml
  setup.cfg
  setup.py
  resource/
  px4_ros2_basics/
    __init__.py
```

---

## 3. Add a Publisher and Subscriber

Create `px4_ros2_basics/talker.py`:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.pub = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(0.5, self.tick)
        self.count = 0

    def tick(self):
        msg = String()
        msg.data = f"Hello PX4 #{self.count}"
        self.pub.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")
        self.count += 1

def main():
    rclpy.init()
    node = Talker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Create `px4_ros2_basics/listener.py`:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.sub = self.create_subscription(String, 'chatter', self.cb, 10)

    def cb(self, msg: String):
        self.get_logger().info(f"I heard: {msg.data}")

def main():
    rclpy.init()
    node = Listener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Expose entry points in `setup.py`:
```python
from setuptools import setup

package_name = 'px4_ros2_basics'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml']),],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tinny-Robot',
    maintainer_email='example@example.com',
    description='ROS 2 basics with PX4 context',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = px4_ros2_basics.talker:main',
            'listener = px4_ros2_basics.listener:main',
        ],
    },
)
```

Update `package.xml` dependencies (ensure these tags exist):
```xml
<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>px4_msgs</depend>
```

---

## 4. Build and Run

```bash
cd ~/px4_ros2_ws
colcon build --packages-select px4_ros2_basics
source install/setup.bash

# Run in two terminals
ros2 run px4_ros2_basics talker
ros2 run px4_ros2_basics listener
```

---

## 5. PX4 QoS Notes (Important)

PX4 topics often use best-effort reliability to optimize performance. When subscribing to PX4 topics:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
# self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', cb, qos)
```

---

## 6. Try Subscribing to a PX4 Topic

With SITL running (`make px4_sitl gz_x500`), add a quick subscriber:

```python
# px4_ros2_basics/odom_echo.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleOdometry

class OdomEcho(Node):
    def __init__(self):
        super().__init__('odom_echo')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST,
                         depth=10)
        self.sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.cb,
            qos)

    def cb(self, msg: VehicleOdometry):
        self.get_logger().info(f"Pose frame: {msg.pose_frame}, Velocity frame: {msg.velocity_frame}")


def main():
    rclpy.init()
    node = OdomEcho()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

Expose it in `setup.py`:
```python
        'console_scripts': [
            'talker = px4_ros2_basics.talker:main',
            'listener = px4_ros2_basics.listener:main',
            'odom_echo = px4_ros2_basics.odom_echo:main',
        ],
```

Build and run:
```bash
cd ~/px4_ros2_ws
colcon build --packages-select px4_ros2_basics
source install/setup.bash
ros2 run px4_ros2_basics odom_echo
```

---

## 7. Summary
- You explored core ROS 2 concepts via CLI and minimal nodes.
- You learned about PX4-friendly QoS and subscribed to an FMU topic.
- You built a small Python package to use as a starting point for PX4 integrations.

Next up: We can add actions/services and a simple offboard control example, or move to Module 3 (SITL workflows).
