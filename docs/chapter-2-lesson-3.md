---
id: chapter-2-lesson-3
title: "Chapter 2 – Lesson 3: rclpy and URDF for Humanoids"
---

# Chapter 2 – Lesson 3: rclpy and URDF for Humanoids

## Introduction to rclpy

rclpy is the Python client library for ROS 2, enabling developers to write ROS 2 nodes and components using Python. It provides an intuitive API to interact with ROS 2 concepts and is particularly well-suited for rapid prototyping and scripting tasks in humanoid robotics.

## rclpy Fundamentals

### Basic Node Creation
```python
import rclpy
from rclpy.node import Node

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')
        self.get_logger().info('Humanoid Controller initialized')

def main(args=None):
    rclpy.init(args=args)
    controller = HumanoidController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()
```

### Parameter Handling
rclpy allows dynamic parameter configuration:

```python
class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')

        # Declare parameters with default values
        self.declare_parameter('walk_speed', 0.5)
        self.declare_parameter('step_height', 0.1)

        # Get parameter values
        self.walk_speed = self.get_parameter('walk_speed').value
        self.step_height = self.get_parameter('step_height').value

        # Add parameter callback for dynamic updates
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'walk_speed':
                self.walk_speed = param.value
                self.get_logger().info(f'Updated walk speed to {self.walk_speed}')
        return SetParametersResult(successful=True)
```

### Timer Usage
Timers are essential for periodic control tasks:

```python
class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')
        self.timer = self.create_timer(0.01, self.control_loop)  # 100Hz control loop
        self.joint_positions = [0.0] * 28  # Example: 28 joints for humanoid

    def control_loop(self):
        # Implement control algorithm here
        self.update_joint_commands()
```

## Working with Humanoid Joint Control

### Joint State Publisher
```python
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.05, self.publish_joint_states)  # 20Hz

        # Define humanoid joint names
        self.joint_names = [
            'left_hip_yaw', 'left_hip_roll', 'left_hip_pitch',
            'left_knee', 'left_ankle_pitch', 'left_ankle_roll',
            'right_hip_yaw', 'right_hip_roll', 'right_hip_pitch',
            'right_knee', 'right_ankle_pitch', 'right_ankle_roll',
            'left_shoulder_pitch', 'left_shoulder_roll', 'left_shoulder_yaw',
            'left_elbow', 'left_wrist_yaw', 'left_wrist_pitch',
            'right_shoulder_pitch', 'right_shoulder_roll', 'right_shoulder_yaw',
            'right_elbow', 'right_wrist_yaw', 'right_wrist_pitch',
            'head_yaw', 'head_pitch', 'torso_yaw', 'torso_pitch'
        ]

        self.joint_positions = [0.0] * len(self.joint_names)

    def publish_joint_states(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)

        self.publisher.publish(msg)
```

### Joint Command Subscriber
```python
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointCommandSubscriber(Node):
    def __init__(self):
        super().__init__('joint_command_subscriber')
        self.subscription = self.create_subscription(
            JointTrajectory,
            'joint_trajectory',
            self.joint_trajectory_callback,
            10
        )

    def joint_trajectory_callback(self, msg):
        # Process joint trajectory commands
        for point in msg.points:
            # Send commands to actuators
            self.execute_joint_commands(point.positions, point.time_from_start)
```

## Understanding URDF for Humanoid Robots

Unified Robot Description Format (URDF) is an XML format for describing all aspects of a robot, including its kinematic and dynamic properties, visual appearance, and collision models.

### Basic URDF Structure
```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Joints connect links -->
  <joint name="base_to_torso" type="revolute">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="torso">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="3.0"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.02"/>
    </inertial>
  </link>
</robot>
```

## Creating Humanoid-Specific URDF Components

### Leg Structure
```xml
<!-- Left Leg -->
<joint name="left_hip_yaw_joint" type="revolute">
  <parent link="torso"/>
  <child link="left_hip"/>
  <origin xyz="-0.05 0.1 -0.2" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-0.5" upper="0.5" effort="200" velocity="2"/>
</joint>

<link name="left_hip">
  <visual>
    <geometry>
      <cylinder length="0.1" radius="0.05"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.1" radius="0.05"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1.0"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0005"/>
  </inertial>
</link>

<joint name="left_hip_pitch_joint" type="revolute">
  <parent link="left_hip"/>
  <child link="left_thigh"/>
  <origin xyz="0 0 -0.05" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="0.5" effort="300" velocity="1.5"/>
</joint>

<link name="left_thigh">
  <visual>
    <geometry>
      <capsule length="0.4" radius="0.04"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <capsule length="0.4" radius="0.04"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="2.0"/>
    <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.001"/>
  </inertial>
</link>
```

### Arm Structure
```xml
<!-- Left Arm -->
<joint name="left_shoulder_pitch_joint" type="revolute">
  <parent link="torso"/>
  <child link="left_shoulder"/>
  <origin xyz="0 0.15 0.1" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="2"/>
</joint>

<link name="left_shoulder">
  <visual>
    <geometry>
      <cylinder length="0.1" radius="0.05"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.1" radius="0.05"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.5"/>
    <inertia ixx="0.0005" ixy="0" ixz="0" iyy="0.0005" iyz="0" izz="0.0002"/>
  </inertial>
</link>

<joint name="left_elbow_joint" type="revolute">
  <parent link="left_upper_arm"/>
  <child link="left_forearm"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="0" upper="2.5" effort="80" velocity="2"/>
</joint>
```

## URDF with rclpy Integration

### Loading URDF in Python
```python
import rclpy
from rclpy.node import Node
import xacro
from urdf_parser_py.urdf import URDF

class URDFProcessor(Node):
    def __init__(self):
        super().__init__('urdf_processor')

        # Load URDF from file
        urdf_file = self.declare_parameter(
            'urdf_file',
            '/path/to/humanoid.urdf'
        ).value

        # Parse URDF
        with open(urdf_file, 'r') as f:
            urdf_string = f.read()

        # Process Xacro if needed
        if urdf_file.endswith('.xacro'):
            urdf_string = xacro.process_file(urdf_file).toprettyxml(indent='  ')

        # Parse the robot description
        self.robot = URDF.from_xml_string(urdf_string)

        self.get_logger().info(f'Loaded robot with {len(self.robot.links)} links and {len(self.robot.joints)} joints')
```

## Kinematics with URDF

### Forward Kinematics
```python
from tf2_ros import TransformBroadcaster
import tf_transformations

class FKCalculator(Node):
    def __init__(self):
        super().__init__('fk_calculator')
        self.tf_broadcaster = TransformBroadcaster(self)

    def broadcast_transforms(self, joint_positions):
        # Calculate and broadcast transforms for all joints
        # This is a simplified example
        for joint_name, position in joint_positions.items():
            # Calculate transform based on joint position
            transform = self.calculate_transform(joint_name, position)
            self.tf_broadcaster.sendTransform(transform)
```

## Best Practices for Humanoid URDF

### Link and Joint Naming
- Use consistent naming conventions
- Group joints by body part (left_arm_, right_leg_, etc.)
- Use descriptive names that indicate function

### Inertial Properties
- Accurately model mass and inertia
- Use CAD tools to calculate inertial properties
- Verify that the center of mass is correctly positioned

### Collision Models
- Use simplified collision models for efficiency
- Ensure collision models are convex
- Test collision detection thoroughly

### Visual Models
- Use high-quality visual models for simulation
- Optimize for real-time rendering
- Include realistic materials and textures

## Summary

rclpy provides the Python interface for ROS 2 development, while URDF defines the physical structure of humanoid robots. Together, they form the foundation for controlling and simulating humanoid robots. Understanding both is essential for developing effective humanoid robot systems.