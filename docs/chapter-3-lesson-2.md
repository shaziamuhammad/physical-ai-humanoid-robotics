---
id: chapter-3-lesson-2
title: "Chapter 3 – Lesson 2: Sensors: LiDAR, Depth Cameras, IMU"
---

# Chapter 3 – Lesson 2: Sensors: LiDAR, Depth Cameras, IMU

## Introduction to Robotic Sensors

Robots rely on various sensors to perceive their environment and understand their state. In Physical AI systems, accurate sensor data is crucial for navigation, manipulation, and interaction with the physical world. This lesson covers three essential sensor types: LiDAR, depth cameras, and IMUs.

## LiDAR Sensors

LiDAR (Light Detection and Ranging) sensors provide precise distance measurements to objects by emitting laser pulses and measuring the time it takes for them to return.

### LiDAR Principles
- **Operation**: Emits laser pulses and measures time-of-flight
- **Resolution**: Can provide thousands of distance measurements per second
- **Accuracy**: High precision distance measurements (typically millimeter accuracy)
- **Range**: Can detect objects from centimeters to hundreds of meters away

### LiDAR in Gazebo Simulation
Configuring a LiDAR sensor in a URDF/SDF file:

```xml
<link name="lidar_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.04"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.04"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>

<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0.1 0 0.2" rpy="0 0 0"/>
</joint>

<gazebo reference="lidar_link">
  <sensor name="lidar" type="ray">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/humanoid</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
    </plugin>
  </sensor>
</gazebo>
```

### LiDAR Applications in Humanoid Robots
- **Navigation**: Creating 2D/3D maps of the environment
- **Obstacle Detection**: Identifying and avoiding obstacles
- **Localization**: Matching sensor data to known maps
- **Path Planning**: Finding safe routes through environments

### LiDAR ROS 2 Interface
LiDAR sensors publish to the `sensor_msgs/LaserScan` topic:

```python
from sensor_msgs.msg import LaserScan
import rclpy
from rclpy.node import Node

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        self.subscription = self.create_subscription(
            LaserScan,
            '/humanoid/scan',
            self.lidar_callback,
            10
        )

    def lidar_callback(self, msg):
        # Process laser scan data
        ranges = msg.ranges
        min_distance = min(ranges) if ranges else float('inf')

        if min_distance < 1.0:  # Obstacle within 1 meter
            self.get_logger().info('Obstacle detected!')
```

## Depth Cameras

Depth cameras provide 3D information about the environment by measuring the distance to objects in each pixel.

### Depth Camera Principles
- **RGB-D Cameras**: Provide both color and depth information
- **Stereo Vision**: Uses two cameras to calculate depth
- **Structured Light**: Projects patterns to calculate depth
- **Time-of-Flight**: Measures light travel time for depth calculation

### Depth Camera in Gazebo
Configuring a depth camera in simulation:

```xml
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.02 0.05 0.02"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.02 0.05 0.02"/>
    </geometry>
  </collision>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.05 0 0.02" rpy="0 0 0"/>
</joint>

<gazebo reference="camera_link">
  <sensor name="depth_camera" type="depth">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>30</update_rate>
    <camera name="depth_cam">
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
      <ros>
        <namespace>/humanoid</namespace>
        <remapping>~/depth/image_raw:=depth/image_raw</remapping>
        <remapping>~/rgb/image_raw:=rgb/image_raw</remapping>
        <remapping>~/rgb/camera_info:=rgb/camera_info</remapping>
      </ros>
      <output_type>sensor_msgs/Image</output_type>
      <camera_name>camera</camera_name>
    </plugin>
  </sensor>
</gazebo>
```

### Depth Camera ROS 2 Interface
Depth cameras publish multiple topics:
- `sensor_msgs/Image` for depth data
- `sensor_msgs/Image` for RGB data
- `sensor_msgs/CameraInfo` for camera parameters

```python
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
import numpy as np

class DepthCameraProcessor(Node):
    def __init__(self):
        super().__init__('depth_camera_processor')
        self.bridge = CvBridge()
        self.depth_subscription = self.create_subscription(
            Image,
            '/humanoid/depth/image_raw',
            self.depth_callback,
            10
        )
        self.rgb_subscription = self.create_subscription(
            Image,
            '/humanoid/rgb/image_raw',
            self.rgb_callback,
            10
        )

    def depth_callback(self, msg):
        # Convert ROS Image to OpenCV
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

        # Process depth information
        height, width = depth_image.shape
        center_depth = depth_image[height//2, width//2]

        if center_depth < 1.0:  # Object within 1 meter
            self.get_logger().info(f'Close object detected at {center_depth:.2f}m')

    def rgb_callback(self, msg):
        # Process RGB image for object recognition
        rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Further processing...
```

## IMU Sensors

IMU (Inertial Measurement Unit) sensors measure orientation, angular velocity, and linear acceleration.

### IMU Principles
- **Accelerometer**: Measures linear acceleration
- **Gyroscope**: Measures angular velocity
- **Magnetometer**: Measures magnetic field (for heading)
- **Fusion**: Combines measurements to estimate orientation

### IMU in Gazebo
Configuring an IMU sensor:

```xml
<link name="imu_link">
  <inertial>
    <mass value="0.01"/>
    <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
  </inertial>
</link>

<joint name="imu_joint" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>

<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <visualize>false</visualize>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/humanoid</namespace>
        <remapping>~/out:=imu</remapping>
      </ros>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
    </plugin>
  </sensor>
</gazebo>
```

### IMU ROS 2 Interface
IMU sensors publish to the `sensor_msgs/Imu` topic:

```python
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion
import rclpy
from rclpy.node import Node

class ImuProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')
        self.subscription = self.create_subscription(
            Imu,
            '/humanoid/imu',
            self.imu_callback,
            10
        )

    def imu_callback(self, msg):
        # Extract orientation from quaternion
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        # Extract angular velocity
        angular_velocity = msg.angular_velocity
        # Extract linear acceleration
        linear_acceleration = msg.linear_acceleration

        # Log orientation for balance control
        self.get_logger().info(f'Roll: {roll:.3f}, Pitch: {pitch:.3f}, Yaw: {yaw:.3f}')

        # Check for balance issues
        if abs(pitch) > 0.5:  # Robot tilting too much
            self.get_logger().warn('Robot may be losing balance!')
```

## Sensor Fusion for Humanoid Robots

### Combining Multiple Sensors
Humanoid robots benefit from sensor fusion to improve state estimation:

- **Visual-Inertial Odometry**: Combining camera and IMU data
- **LIDAR-Inertial Integration**: Fusing LiDAR and IMU for robust localization
- **Multi-Modal Perception**: Using all sensors for comprehensive environment understanding

### ROS 2 Sensor Fusion Tools
- **robot_localization**: Package for sensor fusion
- **message_filters**: For synchronizing sensor messages
- **tf2**: For coordinate frame transformations

## Sensor Calibration and Validation

### Calibration Process
- **Intrinsic Calibration**: Camera internal parameters
- **Extrinsic Calibration**: Sensor positions relative to robot
- **Temporal Calibration**: Synchronizing sensor timestamps

### Validation Techniques
- **Ground Truth Comparison**: Comparing with known positions
- **Cross-Sensor Validation**: Checking consistency between sensors
- **Real-World Testing**: Validating in physical environments

## Challenges and Considerations

### Sensor Limitations
- **Range Limitations**: Each sensor has effective range constraints
- **Environmental Conditions**: Performance varies with lighting, weather
- **Computational Requirements**: Processing sensor data in real-time

### Humanoid-Specific Considerations
- **Sensor Placement**: Positioning sensors for optimal humanoid perception
- **Occlusion Handling**: Managing when robot's own body blocks sensors
- **Motion Compensation**: Accounting for robot movement in sensor data

## Summary

Sensors are the eyes and ears of humanoid robots, providing essential information for navigation, manipulation, and interaction. LiDAR, depth cameras, and IMUs each provide unique information that, when properly integrated, enable robots to understand and interact with their physical environment. The next lesson will explore Unity for visualization and interaction.