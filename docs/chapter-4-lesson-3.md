---
id: chapter-4-lesson-3
title: "Chapter 4 – Lesson 3: Isaac ROS + Nav2 for VSLAM and Navigation"
---

# Chapter 4 – Lesson 3: Isaac ROS + Nav2 for VSLAM and Navigation

## Introduction to Isaac ROS and Navigation

Isaac ROS provides a collection of hardware-accelerated ROS 2 packages that enhance robotic perception and navigation. When combined with the Navigation2 (Nav2) framework, Isaac ROS enables advanced Visual Simultaneous Localization and Mapping (VSLAM) and navigation capabilities for humanoid robots. This lesson explores how to integrate these technologies for robust perception and navigation.

## Isaac ROS Overview

### Key Isaac ROS Packages

#### Isaac ROS Visual SLAM (VSLAM)
Isaac ROS VSLAM provides accelerated visual SLAM capabilities:
- **Hardware Acceleration**: Leverages NVIDIA GPUs for real-time performance
- **Multi-sensor Fusion**: Combines visual and inertial data
- **Robust Tracking**: Maintains tracking even in challenging conditions

#### Isaac ROS Apriltag
Detects and localizes AprilTag markers:
- **GPU Acceleration**: Fast detection using CUDA
- **High Precision**: Accurate pose estimation
- **Robust Detection**: Works in various lighting conditions

#### Isaac ROS Stereo DNN
Performs stereo depth estimation and neural network inference:
- **Real-time Processing**: Accelerated depth estimation
- **Object Detection**: Integrated object detection capabilities
- **Semantic Segmentation**: Pixel-level scene understanding

### Installation and Setup
```bash
# Install Isaac ROS packages
sudo apt update
sudo apt install ros-humble-isaac-ros-common
sudo apt install ros-humble-isaac-ros-visual-slam
sudo apt install ros-humble-isaac-ros-apriltag
sudo apt install ros-humble-isaac-ros-stereo-dnn
```

## Isaac ROS Visual SLAM Integration

### VSLAM Node Configuration
Setting up Isaac ROS VSLAM for humanoid robots:

```yaml
# config/vslam_config.yaml
camera_info_url: "package://robot_description/config/camera_info.yaml"
image_width: 1280
image_height: 720
enable_rectification: true
enable_imu_fusion: true
map_frame: "map"
odom_frame: "odom"
base_frame: "base_link"
publish_tf: true
use_sim_time: true
```

### Launch File for VSLAM
```xml
<!-- launch/vslam.launch.py -->
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('humanoid_navigation'),
        'config',
        'vslam_config.yaml'
    )

    vslam_node = Node(
        package='isaac_ros_visual_slam',
        executable='visual_slam_node',
        name='visual_slam',
        parameters=[config],
        remappings=[
            ('/visual_slam/image_raw', '/camera/rgb/image_raw'),
            ('/visual_slam/camera_info', '/camera/rgb/camera_info'),
            ('/visual_slam/imu', '/imu/data')
        ]
    )

    return LaunchDescription([
        vslam_node
    ])
```

### ROS 2 Node Integration
Integrating VSLAM with ROS 2:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Image, CameraInfo, Imu
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import numpy as np

class IsaacVSLAMIntegrator(Node):
    def __init__(self):
        super().__init__('isaac_vslam_integrator')

        # Subscribe to VSLAM pose output
        self.vslam_pose_sub = self.create_subscription(
            PoseStamped,
            '/visual_slam/pose',
            self.vslam_pose_callback,
            10
        )

        # Subscribe to odometry for comparison
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # TF broadcaster for VSLAM transforms
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publisher for fused pose
        self.fused_pose_pub = self.create_publisher(
            PoseStamped,
            '/fused_pose',
            10
        )

        self.latest_vslam_pose = None
        self.latest_odom = None

    def vslam_pose_callback(self, msg):
        """Handle VSLAM pose updates"""
        self.latest_vslam_pose = msg

        # Broadcast VSLAM transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'vslam_odom'
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation

        self.tf_broadcaster.sendTransform(t)

        # Fuse with other sources if available
        if self.latest_odom:
            fused_pose = self.fuse_poses(msg, self.latest_odom)
            self.fused_pose_pub.publish(fused_pose)

    def odom_callback(self, msg):
        """Handle odometry updates"""
        self.latest_odom = msg

    def fuse_poses(self, vslam_pose, odom):
        """Fuse VSLAM and odometry data"""
        # Simple fusion - in practice, use a more sophisticated filter
        fused_pose = PoseStamped()
        fused_pose.header = vslam_pose.header
        fused_pose.header.frame_id = 'map'

        # Weighted average based on confidence
        vslam_weight = 0.7  # VSLAM is more reliable for global position
        odom_weight = 0.3   # Odometry is more reliable for short-term motion

        fused_pose.pose.position.x = (
            vslam_weight * vslam_pose.pose.position.x +
            odom_weight * odom.pose.pose.position.x
        )
        fused_pose.pose.position.y = (
            vslam_weight * vslam_pose.pose.position.y +
            odom_weight * odom.pose.pose.position.y
        )
        fused_pose.pose.position.z = (
            vslam_weight * vslam_pose.pose.position.z +
            odom_weight * odom.pose.pose.position.z
        )

        # For orientation, prefer VSLAM's global reference
        fused_pose.pose.orientation = vslam_pose.pose.orientation

        return fused_pose
```

## Navigation2 (Nav2) Framework

### Nav2 Architecture
Navigation2 consists of several key components:
- **Navigation Server**: Main orchestrator
- **Local Planner**: Local trajectory planning
- **Global Planner**: Global path planning
- **Controller**: Low-level motion control
- **Recovery Behaviors**: Failure recovery strategies

### Nav2 Configuration for Humanoid Robots
```yaml
# config/nav2_params.yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: "map"
    robot_base_frame: "base_link"
    odom_topic: "/odom"
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    # Specify the path where the BT XML files are located
    default_nav_through_poses_bt_xml: "package://nav2_bt_navigator/behavior_trees/navigate_w_replanning_and_recovery.xml"
    default_nav_to_pose_bt_xml: "package://nav2_bt_navigator/behavior_trees/navigate_w_replanning_and_recovery.xml"
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node
    - nav2_controller_cancel_bt_node
    - nav2_path_longer_on_approach_bt_node
    - nav2_wait_cancel_bt_node

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Progress checker parameters
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    # Goal checker parameters
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True

    # Humanoid-specific controller
    FollowPath:
      plugin: "nav2_rotation_shim_controller::RotationShimController"
      progress_checker_plugin: "progress_checker"
      goal_checker_plugin: "goal_checker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
```

### Humanoid-Specific Navigation Parameters
```yaml
# config/humanoid_nav_params.yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0
      publish_frequency: 5.0
      global_frame: odom
      robot_base_frame: base_footprint
      use_sim_time: True
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05
      transform_tolerance: 0.5
      footprint: "[ [0.3, 0.2], [0.3, -0.2], [-0.3, -0.2], [-0.3, 0.2] ]"
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 10
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_footprint
      use_sim_time: True
      robot_radius: 0.3
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
```

## Isaac ROS and Nav2 Integration

### Launch File for Full Integration
```xml
<!-- launch/navigation.launch.py -->
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    declare_slam = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Enable SLAM mode'
    )

    # Isaac ROS VSLAM
    vslam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('humanoid_navigation'),
                'launch',
                'vslam.launch.py'
            ])
        ]),
        condition=IfCondition(slam)
    )

    # Nav2 bringup
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ])
    )

    # Additional nodes for humanoid navigation
    humanoid_nav_node = Node(
        package='humanoid_navigation',
        executable='humanoid_nav_node',
        name='humanoid_nav_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            PathJoinSubstitution([
                FindPackageShare('humanoid_navigation'),
                'config',
                'humanoid_nav_params.yaml'
            ])
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_slam,
        vslam_launch,
        nav2_launch,
        humanoid_nav_node
    ])
```

### Custom Navigation Node for Humanoid Robots
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
import math

class HumanoidNavigationNode(Node):
    def __init__(self):
        super().__init__('humanoid_navigation_node')

        # Action client for Nav2
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher for navigation goals
        self.goal_publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

        # Service for navigation commands
        self.nav_service = self.create_service(
            NavigationCommand,
            'humanoid_navigate',
            self.navigation_command_callback
        )

    def navigation_command_callback(self, request, response):
        """Handle navigation commands"""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = request.x
        goal_pose.pose.position.y = request.y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.w = 1.0  # Default orientation

        # Transform to map frame if needed
        try:
            goal_pose = self.transform_pose(goal_pose, 'map')
        except Exception as e:
            self.get_logger().error(f'Transform failed: {e}')
            response.success = False
            response.message = f'Transform failed: {e}'
            return response

        # Send navigation goal
        self.send_navigation_goal(goal_pose)

        response.success = True
        response.message = 'Navigation goal sent'
        return response

    def send_navigation_goal(self, pose_stamped):
        """Send navigation goal to Nav2"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped

        self.nav_to_pose_client.wait_for_server()
        self.nav_to_pose_client.send_goal_async(goal_msg)

    def transform_pose(self, pose_stamped, target_frame):
        """Transform pose to target frame"""
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                pose_stamped.header.frame_id,
                rclpy.time.Time()
            )
            return tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
        except Exception as e:
            self.get_logger().error(f'Transform error: {e}')
            raise

    def check_navigation_progress(self):
        """Monitor navigation progress and adjust for humanoid-specific needs"""
        # Check if robot is stuck
        if self.is_robot_stuck():
            self.handle_stuck_robot()

        # Check balance during navigation
        if not self.is_robot_balanced():
            self.slow_down_navigation()

    def is_robot_stuck(self):
        """Check if robot is stuck using various metrics"""
        # Implementation depends on robot sensors
        # Check if robot position hasn't changed significantly
        # Check if robot is oscillating
        # Check if robot is in collision
        pass

    def handle_stuck_robot(self):
        """Handle robot being stuck"""
        # Implement recovery behaviors
        # Back up slightly
        # Rotate in place
        # Request new plan
        pass

    def is_robot_balanced(self):
        """Check if humanoid robot is maintaining balance"""
        # Check IMU data for excessive tilt
        # Check joint positions for balance
        pass

    def slow_down_navigation(self):
        """Reduce navigation speed to maintain balance"""
        # Send command to reduce speed
        # Adjust controller parameters
        pass
```

## VSLAM and Navigation Integration Strategies

### Multi-Sensor Fusion
Combining VSLAM with other sensors for robust navigation:

```python
class MultiSensorFusionNode(Node):
    def __init__(self):
        super().__init__('multi_sensor_fusion')

        # Subscriptions for different sensors
        self.vslam_sub = self.create_subscription(
            PoseStamped,
            '/visual_slam/pose',
            self.vslam_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        # Publisher for fused pose
        self.fused_pose_pub = self.create_publisher(
            PoseStamped,
            '/fused_pose',
            10
        )

        # Extended Kalman Filter for sensor fusion
        self.ekf = self.initialize_ekf()

    def initialize_ekf(self):
        """Initialize Extended Kalman Filter for sensor fusion"""
        # State: [x, y, z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz]
        # Process noise and measurement noise matrices
        # Transition and observation models
        pass

    def vslam_callback(self, msg):
        """Process VSLAM measurements"""
        # Extract position and orientation from VSLAM
        vslam_position = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]

        vslam_orientation = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]

        # Update EKF with VSLAM measurement
        self.update_ekf_with_vslam(vslam_position, vslam_orientation)

    def odom_callback(self, msg):
        """Process odometry measurements"""
        # Extract pose and twist from odometry
        odom_position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]

        # Update EKF with odometry measurement
        self.update_ekf_with_odom(odom_position, msg.twist.twist)

    def update_ekf_with_vslam(self, position, orientation):
        """Update EKF with VSLAM measurement"""
        # Convert to measurement vector
        measurement = np.array([
            position[0], position[1], position[2],
            orientation[0], orientation[1], orientation[2], orientation[3]
        ])

        # Add to measurement queue
        self.ekf.add_measurement(measurement, 'vslam')

    def update_ekf_with_odom(self, position, twist):
        """Update EKF with odometry measurement"""
        # Convert to measurement vector
        measurement = np.array([
            position[0], position[1], position[2],
            twist.linear.x, twist.linear.y, twist.linear.z
        ])

        # Add to measurement queue
        self.ekf.add_measurement(measurement, 'odom')

    def publish_fused_pose(self):
        """Publish the fused pose estimate"""
        fused_state = self.ekf.get_state_estimate()

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = 'map'

        pose_stamped.pose.position.x = fused_state[0]
        pose_stamped.pose.position.y = fused_state[1]
        pose_stamped.pose.position.z = fused_state[2]

        # Convert orientation from state vector
        pose_stamped.pose.orientation = self.state_to_quaternion(fused_state[3:7])

        self.fused_pose_pub.publish(pose_stamped)
```

## Performance Considerations

### Optimization Strategies
- **GPU Utilization**: Maximize GPU usage for Isaac ROS nodes
- **CPU Allocation**: Assign navigation nodes to appropriate CPU cores
- **Memory Management**: Efficiently manage memory for large maps
- **Communication**: Optimize ROS 2 communication for real-time performance

### Real-time Constraints
For humanoid robots, navigation must respect real-time constraints:
- **Balance Maintenance**: Ensure navigation commands don't compromise balance
- **Reaction Time**: Respond quickly to obstacles and changes
- **Smooth Motion**: Generate smooth trajectories for stable locomotion

## Troubleshooting Common Issues

### VSLAM Failures
- **Feature Poor Environments**: Use additional sensors in textureless areas
- **Motion Blur**: Ensure appropriate camera settings and exposure
- **Lighting Changes**: Implement adaptive parameters for different conditions

### Navigation Failures
- **Localization Issues**: Ensure proper map quality and sensor calibration
- **Path Planning**: Verify costmap configuration and inflation parameters
- **Controller Issues**: Tune controller parameters for humanoid dynamics

## Summary

The integration of Isaac ROS VSLAM with Navigation2 provides powerful capabilities for humanoid robot navigation. Isaac ROS leverages NVIDIA's GPU acceleration for real-time perception, while Nav2 provides a robust navigation framework. The combination enables humanoid robots to perform complex navigation tasks with accurate localization and obstacle avoidance. Proper configuration of both systems, along with multi-sensor fusion, ensures reliable and safe navigation in dynamic environments. The next chapter will explore Vision-Language-Action systems for intelligent humanoid control.