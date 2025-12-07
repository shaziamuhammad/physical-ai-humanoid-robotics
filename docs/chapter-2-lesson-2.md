---
id: chapter-2-lesson-2
title: "Chapter 2 – Lesson 2: Services, Actions, and Communication Patterns"
---

# Chapter 2 – Lesson 2: Services, Actions, and Communication Patterns

## Introduction to Communication Patterns

While topics provide asynchronous, one-way communication, ROS 2 offers additional communication patterns for more complex interactions. Services and actions are essential for request/response and long-running operations respectively.

## Services in ROS 2

Services provide synchronous, request/reply communication patterns, typically used for immediate, synchronous operations.

### Service Structure
A service consists of:
- Request message (sent from client to server)
- Response message (sent from server to client)
- Service definition file (.srv) containing both request and response

### Service Example
Creating a simple service to control a humanoid robot's LED:

**Service Definition (SetLED.srv):**
```
bool state
---
bool success
string message
```

**Service Server:**
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class LEDService(Node):
    def __init__(self):
        super().__init__('led_service')
        self.srv = self.create_service(
            SetBool,
            'set_led',
            self.set_led_callback
        )

    def set_led_callback(self, request, response):
        # Simulate LED control
        if request.data:
            self.get_logger().info('Turning LED ON')
            response.success = True
            response.message = 'LED turned ON'
        else:
            self.get_logger().info('Turning LED OFF')
            response.success = True
            response.message = 'LED turned OFF'

        return response

def main(args=None):
    rclpy.init(args=args)
    led_service = LEDService()
    rclpy.spin(led_service)
    rclpy.shutdown()
```

**Service Client:**
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class LEDClient(Node):
    def __init__(self):
        super().__init__('led_client')
        self.cli = self.create_client(SetBool, 'set_led')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = SetBool.Request()

    def send_request(self, led_state):
        self.req.data = led_state
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    led_client = LEDClient()
    response = led_client.send_request(True)  # Turn LED on
    print(f'Result: {response.success}, {response.message}')
    rclpy.shutdown()
```

## Actions in ROS 2

Actions are designed for long-running, asynchronous tasks that provide feedback during execution. They're ideal for complex maneuvers like navigation goals.

### Action Structure
An action includes:
- Goal message (what to do)
- Feedback message (progress updates)
- Result message (final outcome)

### Action Example
Creating an action for humanoid robot navigation:

**Action Definition (NavigateToPose.action):**
```
# Goal
geometry_msgs/PoseStamped pose
---
# Result
bool reached
float32 distance_traveled
---
# Feedback
float32 distance_to_goal
geometry_msgs/PoseStamped current_pose
```

**Action Server:**
```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

class NavigateActionServer(Node):
    def __init__(self):
        super().__init__('navigate_action_server')
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info('Received navigation goal')

        # Simulate navigation progress
        feedback_msg = NavigateToPose.Feedback()
        result = NavigateToPose.Result()

        for i in range(10):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.reached = False
                self.get_logger().info('Navigation canceled')
                return result

            # Update feedback
            feedback_msg.distance_to_goal = 10.0 - float(i)
            goal_handle.publish_feedback(feedback_msg)

            # Sleep to simulate navigation
            time.sleep(1)

        goal_handle.succeed()
        result.reached = True
        result.distance_traveled = 10.0
        self.get_logger().info('Navigation completed successfully')
        return result
```

## Communication Pattern Selection

### When to Use Topics
- Sensor data publishing
- Continuous status updates
- Broadcasting information to multiple subscribers
- Control commands that don't require acknowledgment

### When to Use Services
- Simple, quick operations
- Configuration changes
- One-time requests with immediate responses
- Validation or checking operations

### When to Use Actions
- Long-running operations
- Tasks requiring feedback during execution
- Operations that might be canceled
- Complex maneuvers with intermediate results

## Advanced Communication Patterns

### Latching Topics
Latching ensures that the last published message is sent to new subscribers immediately upon connection:

```python
publisher = self.create_publisher(String, 'latched_topic', 10,
                                 qos_profile=qos_profile_latched_msg)
```

### Time Synchronization
Using message filters for time-based synchronization:

```python
from message_filters import ApproximateTimeSynchronizer, Subscriber

def sync_callback(self, image_msg, depth_msg):
    # Process synchronized image and depth data
    pass

image_sub = Subscriber(self, Image, 'camera/image')
depth_sub = Subscriber(self, Image, 'camera/depth')
sync = ApproximateTimeSynchronizer([image_sub, depth_sub], queue_size=10, slop=0.1)
sync.registerCallback(self.sync_callback)
```

## Communication in Humanoid Robots

Humanoid robots require sophisticated communication patterns:

### Coordination Patterns
- Joint control coordination between multiple controllers
- Sensor fusion across different modalities
- Task-level planning and execution coordination

### Safety Communication
- Emergency stop propagation
- Collision detection and avoidance
- System health monitoring

### Human-Robot Interaction
- Command interpretation
- Status feedback to users
- Multi-modal interaction coordination

## Quality of Service (QoS) Considerations

Different communication patterns may require different QoS settings:

### For Sensors
- Reliable delivery for critical sensors
- Best effort for high-frequency data
- Appropriate history depth for data buffering

### For Control Commands
- Reliable delivery for safety-critical commands
- Appropriate deadline and lifespan settings
- Durability for system state messages

## Summary

Services and actions provide essential communication patterns beyond topics for complex robotic systems. Understanding when to use each pattern is crucial for designing effective humanoid robot systems. In the next lesson, we'll explore rclpy and URDF for humanoid robots.