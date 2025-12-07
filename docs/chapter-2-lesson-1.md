---
id: chapter-2-lesson-1
title: "Chapter 2 – Lesson 1: ROS 2 Basics, Nodes, and Topics"
---

# Chapter 2 – Lesson 1: ROS 2 Basics, Nodes, and Topics

## Introduction to ROS 2

ROS 2 (Robot Operating System 2) is an open-source middleware that provides a standardized way for robotic components to communicate. It facilitates the development of complex robotic applications by abstracting hardware details and providing a robust communication framework. Unlike the original ROS, ROS 2 is built on DDS (Data Distribution Service) for improved real-time performance, security, and reliability.

## Core Concepts of ROS 2

### Nodes
Nodes are executable processes that perform computation in ROS 2. They can be thought of as individual programs that perform specific functions within a robotic system. Examples include:
- Camera driver nodes
- Motor control nodes
- Perception nodes
- Planning nodes

Each node typically runs in its own process and can communicate with other nodes through topics, services, or actions.

### Topics
Topics are anonymous publish/subscribe messaging buses. They enable asynchronous communication between nodes:
- Publisher nodes send data to topics
- Subscriber nodes receive data from topics
- Multiple publishers and subscribers can use the same topic
- Communication is one-way from publisher to subscriber

### Messages
Messages are the data structures that are passed between nodes through topics. They are defined in `.msg` files and can contain various data types including:
- Primitive types (integers, floats, booleans, strings)
- Arrays of primitive types
- Other message types (nested messages)

## Setting Up Your First ROS 2 Node

### Creating a Package
```bash
ros2 pkg create --build-type ament_python my_robot_package
```

### Basic Node Structure
A minimal ROS 2 node in Python includes:

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.get_clock().now().nanoseconds
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Working with Topics

### Publisher Example
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'robot_status', 10)
        self.timer = self.create_timer(1.0, self.publish_status)

    def publish_status(self):
        msg = String()
        msg.data = 'Robot is operational'
        self.publisher.publish(msg)
```

### Subscriber Example
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'robot_status',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
```

## Topic Communication Patterns

### Unidirectional Flow
- Publisher sends data without expecting a response
- Suitable for sensor data, status updates, or control commands

### Multiple Publishers and Subscribers
- Multiple nodes can publish to the same topic
- Multiple nodes can subscribe to the same topic
- Enables flexible system architecture

### Quality of Service (QoS) Settings
ROS 2 allows configuring how messages are delivered:
- Reliability (reliable vs best effort)
- Durability (transient local vs volatile)
- History (keep last N vs keep all messages)

## Best Practices for Topic Usage

### Naming Conventions
- Use descriptive, lowercase names
- Use forward slashes for hierarchical organization
- Example: `/arm/joint_states`, `/camera/rgb/image_raw`

### Message Design
- Keep messages small and efficient
- Use appropriate data types
- Consider bandwidth and processing constraints

### Error Handling
- Handle connection failures gracefully
- Implement timeouts for critical communications
- Log communication issues for debugging

## Humanoid Robot Applications

In humanoid robots, topics are used for:
- Joint state publishing
- Sensor data distribution
- Control command distribution
- Status and diagnostic information
- Coordination between different subsystems

## Summary

Topics and nodes form the foundation of ROS 2 communication. Understanding these concepts is essential for building distributed robotic systems that can coordinate effectively. In the next lesson, we'll explore services and actions for more complex communication patterns.