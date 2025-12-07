---
id: chapter-3-lesson-1
title: "Chapter 3 – Lesson 1: Physics Simulation, Gravity, and Collisions in Gazebo"
---

# Chapter 3 – Lesson 1: Physics Simulation, Gravity, and Collisions in Gazebo

## Introduction to Gazebo Physics Simulation

Gazebo is a powerful 3D robotics simulator that accurately simulates rigid body dynamics, sensor data, and environmental interactions. For Physical AI and humanoid robotics, Gazebo provides a safe and controlled environment to develop, test, and validate robotic algorithms without the need for physical hardware.

## Physics Engine Fundamentals

Gazebo uses the Open Dynamics Engine (ODE) as its default physics engine, though it also supports Bullet and DART. The physics engine handles:

- **Gravity Simulation**: Accurate modeling of gravitational forces
- **Collision Detection**: Identifying when objects make contact
- **Contact Response**: Calculating forces and reactions during collisions
- **Rigid Body Dynamics**: Simulating movement and interaction of solid objects

### Gravity Configuration
In Gazebo, gravity is typically set to Earth's standard gravity (9.81 m/s²) but can be adjusted for different environments:

```xml
<sdf version="1.6">
  <world name="default">
    <gravity>0 0 -9.8</gravity>
    <!-- Other world elements -->
  </world>
</sdf>
```

For humanoid robots, accurate gravity simulation is crucial for:
- Balance and locomotion algorithms
- Walking gait development
- Fall recovery systems
- Manipulation planning

## Collision Detection in Gazebo

Collision detection is fundamental for realistic robot simulation. Gazebo uses several approaches:

### Collision Shapes
Common collision shapes used in humanoid robots:

- **Box**: Simple rectangular collision volumes
- **Sphere**: Spherical collision volumes
- **Cylinder**: Cylindrical collision volumes
- **Capsule**: Capsule-shaped volumes (good for limbs)
- **Mesh**: Complex shapes based on visual models

### Collision Parameters
Collision properties can be configured in URDF/SDF:

```xml
<link name="left_thigh">
  <collision>
    <geometry>
      <capsule>
        <radius>0.04</radius>
        <length>0.4</length>
      </capsule>
    </geometry>
    <surface>
      <friction>
        <ode>
          <mu>1.0</mu>
          <mu2>1.0</mu2>
        </ode>
      </friction>
      <bounce>
        <restitution_coefficient>0.01</restitution_coefficient>
        <threshold>100000</threshold>
      </bounce>
    </surface>
  </collision>
</link>
```

## Physics Properties and Parameters

### Mass and Inertia
Accurate mass and inertia properties are essential for realistic simulation:

- **Mass**: The mass of each link affects how it responds to forces
- **Inertia**: The distribution of mass affects rotational dynamics
- **Center of Mass**: Affects balance and stability

### Friction Properties
Friction is critical for humanoid locomotion:

- **Static Friction (mu)**: Prevents sliding when forces are small
- **Dynamic Friction (mu2)**: Determines resistance during sliding
- **Slip Compliance**: Allows for more realistic contact modeling

### Damping
Damping helps stabilize simulations:
- **Linear Damping**: Reduces linear velocity over time
- **Angular Damping**: Reduces angular velocity over time

## Setting Up Physics for Humanoid Robots

### World Configuration
Creating a physics-appropriate world for humanoid simulation:

```xml
<sdf version="1.6">
  <world name="humanoid_world">
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>
  </world>
</sdf>
```

### Robot-Specific Physics Parameters
Humanoid robots require specific physics considerations:

- **High Update Rates**: 1000+ Hz for stable bipedal control
- **Small Time Steps**: 0.001-0.002s for accurate contact simulation
- **Appropriate Friction**: High friction for feet to prevent slipping
- **Realistic Mass Distribution**: Based on actual humanoid robot designs

## Collision Handling Strategies

### Multi-Contact Points
For stable humanoid feet, multiple contact points may be necessary:

```xml
<link name="left_foot">
  <collision name="left_foot_front">
    <origin xyz="0.05 0 -0.02" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.08 0.04"/>
    </geometry>
  </collision>
  <collision name="left_foot_back">
    <origin xyz="-0.05 0 -0.02" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.08 0.04"/>
    </geometry>
  </collision>
</link>
```

### Self-Collision Avoidance
Humanoid robots need to avoid self-collision:

```xml
<link name="left_upper_arm">
  <collision>
    <geometry>
      <capsule>
        <radius>0.05</radius>
        <length>0.3</length>
      </capsule>
    </geometry>
  </collision>
  <!-- Disable self-collision with adjacent links -->
  <self_collide>false</self_collide>
</link>
```

## Common Physics Challenges in Humanoid Simulation

### Stability Issues
- **Time Step Size**: Too large can cause instability
- **Contact Parameters**: Poor friction settings can cause sliding
- **Mass Distribution**: Incorrect inertia can cause unrealistic movement

### Performance Optimization
- **Collision Simplification**: Use simpler shapes where possible
- **Update Rate Balance**: Balance accuracy with performance
- **Selective Physics**: Apply detailed physics only where needed

### Walking and Balance
Specific challenges for humanoid locomotion:
- **Foot Ground Contact**: Ensuring stable contact during walking
- **Balance Recovery**: Realistic fall and recovery dynamics
- **Stair Navigation**: Proper contact handling for step climbing

## Physics Debugging and Tuning

### Visualization Tools
Gazebo provides physics visualization:
- Contact visualization to see collision points
- Force visualization to see applied forces
- Center of mass visualization

### Parameter Tuning Process
1. Start with realistic physical values
2. Test basic movements (standing, simple motions)
3. Gradually increase complexity
4. Adjust parameters based on behavior
5. Validate against real robot behavior when possible

## Integration with ROS 2

Physics simulation integrates with ROS 2 through:
- Joint state publishers reporting simulated positions
- IMU sensors providing orientation data
- Force/torque sensors for contact detection
- Ground truth publishers for validation

## Summary

Physics simulation in Gazebo is fundamental to developing humanoid robots. Accurate gravity, collision detection, and physical properties enable realistic testing of control algorithms. Proper configuration of physics parameters is essential for stable and realistic humanoid simulation. The next lesson will explore sensor integration in Gazebo.