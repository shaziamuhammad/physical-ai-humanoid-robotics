---
id: chapter-3-lesson-3
title: "Chapter 3 – Lesson 3: Unity for Visualization and Interaction"
---

# Chapter 3 – Lesson 3: Unity for Visualization and Interaction

## Introduction to Unity in Robotics

Unity, a popular real-time 3D development platform, can be leveraged to create rich and interactive visualizations for robotic digital twins. While Gazebo excels at physics simulation, Unity offers superior graphical fidelity and user interaction capabilities. This lesson explores how Unity can complement Gazebo in the robotics development pipeline.

## Unity vs Gazebo: Complementary Strengths

### Gazebo Strengths
- Accurate physics simulation
- Realistic sensor emulation
- Integration with ROS/ROS 2
- Fast simulation for algorithm testing

### Unity Strengths
- High-quality graphics and rendering
- Advanced visualization capabilities
- Intuitive user interface design
- Cross-platform deployment
- Rich interaction models

## Setting Up Unity for Robotics

### Unity Robotics Hub
The Unity Robotics Hub provides essential tools:
- **Unity ROS TCP Connector**: Enables communication between Unity and ROS
- **Unity Robotics Package**: Provides robotics-specific components
- **Sample Environments**: Pre-built robotics scenarios

### Basic ROS Connection
Setting up communication between Unity and ROS:

```csharp
using UnityEngine;
using RosSharp;

public class RobotController : MonoBehaviour
{
    private RosSocket rosSocket;
    private string rosBridgeUrl = "ws://127.0.0.1:9090";

    void Start()
    {
        // Connect to ROS bridge
        rosSocket = new RosSocket(new WebSocketNetSharp(rosBridgeUrl));

        // Subscribe to robot joint states
        rosSocket.Subscribe<sensor_msgs.JointState>("/joint_states", OnJointStateReceived);

        // Publish to robot control topic
        InvokeRepeating("PublishRobotCommands", 0.0f, 0.1f);
    }

    void OnJointStateReceived(sensor_msgs.JointState jointState)
    {
        // Update Unity robot model based on joint states
        UpdateRobotModel(jointState);
    }

    void UpdateRobotModel(sensor_msgs.JointState jointState)
    {
        // Update each joint in the Unity robot model
        for (int i = 0; i < jointState.name.Count; i++)
        {
            string jointName = jointState.name[i];
            float jointPosition = (float)jointState.position[i];

            // Find corresponding joint in Unity model and update
            Transform jointTransform = FindJointByName(jointName);
            if (jointTransform != null)
            {
                // Apply rotation based on joint position
                jointTransform.localRotation = Quaternion.Euler(0, jointPosition * Mathf.Rad2Deg, 0);
            }
        }
    }

    void PublishRobotCommands()
    {
        // Create and publish joint commands
        var jointCmd = new control_msgs.JointTrajectoryControllerState();
        // Populate command data
        rosSocket.Publish("/joint_commands", jointCmd);
    }
}
```

## Unity Robotics Components

### URDF Importer
Unity provides tools to import URDF files directly:

```csharp
using Unity.Robotics;
using Unity.Robotics.ROSTCPConnector;

public class URDFLoader : MonoBehaviour
{
    public string urdfPath;

    void Start()
    {
        // Load URDF robot model into Unity
        LoadRobotFromURDF(urdfPath);
    }

    void LoadRobotFromURDF(string path)
    {
        // Unity's URDF Importer automatically creates a kinematically correct robot
        // based on the URDF file, including proper joint constraints and visualization
    }
}
```

### Joint Control in Unity
Controlling robot joints within Unity:

```csharp
using UnityEngine;

public class JointController : MonoBehaviour
{
    public float jointLimitMin = -1.57f;
    public float jointLimitMax = 1.57f;
    public float jointSpeed = 1.0f;

    private ConfigurableJoint joint;
    private float targetAngle = 0.0f;

    void Start()
    {
        joint = GetComponent<ConfigurableJoint>();
    }

    void Update()
    {
        // Move joint toward target position
        float currentAngle = GetJointAngle();
        float newAngle = Mathf.MoveTowards(currentAngle, targetAngle, jointSpeed * Time.deltaTime);
        SetJointAngle(Mathf.Clamp(newAngle, jointLimitMin, jointLimitMax));
    }

    public void SetTargetAngle(float angle)
    {
        targetAngle = Mathf.Clamp(angle, jointLimitMin, jointLimitMax);
    }

    float GetJointAngle()
    {
        // Calculate current joint angle
        return transform.localEulerAngles.x;
    }

    void SetJointAngle(float angle)
    {
        // Apply new joint angle
        transform.localEulerAngles = new Vector3(angle, 0, 0);
    }
}
```

## Creating Interactive Environments

### Physics in Unity vs Gazebo
While Unity has a physics engine, for robotics simulation it's often used more for visualization:

```csharp
using UnityEngine;

public class InteractiveEnvironment : MonoBehaviour
{
    // Objects that can be interacted with
    public GameObject[] interactableObjects;

    void Start()
    {
        // Set up interactive environment
        SetupEnvironment();
    }

    void SetupEnvironment()
    {
        // Configure objects for interaction
        foreach (GameObject obj in interactableObjects)
        {
            // Add interaction components
            obj.AddComponent<Rigidbody>();
            obj.AddComponent<InteractableObject>();
        }
    }
}

public class InteractableObject : MonoBehaviour
{
    void OnMouseDown()
    {
        // Handle object interaction
        Debug.Log($"Interacted with {gameObject.name}");

        // Could send ROS message to robot controller
        SendInteractionToRobot();
    }

    void SendInteractionToRobot()
    {
        // Send message to ROS indicating interaction
        // This could trigger robot to move to this object
    }
}
```

## Human-Robot Interaction (HRI) in Unity

### Visual Feedback Systems
Creating intuitive interfaces for human-robot interaction:

```csharp
using UnityEngine;
using UnityEngine.UI;

public class HRIInterface : MonoBehaviour
{
    public Text robotStatusText;
    public Image attentionIndicator;
    public Button[] robotCommandButtons;

    void Start()
    {
        // Initialize HRI interface
        InitializeInterface();
    }

    void InitializeInterface()
    {
        // Set up buttons for robot commands
        foreach (Button btn in robotCommandButtons)
        {
            btn.onClick.AddListener(() => SendCommandToRobot(btn.name));
        }
    }

    void UpdateRobotStatus(string status)
    {
        robotStatusText.text = status;

        // Update visual indicators based on status
        if (status.Contains("busy"))
        {
            attentionIndicator.color = Color.yellow;
        }
        else if (status.Contains("error"))
        {
            attentionIndicator.color = Color.red;
        }
        else
        {
            attentionIndicator.color = Color.green;
        }
    }

    void SendCommandToRobot(string command)
    {
        // Send command to robot via ROS
        Debug.Log($"Sending command: {command}");
    }
}
```

### Gesture Recognition
Implementing gesture-based control:

```csharp
using UnityEngine;

public class GestureController : MonoBehaviour
{
    public Camera mainCamera;
    private Vector3 lastMousePosition;
    private bool isDragging = false;

    void Update()
    {
        HandleMouseInput();
    }

    void HandleMouseInput()
    {
        if (Input.GetMouseButtonDown(0))
        {
            lastMousePosition = Input.mousePosition;
            isDragging = true;
        }
        else if (Input.GetMouseButtonUp(0))
        {
            isDragging = false;
        }
        else if (isDragging && Input.GetMouseButton(0))
        {
            Vector3 currentMousePos = Input.mousePosition;
            Vector3 mouseDelta = currentMousePos - lastMousePosition;

            // Interpret mouse movement as gesture
            if (mouseDelta.magnitude > 10.0f)
            {
                InterpretGesture(mouseDelta);
                lastMousePosition = currentMousePos;
            }
        }
    }

    void InterpretGesture(Vector3 delta)
    {
        // Map mouse gestures to robot commands
        if (Mathf.Abs(delta.x) > Mathf.Abs(delta.y))
        {
            // Horizontal movement - turn robot
            string direction = delta.x > 0 ? "turn_right" : "turn_left";
            SendCommandToRobot(direction);
        }
        else
        {
            // Vertical movement - move forward/backward
            string direction = delta.y > 0 ? "move_forward" : "move_backward";
            SendCommandToRobot(direction);
        }
    }
}
```

## Dashboard and Visualization Systems

### Custom Dashboards
Creating custom dashboards to visualize robot data:

```csharp
using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;

public class RobotDashboard : MonoBehaviour
{
    public Text jointPositionText;
    public Text sensorDataText;
    public Slider batteryLevelSlider;
    public Image[] cameraFeeds;

    private Dictionary<string, float> jointPositions;
    private Dictionary<string, float> sensorValues;

    void Start()
    {
        jointPositions = new Dictionary<string, float>();
        sensorValues = new Dictionary<string, float>();

        InvokeRepeating("UpdateDashboard", 0.0f, 0.1f);
    }

    void UpdateDashboard()
    {
        // Update joint positions display
        string jointInfo = "";
        foreach (var kvp in jointPositions)
        {
            jointInfo += $"{kvp.Key}: {kvp.Value:F2} rad\n";
        }
        jointPositionText.text = jointInfo;

        // Update sensor data display
        string sensorInfo = "";
        foreach (var kvp in sensorValues)
        {
            sensorInfo += $"{kvp.Key}: {kvp.Value:F2}\n";
        }
        sensorDataText.text = sensorInfo;

        // Update battery level
        float batteryLevel = GetBatteryLevel();
        batteryLevelSlider.value = batteryLevel;
    }

    public void UpdateJointPosition(string jointName, float position)
    {
        if (jointPositions.ContainsKey(jointName))
        {
            jointPositions[jointName] = position;
        }
        else
        {
            jointPositions.Add(jointName, position);
        }
    }

    public void UpdateSensorValue(string sensorName, float value)
    {
        if (sensorValues.ContainsKey(sensorName))
        {
            sensorValues[sensorName] = value;
        }
        else
        {
            sensorValues.Add(sensorName, value);
        }
    }

    float GetBatteryLevel()
    {
        // In real implementation, this would come from robot
        return 0.85f; // Example: 85% battery
    }
}
```

## Integration with ROS/ROS 2

### ROS TCP Connector
Using the Unity ROS TCP Connector for communication:

```csharp
using UnityEngine;
using RosSharp;

public class UnityRosBridge : MonoBehaviour
{
    private RosSocket rosSocket;
    private string rosBridgeUrl = "ws://localhost:9090";

    [System.Serializable]
    public class RosTopics
    {
        public string jointStates = "/joint_states";
        public string jointCommands = "/joint_group_position_controller/command";
        public string robotStatus = "/robot_status";
    }

    public RosTopics topics;

    void Start()
    {
        ConnectToRosBridge();
        SetupSubscribers();
        SetupPublishers();
    }

    void ConnectToRosBridge()
    {
        try
        {
            rosSocket = new RosSocket(new WebSocketNetSharp(rosBridgeUrl));
            Debug.Log("Connected to ROS Bridge");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Failed to connect to ROS Bridge: {e.Message}");
        }
    }

    void SetupSubscribers()
    {
        rosSocket.Subscribe<sensor_msgs.JointState>(topics.jointStates, OnJointStatesReceived);
        rosSocket.Subscribe<std_msgs.String>(topics.robotStatus, OnRobotStatusReceived);
    }

    void SetupPublishers()
    {
        // Publishers are typically created when needed
    }

    void OnJointStatesReceived(sensor_msgs.JointState jointState)
    {
        // Update Unity robot model
        UpdateUnityRobot(jointState);
    }

    void OnRobotStatusReceived(std_msgs.String status)
    {
        // Update dashboard with robot status
        Debug.Log($"Robot Status: {status.data}");
    }

    void UpdateUnityRobot(sensor_msgs.JointState jointState)
    {
        // Update the Unity representation of the robot based on joint states
        for (int i = 0; i < jointState.name.Count; i++)
        {
            string jointName = jointState.name[i];
            float jointPosition = (float)jointState.position[i];

            // Update corresponding joint in Unity model
            Transform jointTransform = FindJointByName(jointName);
            if (jointTransform != null)
            {
                // Apply the joint position to the Unity model
                ApplyJointPosition(jointTransform, jointPosition);
            }
        }
    }

    Transform FindJointByName(string name)
    {
        // Find joint in Unity hierarchy
        Transform[] allChildren = GetComponentsInChildren<Transform>();
        foreach (Transform child in allChildren)
        {
            if (child.name == name)
                return child;
        }
        return null;
    }

    void ApplyJointPosition(Transform joint, float position)
    {
        // Apply position based on joint type
        joint.localRotation = Quaternion.Euler(0, position * Mathf.Rad2Deg, 0);
    }
}
```

## Educational Applications

### Learning Tools
Unity's visualization capabilities make it excellent for educational purposes:

```csharp
using UnityEngine;

public class RoboticsEducator : MonoBehaviour
{
    public GameObject[] lessonObjects;
    public Text lessonInstructions;
    public Button nextLessonButton;

    private int currentLesson = 0;

    void Start()
    {
        LoadLesson(currentLesson);
    }

    public void LoadLesson(int lessonIndex)
    {
        // Hide all lesson objects
        foreach (GameObject obj in lessonObjects)
        {
            obj.SetActive(false);
        }

        // Show current lesson objects
        if (lessonIndex < lessonObjects.Length)
        {
            lessonObjects[lessonIndex].SetActive(true);
            lessonInstructions.text = GetLessonInstructions(lessonIndex);
        }
    }

    string GetLessonInstructions(int lessonIndex)
    {
        // Return appropriate instructions for the lesson
        switch (lessonIndex)
        {
            case 0:
                return "Learn about robot joints and their range of motion. Use the sliders to control each joint.";
            case 1:
                return "Explore how sensors work. Move the robot around and see how sensor values change.";
            default:
                return "Robotics lesson instructions";
        }
    }

    public void NextLesson()
    {
        if (currentLesson < lessonObjects.Length - 1)
        {
            currentLesson++;
            LoadLesson(currentLesson);
        }
    }
}
```

## Performance Considerations

### Optimization Strategies
- **Level of Detail (LOD)**: Reduce detail for distant objects
- **Occlusion Culling**: Don't render hidden objects
- **Texture Compression**: Optimize textures for real-time rendering
- **Efficient Shaders**: Use simple shaders when possible

### Real-time Requirements
Unity visualization should maintain at least 30 FPS for smooth interaction:
- Keep polygon counts reasonable
- Use efficient lighting systems
- Optimize asset loading and management

## Summary

Unity provides powerful visualization and interaction capabilities that complement Gazebo's physics simulation. By combining both tools, developers can create comprehensive digital twin experiences for humanoid robots with realistic physics and high-quality visualization. Unity's strength in user interface design and interaction makes it ideal for human-robot interaction interfaces, educational tools, and custom dashboards. The next chapter will explore NVIDIA Isaac for AI-driven robotics.