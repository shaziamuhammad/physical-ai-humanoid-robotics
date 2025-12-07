---
id: chapter-5-lesson-2
title: "Chapter 5 – Lesson 2: LLM Planning: Turning 'Clean the Room' into a Plan"
---

# Chapter 5 – Lesson 2: LLM Planning: Turning 'Clean the Room' into a Plan

## Introduction to LLM-Based Task Planning

Large Language Models (LLMs) have revolutionized how we approach task planning in robotics. For Physical AI and humanoid robots, LLMs serve as intelligent intermediaries that can interpret high-level human commands and decompose them into executable robotic actions. This lesson explores how to leverage LLMs for creating detailed action plans from natural language instructions.

## Task Decomposition with LLMs

### The Planning Challenge
Human commands like "Clean the room" are high-level and ambiguous. The planning process involves:
- **Semantic Understanding**: Interpreting the intent behind the command
- **Task Decomposition**: Breaking down the high-level task into sub-tasks
- **Environment Analysis**: Understanding the current state and required actions
- **Action Mapping**: Translating sub-tasks into robot-specific actions

### Planning Architecture
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from action_msgs.msg import GoalStatus
import openai
import json
import re

class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner_node')

        # Configuration
        self.openai_api_key = self.declare_parameter(
            'openai_api_key',
            'your-api-key-here'
        ).value

        # Set up OpenAI client
        openai.api_key = self.openai_api_key

        # Subscribers and publishers
        self.command_sub = self.create_subscription(
            String,
            '/natural_language_command',
            self.command_callback,
            10
        )

        self.plan_pub = self.create_publisher(
            String,
            '/execution_plan',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/planner_status',
            10
        )

        # Robot capabilities (for grounding)
        self.robot_capabilities = [
            "move_to_location",
            "pick_up_object",
            "place_object",
            "grasp_object",
            "release_object",
            "navigate_to",
            "detect_object",
            "manipulate_object"
        ]

        # Object categories the robot can handle
        self.known_objects = [
            "book", "bottle", "cup", "phone", "keys",
            "pen", "paper", "trash", "dust", "dirt",
            "chair", "table", "desk", "floor"
        ]

        self.get_logger().info('LLM Planner initialized')

    def command_callback(self, msg):
        """Process natural language command"""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        try:
            # Generate plan using LLM
            plan = self.generate_plan(command)

            if plan:
                # Publish the plan
                plan_msg = String()
                plan_msg.data = json.dumps(plan)
                self.plan_pub.publish(plan_msg)

                self.get_logger().info(f'Generated plan: {plan}')

                # Publish status
                status_msg = String()
                status_msg.data = f"plan_generated:{len(plan)}_steps"
                self.status_pub.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f'Planning error: {e}')
            error_msg = String()
            error_msg.data = f"error:{str(e)}"
            self.status_pub.publish(error_msg)

    def generate_plan(self, command):
        """Generate execution plan using LLM"""
        prompt = self.create_planning_prompt(command)

        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": self.get_system_prompt()},
                {"role": "user", "content": prompt}
            ],
            temperature=0.3,
            max_tokens=1000,
            response_format={"type": "json_object"}
        )

        plan_text = response.choices[0].message.content
        plan = json.loads(plan_text)

        return self.validate_and_format_plan(plan)

    def create_planning_prompt(self, command):
        """Create prompt for plan generation"""
        prompt = f"""
        Command: "{command}"

        You are a robotic task planner. Your job is to decompose high-level commands into detailed execution steps.

        Available robot capabilities: {self.robot_capabilities}
        Known objects: {self.known_objects}

        Please create a detailed execution plan in JSON format with the following structure:
        {{
            "command": "original command",
            "description": "brief description of the plan",
            "steps": [
                {{
                    "id": step number,
                    "action": "robot action",
                    "parameters": {{"param": "value"}},
                    "description": "what this step does",
                    "dependencies": [list of step IDs this step depends on]
                }}
            ],
            "estimated_duration": "estimated time in seconds"
        }}

        Each action should be specific and executable by the robot.
        """
        return prompt

    def get_system_prompt(self):
        """System prompt for consistent planning"""
        return """
        You are an expert robotic task planner. Your role is to decompose natural language commands into detailed, executable robotic action plans.

        Guidelines:
        1. Each step should be a specific, executable action
        2. Consider the robot's physical capabilities and limitations
        3. Include necessary preconditions for each action
        4. Consider safety and efficiency in the plan order
        5. Be specific about objects, locations, and actions
        6. Handle ambiguous commands by making reasonable assumptions
        """
```

## Advanced Planning with Context

### Environment Context Integration
```python
class ContextAwarePlannerNode(LLMPlannerNode):
    def __init__(self):
        super().__init__()

        # Subscribe to environment information
        self.perception_sub = self.create_subscription(
            String,
            '/environment_state',
            self.perception_callback,
            10
        )

        self.current_environment = {}

    def perception_callback(self, msg):
        """Update environment state"""
        try:
            env_state = json.loads(msg.data)
            self.current_environment.update(env_state)
        except json.JSONDecodeError:
            self.get_logger().error('Failed to parse environment state')

    def generate_plan_with_context(self, command):
        """Generate plan with environment context"""
        context = {
            "current_objects": self.current_environment.get('objects', []),
            "current_location": self.current_environment.get('location', 'unknown'),
            "robot_state": self.current_environment.get('robot_state', {}),
            "known_areas": self.current_environment.get('areas', [])
        }

        prompt = f"""
        Command: "{command}"

        Current Environment Context:
        - Objects in environment: {context['current_objects']}
        - Robot location: {context['current_location']}
        - Robot state: {context['robot_state']}
        - Known areas: {context['known_areas']}

        Available robot capabilities: {self.robot_capabilities}
        Known objects: {self.known_objects}

        Based on the current environment, create a detailed execution plan in JSON format:
        {{
            "command": "{command}",
            "environment_context": {json.dumps(context)},
            "steps": [
                {{
                    "id": step number,
                    "action": "robot action",
                    "parameters": {{"param": "value"}},
                    "description": "what this step does",
                    "location": "where to perform action",
                    "object": "object to interact with",
                    "dependencies": [list of step IDs this step depends on]
                }}
            ]
        }}
        """

        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": self.get_system_prompt()},
                {"role": "user", "content": prompt}
            ],
            temperature=0.2,
            max_tokens=1500,
            response_format={"type": "json_object"}
        )

        plan_text = response.choices[0].message.content
        return json.loads(plan_text)
```

## Specific Example: "Clean the Room" Planning

### Handling the "Clean the Room" Command
```python
class CleanRoomPlannerNode(LLMPlannerNode):
    def __init__(self):
        super().__init__()
        self.command_sub = self.create_subscription(
            String,
            '/natural_language_command',
            self.enhanced_command_callback,
            10
        )

    def enhanced_command_callback(self, msg):
        """Process commands with special handling for cleaning tasks"""
        command = msg.data.lower().strip()

        # Special handling for cleaning commands
        if any(keyword in command for keyword in ["clean", "tidy", "organize", "arrange"]):
            plan = self.handle_cleaning_command(command)
        else:
            plan = self.generate_plan(command)

        if plan:
            plan_msg = String()
            plan_msg.data = json.dumps(plan)
            self.plan_pub.publish(plan_msg)

    def handle_cleaning_command(self, command):
        """Specialized handler for cleaning tasks"""
        # First, get environment context
        env_context = self.get_current_environment_context()

        # Create detailed cleaning-specific prompt
        prompt = f"""
        Cleaning Command: "{command}"

        Environment Context:
        - Current objects detected: {env_context.get('objects', [])}
        - Current room layout: {env_context.get('layout', 'unknown')}
        - Dirty areas detected: {env_context.get('dirty_areas', [])}
        - Cluttered areas detected: {env_context.get('cluttered_areas', [])}

        Robot Capabilities for Cleaning:
        - navigate_to_location
        - detect_dirt
        - detect_clutter
        - pick_up_object
        - place_object_in_correct_location
        - vacuum_area
        - wipe_surface
        - detect_object_category

        Object Categories:
        - trash_items: ["bottle", "wrapper", "paper", "can"]
        - misplaced_items: ["book", "clothes", "keys", "phone", "pen"]
        - cleaning_tools: ["duster", "cloth", "vacuum_attachment"]

        Please create a detailed cleaning plan in JSON format:
        {{
            "command": "{command}",
            "task_type": "cleaning",
            "primary_goal": "room_cleanliness",
            "steps": [
                {{
                    "id": 1,
                    "action": "scan_area",
                    "description": "Scan entire room to identify all items and dirty areas",
                    "location": "room_center",
                    "estimated_time": 10
                }},
                {{
                    "id": 2,
                    "action": "collect_trash",
                    "description": "Pick up all trash items",
                    "target_objects": ["trash_items"],
                    "location": "all_rooms",
                    "estimated_time": 60
                }},
                {{
                    "id": 3,
                    "action": "organize_items",
                    "description": "Place misplaced items in their correct locations",
                    "target_objects": ["misplaced_items"],
                    "location": "room",
                    "estimated_time": 120
                }}
            ],
            "success_criteria": ["room_appears_tidy", "no_trash_detected", "all_items_in_place"]
        }}
        """

        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": self.get_cleaning_system_prompt()},
                {"role": "user", "content": prompt}
            ],
            temperature=0.1,
            max_tokens=2000,
            response_format={"type": "json_object"}
        )

        plan_text = response.choices[0].message.content
        plan = json.loads(plan_text)

        return self.refine_cleaning_plan(plan)

    def get_cleaning_system_prompt(self):
        """System prompt for cleaning task planning"""
        return """
        You are an expert in household cleaning task planning for robots. Your goal is to create detailed, efficient cleaning plans that consider:

        1. Safety: Ensure the robot doesn't collide with obstacles or harm itself
        2. Efficiency: Minimize travel distance and time
        3. Thoroughness: Cover all areas that need cleaning
        4. Organization: Place items in logical, human-friendly locations
        5. Prioritization: Address the most important/obvious issues first

        Always include error handling and verification steps in your plans.
        """

    def refine_cleaning_plan(self, plan):
        """Refine the cleaning plan based on robot capabilities"""
        refined_steps = []

        for step in plan.get('steps', []):
            # Add preconditions and error handling
            refined_step = {
                **step,
                "preconditions": self.get_preconditions(step),
                "error_handling": self.get_error_handling(step),
                "verification": self.get_verification(step)
            }
            refined_steps.append(refined_step)

        plan['steps'] = refined_steps
        return plan

    def get_preconditions(self, step):
        """Get preconditions for a step"""
        action = step.get('action', '')
        if action == 'pick_up_object':
            return {
                "gripper_available": True,
                "object_reachable": True,
                "safety_clear": True
            }
        elif action == 'navigate_to_location':
            return {
                "path_clear": True,
                "destination_known": True,
                "battery_sufficient": True
            }
        return {}

    def get_error_handling(self, step):
        """Get error handling for a step"""
        action = step.get('action', '')
        if action == 'pick_up_object':
            return {
                "object_not_found_retry": 3,
                "gripper_failure_alternative": "detect_alternative_grasp",
                "drop_protection": True
            }
        elif action == 'navigate_to_location':
            return {
                "obstacle_avoidance": True,
                "replan_on_blockage": True,
                "localization_recovery": True
            }
        return {}

    def get_verification(self, step):
        """Get verification criteria for a step"""
        action = step.get('action', '')
        if action == 'pick_up_object':
            return {
                "gripper_status": "object_grasped",
                "object_detection": "object_no_longer_at_location"
            }
        elif action == 'place_object':
            return {
                "gripper_status": "object_released",
                "object_detection": "object_at_target_location"
            }
        return {}
```

## Plan Execution and Monitoring

### Plan Executor Node
```python
import time
from action_msgs.msg import GoalStatus

class PlanExecutorNode(Node):
    def __init__(self):
        super().__init__('plan_executor_node')

        # Subscribe to plans
        self.plan_sub = self.create_subscription(
            String,
            '/execution_plan',
            self.plan_callback,
            10
        )

        # Robot action clients
        self.navigation_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.manipulation_client = ActionClient(self, ManipulateObject, 'manipulate_object')

        # Publishers
        self.status_pub = self.create_publisher(String, '/execution_status', 10)
        self.current_step_pub = self.create_publisher(String, '/current_step', 10)

        self.current_plan = None
        self.current_step_index = 0
        self.execution_active = False

    def plan_callback(self, msg):
        """Execute incoming plan"""
        try:
            plan = json.loads(msg.data)
            self.current_plan = plan
            self.current_step_index = 0

            self.get_logger().info(f'Starting execution of plan with {len(plan["steps"])} steps')

            # Execute plan in separate thread
            import threading
            execution_thread = threading.Thread(target=self.execute_plan)
            execution_thread.start()

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid plan format: {e}')

    def execute_plan(self):
        """Execute the plan step by step"""
        self.execution_active = True

        for step_index, step in enumerate(self.current_plan['steps']):
            if not self.execution_active:
                break

            self.current_step_index = step_index
            self.publish_current_step(step)

            success = self.execute_step(step)

            if not success:
                self.get_logger().error(f'Step {step_index} failed: {step["description"]}')
                self.publish_status(f"step_failed:{step_index}")
                break
            else:
                self.get_logger().info(f'Step {step_index} completed: {step["description"]}')
                self.publish_status(f"step_completed:{step_index}")

        self.execution_active = False
        self.publish_status("plan_execution_complete")

    def execute_step(self, step):
        """Execute a single step"""
        action = step['action']

        try:
            if action == 'navigate_to_location':
                return self.execute_navigation_step(step)
            elif action == 'pick_up_object':
                return self.execute_manipulation_step(step)
            elif action == 'place_object':
                return self.execute_manipulation_step(step)
            elif action == 'scan_area':
                return self.execute_scan_step(step)
            else:
                # Handle other actions
                self.get_logger().info(f'Unknown action: {action}, skipping')
                return True

        except Exception as e:
            self.get_logger().error(f'Error executing step: {e}')
            return False

    def execute_navigation_step(self, step):
        """Execute navigation step"""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'

        # Extract position from step parameters
        params = step.get('parameters', {})
        goal.pose.pose.position.x = params.get('x', 0.0)
        goal.pose.pose.position.y = params.get('y', 0.0)
        goal.pose.pose.position.z = params.get('z', 0.0)

        # Execute navigation goal
        self.navigation_client.wait_for_server()
        future = self.navigation_client.send_goal_async(goal)

        # Wait for result with timeout
        import asyncio
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        try:
            result = loop.run_until_complete(
                asyncio.wait_for(future, timeout=30.0)
            )
            return result.status == GoalStatus.STATUS_SUCCEEDED
        except asyncio.TimeoutError:
            return False
        finally:
            loop.close()

    def execute_manipulation_step(self, step):
        """Execute manipulation step"""
        goal = ManipulateObject.Goal()

        # Configure manipulation based on step
        params = step.get('parameters', {})
        goal.object_name = params.get('object_name', '')
        goal.action = params.get('action', 'grasp')

        # Execute manipulation goal
        self.manipulation_client.wait_for_server()
        future = self.manipulation_client.send_goal_async(goal)

        # Wait for result with timeout
        import asyncio
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        try:
            result = loop.run_until_complete(
                asyncio.wait_for(future, timeout=60.0)
            )
            return result.status == GoalStatus.STATUS_SUCCEEDED
        except asyncio.TimeoutError:
            return False
        finally:
            loop.close()

    def execute_scan_step(self, step):
        """Execute area scanning step"""
        # Publish scan command
        scan_cmd = String()
        scan_cmd.data = 'start_360_scan'

        scan_pub = self.create_publisher(String, '/scan_command', 10)
        scan_pub.publish(scan_cmd)

        # Wait for scan completion
        time.sleep(5.0)  # Wait for scan to complete

        return True

    def publish_current_step(self, step):
        """Publish current step information"""
        step_msg = String()
        step_msg.data = json.dumps({
            'step_id': step['id'],
            'action': step['action'],
            'description': step['description'],
            'estimated_time_remaining': step.get('estimated_time', 30)
        })
        self.current_step_pub.publish(step_msg)

    def publish_status(self, status):
        """Publish execution status"""
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)
```

## Plan Validation and Safety

### Plan Validator
```python
class PlanValidatorNode(Node):
    def __init__(self):
        super().__init__('plan_validator_node')

        self.plan_sub = self.create_subscription(
            String,
            '/execution_plan',
            self.validate_plan_callback,
            10
        )

        self.validated_plan_pub = self.create_publisher(
            String,
            '/validated_execution_plan',
            10
        )

        self.validation_result_pub = self.create_publisher(
            String,
            '/plan_validation_result',
            10
        )

    def validate_plan_callback(self, msg):
        """Validate incoming plan"""
        try:
            plan = json.loads(msg.data)

            validation_result = self.validate_plan(plan)

            if validation_result['valid']:
                # Publish validated plan
                validated_msg = String()
                validated_msg.data = json.dumps(plan)
                self.validated_plan_pub.publish(validated_msg)

                result_msg = String()
                result_msg.data = f"valid:true:steps:{len(plan.get('steps', []))}"
                self.validation_result_pub.publish(result_msg)
            else:
                result_msg = String()
                result_msg.data = f"valid:false:errors:{';'.join(validation_result['errors'])}"
                self.validation_result_pub.publish(result_msg)

        except json.JSONDecodeError:
            result_msg = String()
            result_msg.data = "valid:false:errors:invalid_json"
            self.validation_result_pub.publish(result_msg)

    def validate_plan(self, plan):
        """Validate plan for safety and feasibility"""
        errors = []

        # Check required fields
        required_fields = ['command', 'steps']
        for field in required_fields:
            if field not in plan:
                errors.append(f"Missing required field: {field}")

        # Validate steps
        steps = plan.get('steps', [])
        if not steps:
            errors.append("Plan has no steps")

        for i, step in enumerate(steps):
            # Check step structure
            required_step_fields = ['id', 'action', 'description']
            for field in required_step_fields:
                if field not in step:
                    errors.append(f"Step {i} missing required field: {field}")

        # Check for dangerous actions
        for step in steps:
            action = step.get('action', '')
            if action in ['jump', 'run_fast', 'throw_object']:
                errors.append(f"Dangerous action not allowed: {action}")

        # Check plan complexity
        if len(steps) > 50:
            errors.append("Plan too complex: more than 50 steps")

        # Check for circular dependencies
        dependencies = {}
        for step in steps:
            step_id = step['id']
            deps = step.get('dependencies', [])
            dependencies[step_id] = deps

        # Simple cycle detection (for basic dependency chains)
        if self.has_cycle(dependencies):
            errors.append("Plan has circular dependencies")

        return {
            'valid': len(errors) == 0,
            'errors': errors
        }

    def has_cycle(self, dependencies):
        """Simple cycle detection in dependency graph"""
        visited = set()
        rec_stack = set()

        def dfs(node):
            if node in rec_stack:
                return True
            if node in visited:
                return False

            visited.add(node)
            rec_stack.add(node)

            for dep in dependencies.get(node, []):
                if dfs(dep):
                    return True

            rec_stack.remove(node)
            return False

        for node in dependencies:
            if node not in visited:
                if dfs(node):
                    return True
        return False
```

## Error Recovery and Adaptation

### Adaptive Planner
```python
class AdaptivePlannerNode(LLMPlannerNode):
    def __init__(self):
        super().__init__()

        # Subscribe to execution feedback
        self.feedback_sub = self.create_subscription(
            String,
            '/execution_feedback',
            self.feedback_callback,
            10
        )

        # Plan modification capabilities
        self.failed_steps = []
        self.environment_changes = []

    def feedback_callback(self, msg):
        """Handle execution feedback for plan adaptation"""
        try:
            feedback = json.loads(msg.data)

            if feedback.get('status') == 'failed':
                step_id = feedback.get('step_id')
                reason = feedback.get('reason', 'unknown')

                self.failed_steps.append({
                    'step_id': step_id,
                    'reason': reason,
                    'timestamp': self.get_clock().now().seconds_nanoseconds()
                })

                # Generate recovery plan
                recovery_plan = self.generate_recovery_plan(step_id, reason)
                if recovery_plan:
                    self.publish_recovery_plan(recovery_plan)

        except json.JSONDecodeError:
            pass

    def generate_recovery_plan(self, failed_step_id, failure_reason):
        """Generate recovery plan for failed step"""
        prompt = f"""
        Previous plan execution failed at step {failed_step_id} due to: {failure_reason}

        Original command: {self.current_command}

        Available recovery strategies:
        - Retry with different parameters
        - Skip and continue with next steps
        - Modify approach for the same goal
        - Request human assistance
        - Use alternative objects/tools

        Generate a recovery plan in JSON format:
        {{
            "original_failed_step": {failed_step_id},
            "failure_reason": "{failure_reason}",
            "recovery_strategy": "description of recovery approach",
            "modified_steps": [
                // New or modified steps to handle the failure
            ],
            "continue_original_plan": true/false
        }}
        """

        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": self.get_recovery_system_prompt()},
                {"role": "user", "content": prompt}
            ],
            temperature=0.3,
            max_tokens=800,
            response_format={"type": "json_object"}
        )

        recovery_plan = json.loads(response.choices[0].message.content)
        return recovery_plan

    def get_recovery_system_prompt(self):
        """System prompt for recovery planning"""
        return """
        You are an expert in robotic task recovery planning. When a robot task fails,
        you must generate a recovery plan that considers:

        1. Root cause analysis of the failure
        2. Feasible alternative approaches
        3. Safety considerations
        4. Task completion requirements
        5. Resource availability

        Prioritize solutions that maintain task completion while ensuring robot safety.
        """
```

## Summary

LLM-based planning enables humanoid robots to interpret high-level natural language commands and execute complex tasks like "Clean the room" by breaking them down into detailed, executable steps. The planning process involves semantic understanding, task decomposition, environment context integration, and safety considerations. Advanced planning systems include validation, error recovery, and adaptation capabilities to handle real-world uncertainties and failures. The next lesson will explore how to map these high-level plans to low-level ROS 2 actions.