---
id: humanoid-applications
title: Humanoid Robotics Applications
sidebar_position: 4
description: Connecting ROS 2 concepts to humanoid robotics applications including sensor integration, actuator coordination, and locomotion control.
tags: [ros2, humanoid-robotics, applications, sensors, actuators, locomotion]
learning_objectives: [lo-003]
---

# Humanoid Robotics Applications

Now that you understand ROS 2's core concepts and communication patterns, let's explore how these apply specifically to humanoid robotics. Humanoid robots present unique challenges that make ROS 2's distributed communication architecture particularly valuable.

## ROS 2 in Humanoid Robotics

Humanoid robots are among the most complex robotic systems, requiring coordination between:

- **Multiple sensors** (vision, proprioception, touch, balance)
- **Numerous actuators** (joints in arms, legs, hands, head)
- **Complex processing** (perception, planning, control)
- **Real-time coordination** for stable, natural movement

ROS 2's distributed architecture makes it possible to manage this complexity by allowing specialized nodes to handle different aspects of the robot's operation.

## Sensor Integration with Topics

Humanoid robots rely on extensive sensor arrays to perceive their environment and understand their own state. ROS 2 topics are ideal for streaming this continuous sensor data.

### Vision Sensors

Humanoid robots typically have multiple cameras (head-mounted, hand-mounted) that generate continuous image streams:

```python
# Example: Camera sensor node publishing images
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class HeadCameraNode(Node):
    def __init__(self):
        super().__init__('head_camera')
        self.publisher = self.create_publisher(Image, 'sensors/head_camera/image', 10)
        self.timer = self.create_timer(0.033, self.publish_image)  # 30 FPS
    
    def publish_image(self):
        # Capture image from head-mounted camera
        msg = Image()
        # ... populate with camera data ...
        self.publisher.publish(msg)
```

Multiple processing nodes can subscribe to these image topics:
- **Object detection node** - Identifies objects in the scene
- **Face recognition node** - Recognizes human faces
- **Navigation node** - Uses visual information for localization

### Proprioceptive Sensors

Proprioceptive sensors (joint encoders, IMUs, force sensors) provide continuous feedback about the robot's body state:

```python
# Example: Joint state publisher
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(JointState, 'sensors/joint_states', 10)
        self.timer = self.create_timer(0.01, self.publish_states)  # 100 Hz
    
    def publish_states(self):
        msg = JointState()
        # Populate with joint positions, velocities, efforts
        # ... read from robot hardware ...
        self.publisher.publish(msg)
```

This joint state information flows to:
- **Control nodes** - For closed-loop control of each joint
- **Balance node** - For maintaining stability
- **Planning node** - For motion planning

## Actuator Coordination with Services

Humanoid robots have many actuators (typically 20-40+ joints) that must be coordinated precisely. Services are useful for on-demand actuator control and configuration.

### Coordinated Movement

When a humanoid robot needs to perform a coordinated movement (like reaching for an object), multiple joints must move together:

```python
# Example: Actuator coordination service
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class ArmControlService(Node):
    def __init__(self):
        super().__init__('arm_control')
        self.srv = self.create_service(
            SetBool,
            'control/arm/move_to_pose',
            self.move_arm_callback
        )
    
    def move_arm_callback(self, request, response):
        if request.data:  # If request is to move
            # Coordinate multiple joints to reach target pose
            # Shoulder, elbow, wrist joints move together
            response.success = self.execute_coordinated_movement()
            response.message = "Arm movement executed" if response.success else "Movement failed"
        return response
```

### Configuration Services

Services are also useful for configuring actuator parameters:

```python
# Example: Setting joint limits or control parameters
class JointConfigService(Node):
    def __init__(self):
        super().__init__('joint_config')
        self.srv = self.create_service(
            # Custom service type for joint configuration
            'joint_config',
            self.config_callback
        )
    
    def config_callback(self, request, response):
        # Set joint speed limits, torque limits, etc.
        # This is a synchronous operation - we need confirmation
        response.success = True
        return response
```

## Locomotion Control with Actions

Walking and other locomotion behaviors are long-running tasks that require continuous feedback and the ability to adapt or cancel. Actions are perfect for these scenarios.

### Walking Action

```python
# Example: Walking action for humanoid robot
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
# Custom action type for navigation
from custom_msgs.action import WalkToGoal

class WalkingActionServer(Node):
    def __init__(self):
        super().__init__('walking_server')
        self._action_server = ActionServer(
            self,
            WalkToGoal,
            'locomotion/walk',
            self.walk_callback
        )
    
    def walk_callback(self, goal_handle):
        goal = goal_handle.request
        feedback_msg = WalkToGoal.Feedback()
        
        # Execute walking with continuous feedback
        total_steps = self.calculate_steps(goal.target_pose)
        current_step = 0
        
        while current_step < total_steps:
            # Execute one walking step
            # Update balance, shift weight, lift foot, place foot
            current_step += 1
            
            # Provide feedback
            feedback_msg.current_step = current_step
            feedback_msg.total_steps = total_steps
            feedback_msg.current_pose = self.get_current_pose()
            goal_handle.publish_feedback(feedback_msg)
            
            # Check for cancellation (e.g., obstacle detected)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return WalkToGoal.Result()
        
        # Walking complete
        goal_handle.succeed()
        result = WalkToGoal.Result()
        result.final_pose = self.get_current_pose()
        return result
```

This walking action provides:
- **Continuous feedback** on progress (current step, current pose)
- **Ability to cancel** if an obstacle is detected
- **Final result** with the achieved pose

## Mapping ROS 2 Components to Robot Subsystems

Here's how ROS 2 components map to a humanoid robot's subsystems:

| Robot Subsystem | ROS 2 Component | Communication Pattern | Example |
|----------------|-----------------|----------------------|---------|
| Vision System | Camera nodes | Topics | Head camera → Image processing nodes |
| Proprioception | Sensor nodes | Topics | Joint encoders → State estimation |
| Balance Control | Control nodes | Topics + Services | IMU data (topic) → Balance service calls |
| Arm Control | Actuator nodes | Services | Reach command → Arm coordination service |
| Leg Control | Locomotion nodes | Actions | Walk command → Walking action |
| Planning | Planning nodes | Topics + Services | Sensor data (topics) → Plan requests (services) |
| High-level Control | Behavior nodes | Actions | "Pick up object" → Manipulation action |

## Complete System Example

In a humanoid robot picking up an object:

1. **Vision nodes** publish camera images to topics
2. **Object detection node** subscribes to images, publishes object location to a topic
3. **Planning node** subscribes to object location, calls arm control service to plan trajectory
4. **Arm control service** coordinates multiple joints synchronously
5. **Grasping action** executes the grasp with feedback on finger positions
6. **Balance node** continuously subscribes to IMU data (topic) to maintain stability

This demonstrates how all three communication patterns work together in a real humanoid robot application.

## Key Takeaways

1. **Topics** stream continuous sensor data to multiple processing nodes
2. **Services** coordinate actuators for synchronous, on-demand actions
3. **Actions** execute complex behaviors like walking with feedback and cancellation
4. **Humanoid robots** use all three patterns simultaneously for different subsystems
5. **ROS 2's distributed architecture** makes it possible to manage the complexity of humanoid systems

## Next Steps

Now that you understand how ROS 2 applies to humanoid robotics, proceed to [Workspace Overview](./workspace-overview.md) to learn about ROS 2 workspace structure, or review the [Glossary](./glossary.md) for quick reference to key terms.

