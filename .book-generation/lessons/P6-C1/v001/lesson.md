# Chapter P6-C1: Build a Mobile Robot

**Learning Time**: 50 minutes reading + 18 hours hands-on
**Version**: v001
**Created**: 2025-12-01

---

## Part 1: The Hook

Imagine a robot that can autonomously navigate your home, avoiding obstacles, mapping its environment, and reaching any destination you specify. By the end of this chapter, you'll have built exactly that—both in simulation and in physical hardware.

This isn't just theory. You'll implement a complete system integrating kinematics, dynamics, control, navigation, and AI. You'll start with mathematical foundations, build a simulation model in Gazebo, assemble physical hardware, deploy ROS2 Navigation2, and validate your models against real-world performance.

**Why mobile robots matter:**

Mobile robots are everywhere—warehouse automation (Amazon Kiva), delivery robots (Starship, Nuro), service robots (iRobot Roomba), and exploration (Mars rovers). The core challenge: moving safely and efficiently in unstructured environments. Differential drive robots are the simplest and most common configuration, making them perfect for learning the fundamentals.

**Your measurable objective**: By the end of this chapter, you'll be able to:
- Derive and implement forward/inverse kinematics for differential drive robots
- Model robot dynamics accounting for motor and chassis dynamics
- Design and assemble a physical mobile robot with motors, encoders, and sensors
- Deploy ROS2 Navigation2 for autonomous navigation
- Validate simulation models against physical hardware
- Debug and tune control parameters for stable operation

You'll build this through hands-on labs in both Gazebo (simulation) and Raspberry Pi hardware (physical), culminating in a project where you create an autonomous delivery robot that navigates to multiple waypoints.

---

## Part 2: The Concept (Theory)

### Differential Drive Kinematics: The Foundation

A differential drive mobile robot has two independently driven wheels on a common axis, with a caster wheel (or two) for support. This simple configuration enables omnidirectional movement through rotation and forward/backward motion, but with a critical constraint: **you cannot move sideways directly**.

**Visual Intuition: The Shopping Cart Analogy**

Think of a shopping cart: you can push it forward, pull it backward, and rotate it in place, but you can't slide it sideways. To reach a point to your left, you must first rotate, then move forward. This is exactly how differential drive robots work.

**Forward Kinematics: From Wheel Velocities to Robot Motion**

Forward kinematics answers: "If my wheels are rotating at speeds v_left and v_right, where does the robot go?"

The robot's motion is described by:
- **Linear velocity** v: How fast the robot moves forward/backward
- **Angular velocity** ω: How fast the robot rotates

From wheel velocities:
```
v = (r/2) × (ω_left + ω_right)
ω = (r/L) × (ω_right - ω_left)
```

Where:
- r = wheel radius
- L = wheelbase (distance between wheel centers)
- ω_left, ω_right = angular velocities of left and right wheels

The robot's position (x, y, θ) evolves as:
```
ẋ = v × cos(θ)
ẏ = v × sin(θ)
θ̇ = ω
```

**Inverse Kinematics: From Desired Motion to Wheel Commands**

Inverse kinematics answers: "I want the robot to move at velocity v with angular velocity ω. What wheel speeds should I command?"

Solving the forward kinematics equations:
```
ω_left = (2v - L×ω) / (2r)
ω_right = (2v + L×ω) / (2r)
```

**Non-Holonomic Constraints: Why You Can't Move Sideways**

The constraint equation is:
```
ẏ = 0  (no sideways motion)
```

This means the robot's velocity vector is always aligned with its heading. To change direction, you must rotate first. This constraint fundamentally shapes path planning algorithms.

> **💡 Pattern**: Non-holonomic constraints appear whenever a system has fewer control inputs than degrees of freedom. Differential drive has 2 controls (wheel speeds) but 3 DOF (x, y, θ).

### Dynamic Modeling: Beyond Kinematics

Kinematics tells you where the robot goes, but dynamics tells you **how forces and torques create that motion**. This is essential for accurate control.

**Motor Dynamics**

Real motors don't respond instantly. The relationship between commanded current and actual torque is:
```
τ_motor = K_t × I - K_v × ω_motor
```

Where:
- K_t = torque constant (Nm/A)
- K_v = back-EMF constant (Vs/rad)
- I = motor current
- ω_motor = motor angular velocity

**Chassis Dynamics**

The robot's acceleration depends on applied forces:
```
F = m × a  (Newton's second law)
τ = I × α  (rotational dynamics)
```

Combined with motor dynamics, this gives the full dynamic model:
```
M(q) × [v̇, ω̇]ᵀ + C(q,q̇) + G(q) = [F, τ]ᵀ
```

Where:
- M(q) = mass/inertia matrix (configuration-dependent)
- C(q,q̇) = Coriolis and centrifugal forces
- G(q) = gravity forces
- F, τ = applied force and torque

> **⚠️ Common Mistake**: Using pure kinematic control ignores motor dynamics and friction, leading to poor trajectory tracking. Dynamic models account for these effects.

### ROS2 Navigation2: The Autonomy Stack

ROS2 Navigation2 provides a complete framework for autonomous mobile robot navigation:

**Core Components**:
1. **Planner**: Computes global path from start to goal (A*, RRT*)
2. **Controller**: Follows the path using local trajectory tracking (DWB, TEB)
3. **Recovery Behaviors**: Handles stuck situations (rotate in place, clear costmap)
4. **Costmap**: Grid-based obstacle representation (static + dynamic layers)

**Architecture Flow**:
```
Goal → Planner → Global Path → Controller → Local Trajectory → Motor Commands
         ↓                                              ↑
      Costmap ←─────────────── Sensor Data (LiDAR, Odometry)
```

> **🎯 Key Insight**: Navigation2 separates global planning (long-term strategy) from local control (short-term execution). This hierarchical approach enables robust navigation in complex environments.

---

## Part 3: The Walkthrough (I Do / We Do)

### Example 1: Forward Kinematics Implementation

Let's implement forward kinematics in Python:

```python
import numpy as np

def forward_kinematics(v_left, v_right, L, r, dt, current_pose):
    """
    Compute robot pose after time dt given wheel velocities.
    
    Args:
        v_left: Left wheel linear velocity (m/s)
        v_right: Right wheel linear velocity (m/s)
        L: Wheelbase (m)
        r: Wheel radius (m)
        dt: Time step (s)
        current_pose: [x, y, theta] current robot pose
    
    Returns:
        [x_new, y_new, theta_new] updated pose
    """
    x, y, theta = current_pose
    
    # Compute robot linear and angular velocities
    v = (r/2) * (v_left + v_right)
    omega = (r/L) * (v_right - v_left)
    
    # Integrate to get new pose
    x_new = x + v * np.cos(theta) * dt
    y_new = y + v * np.sin(theta) * dt
    theta_new = theta + omega * dt
    
    return [x_new, y_new, theta_new]

# Test with example values
pose = [0.0, 0.0, 0.0]  # Start at origin, facing +x
L = 0.3  # 30cm wheelbase
r = 0.05  # 5cm wheel radius
dt = 0.1  # 100ms time step

# Move forward: both wheels at 1 m/s
pose = forward_kinematics(1.0, 1.0, L, r, dt, pose)
print(f"After 0.1s forward: {pose}")  # [0.005, 0.0, 0.0]

# Rotate in place: wheels opposite directions
pose = forward_kinematics(-1.0, 1.0, L, r, dt, pose)
print(f"After 0.1s rotation: {pose}")  # [0.005, 0.0, 0.333]
```

**What's happening**: The function computes robot velocity from wheel speeds, then integrates to update position. Notice how rotation changes θ without changing x, y (rotating in place).

### Example 2: Inverse Kinematics with Velocity Limits

Now let's compute required wheel speeds for desired robot motion:

```python
def inverse_kinematics(v_desired, omega_desired, L, r, v_max):
    """
    Compute required wheel velocities for desired robot motion.
    Includes velocity saturation to respect motor limits.
    
    Args:
        v_desired: Desired linear velocity (m/s)
        omega_desired: Desired angular velocity (rad/s)
        L: Wheelbase (m)
        r: Wheel radius (m)
        v_max: Maximum wheel linear velocity (m/s)
    
    Returns:
        (v_left, v_right) wheel velocities (m/s)
    """
    # Compute required wheel velocities
    v_left = v_desired - (L/2) * omega_desired
    v_right = v_desired + (L/2) * omega_desired
    
    # Check for saturation
    if abs(v_left) > v_max or abs(v_right) > v_max:
        # Scale both velocities proportionally
        scale = v_max / max(abs(v_left), abs(v_right))
        v_left *= scale
        v_right *= scale
    
    return v_left, v_right

# Example: Move forward at 0.5 m/s while rotating at 1 rad/s
v_left, v_right = inverse_kinematics(0.5, 1.0, 0.3, 0.05, 2.0)
print(f"Wheel velocities: left={v_left:.2f}, right={v_right:.2f} m/s")
# Output: left=0.35, right=0.65 m/s
```

**Saturation handling**: When desired motion exceeds motor limits, we scale both wheels proportionally to maintain the desired direction while respecting limits.

### Example 3: Gazebo URDF Model

Create a Gazebo model for differential drive robot:

```xml
<?xml version="1.0"?>
<robot name="differential_drive_robot">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Left Wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Left Wheel Joint -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.15 0" rpy="0 1.57 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Right Wheel (similar structure) -->
  <link name="right_wheel">
    <!-- ... same as left wheel ... -->
  </link>
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.15 0" rpy="0 1.57 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Caster Wheel -->
  <link name="caster">
    <visual>
      <geometry>
        <sphere radius="0.02"/>
      </geometry>
    </visual>
  </link>
  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster"/>
    <origin xyz="0.1 0 -0.05"/>
  </joint>

  <!-- Gazebo Differential Drive Plugin -->
  <gazebo>
    <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
      <ros>
        <namespace>/</namespace>
      </ros>
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.3</wheel_separation>
      <wheel_diameter>0.1</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>true</publish_wheel_tf>
    </plugin>
  </gazebo>
</robot>
```

**Key elements**: Base link with mass/inertia, two wheel joints (continuous type for rotation), caster for support, and Gazebo plugin for differential drive control.

### Example 4: ROS2 Motor Control Node

Create a ROS2 node that subscribes to `/cmd_vel` and publishes wheel commands:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class DifferentialDriveController(Node):
    def __init__(self):
        super().__init__('differential_drive_controller')
        
        # Robot parameters
        self.wheelbase = 0.3  # meters
        self.wheel_radius = 0.05  # meters
        self.max_wheel_velocity = 2.0  # m/s
        
        # Subscribers and publishers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.left_wheel_pub = self.create_publisher(Float64, '/left_wheel/command', 10)
        self.right_wheel_pub = self.create_publisher(Float64, '/right_wheel/command', 10)
        
        self.get_logger().info('Differential drive controller started')
    
    def cmd_vel_callback(self, msg):
        """Convert Twist command to wheel velocities."""
        v = msg.linear.x  # Forward velocity
        omega = msg.angular.z  # Angular velocity
        
        # Inverse kinematics
        v_left = v - (self.wheelbase/2) * omega
        v_right = v + (self.wheelbase/2) * omega
        
        # Saturation
        if abs(v_left) > self.max_wheel_velocity or abs(v_right) > self.max_wheel_velocity:
            scale = self.max_wheel_velocity / max(abs(v_left), abs(v_right))
            v_left *= scale
            v_right *= scale
        
        # Convert to wheel angular velocity (rad/s)
        omega_left = v_left / self.wheel_radius
        omega_right = v_right / self.wheel_radius
        
        # Publish commands
        self.left_wheel_pub.publish(Float64(data=omega_left))
        self.right_wheel_pub.publish(Float64(data=omega_right))

def main(args=None):
    rclpy.init(args=args)
    node = DifferentialDriveController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Integration**: This node bridges ROS2 Navigation2 (which publishes `/cmd_vel`) with motor hardware (which receives wheel commands).

---

## Part 4: The Challenge (You Do)

**Challenge: Build a Complete Navigation System**

Your task is to create a complete autonomous navigation system for a differential drive robot, integrating all concepts from this chapter.

**Requirements**:

1. **Kinematics Library** (Specification-driven):
   - Write a specification for a `DifferentialDriveKinematics` class
   - Include: forward kinematics, inverse kinematics, velocity saturation
   - AI generates initial implementation from your spec
   - Grade on: Code quality (40%) + Spec alignment (60%)

2. **Simulation Model**:
   - Create Gazebo URDF model with accurate physics
   - Configure ROS2 Navigation2 stack
   - Test trajectory tracking in simulation

3. **Physical Implementation** (if hardware available):
   - Assemble robot with motors, encoders, IMU
   - Calibrate sensors and measure actual parameters
   - Deploy Navigation2 on physical robot
   - Compare simulation vs. physical performance

4. **Validation Report**:
   - Document kinematic validation (simulation vs. theory)
   - Analyze sources of error (slippage, encoder resolution, etc.)
   - Propose improvements

**Success Criteria**:
- ✅ Forward kinematics matches simulation within 5% error
- ✅ Robot successfully navigates to waypoints in simulation
- ✅ Navigation2 stack properly configured
- ✅ (If physical) Physical robot matches simulation behavior within 10% error

**Iteration Guidance**:
- Start with kinematics library (spec → implementation → test)
- Then simulation model (URDF → Navigation2 → test)
- Finally physical deployment (if available)
- Iterate based on validation results

---

## Part 5: Key Takeaways

1. **Differential drive kinematics are simple but powerful** - The forward/inverse kinematic equations are straightforward but enable complex navigation behaviors.

2. **Dynamic modeling improves control performance** - Accounting for motor dynamics and friction leads to better trajectory tracking than pure kinematic control.

3. **Simulation accelerates development** - Test algorithms safely and rapidly in simulation before deploying to hardware.

4. **Hardware calibration is essential** - Actual parameters (L, r, friction) often differ from design values; measure and calibrate.

5. **ROS2 Navigation2 is production-ready** - The Navigation2 stack provides robust path planning and obstacle avoidance out of the box.

6. **Sim-to-real transfer requires iteration** - Expect differences between simulation and hardware; refine models based on physical testing.

7. **Power management matters** - Autonomous operation requires careful battery management and energy-efficient control.

8. **Sensor fusion improves robustness** - Combining odometry, IMU, and LiDAR provides more reliable localization than any single sensor.

9. **Control tuning is iterative** - PID gains and other parameters require systematic tuning based on performance metrics.

10. **Documentation enables debugging** - Keep detailed logs of parameters, test results, and issues for effective troubleshooting.

**Common Mistakes to Avoid**:
- Ignoring non-holonomic constraints (attempting to move sideways directly)
- Incorrect wheelbase measurement (leads to systematic kinematic errors)
- Neglecting motor dynamics (pure kinematic control may be unstable)
- Insufficient encoder resolution (causes odometry drift)
- Poor power distribution (voltage drops cause erratic behavior)
- Inadequate testing (deploying untested code to hardware causes failures)

---

## Part 6: Learn with AI

This section provides AI prompts for deeper exploration of mobile robot concepts. Use these with your AI assistant to deepen your understanding.

### AI Tutor: Deep-Dive Queries

**Prompt 1: Understanding Non-Holonomic Constraints**
```
Explain non-holonomic constraints in mobile robotics using analogies. 
Why can't a differential drive robot move sideways? 
What are the implications for path planning algorithms?
Compare with holonomic systems (like omni-directional wheels).
```

**Prompt 2: Dynamic Modeling Complexity**
```
Walk me through deriving the dynamic model for a differential drive robot 
using Lagrange formulation. Explain each term (mass matrix, Coriolis forces, 
gravity) physically. How does this differ from Newton-Euler approach?
```

**Prompt 3: ROS2 Navigation2 Architecture**
```
Explain the ROS2 Navigation2 stack architecture in detail. 
How does the planner interact with the controller? 
What is a costmap and how is it updated? 
Describe the recovery behavior system.
```

### AI Collaboration: Code Refinement

**Prompt 4: Optimize Kinematics Implementation**
```
Review my forward kinematics implementation:
[Paste your code]

Suggest improvements for:
1. Numerical stability (integration method)
2. Error handling (invalid inputs)
3. Performance optimization
4. Code documentation
```

**Prompt 5: Debug Navigation2 Configuration**
```
I'm having issues with ROS2 Navigation2. My robot overshoots waypoints 
and sometimes gets stuck. Here's my configuration:
[Paste nav2_params.yaml]

Analyze potential issues and suggest parameter tuning strategies.
```

### AI System Analyzer: Performance Analysis

**Prompt 6: Compare Simulation vs. Physical Performance**
```
I've collected data from both simulation and physical robot:
- Simulation: trajectory tracking error = 2cm
- Physical: trajectory tracking error = 8cm

Analyze potential sources of the discrepancy:
1. Modeling errors (what might be missing?)
2. Sensor noise and calibration
3. Actuator dynamics differences
4. Environmental factors

Propose a systematic debugging approach.
```

### AI Challenge Generator: Specification-Driven Development

**Prompt 7: Generate Advanced Navigation Challenge**
```
Create a specification for an advanced navigation challenge:
- Multi-waypoint navigation with time constraints
- Dynamic obstacle avoidance
- Energy-efficient path planning
- Recovery from localization failures

The spec should include:
1. Functional requirements
2. Performance metrics
3. Test scenarios
4. Success criteria

Then generate an implementation plan from this spec.
```

### Reusable Intelligence Design

**Prompt 8: Create Mobile Robot Skill Blueprint**
```
Based on this chapter, design a reusable "Differential Drive Mobile Robot" 
skill/component that can be used in other projects.

Define:
1. Three non-negotiable instructions for the component
2. Input/output interface
3. Configuration parameters
4. Error handling strategy
5. Testing requirements

This component should encapsulate kinematics, dynamics, and basic control.
```

---

**Lesson Status**: ✅ Complete
**Next Steps**: Proceed to writer-agent for prose generation


