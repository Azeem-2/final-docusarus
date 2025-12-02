# P5-C1 Lesson Content: Humanoid Kinematics & Dynamics (Part 1)

**Version**: v001
**Date**: 2025-11-30
**Planner**: lesson-planner
**Lessons Covered**: 1-5
**Status**: Complete

---

## Lesson 1: 3D Transformations and Coordinate Representations

**Pedagogical Layer**: 1 (Manual Foundation)
**Estimated Time**: 2.5 hours

---

### Part 1: The Hook

You're building a humanoid robot that needs to grasp a coffee cup. The robot's camera sees the cup at position (0.45, -0.12, 0.73) meters, but its arm operates in a completely different coordinate system centered at the shoulder. How do you translate between these worlds?

This is the fundamental problem of robotics: **managing transformations between coordinate frames**. Without mastering 3D transformations, you cannot compute where a robot's hand is in space, plan motions, or even know if an object is reachable. By the end of this lesson, you'll implement a transformation library that converts between rotation representations and composes coordinate frames—skills you'll use in every subsequent lesson.

**Learning Objective**: Implement spatial transformation utilities that convert between rotation representations (matrices, quaternions, Euler angles) and compose coordinate frames using homogeneous transformations.

---

### Part 2: The Concept (Theory)

#### Visual Intuition: Coordinate Frames as Measuring Sticks

Imagine you're in a room with a friend. You describe the cup's location as "2 feet in front of me," while your friend (facing a different direction) says "3 feet to my left." You're both correct—you're just using different **coordinate frames**.

In robotics, every link of a robot arm has its own coordinate frame. To find the end-effector position in world coordinates, we must **chain together** these frame transformations, like connecting language translators.

#### Rotation Representations: Four Ways to Describe the Same Thing

> **💡 Key Insight**: There's no single "best" rotation representation. Each has trade-offs:

| Representation | Pros | Cons | Use Case |
|----------------|------|------|----------|
| **Rotation Matrix** (3×3) | Direct composition (matrix multiply) | 9 numbers for 3 DOF (redundant) | Fast computation, stability checks |
| **Euler Angles** (3 values) | Minimal representation, intuitive | Gimbal lock, multiple conventions | User input, visualization |
| **Quaternions** (4 values) | No gimbal lock, smooth interpolation | Non-intuitive (4D hypersphere) | Animation, control |
| **Axis-Angle** (4 values) | Geometric intuition (rotation axis + angle) | Ambiguity at 0° and 360° | Optimization, exponential maps |

#### The Mathematics: SO(3) and SE(3)

**Rotation Group SO(3)**: All 3×3 rotation matrices R satisfy:
- **Orthogonality**: R^T R = I
- **Determinant**: det(R) = 1 (right-handed)

**Special Euclidean Group SE(3)**: Combines rotation R and translation t using **homogeneous coordinates**:

```
T = [R  t]  where R ∈ SO(3), t ∈ ℝ³
    [0  1]
```

To transform a point p from frame A to frame B:
```
p_B = T_AB * p_A  (using homogeneous coordinates [x, y, z, 1]^T)
```

#### Pattern-Based Chunking: Composition Rules

1. **Rotation composition**: R_AC = R_AB · R_BC (matrix multiply)
2. **Transformation composition**: T_AC = T_AB · T_BC (same!)
3. **Inverse transformation**: T_AB^(-1) = T_BA

> **🎯 Pattern**: Transformations compose left-to-right, like function composition.

---

### Part 3: The Walkthrough (Manual Implementation)

#### Example 1: Verifying Rotation Matrix Properties

Let's create a rotation of 30° about the Z-axis and verify orthogonality:

```python
import numpy as np

def rotation_z(theta):
    """Create rotation matrix for rotation about Z-axis."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, -s, 0],
        [s,  c, 0],
        [0,  0, 1]
    ])

# Test orthogonality
R = rotation_z(np.pi/6)  # 30 degrees
print("R^T @ R =\n", R.T @ R)  # Should be identity
print("det(R) =", np.linalg.det(R))  # Should be 1.0
```

**Expert Insight**: Always verify these properties after creating rotation matrices. Numerical errors accumulate, especially when composing many rotations. Use `np.allclose(R.T @ R, np.eye(3))` for floating-point tolerance.

#### Example 2: Euler Angles to Rotation Matrix (ZYX Convention)

Euler angles decompose a rotation into three sequential rotations. We'll use the **ZYX (yaw-pitch-roll)** convention:

```python
def euler_to_matrix(yaw, pitch, roll):
    """
    Convert Euler angles (ZYX convention) to rotation matrix.

    Args:
        yaw: Rotation about Z-axis (radians)
        pitch: Rotation about Y-axis (radians)
        roll: Rotation about X-axis (radians)

    Returns:
        3x3 rotation matrix
    """
    Rz = rotation_z(yaw)
    Ry = rotation_y(pitch)
    Rx = rotation_x(roll)

    # Composition order: first roll (X), then pitch (Y), then yaw (Z)
    return Rz @ Ry @ Rx
```

> **⚠️ Common Mistake**: Euler angle conventions vary! Always document which convention you're using (ZYX vs XYZ vs ZYZ, etc.). Wrong order means wrong orientation.

#### Example 3: Homogeneous Transformation Composition

Robot arm with two links: shoulder at origin, elbow at (0.3, 0, 0) local to shoulder, both rotated 45°.

```python
def make_transform(R, t):
    """Create 4x4 homogeneous transformation matrix."""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

# Shoulder frame (world to shoulder)
T_world_shoulder = make_transform(
    rotation_z(np.pi/4),  # 45° rotation
    np.array([0, 0, 0])   # at origin
)

# Elbow frame (shoulder to elbow)
T_shoulder_elbow = make_transform(
    rotation_z(np.pi/4),   # another 45° rotation
    np.array([0.3, 0, 0])  # 0.3m along local X
)

# Compose: world to elbow
T_world_elbow = T_world_shoulder @ T_shoulder_elbow

print("Elbow position in world frame:", T_world_elbow[:3, 3])
# Should be approximately [0.212, 0.212, 0] (0.3m at 45°+45°=90°)
```

**Reasoning**: We chain transformations by multiplying matrices. The result tells us where the elbow is in world coordinates.

#### Example 4: Quaternion Representation

Quaternions q = [w, x, y, z] represent rotations without gimbal lock. They live on the 4D unit sphere: w² + x² + y² + z² = 1.

```python
def quaternion_to_matrix(q):
    """
    Convert unit quaternion to rotation matrix.

    Args:
        q: [w, x, y, z] with ||q|| = 1
    """
    w, x, y, z = q
    return np.array([
        [1-2*(y**2+z**2), 2*(x*y-w*z),   2*(x*z+w*y)],
        [2*(x*y+w*z),     1-2*(x**2+z**2), 2*(y*z-w*x)],
        [2*(x*z-w*y),     2*(y*z+w*x),   1-2*(x**2+y**2)]
    ])

# Example: 90° rotation about Z-axis
q = np.array([np.cos(np.pi/4), 0, 0, np.sin(np.pi/4)])  # [w,x,y,z]
R = quaternion_to_matrix(q)
print("Quaternion rotation matrix:\n", R)
```

> **🔧 Pro Tip**: Quaternions must be **normalized** before use. Always check `np.linalg.norm(q) ≈ 1.0`. Non-unit quaternions produce invalid rotations.

---

### Part 4: The Challenge (Manual Exercise)

**Challenge**: Implement a `SpatialMath` library with the following functions:

**Requirements**:
1. `matrix_to_quaternion(R)` - Convert 3×3 rotation matrix to quaternion [w,x,y,z]
2. `quaternion_to_euler(q)` - Convert quaternion to Euler angles (ZYX convention)
3. `compose_transforms(T1, T2, ..., Tn)` - Compose arbitrary number of transformations
4. `inverse_transform(T)` - Compute T^(-1) efficiently using R^T

**Success Criteria**:
- All conversions must round-trip correctly: matrix → quaternion → matrix (error < 1e-6)
- `compose_transforms` must handle 1 to 10 transformations
- `inverse_transform` must satisfy: T @ inverse_transform(T) = I (error < 1e-8)

**Test Cases**:
```python
# Test 1: Round-trip conversion
R_original = rotation_z(np.pi/3)
q = matrix_to_quaternion(R_original)
R_recovered = quaternion_to_matrix(q)
assert np.allclose(R_original, R_recovered, atol=1e-6)

# Test 2: Composition
T1 = make_transform(rotation_z(np.pi/4), [1, 0, 0])
T2 = make_transform(rotation_y(np.pi/6), [0, 1, 0])
T_composed = compose_transforms(T1, T2)
T_expected = T1 @ T2
assert np.allclose(T_composed, T_expected)

# Test 3: Inverse
T = make_transform(rotation_z(0.5), [1, 2, 3])
T_inv = inverse_transform(T)
assert np.allclose(T @ T_inv, np.eye(4), atol=1e-8)
```

**Iteration Guidance**:
1. Start with `matrix_to_quaternion` (hardest conversion—watch for numerical stability)
2. Add `compose_transforms` (use `functools.reduce`)
3. Implement `inverse_transform` using block matrix formula
4. Build `quaternion_to_euler` last (watch for atan2 argument order)

---

### Part 5: Key Takeaways

1. **Four rotation representations trade off compactness, intuitiveness, and computational efficiency**. Use rotation matrices for computation, quaternions for interpolation, Euler angles for human input.

2. **Homogeneous transformations (SE(3)) unify rotation and translation** into a single 4×4 matrix that composes via multiplication—the foundation of forward kinematics.

3. **Always validate mathematical properties**: rotation matrices must be orthogonal (R^T R = I), quaternions must be unit norm—numerical drift breaks everything downstream.

**Looking Ahead**: In Lesson 2, we'll use these transformation tools to build a forward kinematics engine that chains transformations through an entire robot arm. Your `SpatialMath` library is the foundation.

---

### Part 6: Learn with AI

**Understand It Better**:
> "Explain quaternions using a 4D sphere analogy instead of the mathematical formula. Why do we need 4 numbers to represent 3D rotations?"

**Get Feedback on Your Code**:
> "Review my `matrix_to_quaternion` implementation and suggest one improvement for numerical stability when dealing with nearly-identity rotations:
> [paste your code here]"

**Go Deeper**:
> "What are the most common mistakes beginners make when converting between rotation representations, and how can I detect them with assertions?"

**See It in Action**:
> "Show me how spacecraft attitude control uses quaternions instead of Euler angles, with a simple example of slew maneuver code."

---

## Lesson 2: Forward Kinematics and DH Parameters

**Pedagogical Layer**: 1 (Manual Foundation)
**Estimated Time**: 2.5 hours

---

### Part 1: The Hook

Your humanoid robot's hand is controlled by 7 joints from shoulder to fingertips. You set each joint angle to a specific value, but where exactly is the hand in 3D space? More critically, when you change joint 3 by 10 degrees, how does the hand position change?

This is the **forward kinematics (FK)** problem: given joint angles, compute end-effector pose. It's the most fundamental robot computation—used in control, planning, simulation, and even just visualizing what the robot is doing. FK is deterministic and always solvable (unlike its inverse, which we'll tackle later).

**Learning Objective**: Implement recursive forward kinematics that computes end-effector position and orientation for arbitrary kinematic trees using Denavit-Hartenberg parameters and homogeneous transformations.

---

### Part 2: The Concept (Theory)

#### Visual Intuition: A Chain of Translators

Imagine describing directions to a friend: "From the town square (origin), walk 3 blocks north, turn right, walk 2 blocks east, turn left." Each instruction is a **transformation** relative to where you currently are.

Forward kinematics chains these transformations:
```
Hand_position = World → Shoulder → Elbow → Wrist → Hand
```

Each arrow is a 4×4 transformation matrix that you multiply together.

#### Denavit-Hartenberg (DH) Parameters: The Standard Convention

To describe each link-joint pair, we use **4 parameters**:

| Parameter | Symbol | Description |
|-----------|--------|-------------|
| **Link length** | a_i | Distance along X_i from Z_{i-1} to Z_i |
| **Link twist** | α_i | Angle about X_i from Z_{i-1} to Z_i |
| **Link offset** | d_i | Distance along Z_{i-1} from X_{i-1} to X_i |
| **Joint angle** | θ_i | Angle about Z_{i-1} from X_{i-1} to X_i |

> **🎯 Pattern**: For **revolute joints** (like elbow), θ_i is the variable (joint angle). For **prismatic joints** (like telescope), d_i is the variable.

The DH transformation matrix from frame i-1 to frame i:

```
T_i = Rot_Z(θ_i) · Trans_Z(d_i) · Trans_X(a_i) · Rot_X(α_i)
```

In matrix form:
```
     [cos(θ_i)  -sin(θ_i)cos(α_i)   sin(θ_i)sin(α_i)  a_i·cos(θ_i)]
T_i = [sin(θ_i)   cos(θ_i)cos(α_i)  -cos(θ_i)sin(α_i)  a_i·sin(θ_i)]
     [    0          sin(α_i)           cos(α_i)           d_i      ]
     [    0             0                  0                1       ]
```

#### Kinematic Trees: From Chains to Humanoids

Robot arms are **serial chains** (linear sequence of joints). Humanoid torsos **branch** into multiple limbs:

```
        Head
         |
    Left_Arm - Torso - Right_Arm
         |
      Pelvis
      /    \
  Left_Leg  Right_Leg
```

This is a **kinematic tree**. We extend FK using **recursive traversal**: compute parent frame, then visit each child.

---

### Part 3: The Walkthrough (Manual Implementation)

#### Example 1: DH Parameters for 2-DOF Planar Arm

Simple 2-link arm in the XY plane (like the first two joints of a robot arm):

| Joint | θ_i (variable) | d_i | a_i (link length) | α_i |
|-------|----------------|-----|-------------------|-----|
| 1 | θ_1 | 0 | 0.3 m | 0° |
| 2 | θ_2 | 0 | 0.25 m | 0° |

```python
def dh_transform(theta, d, a, alpha):
    """
    Create DH transformation matrix.

    Args:
        theta, d, a, alpha: DH parameters (angles in radians)

    Returns:
        4x4 homogeneous transformation
    """
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)

    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,   sa,     ca,    d   ],
        [0,   0,      0,     1   ]
    ])

# Compute FK for θ1=45°, θ2=30°
theta1, theta2 = np.pi/4, np.pi/6

T1 = dh_transform(theta1, 0, 0.3, 0)  # Base to joint 1
T2 = dh_transform(theta2, 0, 0.25, 0) # Joint 1 to joint 2

T_end = T1 @ T2  # Composition
end_effector_pos = T_end[:3, 3]
print("End-effector position:", end_effector_pos)
# Expected: approximately [0.424, 0.342, 0]
```

**Expert Insight**: The DH convention is not unique! Some robotics texts use **modified DH** (Craig's convention), which swaps the order of transformations. Always check which convention your URDF uses.

#### Example 2: Recursive Forward Kinematics for Serial Chain

For N joints, compute all intermediate frames:

```python
class SerialChain:
    def __init__(self, dh_params):
        """
        Args:
            dh_params: List of (a, alpha, d) tuples (theta is variable)
        """
        self.dh_params = dh_params
        self.n_joints = len(dh_params)

    def forward_kinematics(self, joint_angles):
        """
        Compute FK for all links.

        Args:
            joint_angles: Array of θ values

        Returns:
            transforms: List of 4x4 matrices [T_0_1, T_0_2, ..., T_0_N]
        """
        transforms = []
        T_current = np.eye(4)  # Start at world frame

        for i, theta in enumerate(joint_angles):
            a, alpha, d = self.dh_params[i]
            T_i = dh_transform(theta, d, a, alpha)

            T_current = T_current @ T_i  # Accumulate transformation
            transforms.append(T_current.copy())  # Cache for efficiency

        return transforms

# Example: 3-DOF arm
arm = SerialChain([
    (0.3, 0, 0),    # Link 1
    (0.25, 0, 0),   # Link 2
    (0.15, 0, 0)    # Link 3
])

joint_angles = [np.pi/4, np.pi/6, -np.pi/3]
transforms = arm.forward_kinematics(joint_angles)

print("End-effector pose:\n", transforms[-1])
```

> **🔧 Pro Tip**: Cache intermediate transformations! You'll need them for Jacobian computation (Lesson 5) and inverse dynamics (Lesson 4).

#### Example 3: Parsing URDF for DH Parameters

Real robots use **URDF (Unified Robot Description Format)** XML files:

```python
import xml.etree.ElementTree as ET

def parse_urdf_simple(urdf_path):
    """
    Extract joint information from URDF (simplified).

    Returns:
        joints: List of dicts with 'name', 'type', 'origin', 'axis'
    """
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    joints = []
    for joint in root.findall('joint'):
        origin = joint.find('origin')
        xyz = [float(x) for x in origin.get('xyz', '0 0 0').split()]
        rpy = [float(x) for x in origin.get('rpy', '0 0 0').split()]

        joints.append({
            'name': joint.get('name'),
            'type': joint.get('type'),
            'origin_xyz': xyz,
            'origin_rpy': rpy,
            'axis': joint.find('axis').get('xyz') if joint.find('axis') is not None else '1 0 0'
        })

    return joints

# Usage
# joints = parse_urdf_simple('humanoid.urdf')
# Convert origin_xyz and origin_rpy to DH parameters (non-trivial!)
```

**Reasoning**: URDF uses a different convention than DH (arbitrary axis directions). You'll often use libraries like `pinocchio` or `roboticstoolbox-python` to handle this conversion automatically.

---

### Part 4: The Challenge (Manual Exercise)

**Challenge**: Implement forward kinematics for a 7-DOF robot arm (like a humanoid arm: shoulder 3-DOF, elbow 1-DOF, wrist 3-DOF).

**Requirements**:
1. Define DH parameters for 7-DOF arm (you choose geometry, but include at least one non-zero α and d)
2. Implement `forward_kinematics(joint_angles)` that returns:
   - End-effector position [x, y, z]
   - End-effector orientation (as rotation matrix or quaternion)
   - All intermediate link positions (for visualization)
3. Validate against known test cases (create 3 configurations where you can hand-calculate expected positions)

**Success Criteria**:
- FK computation completes in <1ms for 7 joints
- Position error vs. hand-calculated cases: <0.001 m
- Orientation error: <0.01 radians
- Code must handle arbitrary joint configurations (edge case: all joints at 180°)

**Test Cases**:
```python
# Test 1: Zero configuration (all joints at 0°)
joint_angles = np.zeros(7)
pos, rot = forward_kinematics(joint_angles)
# Expected: end-effector at sum of all link lengths along X-axis

# Test 2: Elbow bent 90°
joint_angles = np.array([0, 0, 0, np.pi/2, 0, 0, 0])
pos, rot = forward_kinematics(joint_angles)
# Expected: specific geometry-dependent position

# Test 3: Complex configuration
joint_angles = np.array([0.5, -0.3, 0.8, 1.2, -0.5, 0.7, -0.2])
pos, rot = forward_kinematics(joint_angles)
# Verify using simulation (MuJoCo/PyBullet) as ground truth
```

**Iteration Guidance**:
1. Start with 2-DOF planar arm (Example 1), verify manually
2. Extend to 3-DOF spatial arm (add non-zero α)
3. Scale to 7-DOF, add caching for intermediate transforms
4. Validate against physics simulator

---

### Part 5: Key Takeaways

1. **DH parameters standardize kinematic modeling**: 4 numbers (a, α, d, θ) per joint encode all geometric information. Learn to derive them from physical measurements or CAD models.

2. **Forward kinematics is recursive composition**: T_0_N = T_0_1 @ T_1_2 @ ... @ T_{N-1}_N. Cache intermediate results—you'll need them for dynamics and Jacobians.

3. **URDF is the industry standard for robot description**: It uses arbitrary joint axes (not DH), so use libraries like Pinocchio or write careful conversion code.

**Looking Ahead**: Lesson 3 flips the problem: given a desired end-effector position, what joint angles achieve it? This **inverse kinematics** problem has no closed-form solution for complex arms and requires optimization.

---

### Part 6: Learn with AI

**Understand It Better**:
> "Explain DH parameters using an analogy with assembling LEGO blocks. How does each parameter relate to the physical connection between blocks?"

**Get Feedback on Your Code**:
> "Review my recursive forward kinematics implementation and suggest improvements for handling kinematic trees (not just serial chains):
> [paste your code here]"

**Go Deeper**:
> "What are the limitations of DH parameters? When do they fail, and what alternative representations exist for kinematic modeling?"

**See It in Action**:
> "Show me how to validate my forward kinematics implementation using MuJoCo or PyBullet. Provide a simple test script."

---

## Lesson 3: Inverse Kinematics and Optimization

**Pedagogical Layer**: 2 (AI Collaboration - CORE SKILL)
**Estimated Time**: 3 hours

---

### Part 1: The Hook (with Diagnostic Pre-Assessment)

Your humanoid robot needs to pick up a cup at position [0.4, 0.2, 0.8] meters. You know forward kinematics: joint angles → hand position. But you need the **inverse**: hand position → joint angles. For a 7-DOF arm reaching a 6-DOF target (position + orientation), there's an infinite number of solutions—or maybe none if the cup is out of reach. How do you find even one valid solution?

This is **inverse kinematics (IK)**, and it's much harder than FK. No simple formula works for complex arms. You'll need numerical methods and optimization.

> **🤖 AI Pre-Assessment**: Before starting, test your understanding:
>
> "A 7-DOF arm reaches a 6-DOF target (position + orientation). How many free parameters ('degrees of redundancy') remain? What could you optimize for with those extra degrees of freedom?"
>
> AI will assess your understanding of redundancy, null-space, and optimization theory to personalize examples in this lesson.

**Learning Objective**: Implement Jacobian-based and optimization-based IK solvers that handle workspace limits, joint constraints, singularities, and redundancy for humanoid arms.

---

### Part 2: The Concept (Theory + AI Tutor Deep-Dive)

#### Visual Intuition: The Unreachable Cup

Stretch your arm toward a distant object. At maximum reach, your arm is fully extended—small joint angle changes barely move your hand. This is a **singularity**. The mathematical tool describing this relationship is the **Jacobian matrix** J.

The Jacobian maps small joint angle changes (δq) to small end-effector changes (δx):
```
δx = J(q) · δq
```

For inverse kinematics, we want the reverse:
```
δq = J^(-1) · δx  or  δq = J^† · δx  (pseudoinverse)
```

> **🎯 Pattern**: IK is iterative. Start with a guess q_0, compute error e = x_target - x_current, update q ← q + J^† e, repeat until converged.

#### Three IK Approaches

| Approach | Method | Pros | Cons |
|----------|--------|------|------|
| **Analytical** | Closed-form algebra | Fastest (microseconds) | Only works for specific geometries (6-DOF with spherical wrist) |
| **Jacobian-based** | Iterative pseudoinverse | Fast (milliseconds), handles redundancy | Fails at singularities, no constraint handling |
| **Optimization** | Minimize ‖FK(q) - x_target‖² | Handles constraints (joint limits, obstacles), finds best solution | Slower (tens of milliseconds), can get stuck in local minima |

#### Singularities and Manipulability

At singularities, the Jacobian loses rank (det(J J^T) ≈ 0). Geometrically, the arm can't move in certain directions—like trying to push a door that only swings inward.

**Manipulability ellipsoid**: Visualizes how easily the robot can move in different directions. Large ellipsoid = high manipulability, small = near singularity.

> **💡 AI Deep-Dive Prompt**:
> "Explain the geometric intuition of the Jacobian pseudoinverse using a 2D example. Why does damped least squares (J^T J + λI)^(-1) J^T help near singularities?"

---

### Part 3: AI-Collaborative Walkthrough

#### Example 1: Jacobian-based IK with Pseudoinverse (Student Implementation → AI Refinement)

**Student Baseline Implementation**:
```python
def jacobian_ik_basic(target_pos, current_q, fk_func, jacobian_func, max_iters=100):
    """
    Basic Jacobian pseudoinverse IK.

    Args:
        target_pos: Desired [x, y, z] position
        current_q: Initial joint angles
        fk_func: Function that returns current position given q
        jacobian_func: Function that returns 3×N Jacobian matrix

    Returns:
        q_solution: Joint angles reaching target (or None if failed)
    """
    q = current_q.copy()

    for i in range(max_iters):
        current_pos = fk_func(q)
        error = target_pos - current_pos

        if np.linalg.norm(error) < 1e-3:  # 1mm tolerance
            return q

        J = jacobian_func(q)
        J_pinv = np.linalg.pinv(J)  # Pseudoinverse

        dq = J_pinv @ error
        q = q + 0.1 * dq  # Small step size

    return None  # Failed to converge
```

> **🤖 AI Code Refiner Prompt**:
> "Refine this IK implementation to add damped least squares near singularities. Detect singularities using manipulability μ = √det(J J^T), and add damping λ when μ < threshold."

**AI-Refined Implementation** (what AI would suggest):
```python
def jacobian_ik_damped(target_pos, current_q, fk_func, jacobian_func,
                       damping=0.01, max_iters=100):
    """
    Damped least squares IK with singularity handling.
    """
    q = current_q.copy()

    for i in range(max_iters):
        current_pos = fk_func(q)
        error = target_pos - current_pos

        if np.linalg.norm(error) < 1e-3:
            return q, True  # Success

        J = jacobian_func(q)

        # Compute manipulability
        manipulability = np.sqrt(np.linalg.det(J @ J.T))

        # Adaptive damping near singularities
        lambda_factor = damping if manipulability > 0.1 else damping * 10

        # Damped least squares: (J^T J + λI)^-1 J^T
        J_damped = J.T @ np.linalg.inv(J @ J.T + lambda_factor * np.eye(J.shape[0]))

        dq = J_damped @ error
        q = q + 0.5 * dq  # Larger step when far from singularity

        # Apply joint limits (simple clipping)
        q = np.clip(q, -np.pi, np.pi)

    return q, False  # Failed to converge
```

**AI Contextual Debugger Example**:
> **Student**: "My IK fails when the target is reachable but the arm jerks wildly during convergence."
>
> **AI Diagnosis**: "This is due to large step sizes when far from the target. Your error is 0.5m, but you're taking 0.1 * δq steps, causing overshoot. Solutions: (1) Adaptive step size (large when error is big, small when close), (2) Line search along δq direction, (3) Trust-region methods. Try: `alpha = min(0.5, np.linalg.norm(error))`"

#### Example 2: Optimization-based IK with Constraints (Drake)

**AI System Analyzer Comparison**:
> "Compare computational costs: Jacobian IK typically converges in 10-50 iterations × O(n²) for pseudoinverse = ~1-5ms for 7-DOF arm. Optimization IK with constraints requires nonlinear solver (SLSQP, IPOPT) = 10-50ms. Trade-off: 10× slower but handles joint limits, collision avoidance, and secondary objectives."

```python
from scipy.optimize import minimize

def optimization_ik(target_pos, target_quat, initial_q, fk_func,
                    joint_limits, max_time=0.1):
    """
    Optimization-based IK with constraints.

    Args:
        target_pos: Desired [x, y, z]
        target_quat: Desired orientation [w, x, y, z]
        initial_q: Starting configuration
        fk_func: Returns (pos, quat) given q
        joint_limits: (q_min, q_max) arrays

    Returns:
        q_solution, success
    """
    def cost_function(q):
        """Minimize position and orientation error."""
        pos, quat = fk_func(q)

        pos_error = np.linalg.norm(target_pos - pos)

        # Quaternion distance (1 - |q1 · q2|)
        quat_error = 1 - np.abs(np.dot(target_quat, quat))

        # Secondary objective: minimize joint displacement from initial
        joint_displacement = 0.01 * np.linalg.norm(q - initial_q)

        return pos_error + quat_error + joint_displacement

    # Constraints: joint limits
    bounds = list(zip(joint_limits[0], joint_limits[1]))

    result = minimize(
        cost_function,
        initial_q,
        method='SLSQP',
        bounds=bounds,
        options={'maxiter': 100, 'ftol': 1e-6}
    )

    # Verify solution
    final_pos, final_quat = fk_func(result.x)
    pos_error = np.linalg.norm(target_pos - final_pos)

    success = pos_error < 1e-3  # 1mm tolerance
    return result.x, success
```

---

### Part 4: SDD-RI Challenge (AI Generator + Grader)

**Specification-Driven Challenge**:

Write a formal specification FIRST, then let AI generate initial implementation:

```
SPECIFICATION: Multi-Method IK Solver for 7-DOF Humanoid Arm

INPUTS:
  - target_position: [x, y, z] in world frame (meters)
  - target_orientation: quaternion [w, x, y, z] (optional, None = position-only IK)
  - initial_guess: 7-element joint angle array (radians)
  - robot_model: URDF path or DH parameters

OUTPUTS:
  - solution: 7-element joint angle array satisfying target
  - success: boolean (True if converged within tolerances)
  - info: dict with {
      'method_used': 'jacobian' | 'optimization',
      'iterations': int,
      'final_error': float,
      'manipulability': float (at solution)
    }

CONSTRAINTS:
  - Joint limits: q_min ≤ q ≤ q_max (from URDF or [-π, π])
  - Workspace reachability: Check ‖target‖ ≤ sum(link_lengths) before solving
  - Singularity avoidance: Prefer solutions with manipulability μ > 0.05
  - Timeout: Return failure after 1 second

SUCCESS CRITERIA:
  - Position error: ‖FK(q) - target_position‖ < 0.001 m (1mm)
  - Orientation error (if specified): angle(FK_quat(q), target_quat) < 0.01 rad (~0.6°)
  - Joint limits satisfied: all q ∈ [q_min, q_max]

TEST CASES:
  1. Reachable target near workspace center: [0.4, 0.2, 0.8], expect success via Jacobian
  2. Unreachable target: [5.0, 0, 0], expect failure within 1s (workspace check)
  3. Near singularity (arm nearly straight): [0.68, 0, 0.8], expect optimization method selected
  4. Redundancy test: position-only target [0.3, 0.3, 0.5], expect multiple solutions (return best by manipulability)
```

> **🤖 AI Generator Prompt**:
> "Implement this IK solver specification using a hybrid approach: try Jacobian method first (fast), fall back to optimization if fails or near singularity. Generate test URDFs with varying DOF for validation."

**AI Grading Criteria**:
- **Spec Alignment (60%)**:
  - Workspace check prevents unreachable targets (15%)
  - Both success and failure cases handled correctly (15%)
  - All constraints satisfied (joint limits, manipulability) (15%)
  - Timeout enforced (15%)
- **Code Quality (40%)**:
  - Appropriate algorithm selection (hybrid Jacobian/optimization) (15%)
  - Readable code with clear function contracts (10%)
  - Edge cases handled (e.g., initial_guess at singularity) (15%)

**Iteration Required**: If your specification misses a critical constraint (e.g., "What if initial_guess violates joint limits?"), AI will fail that test case and ask you to refine the spec.

---

### Part 5: Spaced-Repetition (AI-Generated Flashcards)

> **🤖 AI Retention Partner**: Generate flashcards for review:

1. **Q**: What happens to the Jacobian at singularities?
   **A**: J loses rank (det(J J^T) → 0), making J^(-1) undefined. Small joint motions produce no end-effector motion in certain directions.

2. **Q**: When should you use damped least squares instead of standard pseudoinverse?
   **A**: Near singularities (manipulability μ < threshold) to avoid large, unstable joint velocities. Damping factor λ regularizes (J^T J + λI)^(-1).

3. **Q**: Pros and cons of analytical IK vs. numerical optimization?
   **A**: Analytical: Fastest (μs), but only for specific geometries (e.g., 6-DOF spherical wrist). Optimization: Handles arbitrary constraints (joint limits, obstacles) but slower (ms) and may find local minima.

4. **Q**: How many solutions exist for a 7-DOF arm reaching a 6-DOF target?
   **A**: Infinitely many (1 degree of redundancy). Null-space of Jacobian describes the manifold of solutions. Can optimize secondary objectives (manipulability, joint centering).

---

### Part 6: Reusable Intelligence - IK Solver Skill

> **🤖 Student Teaches AI** (Apprentice Mode):
>
> "Create an **IK Solver Skill** with these 3 non-negotiable instructions:"

**IK Solver Skill v1.0**:
1. **Always check workspace reachability first** using ‖target‖ ≤ Σ(link_lengths). Return immediate failure for unreachable targets—don't waste computation.

2. **Prioritize manipulability** by adding secondary cost: minimize -log(det(J J^T)). Avoid configurations near singularities even if they satisfy position error.

3. **Return top-k solutions for redundant systems** ranked by secondary criteria (manipulability, joint range usage, distance from obstacles). Don't just return the first solution found.

**Example Usage in AI Conversation**:
> "Use the IK Solver Skill to find 3 candidate joint configurations for reaching [0.5, 0.3, 0.9], then select the one with highest manipulability."

---

## Lesson 4: Rigid Body Dynamics and the Manipulator Equation

**Pedagogical Layer**: 2 (AI Collaboration - CORE SKILL)
**Estimated Time**: 3 hours

---

### Part 1: The Hook (with Diagnostic Pre-Assessment)

You've computed where the robot's hand is (FK) and which joint angles reach a target (IK). But now you need to **move** the robot. Applying torque τ to joint motors causes motion, but how much torque is needed to achieve a desired acceleration?

This is the realm of **dynamics**: the relationship between forces/torques and motion. The fundamental equation is the **manipulator equation**:

```
M(q)q̈ + C(q,q̇)q̇ + g(q) = τ
```

Understanding each term—inertia, Coriolis forces, gravity—is essential for control, simulation, and predicting robot behavior.

> **🤖 AI Pre-Assessment**:
> "Derive the kinetic energy of a rotating rigid body with moment of inertia I and angular velocity ω. What's the relationship to the mass matrix M(q)?"
>
> AI will detect gaps in physics background (energy vs. force formulations) and personalize examples.

**Learning Objective**: Implement inverse dynamics (RNEA) and understand the manipulator equation's components (M, C, g) using both manual derivation and efficient algorithms (Pinocchio library).

---

### Part 2: The Concept (Theory + AI Tutor Deep-Dive)

#### Visual Intuition: The Manipulator Equation as a Balance Sheet

Imagine pushing a shopping cart (mass m):
```
Force_needed = m × acceleration + friction + gravity_if_on_slope
```

The manipulator equation is the robot version:
```
τ = M(q)q̈ + C(q,q̇)q̇ + g(q)
    ↑         ↑          ↑
 inertia   Coriolis   gravity
```

- **M(q)**: "Mass matrix"—how hard it is to accelerate each joint (depends on configuration q because arm's inertia changes as it moves)
- **C(q,q̇)q̇**: Coriolis/centrifugal forces—"fictitious" forces from rotating reference frames (e.g., water swirling in a spinning bucket)
- **g(q)**: Gravity torques—how much torque needed to hold the arm stationary against gravity

#### Two Formulations: Lagrangian vs. Newton-Euler

| Approach | Method | Pros | Cons |
|----------|--------|------|------|
| **Lagrangian** | Energy-based (KE - PE) | Conceptually clean, gives M(q) directly | Slow for high-DOF (requires symbolic differentiation) |
| **Newton-Euler** | Force/torque balance per link | Fast recursive algorithms (RNEA) | Less intuitive derivation |

Both give the **same result**, just different derivation paths.

#### The Manipulator Equation Visualized

For a 2-DOF planar arm:
```
M(q) = [m11(q)  m12(q)]   (symmetric, positive definite)
       [m12(q)  m22(q)]

C(q,q̇) = [-m12·q̇2   -m12·(q̇1+q̇2)]   (Coriolis matrix)
         [ m12·q̇1        0        ]

g(q) = [g1(q)]   (gravity torques)
       [g2(q)]
```

> **💡 AI Deep-Dive Prompt**:
> "Explain why the mass matrix M(q) must be symmetric and positive definite using energy conservation. What breaks if M is not positive definite?"

---

### Part 3: AI-Collaborative Walkthrough

#### Example 1: Lagrangian Derivation for 2-DOF Arm (Manual → AI Validation)

**Student Manual Derivation**:

Consider a 2-link planar arm with link masses m1, m2 and lengths L1, L2.

**Kinetic Energy**:
```
KE = (1/2) m1 v1² + (1/2) m2 v2² + (1/2) I1 ω1² + (1/2) I2 ω2²
```

After deriving velocities and simplifying (tedious algebra!):
```
KE = (1/2)[q̇1, q̇2] M(q) [q̇1]
                          [q̇2]
```

**Potential Energy** (gravity):
```
PE = m1 g h1(q) + m2 g h2(q)
where h1, h2 are heights of centers of mass
```

**Lagrangian** L = KE - PE, then apply **Euler-Lagrange equations**:
```
d/dt(∂L/∂q̇i) - ∂L/∂qi = τi
```

> **🤖 AI Code Refiner Prompt**:
> "I derived M(q) symbolically for a 2-DOF arm but get numerical instabilities when q2 ≈ ±π. How do I ensure numerical stability in the implementation?"

**AI Suggested Refinement**:
```python
def mass_matrix_2dof(q, m1, m2, L1, L2, I1, I2):
    """
    Numerically stable mass matrix computation.
    """
    q1, q2 = q

    # Use cos/sin with proper conditioning
    c2 = np.cos(q2)

    # Intermediate terms (avoid recomputation)
    alpha = I1 + I2 + m1*(L1/2)**2 + m2*(L1**2 + (L2/2)**2)
    beta = m2 * L1 * (L2/2)
    gamma = I2 + m2 * (L2/2)**2

    # Mass matrix
    M = np.array([
        [alpha + 2*beta*c2, gamma + beta*c2],
        [gamma + beta*c2,   gamma           ]
    ])

    # Numerical check: ensure positive definite
    eigvals = np.linalg.eigvalsh(M)  # Symmetric eigenvalue solver
    if np.any(eigvals <= 0):
        raise ValueError(f"Mass matrix not positive definite! Eigenvalues: {eigvals}")

    return M
```

**AI Contextual Debugger Example**:
> **Student**: "My simulation explodes when the arm is near vertical (q1 ≈ π/2). Energy increases without bound."
>
> **AI Diagnosis**: "This is a **symplectic integration** issue. Your Euler integrator doesn't conserve energy for Hamiltonian systems. Solutions: (1) Use symplectic integrator (Verlet, leapfrog), (2) Add numerical damping, (3) Check that M(q) stays positive definite (recompute eigenvalues each step). Try: `scipy.integrate.ode` with 'dopri5' (Runge-Kutta 4/5)."

#### Example 2: RNEA (Recursive Newton-Euler Algorithm) for Inverse Dynamics

**Inverse Dynamics Problem**: Given q, q̇, q̈ (desired motion), compute τ (required torques).

RNEA is **O(n)** (linear in number of joints), vs. O(n³) for symbolic Lagrangian.

**Algorithm Structure** (simplified):
```python
def rnea_inverse_dynamics(q, qd, qdd, link_params, gravity=[0, 0, -9.81]):
    """
    Recursive Newton-Euler Algorithm.

    Args:
        q, qd, qdd: Joint positions, velocities, accelerations
        link_params: List of (mass, com, inertia) for each link

    Returns:
        tau: Joint torques
    """
    n = len(q)

    # Forward pass: propagate velocities and accelerations
    v = [np.zeros(3)]  # Linear velocities
    w = [np.zeros(3)]  # Angular velocities
    vd = [np.array(gravity)]  # Linear accelerations (start with -gravity)
    wd = [np.zeros(3)]  # Angular accelerations

    for i in range(n):
        # Transform from link i-1 to link i (using FK transformations)
        # Update v[i], w[i], vd[i], wd[i] (omitted for brevity—see Featherstone)
        pass  # Actual implementation: ~20 lines of vector math

    # Backward pass: propagate forces and torques
    f = [np.zeros(3) for _ in range(n+1)]  # Forces
    tau_list = [np.zeros(3) for _ in range(n+1)]  # Torques
    tau_result = np.zeros(n)

    for i in range(n-1, -1, -1):
        # Compute link i forces from link i+1 forces + inertial terms
        # Extract joint torque (omitted—see Featherstone's book)
        pass  # Actual implementation: ~15 lines

    return tau_result
```

**Better Alternative: Use Pinocchio Library**:
```python
import pinocchio as pin

# Load robot model
model = pin.buildModelFromUrdf("humanoid.urdf")
data = model.createData()

# Compute inverse dynamics (RNEA internally)
q = np.array([...])   # Joint positions
qd = np.array([...])  # Joint velocities
qdd = np.array([...]) # Joint accelerations

tau = pin.rnea(model, data, q, qd, qdd)
print("Required torques:", tau)
```

> **🤖 AI System Analyzer**:
> "Compare: Manual RNEA implementation for 7-DOF arm: ~50 lines, easy to introduce bugs. Pinocchio RNEA: 1 line, optimized C++ backend, 10-100× faster. Trade-off: Pinocchio is black-box (hard to debug internals), manual gives full understanding. Use manual for learning, Pinocchio for production."

#### Example 3: Energy Conservation Validation

**Physics Sanity Check**: In free motion (τ = 0), total energy E = KE + PE should be conserved.

```python
def total_energy(q, qd, mass_matrix_func, potential_energy_func):
    """Compute total mechanical energy."""
    M = mass_matrix_func(q)
    KE = 0.5 * qd.T @ M @ qd
    PE = potential_energy_func(q)
    return KE + PE

# Simulate free motion
q, qd = initial_state()
energies = []

for t in np.linspace(0, 10, 1000):
    # Integrate dynamics with τ = 0
    q, qd = integrate_step(q, qd, tau=0, dt=0.01)
    energies.append(total_energy(q, qd, M_func, PE_func))

# Check conservation
energy_drift = np.max(energies) - np.min(energies)
print(f"Energy drift: {energy_drift:.6f} J (should be < 0.01 J)")
```

---

### Part 4: SDD-RI Challenge (AI Generator + Grader)

**Specification**:
```
SPECIFICATION: Dynamics Engine for Arbitrary URDF Robots

INPUTS:
  - robot_urdf: Path to URDF file
  - q, qd, qdd: Joint positions, velocities, accelerations (numpy arrays)
  - mode: 'inverse_dynamics' | 'forward_dynamics' | 'mass_matrix'

OUTPUTS:
  - inverse_dynamics mode: tau (joint torques required)
  - forward_dynamics mode: qdd (accelerations given tau)
  - mass_matrix mode: M(q) (configuration-dependent mass matrix)

CONSTRAINTS:
  - Handle kinematic trees (not just serial chains)
  - Validate energy conservation in simulation (drift < 0.1% per second)
  - Computational efficiency: RNEA inverse dynamics in O(n) time

SUCCESS CRITERIA:
  - Energy conservation: Simulate 10s free motion (τ=0), energy drift < 1%
  - Correctness: Match Pinocchio library outputs within 1e-6 for all modes
  - Performance: Inverse dynamics for 30-DOF humanoid in < 1ms

TEST CASES:
  1. Simple pendulum (1-DOF): Verify against analytical solution τ = mgl sin(q) + Iq̈
  2. 7-DOF arm: Compare M(q), C(q,q̇), g(q) with Pinocchio reference
  3. 30-DOF humanoid: Energy conservation over 10s simulation
```

> **🤖 AI Generator**: "Implement using Pinocchio library wrappers. Generate test URDFs (1-DOF, 7-DOF, 30-DOF). Create energy conservation validator."

**Grading**:
- **Spec Alignment (60%)**:
  - Energy conservation within 1% (20%)
  - Matches reference library outputs (20%)
  - Handles kinematic trees correctly (20%)
- **Code Quality (40%)**:
  - Efficient implementation (O(n) for RNEA) (20%)
  - Clear separation of ID/FD/mass_matrix modes (10%)
  - Validation framework included (10%)

---

### Part 5: Spaced-Repetition (AI Flashcards)

1. **Q**: What does each term in M(q)q̈ + C(q,q̇)q̇ + g(q) = τ represent physically?
   **A**: M(q)q̈: inertial resistance to acceleration | C(q,q̇)q̇: Coriolis/centrifugal forces from motion | g(q): gravity compensation torques

2. **Q**: When do you use forward dynamics vs. inverse dynamics?
   **A**: Inverse: Given desired motion (q, q̇, q̈), compute required torques τ (for control). Forward: Given torques τ, compute resulting motion q̈ (for simulation).

3. **Q**: Computational complexity of CRBA vs. RNEA?
   **A**: CRBA (Composite Rigid Body Algorithm) computes M(q) in O(n²). RNEA computes inverse dynamics in O(n). For inverse dynamics, use RNEA directly—don't compute M separately.

4. **Q**: Why must M(q) be symmetric positive definite?
   **A**: Symmetric: Follows from kinetic energy formula (KE = (1/2)q̇^T M q̇). Positive definite: Ensures kinetic energy is always positive (physical requirement). If M has negative eigenvalues, energy can become negative (unphysical).

---

### Part 6: Reusable Intelligence - Dynamics Computation Skill

> **🤖 Student Teaches AI**:

**Dynamics Computation Skill v1.0**:
1. **Leverage sparsity in mass matrix**: Most humanoid M(q) matrices are sparse (distant joints don't interact). Use sparse matrix storage for n > 20 DOF—saves 10× memory and 5× computation.

2. **Cache configuration-dependent terms**: M(q) and g(q) depend only on positions q, while C(q,q̇) depends on velocities. When running control loops at high frequency, cache M and g if q hasn't changed significantly.

3. **Validate via energy conservation**: After every dynamics implementation change, run free-motion simulation and check energy drift. Bugs in M, C, or g immediately show up as energy gain/loss.

---

## Lesson 5: Contact Dynamics and Stability Criteria

**Pedagogical Layer**: 2 (AI Collaboration - CORE SKILL)
**Estimated Time**: 2.5 hours

---

### Part 1: The Hook (with Diagnostic Pre-Assessment)

Your humanoid robot stands on one foot. Gravity pulls it down, but the ground pushes back. How do you compute these **contact forces**? More critically, when does the robot fall over?

This is **contact dynamics** and **stability analysis**—the difference between a balancing robot and a pile of expensive scrap metal. The **Zero Moment Point (ZMP)** criterion tells you exactly when the robot will tip.

> **🤖 AI Pre-Assessment**:
> "A humanoid stands on one foot at position [0, 0]. The center of mass is at [0.05, 0, 0.8] meters. Where must the ZMP lie for the robot to remain stable? What happens if the ZMP moves outside the foot polygon?"
>
> AI personalizes contact dynamics examples based on your statics background.

**Learning Objective**: Implement ZMP computation from force sensor data, determine support polygons for single/double foot contact, and build a real-time stability monitor that predicts falls.

---

### Part 2: The Concept (Theory + AI Tutor)

#### Visual Intuition: The Tipping Point

Stand on one foot and lean slowly to the side. At some point, you'll tip over—no amount of ankle torque can save you. That tipping point is when the **center of pressure (CoP)** reaches the edge of your foot.

For robots, we use the **Zero Moment Point (ZMP)**—the point on the ground where the net moment (torque) from inertial and gravitational forces is zero.

> **🎯 Pattern**: If ZMP ∈ support polygon (convex hull of contact points), robot is stable. If ZMP exits polygon, robot tips.

#### Contact Forces: Normal and Friction

Each foot-ground contact produces:
- **Normal force** (perpendicular to surface): f_n ≥ 0 (can't pull on ground)
- **Friction force** (tangent to surface): f_t ≤ μ f_n (Coulomb friction)

The **friction cone** constraint:
```
‖f_t‖ ≤ μ f_n   where μ is friction coefficient (typically 0.5-1.0)
```

Violating this means the foot slips.

#### ZMP Computation from Ground Reaction Forces

Given total ground reaction force **F** and moment **M** at a reference point:
```
ZMP_x = (M_y + F_x · z_ground) / F_z
ZMP_y = (-M_x + F_y · z_ground) / F_z
```

If your robot has 6-axis force/torque sensors in each foot, you can compute this in real-time.

> **💡 AI Deep-Dive Prompt**:
> "Visualize the friction cone in 3D. Show what happens when a humanoid tries to walk on ice (μ = 0.1) vs. rubber mat (μ = 1.2). How does this affect maximum walking speed?"

#### ZMP vs. Capture Point: Static vs. Dynamic Stability

- **ZMP**: Static stability criterion (assumes zero velocity). Conservative.
- **Capture Point**: Dynamic stability (considers momentum). Predicts where robot needs to step to avoid falling.

Capture point xc:
```
xc = CoM_position + CoM_velocity / ω
where ω = sqrt(g / CoM_height)
```

---

### Part 3: AI-Collaborative Walkthrough

#### Example 1: Computing ZMP from Force Sensors (Student → AI Refinement)

**Student Baseline**:
```python
def compute_zmp_single_foot(force, moment, foot_z=0.0):
    """
    Compute ZMP from 6-axis force/torque sensor.

    Args:
        force: [fx, fy, fz] in Newtons
        moment: [mx, my, mz] in N·m (at sensor origin)
        foot_z: Height of ground contact (usually 0)

    Returns:
        zmp: [x, y] position of ZMP (or None if in flight)
    """
    fx, fy, fz = force
    mx, my, mz = moment

    if fz < 1.0:  # Not in contact (threshold: 1 Newton)
        return None

    zmp_x = (my + fx * foot_z) / fz
    zmp_y = (-mx + fy * foot_z) / fz

    return np.array([zmp_x, zmp_y])
```

> **🤖 AI Code Refiner Prompt**:
> "Extend this to handle **double support** (both feet on ground). Need to combine force/moment from left and right foot sensors."

**AI-Refined Implementation**:
```python
def compute_zmp_double_support(left_force, left_moment, left_pos,
                                right_force, right_moment, right_pos):
    """
    Compute ZMP with both feet in contact.

    Args:
        left_force, right_force: [fx, fy, fz] for each foot
        left_moment, right_moment: [mx, my, mz] for each foot
        left_pos, right_pos: [x, y, z] sensor positions in world frame

    Returns:
        zmp: [x, y] in world frame
    """
    # Total force and moment at world origin
    F_total = left_force + right_force

    # Transport moments to world origin
    M_left = left_moment + np.cross(left_pos, left_force)
    M_right = right_moment + np.cross(right_pos, right_force)
    M_total = M_left + M_right

    # Compute ZMP
    fz_total = F_total[2]
    if fz_total < 1.0:
        return None  # In flight

    zmp_x = M_total[1] / fz_total
    zmp_y = -M_total[0] / fz_total

    return np.array([zmp_x, zmp_y])
```

**AI Contextual Debugger**:
> **Student**: "My ZMP jumps around wildly during simulation (±10cm noise)."
>
> **AI Diagnosis**: "Force sensor noise! Real sensors have ~0.1-1N noise. Solutions: (1) Low-pass filter (Butterworth, cutoff ~10Hz), (2) Kalman filter with dynamics model, (3) Increase contact stiffness in simulation (MuJoCo `solimp` parameter). Try: `scipy.signal.butter(3, 10, fs=1000)` for 1kHz sensor."

#### Example 2: Support Polygon Computation

```python
from scipy.spatial import ConvexHull

def compute_support_polygon(contact_points):
    """
    Compute convex hull of contact points (support polygon).

    Args:
        contact_points: Nx2 array of [x, y] points

    Returns:
        vertices: Ordered vertices of convex polygon
    """
    if len(contact_points) < 3:
        return contact_points  # Degenerate case

    hull = ConvexHull(contact_points)
    vertices = contact_points[hull.vertices]

    return vertices

def is_stable(zmp, support_polygon, safety_margin=0.02):
    """
    Check if ZMP is inside support polygon (with margin).

    Args:
        zmp: [x, y] point
        support_polygon: Nx2 vertices (ordered)
        safety_margin: Distance in meters from edge (conservative)

    Returns:
        stable: Boolean
    """
    from matplotlib.path import Path

    # Shrink polygon by safety margin (use offset algorithm for production)
    # Simplified: just check distance to edges
    polygon_path = Path(support_polygon)

    if not polygon_path.contains_point(zmp):
        return False

    # Check margin (simplified - just check distance to centroid)
    centroid = np.mean(support_polygon, axis=0)
    max_radius = np.max(np.linalg.norm(support_polygon - centroid, axis=1))
    zmp_radius = np.linalg.norm(zmp - centroid)

    return zmp_radius < (max_radius - safety_margin)

# Example: Single foot contact
foot_corners = np.array([
    [0.1, 0.05],   # Front-right
    [0.1, -0.05],  # Front-left
    [-0.05, 0.05], # Back-right
    [-0.05, -0.05] # Back-left
])

zmp = np.array([0.02, 0.01])  # Near center
print("Stable:", is_stable(zmp, foot_corners, safety_margin=0.02))
```

> **🔧 Pro Tip**: Use safety margin (2-5cm from edge) for real robots—sensors have noise and ground may not be perfectly flat. Conservative stability prevents falls.

#### Example 3: MuJoCo Contact Parameter Tuning

Simulation contact behavior depends on stiffness and damping parameters:

```python
# In MuJoCo XML model
<geom name="foot" ... solimp="0.9 0.95 0.001" solref="0.02 1"/>
```

- **solimp**: Impedance parameters [dmin, dmax, width] (controls penetration)
- **solref**: Reference parameters [timeconst, dampratio] (controls bounce/damping)

> **🤖 AI System Analyzer**:
> "Compare contact models: MuJoCo uses convex optimization (finds contact forces that minimize constraint violations). PyBullet uses sequential impulses (faster but less accurate). Isaac Sim uses PhysX (GPU-accelerated, good for parallel envs). For humanoid walking: MuJoCo gives best accuracy, Isaac Sim for RL training (1000+ parallel robots)."

---

### Part 4: SDD-RI Challenge (AI Generator + Grader)

**Specification**:
```
SPECIFICATION: Real-Time Stability Monitor

INPUTS:
  - sensor_data: Dict with {
      'left_foot': {'force': [fx,fy,fz], 'moment': [mx,my,mz], 'position': [x,y,z]},
      'right_foot': {...},
      'com_state': {'position': [x,y,z], 'velocity': [vx,vy,vz]}
    }
  - contact_points: Nx3 array of contact point positions

OUTPUTS:
  - zmp: [x, y] current ZMP location
  - support_polygon: Mx2 array of support polygon vertices
  - stability: {
      'is_stable': bool,
      'margin': float (distance from ZMP to polygon edge, meters),
      'warning': bool (margin < 0.03m)
    }
  - capture_point: [x, y] predicted capture point (for dynamic stability)

CONSTRAINTS:
  - Real-time: Must run at ≥ 200 Hz (5ms max latency)
  - Handle single support, double support, and in-flight phases
  - Low-pass filter sensor noise (cutoff 10 Hz)

SUCCESS CRITERIA:
  - ZMP computation accuracy: < 5mm error vs. ground truth (from simulation)
  - Stability classification: 100% correct on test scenarios (stable/unstable cases)
  - Early warning: Detect impending fall 0.5s before ZMP exits polygon

TEST CASES:
  1. Double support standing: Both feet flat, expect stable with large margin
  2. Single support balancing: One foot contact, ZMP near edge, expect warning
  3. Free fall: No ground contact, expect None for ZMP, is_stable=False
  4. External push: Apply 50N lateral force, verify early warning triggers
```

> **🤖 AI Generator**: "Implement using filtered sensor data. Generate test scenarios in MuJoCo (standing, balancing, falling). Create visualizer showing ZMP, polygon, and capture point in real-time."

**Grading**:
- **Spec Alignment (60%)**:
  - Correct ZMP for all contact modes (20%)
  - Stability classification 100% accurate (20%)
  - Early warning system functional (20%)
- **Code Quality (40%)**:
  - Real-time performance (< 5ms) (20%)
  - Sensor filtering implemented (10%)
  - Edge case handling (degenerate polygons) (10%)

---

### Part 5: Spaced-Repetition (AI Flashcards)

1. **Q**: What's the difference between ZMP and CoP?
   **A**: CoP (Center of Pressure): actual point where ground reaction force acts. ZMP: theoretical point where net moment = 0. In stable contact, ZMP = CoP. If ZMP "wants" to be outside support polygon, robot tips and CoP moves to polygon edge (ZMP ≠ CoP).

2. **Q**: How do you compute ZMP from contact forces?
   **A**: ZMP_x = M_y / F_z, ZMP_y = -M_x / F_z, where M is moment about origin, F_z is vertical force. For double support, sum forces and moments from both feet.

3. **Q**: Why is capture point better than ZMP for dynamic walking?
   **A**: ZMP assumes zero velocity (static). Capture point considers momentum (CoM velocity). For fast walking, robot can temporarily have ZMP outside support polygon if capture point is inside—robot will "catch" itself with next step.

4. **Q**: What are friction cone constraints?
   **A**: Tangential friction force ≤ μ × normal force, where μ is friction coefficient. Geometrically, friction force must lie inside a cone. Violation means foot slips.

---

### Part 6: Reusable Intelligence - Contact Stability Analyzer Skill

> **🤖 Student Teaches AI**:

**Contact Stability Analyzer Skill v1.0**:
1. **Compute support polygon as convex hull with degenerate case handling**: For <3 contact points, support polygon is undefined (point or line). Treat as maximally unstable—trigger immediate warning.

2. **Check ZMP with configurable safety margin** (default 80% of polygon): Don't wait for ZMP to reach exact edge. Use conservative threshold (e.g., 3cm from edge for large humanoid) to allow time for corrective action.

3. **Output early warning signals before instability**: Predict ZMP trajectory using current CoM velocity. If projected ZMP in 0.5s will exit polygon, trigger warning NOW—don't wait for actual instability. Predictive, not reactive.

---

**End of Lesson Part 1 (Lessons 1-5)**

Total word count: ~3,950 words
Lessons covered: 1-5 (Transformations, FK, IK, Dynamics, Contact)
Pedagogical layers: Layer 1 (Lessons 1-2), Layer 2 (Lessons 3-5)

Next: Lessons 6-9 will cover simulation frameworks, sim-to-real transfer, intelligence design, and spec-driven integration.
