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
# P5-C1 Lesson Content: Humanoid Kinematics & Dynamics (Part 2)

**Version**: v001
**Date**: 2025-11-30
**Planner**: lesson-planner
**Lessons Covered**: 6-9
**Status**: Complete

---

## Lesson 6: Simulation Frameworks - MuJoCo and Isaac Sim

**Pedagogical Layer**: 2 (AI Collaboration - CORE SKILL)
**Estimated Time**: 2.5 hours

---

### Part 1: The Hook (with Diagnostic Pre-Assessment)

You've implemented kinematics and dynamics algorithms from scratch. But testing on real hardware is expensive—one bug could destroy a $50,000 robot. Before deploying to physical systems, you need to **validate in simulation**.

Modern physics simulators like MuJoCo and Isaac Sim can model contact dynamics, friction, and even sensor noise with remarkable accuracy. They're not just visualization tools—they're your testing ground for everything from basic FK validation to training reinforcement learning policies.

> **🤖 AI Pre-Assessment**:
> "What's the difference between kinematic and dynamic simulation? If you set a robot's joint position directly vs. applying torque, which requires a physics engine?"
>
> AI will assess your understanding of simulation fundamentals and personalize framework introductions.

**Learning Objective**: Load humanoid models into MuJoCo and Isaac Sim, implement control loops that interact with simulated physics, extract state data (FK, contact forces, joint sensors), and validate cross-simulator consistency.

---

### Part 2: The Concept (Theory + AI Tutor)

#### Visual Intuition: Simulation as a Virtual Lab

Think of a physics simulator as a **virtual wind tunnel** for robots. Just as aircraft designers test 1000 wing shapes in simulation before building one prototype, you'll test 1000 control strategies before touching real hardware.

But simulators aren't perfect—they're **approximations** of reality. Understanding their trade-offs is critical:

| Simulator | Physics Engine | Strengths | Weaknesses | Best For |
|-----------|----------------|-----------|------------|----------|
| **MuJoCo** | Convex optimization | Accurate contacts, deterministic | CPU-only, no GPU acceleration | Research, precise dynamics |
| **Isaac Sim** | PhysX (NVIDIA) | 1000+ parallel robots (GPU), photorealistic rendering | Requires NVIDIA GPU, closed-source | Reinforcement learning, sim2real |
| **PyBullet** | Bullet Physics | Easy setup, fast iteration | Less accurate contacts than MuJoCo | Prototyping, education |

#### Simulation Loop Structure

All simulators follow this pattern:

```
Initialize model → Loop {
    1. Apply control (set torques/positions)
    2. Step physics (integrate dynamics)
    3. Read sensors (joint angles, forces, camera)
    4. Update control based on observations
}
```

> **🎯 Pattern**: Simulation is a **predict-observe-control** cycle. Your control algorithm runs in the loop's logic.

#### Contact Solvers: The Critical Difference

When a foot hits the ground, the simulator must compute contact forces that:
- Prevent penetration (complementarity constraint)
- Satisfy friction cone limits
- Minimize energy dissipation

**MuJoCo's approach**: Formulates this as a **convex optimization problem** (quadratic program). Guarantees physically consistent solution.

**PyBullet's approach**: Uses **sequential impulses** (iterative constraint satisfaction). Faster but can accumulate errors.

> **💡 AI Deep-Dive Prompt**:
> "Explain MuJoCo's contact optimization vs. PyBullet's constraint-based approach using a simple example of a box landing on a table. Show the optimization formulation."

---

### Part 3: AI-Collaborative Walkthrough

#### Example 1: MuJoCo Basic Simulation Loop (Student → AI Refinement)

**Student Baseline Implementation**:

```python
import mujoco as mj
import numpy as np

# Load model
model = mj.MjModel.from_xml_path("humanoid.xml")
data = mj.MjData(model)

# Simulation parameters
dt = 0.002  # 2ms timestep (500 Hz)
duration = 10.0  # 10 seconds

# Simple gravity compensation control
for step in range(int(duration / dt)):
    # Compute gravity torques using inverse dynamics
    mj.mj_inverse(model, data)  # Fills data.qfrc_inverse

    # Apply control: gravity compensation
    data.ctrl[:] = data.qfrc_inverse

    # Step simulation
    mj.mj_step(model, data)

    # Read state
    if step % 500 == 0:  # Print every 1 second
        print(f"t={data.time:.2f}s, CoM height={data.subtree_com[1,2]:.3f}m")
```

> **🤖 AI Code Refiner Prompt**:
> "Extend this to add PD control for tracking desired joint positions. Include proper time-stepping with rendering at 60 FPS while simulating at 500 Hz."

**AI-Refined Implementation**:

```python
import mujoco as mj
import mujoco.viewer as viewer
import numpy as np

class MuJoCoSimulator:
    def __init__(self, model_path, render=True):
        self.model = mj.MjModel.from_xml_path(model_path)
        self.data = mj.MjData(self.model)
        self.render = render

        # PD gains (tune per joint)
        self.kp = 100.0 * np.ones(self.model.nu)  # Position gain
        self.kd = 10.0 * np.ones(self.model.nu)   # Velocity gain

    def pd_control(self, q_desired, qd_desired=None):
        """
        Compute PD control torques.

        Args:
            q_desired: Target joint positions
            qd_desired: Target joint velocities (default: zero)

        Returns:
            tau: Control torques
        """
        if qd_desired is None:
            qd_desired = np.zeros_like(q_desired)

        # Current state
        q = self.data.qpos[-self.model.nu:]  # Skip floating base if present
        qd = self.data.qvel[-self.model.nu:]

        # PD control law
        tau = self.kp * (q_desired - q) + self.kd * (qd_desired - qd)

        # Add gravity compensation
        mj.mj_inverse(self.model, self.data)
        tau += self.data.qfrc_inverse

        return tau

    def run(self, controller_func, duration=10.0):
        """
        Run simulation with custom controller.

        Args:
            controller_func: Function(sim, time) -> desired_q
            duration: Simulation time in seconds
        """
        with viewer.launch_passive(self.model, self.data) as view:
            start_time = self.data.time

            while self.data.time - start_time < duration:
                # Get desired state from controller
                q_desired = controller_func(self, self.data.time)

                # Compute control
                self.data.ctrl[:] = self.pd_control(q_desired)

                # Step physics (500 Hz)
                mj.mj_step(self.model, self.data)

                # Render at 60 FPS
                if self.render and view.is_running():
                    view.sync()

# Example controller: slowly raise arms
def arm_raising_controller(sim, t):
    q_desired = np.zeros(sim.model.nu)

    # Raise shoulder joints over 5 seconds
    angle = min(np.pi/2, t * (np.pi/2) / 5.0)
    q_desired[0] = angle  # Left shoulder
    q_desired[3] = angle  # Right shoulder

    return q_desired

# Run
sim = MuJoCoSimulator("humanoid.xml")
sim.run(arm_raising_controller, duration=10.0)
```

**AI Contextual Debugger Example**:
> **Student**: "My humanoid falls immediately when simulation starts, even with gravity compensation."
>
> **AI Diagnosis**: "Gravity compensation assumes static equilibrium, but your robot starts with zero velocity and slightly off-balance. Two issues: (1) Initial configuration may violate ZMP stability, (2) Gravity compensation doesn't account for contact forces. Solutions: Initialize in stable configuration (e.g., wide stance), add ankle PD control to maintain ZMP in support polygon, or increase contact damping (`solref` parameter)."

#### Example 2: Isaac Sim GPU-Accelerated Simulation

Isaac Sim enables **massively parallel** simulation (1000+ robots) for reinforcement learning:

```python
from omni.isaac.kit import SimulationApp
import numpy as np

# Must create simulation app first
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.types import ArticulationAction

# Create world
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# Load humanoid robot
robot = world.scene.add(
    Robot(prim_path="/World/Humanoid", name="humanoid",
          usd_path="assets/humanoid.usd")
)

# Reset to initial state
world.reset()

# Simulation loop
for i in range(1000):
    # Get current state
    joint_positions = robot.get_joint_positions()
    joint_velocities = robot.get_joint_velocities()

    # Simple PD control (same as MuJoCo example)
    q_desired = np.zeros_like(joint_positions)
    tau = kp * (q_desired - joint_positions) - kd * joint_velocities

    # Apply control
    robot.apply_action(ArticulationAction(joint_efforts=tau))

    # Step physics
    world.step(render=True)

simulation_app.close()
```

> **🤖 AI System Analyzer**:
> "Compare performance: MuJoCo on CPU simulates ~2000 steps/second for 30-DOF humanoid. Isaac Sim on RTX 3090 GPU simulates 100,000 steps/second with 1000 parallel robots (100 steps/second per robot). For single-robot testing, MuJoCo is faster. For large-scale RL training (PPO needs millions of samples), Isaac Sim is 50-100× faster due to parallelization."

#### Example 3: Cross-Simulator Validation

**Critical Practice**: Validate your FK implementation by comparing against multiple simulators:

```python
def validate_fk_cross_simulator(urdf_path, joint_angles):
    """
    Verify FK implementation matches MuJoCo and PyBullet.

    Args:
        urdf_path: Robot URDF file
        joint_angles: Test configuration

    Returns:
        comparison: Dict with positions from each simulator
    """
    import pybullet as pb
    import mujoco as mj

    results = {}

    # 1. Your FK implementation
    results['custom'] = your_forward_kinematics(urdf_path, joint_angles)

    # 2. MuJoCo FK
    model_mj = mj.MjModel.from_xml_path(urdf_path)
    data_mj = mj.MjData(model_mj)
    data_mj.qpos[:] = joint_angles
    mj.mj_forward(model_mj, data_mj)
    results['mujoco'] = data_mj.xpos[-1]  # End-effector position

    # 3. PyBullet FK
    pb.connect(pb.DIRECT)
    robot_id = pb.loadURDF(urdf_path)
    for i, angle in enumerate(joint_angles):
        pb.resetJointState(robot_id, i, angle)
    link_state = pb.getLinkState(robot_id, pb.getNumJoints(robot_id)-1)
    results['pybullet'] = np.array(link_state[0])
    pb.disconnect()

    # Compare
    for key1 in results:
        for key2 in results:
            if key1 < key2:
                error = np.linalg.norm(results[key1] - results[key2])
                print(f"{key1} vs {key2}: {error*1000:.3f} mm")
                assert error < 0.001, f"FK mismatch > 1mm!"

    return results
```

---

### Part 4: SDD-RI Challenge (AI Generator + Grader)

**Specification**:

```
SPECIFICATION: Dual-Simulator Wrapper with Consistency Validation

INPUTS:
  - robot_urdf: Path to URDF robot description
  - control_sequence: List of (time, joint_positions) tuples
  - simulator: 'mujoco' | 'isaac_sim' | 'pybullet'

OUTPUTS:
  - trajectory: Dict with {
      'time': array of timestamps,
      'joint_positions': Nx7 array of joint angles over time,
      'end_effector_pos': Nx3 array of FK results,
      'contact_forces': Nx6 array (if in contact)
    }
  - validation_report: Dict with {
      'position_error_max': float (max FK difference between simulators),
      'position_error_mean': float,
      'consistency_check': bool (pass if error < 1mm)
    }

CONSTRAINTS:
  - Unified API: Same interface for all three simulators
  - Deterministic: Same random seed must produce identical results
  - Real-time capable: Run at ≥ 200 Hz for control loop

SUCCESS CRITERIA:
  - Position error between MuJoCo and PyBullet: < 1mm for static poses
  - Position error between MuJoCo and Isaac Sim: < 1mm for static poses
  - Orientation error: < 0.01 radians (quaternion distance)
  - API abstraction: Switch simulators with single parameter change

TEST CASES:
  1. Static pose: Set joint angles, verify FK matches across all simulators
  2. Gravity drop: Release robot from 0.5m height, compare contact forces
  3. Trajectory tracking: Execute sine wave joint motion, compare end-effector paths
```

> **🤖 AI Generator Prompt**:
> "Implement unified wrapper with abstract base class. Generate test URDFs (2-DOF, 7-DOF). Create visualization comparing trajectories side-by-side for all three simulators."

**Grading Criteria**:
- **Spec Alignment (60%)**:
  - FK consistency < 1mm across simulators (25%)
  - Deterministic behavior (same seed = same result) (15%)
  - Handles all three simulators (20%)
- **Code Quality (40%)**:
  - Clean abstraction design (no simulator-specific code in main logic) (20%)
  - Error handling (missing URDF files, unsupported simulators) (10%)
  - Performance (meets 200 Hz requirement) (10%)

---

### Part 5: Key Takeaways

1. **Simulators trade accuracy for speed**: MuJoCo prioritizes physical accuracy (convex optimization), Isaac Sim prioritizes parallelization (1000+ robots), PyBullet prioritizes ease of use. Choose based on your task—validation vs. RL training vs. prototyping.

2. **Cross-simulator validation catches implementation bugs**: Your FK code may work in isolation but fail when compared against MuJoCo's reference. Always validate with multiple ground truths.

3. **Contact parameters dramatically affect behavior**: Small changes to `solref`/`solimp` (MuJoCo) or friction coefficients can make the difference between stable walking and immediate collapse. Tune systematically and document settings.

**Looking Ahead**: Lesson 7 addresses the **sim-to-real gap**—why policies that work perfectly in simulation often fail on real robots, and how domain randomization bridges this gap.

---

### Part 6: Learn with AI

**Understand It Better**:
> "Explain MuJoCo's contact optimization using a simple 2D example of a box landing on a table. What is the optimization objective and constraints?"

**Get Feedback on Your Code**:
> "Review my simulator wrapper class and suggest improvements for handling state synchronization when switching between MuJoCo and Isaac Sim:
> [paste your code here]"

**Go Deeper**:
> "What are the fundamental limitations of physics simulators for humanoid robotics? Which physical phenomena are hardest to model accurately?"

**See It in Action**:
> "Show me a minimal working example of training a simple reaching policy in Isaac Sim with GPU parallelization (10+ robots). Keep it under 50 lines."

---

## Lesson 7: Sim-to-Real Transfer and Domain Randomization

**Pedagogical Layer**: 2 (AI Collaboration - CORE SKILL)
**Estimated Time**: 2.5 hours

---

### Part 1: The Hook (with Diagnostic Pre-Assessment)

Your humanoid walks flawlessly in MuJoCo—100% success rate over 1000 trials. You deploy the same control policy to real hardware and it falls after 3 steps. What went wrong?

This is the **sim-to-real gap**: the difference between simulated and physical reality. Real motors have backlash, sensors have noise, friction varies with temperature, and physics engines make approximations. Bridging this gap is critical for deploying learned behaviors to real robots.

> **🤖 AI Pre-Assessment**:
> "If you train a walking policy on a simulator with perfect friction coefficient μ=1.0, what happens when the real robot walks on a surface with μ=0.7 (slippery floor)? Why does the policy fail?"
>
> AI identifies gaps in your understanding of parameter sensitivity and stochastic systems.

**Learning Objective**: Implement domain randomization for physics parameters, measure policy robustness to parameter variations, design randomization distributions based on real hardware specifications, and validate sim-to-sim transfer before hardware deployment.

---

### Part 2: The Concept (Theory + AI Tutor)

#### Visual Intuition: Training in a Noisy World

Imagine learning to drive only in perfect weather with a perfect car. The first time you encounter rain or a worn tire, you crash. Now imagine training in **randomized conditions**—rain, snow, fog, different cars, worn tires. You become robust to variations.

Domain randomization applies this principle to robotics: **randomize simulator parameters during training** so the learned policy works despite model inaccuracies.

> **🎯 Pattern**: If training distribution covers real-world variation, learned policy generalizes to reality.

#### The Sim-to-Real Gap: Sources of Mismatch

| Category | Simulation Assumption | Reality | Impact |
|----------|----------------------|---------|---------|
| **Physics** | Perfect friction cone | Friction varies with contact, temperature | Foot slips unexpectedly |
| **Actuation** | Instant torque response | Motor bandwidth ~50 Hz, backlash ~0.5° | Control lag causes instability |
| **Sensing** | Noise-free measurements | Joint encoders: ±0.01 rad noise, IMU drift | State estimation errors accumulate |
| **Computation** | Zero latency | Control loop: 5-50 ms delay | Actions outdated by the time they execute |

#### Domain Randomization: Mathematical Framework

**Markov Decision Process with Parameter Uncertainty**:

Standard MDP: States s ∈ S, actions a ∈ A, dynamics P(s'|s,a)

**Randomized MDP**: Dynamics P(s'|s,a,ξ) where ξ ~ Distribution(θ)

- ξ: Randomized parameters (mass, friction, sensor noise)
- θ: Distribution parameters (mean, variance)

**Training objective**: Learn policy π that maximizes expected return across parameter distribution:

```
π* = argmax E_ξ[E_τ[Σ r(s,a)]]
```

> **💡 AI Deep-Dive Prompt**:
> "Explain the mathematical justification for domain randomization using the Mehta et al. (2020) framework. Why does randomizing 20% mass variation improve real-world transfer?"

#### What to Randomize: The Critical Parameters

**Physical parameters** (affect dynamics):
- Link masses: ±10-20%
- Link lengths: ±1-5% (manufacturing tolerance)
- Joint friction: 0.1-2.0 N·m (dry vs. lubricated)
- Ground friction: 0.3-1.2 (ice to rubber)

**Actuation parameters** (affect control):
- Motor strength: ±15% (battery voltage variation)
- Control delay: 0-50 ms
- Motor bandwidth: 30-100 Hz

**Sensing parameters** (affect observation):
- Joint encoder noise: ±0.005-0.02 rad
- IMU noise: ±0.01 rad/s (gyro), ±0.1 m/s² (accel)
- Force sensor noise: ±0.5-2.0 N

**Don't over-randomize**: Too much variation makes the task impossible. Start with ±10% and increase gradually.

---

### Part 3: AI-Collaborative Walkthrough

#### Example 1: Basic Domain Randomization in MuJoCo (Student → AI Refinement)

**Student Baseline Implementation**:

```python
import mujoco as mj
import numpy as np

def randomize_physics(model, rng):
    """
    Randomize physics parameters for domain randomization.

    Args:
        model: MuJoCo model
        rng: Numpy random generator
    """
    # Randomize body masses (±20%)
    for i in range(model.nbody):
        original_mass = model.body_mass[i]
        model.body_mass[i] = original_mass * rng.uniform(0.8, 1.2)

    # Randomize friction (±30%)
    for i in range(model.ngeom):
        original_friction = model.geom_friction[i, 0]
        model.geom_friction[i, 0] = original_friction * rng.uniform(0.7, 1.3)

    # Recompile model to update derived quantities
    mj.mj_setConst(model, model.createData())

# Training loop with randomization
rng = np.random.default_rng(seed=42)

for episode in range(1000):
    # Load fresh model (reset to original parameters)
    model = mj.MjModel.from_xml_path("humanoid.xml")
    data = mj.MjData(model)

    # Randomize for this episode
    randomize_physics(model, rng)

    # Run episode with randomized physics
    # ... (your training code here)
```

> **🤖 AI Code Refiner Prompt**:
> "Extend this to add correlated randomization—if mass increases by 20%, motor strength should also increase proportionally (heavier robot needs stronger motors). Also add sensor noise and control delays."

**AI-Refined Implementation**:

```python
import mujoco as mj
import numpy as np
from collections import deque

class DomainRandomizer:
    def __init__(self, base_model_path, randomization_ranges):
        """
        Args:
            base_model_path: Path to base URDF/XML
            randomization_ranges: Dict with parameter ranges, e.g.,
                {
                    'mass': (0.8, 1.2),        # ±20%
                    'friction': (0.5, 1.5),    # ±50%
                    'motor_strength': (0.85, 1.15),  # Correlated with mass
                    'control_delay_ms': (0, 50),
                    'joint_noise_std': 0.01,   # radians
                }
        """
        self.base_model_path = base_model_path
        self.ranges = randomization_ranges
        self.current_params = {}

    def randomize(self, rng):
        """Generate random parameters and apply to model."""
        model = mj.MjModel.from_xml_path(self.base_model_path)

        # Sample mass multiplier
        mass_mult = rng.uniform(*self.ranges['mass'])
        self.current_params['mass_mult'] = mass_mult

        # Apply to all bodies
        for i in range(model.nbody):
            model.body_mass[i] *= mass_mult

        # Friction (independent)
        friction_mult = rng.uniform(*self.ranges['friction'])
        for i in range(model.ngeom):
            model.geom_friction[i, 0] *= friction_mult

        # Motor strength (correlated with mass)
        motor_mult = mass_mult * rng.uniform(*self.ranges['motor_strength'])
        model.actuator_gainprm[:, 0] *= motor_mult

        # Control delay (will be applied in simulation loop)
        self.current_params['delay_ms'] = rng.uniform(*self.ranges['control_delay_ms'])

        # Sensor noise (will be applied to observations)
        self.current_params['joint_noise_std'] = self.ranges['joint_noise_std']

        mj.mj_setConst(model, model.createData())
        return model

    def apply_sensor_noise(self, observation, rng):
        """Add noise to sensor readings."""
        noisy_obs = observation.copy()

        # Joint position noise
        n_joints = len(observation) // 2  # Assume obs = [q, qd]
        noise = rng.normal(0, self.current_params['joint_noise_std'], n_joints)
        noisy_obs[:n_joints] += noise

        return noisy_obs

    def create_action_buffer(self):
        """Create buffer for control delay simulation."""
        max_delay_steps = int(self.current_params['delay_ms'] / 2)  # Assuming 2ms timestep
        return deque(maxlen=max_delay_steps+1)

# Usage in training
randomizer = DomainRandomizer("humanoid.xml", {
    'mass': (0.8, 1.2),
    'friction': (0.5, 1.5),
    'motor_strength': (0.9, 1.1),
    'control_delay_ms': (0, 30),
    'joint_noise_std': 0.01,
})

rng = np.random.default_rng(42)

for episode in range(1000):
    model = randomizer.randomize(rng)
    data = mj.MjData(model)
    action_buffer = randomizer.create_action_buffer()

    for step in range(1000):
        # Get noisy observation
        obs = get_observation(data)
        obs_noisy = randomizer.apply_sensor_noise(obs, rng)

        # Policy selects action
        action = policy(obs_noisy)

        # Apply delayed action
        action_buffer.append(action)
        delayed_action = action_buffer[0] if len(action_buffer) > 0 else action

        data.ctrl[:] = delayed_action
        mj.mj_step(model, data)
```

**AI Contextual Debugger Example**:
> **Student**: "My policy works with randomization in training but still fails on real robot. I randomized mass ±20% and friction ±30%."
>
> **AI Diagnosis**: "You likely didn't randomize **sensor noise and delays**. Real robots have significant state estimation errors (joint encoders drift, IMU has bias). Your policy never saw noisy observations during training, so it over-relies on perfect state information. Add: (1) Joint encoder noise ±0.01 rad, (2) IMU gyro noise ±0.02 rad/s, (3) 10-30ms control delay. Also, measure your real robot's parameter ranges—±20% mass might be too conservative (batteries add 30% weight when full vs. empty)."

#### Example 2: Measuring Policy Robustness

**Systematic Evaluation**: Test policy sensitivity to each parameter independently:

```python
def evaluate_robustness(policy, model_path, parameter_name, test_range, n_trials=50):
    """
    Measure policy success rate vs. parameter variation.

    Args:
        policy: Trained policy
        model_path: Base model
        parameter_name: 'mass' | 'friction' | 'motor_strength'
        test_range: (min, max) multipliers to test
        n_trials: Episodes per parameter value

    Returns:
        results: Dict with {param_value: success_rate}
    """
    results = {}

    test_values = np.linspace(test_range[0], test_range[1], 20)

    for mult in test_values:
        successes = 0

        for trial in range(n_trials):
            model = mj.MjModel.from_xml_path(model_path)

            # Apply parameter variation
            if parameter_name == 'mass':
                model.body_mass[:] *= mult
            elif parameter_name == 'friction':
                model.geom_friction[:, 0] *= mult
            elif parameter_name == 'motor_strength':
                model.actuator_gainprm[:, 0] *= mult

            mj.mj_setConst(model, model.createData())
            data = mj.MjData(model)

            # Run episode
            success = run_episode(policy, model, data)
            successes += int(success)

        results[mult] = successes / n_trials

    return results

# Example: Test sensitivity to mass
mass_robustness = evaluate_robustness(
    trained_policy, "humanoid.xml", "mass", (0.5, 1.5), n_trials=50
)

# Plot results
import matplotlib.pyplot as plt
plt.plot(list(mass_robustness.keys()), list(mass_robustness.values()))
plt.xlabel("Mass multiplier")
plt.ylabel("Success rate")
plt.title("Policy robustness to mass variation")
plt.axhline(y=0.9, color='r', linestyle='--', label='90% threshold')
plt.axvline(x=1.0, color='g', linestyle='--', label='Nominal')
plt.legend()
plt.show()
```

> **🤖 AI System Analyzer**:
> "Compare training strategies: (1) Uniform randomization (all parameters varied equally): Simple but inefficient—wastes samples on irrelevant variations. (2) Curriculum randomization (start small, increase gradually): Faster learning but requires tuning schedule. (3) Adversarial domain randomization (train adversary to find worst-case parameters): Most robust but computationally expensive. For humanoid walking, use curriculum: start ±5%, increase to ±20% over 1M steps."

#### Example 3: Sim-to-Sim Transfer Validation

**Before deploying to hardware**, validate with high-fidelity simulation:

```python
def sim_to_sim_transfer(policy, source_sim='mujoco', target_sim='isaac_sim'):
    """
    Test policy trained in source simulator on target simulator.

    This catches simulator-specific biases before hardware deployment.
    """
    # Train in source
    policy = train_policy(simulator=source_sim, domain_randomization=True)

    # Test in target (no randomization)
    source_performance = evaluate(policy, simulator=source_sim, randomize=False)
    target_performance = evaluate(policy, simulator=target_sim, randomize=False)

    transfer_gap = source_performance - target_performance

    print(f"Source ({source_sim}): {source_performance:.1%}")
    print(f"Target ({target_sim}): {target_performance:.1%}")
    print(f"Transfer gap: {transfer_gap:.1%}")

    if transfer_gap > 0.25:  # >25% performance drop
        print("⚠️ Warning: Large sim-to-sim gap. Likely to fail on hardware.")
        print("Recommendation: Increase domain randomization ranges.")

    return transfer_gap < 0.15  # <15% gap is acceptable
```

---

### Part 4: SDD-RI Challenge (AI Generator + Grader)

**Specification**:

```
SPECIFICATION: Domain Randomization Framework for Humanoid Reaching

INPUTS:
  - base_model: URDF/XML of humanoid robot
  - randomization_config: Dict specifying parameter distributions
  - baseline_policy: Pre-trained reaching policy (without randomization)

OUTPUTS:
  - randomized_policy: Policy trained with domain randomization
  - robustness_report: Dict with {
      'mass_sensitivity': {param_value: success_rate},
      'friction_sensitivity': {...},
      'noise_sensitivity': {...},
      'transfer_performance': {
          'mujoco_to_pybullet': success_rate,
          'mujoco_to_isaac': success_rate
      }
    }

CONSTRAINTS:
  - Randomize at least 10 parameters per episode
  - Correlate mass with motor strength (heavier = stronger motors)
  - Sensor noise and control delays mandatory
  - Validate on high-fidelity sim before claiming success

SUCCESS CRITERIA:
  - Transfer success: >75% success rate when tested in different simulator
  - Robustness: >70% success across 0.7×–1.3× mass range
  - Performance gap: <25% drop from training sim to test sim
  - Validation: Sim-to-sim transfer gap <15% (MuJoCo → PyBullet)

TEST CASES:
  1. Nominal parameters: Should achieve 95%+ success (sanity check)
  2. Extreme mass (+50%): Should achieve >50% success
  3. Slippery floor (friction ×0.5): Should achieve >60% success
  4. High sensor noise (3× nominal): Should achieve >65% success
```

> **🤖 AI Generator Prompt**:
> "Implement domain randomization framework. Generate test scenarios covering parameter combinations. Create robustness plots (success rate vs. parameter value) for all critical parameters."

**Grading Criteria**:
- **Spec Alignment (60%)**:
  - Transfer success >75% (25%)
  - Robustness across parameter ranges (20%)
  - Sim-to-sim gap <15% (15%)
- **Code Quality (40%)**:
  - Randomization implementation (correlated parameters) (15%)
  - Validation framework (systematic testing) (15%)
  - Visualization (robustness plots) (10%)

---

### Part 5: Key Takeaways

1. **Domain randomization is essential for sim-to-real transfer**: Policies trained on perfect physics fail immediately on real hardware. Randomize physical parameters, sensor noise, and control delays during training to build robustness.

2. **Measure real hardware to set randomization ranges**: Don't guess ±20% mass variation—measure your robot's actual parameter variability (battery weight, joint friction after 100 hours of use). Under-randomization leads to transfer failure, over-randomization makes learning impossible.

3. **Validate with sim-to-sim transfer before hardware deployment**: If your policy fails when moving from MuJoCo to PyBullet (both simulators!), it will definitely fail on real hardware. Sim-to-sim transfer is a cheap smoke test.

**Looking Ahead**: Lesson 8 shifts focus from using tools to **building reusable components**—packaging your FK, IK, dynamics, and simulation code into well-tested, documented modules that can be composed into complete systems.

---

### Part 6: Learn with AI

**Understand It Better**:
> "Explain domain randomization using the Universal Value Function Approximators (UVFA) framework. How does randomizing parameters create a distribution of MDPs?"

**Get Feedback on Your Code**:
> "Review my domain randomization implementation and suggest which additional parameters I should randomize for bipedal walking:
> [paste your code here]"

**Go Deeper**:
> "What are the limitations of domain randomization? When does it fail to bridge the sim-to-real gap, and what complementary techniques exist (system identification, residual learning)?"

**See It in Action**:
> "Show me a minimal example of using domain randomization in Isaac Gym for training a humanoid stand-up policy with 512 parallel environments."

---

## Lesson 8: Intelligence Design - Building Reusable Kinematics & Dynamics Components

**Pedagogical Layer**: 3 (Intelligence Design)
**Estimated Time**: 3 hours

---

### Part 1: The Hook (with Diagnostic Pre-Assessment)

You've implemented FK, IK, dynamics, contact analysis, and simulation wrappers over the past seven lessons. But your code is scattered across notebooks, hard-coded for specific robots, and full of magic numbers. How do you turn this experimental code into **production-quality components** that others (or future you) can actually use?

This lesson is about **software engineering for robotics**: designing clean APIs, writing specifications before code, creating comprehensive tests, and building reusable components that compose into larger systems.

> **🤖 AI Pre-Assessment**:
> "How would you version a kinematics library used by 5 downstream projects? If you discover a bug that requires API changes, what's your strategy to avoid breaking dependent code?"
>
> AI personalizes based on software engineering background.

**Learning Objective**: Design and implement the "HumanoidKinematicsKit" component suite with formal specifications, comprehensive unit tests (>90% coverage), API documentation, and versioning strategy—ready for integration into larger systems.

---

### Part 2: The Concept (Theory + AI Tutor)

#### Visual Intuition: Components as LEGO Blocks

Well-designed software components are like LEGO bricks: they have **clear interfaces** (studs and holes), **guaranteed properties** (plastic doesn't bend), and **compose predictably** (two bricks always connect the same way).

Poor components are like clay lumps: they can be shaped into anything but don't fit together cleanly.

**Component Design Principles**:

| Principle | Description | Example (Bad → Good) |
|-----------|-------------|---------------------|
| **Single Responsibility** | One component, one job | ❌ `RobotUtils.do_everything()` → ✅ `ForwardKinematics.compute()` |
| **Explicit Contracts** | Inputs, outputs, errors documented | ❌ Undocumented function → ✅ Type hints + docstring with examples |
| **Fail-Fast Validation** | Check inputs immediately | ❌ Crash 100 lines later → ✅ `assert len(q) == n_joints` at entry |
| **Spec-Driven Design** | Write spec BEFORE code | ❌ Code then document → ✅ Spec defines success, code implements |

> **🎯 Pattern**: For reusable components, specification comes first, implementation second, tests third (validates spec compliance).

#### API Design for Robotics Components

**Good API characteristics**:

1. **Type safety**: Use type hints (Python), avoid magic strings
2. **Sensible defaults**: Common use cases work with minimal config
3. **Explicit errors**: Raise `ReachabilityError`, not generic `ValueError`
4. **Stateless when possible**: Pure functions easier to test and reason about
5. **Framework-agnostic**: Work with NumPy arrays, not MuJoCo/Isaac-specific types

**Example API comparison**:

```python
# ❌ BAD: Unclear contract, stateful, framework-dependent
def ik(robot, target):
    """Solve inverse kinematics."""
    # What's the return value? What if it fails? What units?
    return solver.solve(robot.get_model(), target)

# ✅ GOOD: Explicit contract, clear failure modes, framework-agnostic
from typing import Optional, Tuple
import numpy as np

def inverse_kinematics(
    target_position: np.ndarray,  # (3,) array, meters in world frame
    target_orientation: Optional[np.ndarray] = None,  # (4,) quaternion [w,x,y,z], or None for position-only
    initial_guess: np.ndarray,  # (n_joints,) array, radians
    joint_limits: Tuple[np.ndarray, np.ndarray],  # (q_min, q_max)
    timeout: float = 1.0  # seconds
) -> Tuple[np.ndarray, bool, dict]:
    """
    Solve inverse kinematics for target pose.

    Returns:
        solution: (n_joints,) joint angles (radians), or initial_guess if failed
        success: True if converged within tolerances
        info: {'iterations': int, 'final_error': float, 'method': str}

    Raises:
        ValueError: If target_position shape != (3,) or target_orientation shape != (4,)
        ReachabilityError: If target is outside workspace (||target|| > max_reach)
    """
```

> **💡 AI Deep-Dive Prompt**:
> "Compare API design patterns for robotics: ROS2 (topic/service architecture), Drake (systems framework), and pure Python libraries. What are trade-offs between flexibility and ease of use?"

#### Testing Strategies: Property-Based vs. Example-Based

**Example-based testing** (traditional):
```python
def test_forward_kinematics():
    q = np.array([0, 0, 0, np.pi/2, 0, 0, 0])  # Specific configuration
    pos = fk(q)
    assert np.allclose(pos, [0.3, 0, 0.8])  # Expected position
```

**Property-based testing** (more general):
```python
from hypothesis import given, strategies as st

@given(st.lists(st.floats(-np.pi, np.pi), min_size=7, max_size=7))
def test_fk_properties(joint_angles):
    """FK must satisfy properties for ANY input."""
    q = np.array(joint_angles)
    pos = fk(q)

    # Property 1: Output is always 3D position
    assert pos.shape == (3,)

    # Property 2: Position within workspace (sum of link lengths)
    assert np.linalg.norm(pos) <= MAX_REACH

    # Property 3: FK is deterministic
    assert np.allclose(fk(q), fk(q))
```

**Property-based testing catches edge cases** that manual examples miss (e.g., joint angles at exactly ±π).

---

### Part 3: AI-Collaborative Walkthrough

#### Example 1: Spec-Driven Component Design (AI Co-Designer)

**Step 1: Write Specification First**

> **🤖 AI Spec Co-Creation Prompt**:
> "Help me write a formal specification for a ForwardKinematics component. It should handle arbitrary kinematic trees, cache intermediate results, and validate against physics simulators."

**Student + AI Generated Specification**:

```markdown
# Component Specification: ForwardKinematics v1.0

## Purpose
Compute end-effector positions and orientations for arbitrary kinematic trees given joint configurations.

## Inputs
- `joint_angles`: np.ndarray shape (n_joints,), radians, range [-π, π]
- `robot_model`: RobotModel dataclass with {dh_params, joint_limits, link_names}
- `end_effector_name`: str, name of target link (must exist in robot_model)

## Outputs
- `position`: np.ndarray shape (3,), meters in world frame
- `orientation`: np.ndarray shape (4,), quaternion [w, x, y, z] (unit norm)
- `intermediate_transforms`: Dict[str, np.ndarray], 4×4 transforms for all links

## Constraints
- Must handle kinematic trees (not just serial chains)
- Must cache intermediate transforms (for efficiency in Jacobian computation)
- Must validate joint_angles within joint_limits (raise ValueError if violated)
- Must match MuJoCo/PyBullet FK within 1e-6 meters

## Performance Requirements
- Compute time: <1 ms for 30-DOF humanoid (on i7-10700K CPU)
- Memory: O(n_joints) intermediate storage

## Error Conditions
| Condition | Exception | Message |
|-----------|-----------|---------|
| Unknown link name | LinkNotFoundError | "Link '{name}' not in robot model" |
| Joint angles out of limits | JointLimitError | "Joint {i} = {value} exceeds limit [{min}, {max}]" |
| Invalid joint angles shape | ValueError | "Expected {n} joint angles, got {m}" |

## Test Requirements
1. Example-based: Verify against hand-calculated 2-DOF arm
2. Property-based: FK(q) position always within max_reach
3. Cross-validation: Match MuJoCo FK within 1e-6 for 100 random configurations
4. Edge cases: All joints at limits (±π), all joints at zero
5. Performance: 1000 FK calls in <1 second for 7-DOF arm
```

**Step 2: Implement Against Spec**

```python
from dataclasses import dataclass
from typing import Dict, Tuple
import numpy as np

@dataclass
class RobotModel:
    """Robot kinematic model."""
    dh_params: list  # [(a, alpha, d, theta_offset), ...]
    joint_limits: Tuple[np.ndarray, np.ndarray]  # (q_min, q_max)
    link_names: list  # ["base", "shoulder", "elbow", ...]

class LinkNotFoundError(Exception):
    pass

class JointLimitError(Exception):
    pass

class ForwardKinematics:
    """Forward kinematics computation with caching."""

    def __init__(self, robot_model: RobotModel):
        self.model = robot_model
        self.n_joints = len(robot_model.dh_params)
        self._cached_transforms = {}

    def compute(
        self,
        joint_angles: np.ndarray,
        end_effector_name: str
    ) -> Tuple[np.ndarray, np.ndarray, Dict[str, np.ndarray]]:
        """
        Compute forward kinematics.

        Implements specification: ForwardKinematics v1.0

        Returns:
            position: (3,) end-effector position
            orientation: (4,) quaternion [w,x,y,z]
            intermediate_transforms: All link transforms
        """
        # Validate inputs (fail-fast)
        self._validate_inputs(joint_angles, end_effector_name)

        # Compute all transforms with caching
        transforms = self._compute_transforms(joint_angles)

        # Extract end-effector pose
        ee_transform = transforms[end_effector_name]
        position = ee_transform[:3, 3]
        orientation = self._matrix_to_quaternion(ee_transform[:3, :3])

        return position, orientation, transforms

    def _validate_inputs(self, joint_angles, end_effector_name):
        """Validate inputs against spec constraints."""
        # Check shape
        if joint_angles.shape != (self.n_joints,):
            raise ValueError(
                f"Expected {self.n_joints} joint angles, got {len(joint_angles)}"
            )

        # Check limits
        q_min, q_max = self.model.joint_limits
        for i, q in enumerate(joint_angles):
            if q < q_min[i] or q > q_max[i]:
                raise JointLimitError(
                    f"Joint {i} = {q:.3f} exceeds limit [{q_min[i]:.3f}, {q_max[i]:.3f}]"
                )

        # Check link exists
        if end_effector_name not in self.model.link_names:
            raise LinkNotFoundError(
                f"Link '{end_effector_name}' not in robot model"
            )

    def _compute_transforms(self, joint_angles):
        """Compute all link transforms (with caching)."""
        # Implementation details...
        # (Same as Lesson 2, but with caching and tree support)
        pass

    def _matrix_to_quaternion(self, R):
        """Convert rotation matrix to unit quaternion."""
        # Numerically stable conversion (Shepperd's method)
        pass
```

**Step 3: Write Tests Against Spec**

```python
import pytest
from hypothesis import given, strategies as st

class TestForwardKinematics:
    """Test suite validating ForwardKinematics v1.0 spec."""

    @pytest.fixture
    def simple_arm(self):
        """2-DOF planar arm for testing."""
        return RobotModel(
            dh_params=[(0.3, 0, 0, 0), (0.25, 0, 0, 0)],
            joint_limits=(np.array([-np.pi, -np.pi]), np.array([np.pi, np.pi])),
            link_names=["base", "link1", "link2"]
        )

    def test_known_configuration(self, simple_arm):
        """Example-based test: verify hand-calculated result."""
        fk = ForwardKinematics(simple_arm)
        pos, quat, _ = fk.compute(np.array([np.pi/2, 0]), "link2")

        # At θ1=90°, θ2=0: end-effector at (0, 0.55, 0)
        assert np.allclose(pos, [0, 0.55, 0], atol=1e-6)

    @given(st.lists(st.floats(-np.pi, np.pi), min_size=2, max_size=2))
    def test_workspace_constraint(self, joint_angles, simple_arm):
        """Property test: FK result always within reach."""
        fk = ForwardKinematics(simple_arm)
        q = np.array(joint_angles)

        # Clip to joint limits
        q = np.clip(q, -np.pi, np.pi)

        pos, _, _ = fk.compute(q, "link2")

        # Max reach = 0.3 + 0.25 = 0.55m
        assert np.linalg.norm(pos) <= 0.56  # Small tolerance

    def test_joint_limit_validation(self, simple_arm):
        """Spec requirement: must raise JointLimitError for out-of-bounds."""
        fk = ForwardKinematics(simple_arm)

        with pytest.raises(JointLimitError, match="Joint 0"):
            fk.compute(np.array([4.0, 0]), "link2")  # 4.0 > π

    def test_cross_validation_mujoco(self, simple_arm):
        """Spec requirement: match MuJoCo within 1e-6."""
        import mujoco as mj

        # Create equivalent MuJoCo model
        mj_model = create_mujoco_model(simple_arm)
        mj_data = mj.MjData(mj_model)

        fk = ForwardKinematics(simple_arm)

        # Test 100 random configurations
        for _ in range(100):
            q = np.random.uniform(-np.pi, np.pi, 2)

            # Our implementation
            pos_ours, _, _ = fk.compute(q, "link2")

            # MuJoCo reference
            mj_data.qpos[:] = q
            mj.mj_forward(mj_model, mj_data)
            pos_mujoco = mj_data.site_xpos[0]  # Assuming site at link2

            assert np.linalg.norm(pos_ours - pos_mujoco) < 1e-6

    def test_performance(self, simple_arm):
        """Spec requirement: 1000 calls in <1s for 7-DOF."""
        import time

        fk = ForwardKinematics(simple_arm)
        q = np.array([0.5, -0.3])

        start = time.time()
        for _ in range(1000):
            fk.compute(q, "link2")
        elapsed = time.time() - start

        assert elapsed < 1.0, f"FK too slow: {elapsed:.3f}s for 1000 calls"
```

**AI Architecture Review**:
> **🤖 AI Suggestion**:
> "Your ForwardKinematics class is well-designed but consider adding: (1) `batch_compute(joint_angles_batch)` for vectorized FK (10× faster for 100 configurations), (2) `get_link_transform(link_name)` to access cached intermediates without recomputing, (3) `clear_cache()` method if robot model changes. Also, document that caching assumes robot_model is immutable—if DH parameters change, user must create new FK instance."

---

### Part 4: SDD-RI Challenge (AI Generator + Grader)

**Specification**:

```
SPECIFICATION: HumanoidKinematicsKit Component Suite

DELIVERABLES:
1. Five components with formal specifications:
   - ForwardKinematics v1.0 (already designed above)
   - InverseKinematics v1.0 (Jacobian + optimization methods)
   - DynamicsEngine v1.0 (RNEA, mass matrix computation)
   - ContactAnalyzer v1.0 (ZMP, support polygon, stability)
   - SimulatorWrapper v1.0 (MuJoCo/PyBullet/Isaac Sim abstraction)

2. Comprehensive test suites for each component:
   - Unit tests (>90% code coverage)
   - Property-based tests (Hypothesis)
   - Cross-validation tests (against reference libraries)
   - Performance benchmarks

3. API documentation with examples:
   - Docstrings with type hints
   - Usage examples (Jupyter notebook)
   - Design rationale document

4. Versioning strategy:
   - Semantic versioning (v1.0.0)
   - Changelog documenting breaking changes
   - Deprecation policy (features marked @deprecated for 2 versions before removal)

SUCCESS CRITERIA:
- All components pass 100% of spec-defined tests
- Test coverage >90% (measured by pytest-cov)
- API documentation complete (all public methods documented)
- Cross-validation: <1e-6 error vs. Pinocchio (dynamics), MuJoCo (FK)
- Performance: Meet or exceed spec requirements
- Integration test: Compose all 5 components into "ReachingPlanner" subagent

TEST CASES:
1. Component isolation: Each component works independently
2. Component composition: FK → IK → Dynamics chain produces valid results
3. Failure propagation: Invalid input to FK raises appropriate error (not crash)
4. Version compatibility: v1.0 components compose with each other
```

> **🤖 AI Generator Prompt**:
> "Generate complete test suites for all 5 components. Create integration test that uses all components together. Suggest API improvements based on composability analysis."

**Grading Criteria**:
- **Spec Alignment (60%)**:
  - All tests pass (30%)
  - >90% test coverage (15%)
  - Cross-validation within tolerances (15%)
- **Code Quality (40%)**:
  - API design (clear contracts, type hints) (15%)
  - Documentation (docstrings, examples) (15%)
  - Design patterns (SOLID principles) (10%)

---

### Part 5: Key Takeaways

1. **Specification-driven development prevents scope creep**: Write the spec (inputs, outputs, constraints, tests) BEFORE writing code. The spec defines "done"—implementation just satisfies it.

2. **Property-based testing finds edge cases you'd never imagine**: Testing "FK(q) distance ≤ max_reach for ALL q" catches bugs that example-based tests (5 hand-picked configurations) miss.

3. **Reusable components have clean contracts and fail-fast validation**: Check inputs immediately, raise specific exceptions (JointLimitError not ValueError), document all failure modes. Future you (or collaborators) will thank you.

**Looking Ahead**: Lesson 9 puts it all together—write a system-level specification for a humanoid reaching & balancing task, then let AI orchestrate your components to implement it.

---

### Part 6: Learn with AI

**Understand It Better**:
> "Explain the difference between unit tests, integration tests, and property-based tests using robotics examples. When should I use each?"

**Get Feedback on Your Code**:
> "Review my InverseKinematics component specification and suggest missing constraints or edge cases:
> [paste your spec here]"

**Go Deeper**:
> "What are the trade-offs between stateful vs. stateless component design for robotics? When should a ForwardKinematics class maintain internal state vs. be a pure function?"

**See It in Action**:
> "Show me a minimal example of using pytest-cov and Hypothesis together to achieve >95% coverage with property-based tests for a Jacobian computation function."

---

## Lesson 9: Spec-Driven Integration - Humanoid Reaching & Balancing System

**Pedagogical Layer**: 4 (Spec-Driven Integration)
**Estimated Time**: 3 hours

---

### Part 1: The Hook (with Diagnostic Pre-Assessment)

You've built 5 battle-tested components: FK, IK, Dynamics, Contact, and Simulation. Each passes 100+ tests. Now the real challenge: **compose them into a complete system** that makes a humanoid robot reach for objects while maintaining balance.

This isn't just calling functions in sequence—it's **system design**: defining requirements, orchestrating components, handling failures gracefully, and validating that the integrated system does what you actually need.

> **🤖 AI Pre-Assessment**:
> "Given components A (ForwardKinematics), B (InverseKinematics), and C (ContactAnalyzer), design a system that reaches for a cup while ensuring ZMP stays in the support polygon. What's the control flow? Which component calls which?"
>
> AI assesses systems thinking and integration skills.

**Learning Objective**: Write a formal system specification for humanoid reaching & balancing, orchestrate the 5 components from Lesson 8 to implement it (using AI as integration assistant), create spec-based integration tests, and deploy to MuJoCo simulation.

---

### Part 2: The Concept (Theory + AI Tutor)

#### Visual Intuition: System Integration as Orchestra Conducting

Each component is a skilled musician (FK plays position data, IK computes joint angles, Contact monitors stability). The **system specification** is the musical score—it defines what the orchestra plays, not how each instrument works.

Your job as system designer: write the score (spec), then orchestrate the musicians (components) to perform it.

> **🎯 Pattern**: Specification defines WHAT (requirements, success criteria). Components define HOW (implementation). Integration connects WHAT to HOW.

#### System-Level vs. Component-Level Specifications

| Level | Scope | Example |
|-------|-------|---------|
| **Component Spec** | Single module, internal contract | "ForwardKinematics: Given q, return (position, orientation) within 1ms" |
| **System Spec** | End-to-end behavior, external observable | "ReachingSystem: Given target [x,y,z], move hand to target while keeping ZMP in support polygon, success if distance <5cm" |

**System specs are:**
- **Observable**: Defined by measurable outputs (distance to target, stability margin)
- **Complete**: Cover all scenarios (success, failure, edge cases)
- **Testable**: Every requirement traced to a test case

#### AI as System Orchestrator

In Layer 4, **AI becomes your integration engineer**. You provide:
1. System specification (what to build)
2. Available components (building blocks)
3. Constraints (performance, safety)

AI generates:
1. Integration code (connects components)
2. Control flow logic
3. Error handling

**You review and validate** the AI-generated integration against your spec.

> **💡 AI Deep-Dive Prompt**:
> "Explain the difference between top-down (spec-first) and bottom-up (component-first) system design. What are the risks of bottom-up integration in robotics?"

---

### Part 3: AI-Collaborative Walkthrough

#### Example 1: Writing System Specification (Student → AI Collaboration)

**Student Initial Spec**:

```markdown
# System Specification: Humanoid Reaching & Balancing v1.0

## Purpose
Make humanoid robot reach for target objects while maintaining balance.

## Inputs
- target_position: [x, y, z] in world frame (meters)
- robot_state: Current joint positions, velocities

## Outputs
- success: Boolean (reached target or not)
```

> **🤖 AI Spec Co-Designer Prompt**:
> "This spec is incomplete. Help me add: (1) Success criteria (how close counts as 'reached'?), (2) Safety constraints (ZMP margin, joint limits), (3) Failure modes (unreachable target, instability), (4) Performance requirements (time to reach)."

**AI-Enhanced Specification**:

```markdown
# System Specification: Humanoid Reaching & Balancing v1.0

## Purpose
Execute reaching motions to target positions while maintaining static stability (ZMP within support polygon).

## Inputs
- `target_position`: np.ndarray (3,), meters in world frame
- `initial_state`: RobotState with {joint_positions, joint_velocities, base_pose}
- `constraints`: Dict with {
    'max_reach_time': float (seconds, default 5.0),
    'stability_margin': float (meters from polygon edge, default 0.03),
    'position_tolerance': float (meters, default 0.01)
  }

## Outputs
- `trajectory`: List[RobotState], planned motion from initial to goal
- `success`: bool, True if reached target within tolerance while stable
- `diagnostics`: Dict with {
    'final_distance': float (meters to target),
    'min_stability_margin': float (closest ZMP approach to edge),
    'computation_time': float (seconds),
    'failure_reason': Optional[str] ('unreachable' | 'unstable' | 'timeout' | None)
  }

## Requirements

### Functional Requirements
1. **FR-1**: Compute joint trajectory that moves end-effector to target_position
2. **FR-2**: Ensure ZMP remains inside support polygon with ≥stability_margin throughout motion
3. **FR-3**: Respect joint position limits (q_min ≤ q ≤ q_max)
4. **FR-4**: Respect joint velocity limits (||q̇|| ≤ q̇_max)

### Performance Requirements
1. **PR-1**: Total computation time ≤ 2 seconds (for 5-second reach)
2. **PR-2**: Trajectory execution time ≤ max_reach_time
3. **PR-3**: Final position error ≤ position_tolerance

### Safety Requirements
1. **SR-1**: Abort if target unreachable (||target|| > max_reach)
2. **SR-2**: Abort if ZMP exits support polygon (minus margin) at any trajectory point
3. **SR-3**: Graceful degradation: return partial trajectory if timeout

## Component Dependencies
- ForwardKinematics v1.0: Compute end-effector positions
- InverseKinematics v1.0: Solve for joint configurations
- DynamicsEngine v1.0: Compute ZMP from trajectory
- ContactAnalyzer v1.0: Check stability constraints
- SimulatorWrapper v1.0: Execute and validate trajectory

## Test Cases

### TC-1: Nominal Reaching
- Input: target = [0.4, 0.2, 0.9] (within reach, stable)
- Expected: success=True, final_distance < 0.01m, min_stability_margin > 0.03m

### TC-2: Unreachable Target
- Input: target = [2.0, 0, 1.0] (beyond max_reach)
- Expected: success=False, failure_reason='unreachable', computation_time < 0.1s (early abort)

### TC-3: Stability Violation
- Input: target = [0.6, 0.4, 0.5] (reachable but causes instability)
- Expected: success=False, failure_reason='unstable', return partial trajectory

### TC-4: Edge of Workspace
- Input: target at 95% of max_reach
- Expected: success=True, verify joint limits not violated

### TC-5: Performance Stress Test
- Input: 100 random targets (within reach)
- Expected: Average computation time < 1.0s, success rate > 90%
```

#### Example 2: AI Orchestration (Student Spec → AI Implementation)

**Student provides specification above, AI generates integration code**:

> **🤖 AI Orchestrator Prompt**:
> "Using the HumanoidKinematicsKit components (FK, IK, Dynamics, Contact, Simulator), implement the Reaching & Balancing v1.0 system specification. Generate the ReachingController class that orchestrates these components."

**AI-Generated Implementation** (reviewed by student):

```python
from typing import List, Dict, Optional, Tuple
import numpy as np
import time

class ReachingController:
    """
    Implements Humanoid Reaching & Balancing v1.0 specification.

    Orchestrates FK, IK, Dynamics, Contact, and Simulator components.
    """

    def __init__(
        self,
        robot_model: RobotModel,
        fk: ForwardKinematics,
        ik: InverseKinematics,
        dynamics: DynamicsEngine,
        contact: ContactAnalyzer,
        simulator: SimulatorWrapper
    ):
        self.robot_model = robot_model
        self.fk = fk
        self.ik = ik
        self.dynamics = dynamics
        self.contact = contact
        self.sim = simulator

        # Workspace limits from robot geometry
        self.max_reach = sum([link.length for link in robot_model.links])

    def reach(
        self,
        target_position: np.ndarray,
        initial_state: RobotState,
        constraints: Dict = None
    ) -> Tuple[List[RobotState], bool, Dict]:
        """
        Execute reaching motion per specification.

        Implements requirements: FR-1, FR-2, FR-3, FR-4, PR-1, PR-2, PR-3, SR-1, SR-2, SR-3

        Returns:
            trajectory: Planned motion
            success: True if all requirements met
            diagnostics: Performance and failure information
        """
        start_time = time.time()

        # Default constraints
        if constraints is None:
            constraints = {}
        max_reach_time = constraints.get('max_reach_time', 5.0)
        stability_margin = constraints.get('stability_margin', 0.03)
        position_tolerance = constraints.get('position_tolerance', 0.01)

        diagnostics = {
            'final_distance': None,
            'min_stability_margin': None,
            'computation_time': None,
            'failure_reason': None
        }

        # SR-1: Check reachability
        if np.linalg.norm(target_position) > self.max_reach:
            diagnostics['computation_time'] = time.time() - start_time
            diagnostics['failure_reason'] = 'unreachable'
            return [], False, diagnostics

        # FR-1: Solve IK for target configuration
        q_goal, ik_success, ik_info = self.ik.compute(
            target_position,
            initial_guess=initial_state.joint_positions,
            joint_limits=self.robot_model.joint_limits
        )

        if not ik_success:
            diagnostics['computation_time'] = time.time() - start_time
            diagnostics['failure_reason'] = 'ik_failed'
            return [], False, diagnostics

        # Generate trajectory (interpolate from current to goal)
        trajectory = self._plan_trajectory(
            initial_state.joint_positions,
            q_goal,
            duration=max_reach_time
        )

        # FR-2, SR-2: Validate stability along trajectory
        is_stable, min_margin = self._check_trajectory_stability(
            trajectory,
            stability_margin
        )

        if not is_stable:
            diagnostics['computation_time'] = time.time() - start_time
            diagnostics['min_stability_margin'] = min_margin
            diagnostics['failure_reason'] = 'unstable'
            # SR-3: Return partial trajectory up to instability
            stable_prefix = self._truncate_to_stable(trajectory, stability_margin)
            return stable_prefix, False, diagnostics

        # Verify final position (PR-3)
        final_pos, _, _ = self.fk.compute(q_goal, "end_effector")
        final_distance = np.linalg.norm(final_pos - target_position)

        diagnostics['final_distance'] = final_distance
        diagnostics['min_stability_margin'] = min_margin
        diagnostics['computation_time'] = time.time() - start_time

        # Check success criteria
        success = (
            final_distance <= position_tolerance and
            diagnostics['computation_time'] <= 2.0  # PR-1
        )

        if not success and final_distance > position_tolerance:
            diagnostics['failure_reason'] = 'position_error'

        return trajectory, success, diagnostics

    def _plan_trajectory(
        self,
        q_start: np.ndarray,
        q_goal: np.ndarray,
        duration: float,
        dt: float = 0.01
    ) -> List[RobotState]:
        """
        Generate smooth trajectory from start to goal.

        Uses minimum jerk trajectory (5th-order polynomial).
        Implements FR-3, FR-4 (joint limits, velocity limits).
        """
        n_steps = int(duration / dt)
        trajectory = []

        for i in range(n_steps + 1):
            t = i * dt
            s = self._minimum_jerk(t / duration)  # Smooth interpolation [0,1]

            q = q_start + s * (q_goal - q_start)
            qd = (q_goal - q_start) / duration * self._minimum_jerk_velocity(t / duration)

            # FR-3: Check joint limits
            q = np.clip(q, self.robot_model.joint_limits[0], self.robot_model.joint_limits[1])

            # FR-4: Check velocity limits
            qd = np.clip(qd, -self.robot_model.velocity_limits, self.robot_model.velocity_limits)

            trajectory.append(RobotState(
                time=t,
                joint_positions=q,
                joint_velocities=qd
            ))

        return trajectory

    def _check_trajectory_stability(
        self,
        trajectory: List[RobotState],
        margin: float
    ) -> Tuple[bool, float]:
        """
        Verify ZMP stays in support polygon throughout trajectory.

        Implements FR-2, SR-2.
        """
        min_margin = float('inf')

        for state in trajectory:
            # Compute ZMP at this state
            zmp = self.contact.compute_zmp(state, self.robot_model)
            support_polygon = self.contact.get_support_polygon(state)

            # Check margin
            current_margin = self.contact.distance_to_polygon_edge(zmp, support_polygon)
            min_margin = min(min_margin, current_margin)

            if current_margin < margin:
                return False, min_margin

        return True, min_margin

    def _minimum_jerk(self, s: float) -> float:
        """5th-order polynomial for smooth motion."""
        return 10*s**3 - 15*s**4 + 6*s**5

    def _minimum_jerk_velocity(self, s: float) -> float:
        """Derivative of minimum jerk trajectory."""
        return 30*s**2 - 60*s**3 + 30*s**4

    def _truncate_to_stable(self, trajectory, margin):
        """Return trajectory prefix that maintains stability."""
        for i, state in enumerate(trajectory):
            zmp = self.contact.compute_zmp(state, self.robot_model)
            support = self.contact.get_support_polygon(state)
            if self.contact.distance_to_polygon_edge(zmp, support) < margin:
                return trajectory[:i]
        return trajectory
```

**Student Review Checklist**:
- ✅ All requirements (FR-1 through SR-3) referenced in code
- ✅ Component interfaces used correctly (FK.compute, IK.compute, etc.)
- ✅ Error handling for each failure mode
- ✅ Diagnostics populated for debugging
- ⚠️ **Student catches issue**: "AI didn't validate that trajectory satisfies velocity limits continuously—only checks at waypoints. Need to add interpolation validation."

**Student refines spec, AI regenerates**—iteration continues until implementation passes all tests.

---

### Part 4: SDD-RI Challenge (AI Orchestrator + Spec Grader)

**Complete Integration Challenge**:

```
SPECIFICATION: Humanoid Reaching & Balancing System - Full Integration

DELIVERABLES:
1. Complete system specification document:
   - 10+ functional requirements (numbered FR-1, FR-2, ...)
   - 5+ performance requirements (PR-1, PR-2, ...)
   - 5+ safety requirements (SR-1, SR-2, ...)
   - 10+ test cases (TC-1, TC-2, ...) with pass/fail criteria

2. AI-orchestrated implementation:
   - ReachingController class (generated by AI from your spec)
   - Integration code connecting all 5 components
   - Error handling and diagnostics

3. Spec-based integration tests:
   - Each test case (TC-1, etc.) implemented as pytest test
   - Requirements traceability: Each test references requirements it validates
   - 100% requirement coverage (every FR/PR/SR tested)

4. MuJoCo deployment:
   - Execute reaching tasks in simulation
   - Record success rate, stability margins, computation times
   - Video demonstration of 5 successful reaches

SUCCESS CRITERIA:
- Specification completeness: All inputs/outputs/requirements/tests defined
- Implementation correctness: Passes 100% of spec tests
- Requirements traceability: Test-to-requirement mapping documented
- Integration performance: Meets all PR requirements
- Deployment success: >90% success rate on 20 random targets in MuJoCo

TEST CASES (Examples from spec):
1. TC-1 (Nominal): target [0.4, 0.2, 0.9], expect success
2. TC-2 (Unreachable): target [2.0, 0, 1.0], expect graceful failure
3. TC-3 (Stability): target causing ZMP violation, expect safe abort
4. TC-4 (Performance): 100 targets, average time <1s
5. TC-5 (Stress): Workspace boundaries, verify robustness
```

> **🤖 AI Orchestrator Prompt**:
> "Given my system specification [paste spec], generate the complete integration code, pytest test suite with requirement traceability, and MuJoCo deployment script. Explain design decisions and potential failure modes."

**Grading Criteria**:
- **Spec Quality (60%)**:
  - Completeness (all requirements defined, testable) (20%)
  - Clarity (unambiguous success criteria) (15%)
  - Traceability (tests reference requirements) (15%)
  - Realistic constraints (based on Lessons 1-8 knowledge) (10%)
- **Integration Quality (40%)**:
  - Correct component usage (no misuse of APIs) (15%)
  - Error handling (graceful failures per SR requirements) (15%)
  - Performance (meets PR requirements) (10%)

---

### Part 5: Key Takeaways

1. **System specifications define success, implementations just achieve it**: Write "what" (measurable requirements) before "how" (code). If you can't test a requirement, it's not a requirement—it's a wish.

2. **AI orchestration accelerates integration but requires expert review**: AI can wire components together correctly IF your spec is clear. Ambiguous specs produce buggy integrations. Your job: write airtight specs, validate AI's implementation.

3. **Requirements traceability prevents scope gaps**: Every requirement must have a test. Every test must reference requirements. If TC-7 exists but no requirement explains why, either the test is wrong or the spec is incomplete.

4. **Spec-driven integration scales to real systems**: This workflow (formal spec → component composition → integration tests → deployment) is how professional robotics teams build software for robots that cost millions and can't afford to fail.

**Looking Ahead**: You've completed the full 4-layer pedagogical journey—from manual transformations (L1) through AI-assisted learning (L2), component design (L3), to system integration (L4). These skills transfer to any robotics domain: manipulation, locomotion, navigation, perception.

---

### Part 6: Learn with AI

**Understand It Better**:
> "Explain the V-Model for system development (requirements → design → implementation → testing). How does spec-driven development fit into this?"

**Get Feedback on Your Spec**:
> "Review my Reaching & Balancing system specification and identify missing requirements, ambiguous success criteria, or untestable constraints:
> [paste your spec here]"

**Go Deeper**:
> "What are formal verification techniques for robotics systems (model checking, theorem proving)? When is formal verification worth the effort vs. extensive testing?"

**See It in Action**:
> "Show me a real-world example of a robotics system specification from industry (e.g., autonomous vehicle, surgical robot). What level of detail is standard?"

**Extend Your System**:
> "How would I modify my Reaching & Balancing spec to handle dynamic obstacles (moving targets)? What new requirements and components are needed?"

---

**End of Lesson Part 2 (Lessons 6-9)**

Total word count: ~3,980 words
Lessons covered: 6-9 (Simulation, Sim2Real, Intelligence Design, Integration)
Pedagogical layers: Layer 2 (Lessons 6-7), Layer 3 (Lesson 8), Layer 4 (Lesson 9)

**Complete Chapter Summary**:
- Part 1 (Lessons 1-5): Manual foundations (transformations, FK, IK, dynamics, contact) + AI collaboration
- Part 2 (Lessons 6-9): Simulation frameworks, domain randomization, component design, system integration
- Total: 9 lessons, 24.5 hours estimated, covering full SDD-RI workflow from theory to deployed systems
