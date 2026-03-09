# Commit 1: URDF and Description Changes

**Commit:** `1b041112` - "feat: wrist roll urdf (todo: correct inertial properties)"  
**Date:** November 19, 2025  
**Package:** `open_manipulator_description`

## Overview

This commit implements the core mechanical structure changes by modifying the URDF to split the original link5 into two separate links with a new revolute joint between them.

## Files Modified

### 1. New Mesh Files Added
```
open_manipulator_description/meshes/open_manipulator_x/
├── wrist_link.stl                      # New: Wrist base with motor housing
├── wrist_link.bak.stl                  # Backup
├── gripper_link_with_bracket.stl       # New: Gripper base with bracket
└── gripper_link_with_bracket.bak.stl   # Backup
```

**Why:** The original `link5.stl` was a single piece containing both the wrist and gripper base. To add a rotational joint, this had to be split into two separate parts with a motor mounting interface.

### 2. URDF File: `open_manipulator_x_arm.urdf.xacro`

#### Change 1: Link5 Mesh Update
```xml
<!-- BEFORE -->
<link name="${prefix}link5">
  <visual>
    <geometry>
      <mesh filename="${meshes_file_direction}/link5.stl" scale="0.001 0.001 0.001"/>
    </geometry>
  </visual>
  <inertial>
    <origin xyz="4.4206755e-02 3.6839985e-07 8.9142216e-03" />
    <mass value="1.4327573e-01" />
    ...
  </inertial>
</link>

<!-- AFTER -->
<link name="${prefix}link5">
  <visual>
    <geometry>
      <mesh filename="${meshes_file_direction}/wrist_link.stl" scale="0.001 0.001 0.001"/>
    </geometry>
  </visual>
  <inertial>
    <origin xyz="2.5e-02 0.0 0.0" />
    <mass value="8.5e-02" />
    ...
  </inertial>
</link>
```

**Why this change:**
- **Mesh:** Changed from `link5.stl` to `wrist_link.stl` to use the new separated wrist portion
- **Inertial properties:** Updated to reflect only the wrist portion mass and COM:
  - Mass reduced from 0.143 kg → 0.085 kg (wrist portion only)
  - COM moved from [0.044, 0, 0.009] → [0.025, 0, 0] (closer to joint)
  - Inertia tensor simplified (estimates, noted in commit message as TODO)

**Purpose:** Link5 now represents only the wrist base that houses the Dynamixel motor, not the entire end effector assembly.

---

#### Change 2: New Wrist Roll Joint
```xml
<!-- NEW JOINT ADDED -->
<joint name="${prefix}joint5_roll" type="revolute">
  <parent link="${prefix}link5"/>
  <child  link="${prefix}wrist_roll_link"/>
  <origin xyz="0.0335 0.0 0.0" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="-3.141592654" upper="3.141592654" velocity="4.8" effort="1000"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

**Why this change:**
- **Type:** `revolute` - allows rotation within specified limits
- **Parent:** `link5` (wrist base)
- **Child:** `wrist_roll_link` (gripper base)
- **Origin:** `[0.0335, 0, 0]` - 33.5mm forward from link5 origin (motor length)
- **Axis:** `[1, 0, 0]` - X-axis (roll about arm longitudinal axis)
- **Limits:** ±180° (±π radians) - full rotation capability
- **Velocity:** 4.8 rad/s (same as other joints)
- **Effort:** 1000 (placeholder for position control)
- **Damping:** 0.1 (standard damping for stability)

**Purpose:** This is the **new 5th DOF** that provides wrist roll capability, allowing the gripper to rotate about its approach axis.

---

#### Change 3: New Wrist Roll Link
```xml
<!-- NEW LINK ADDED -->
<link name="${prefix}wrist_roll_link">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <mesh filename="${meshes_file_direction}/gripper_link_with_bracket.stl" scale="0.001 0.001 0.001"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <mesh filename="${meshes_file_direction}/gripper_link_with_bracket.stl" scale="0.001 0.001 0.001"/>
    </geometry>
  </collision>
  <inertial>
    <origin xyz="4.5e-02 0.0 0.0" />
    <mass value="6.5e-02" />
    <inertia ixx="2.5e-05" ixy="0.0" ixz="0.0"
             iyy="3.0e-05" iyz="0.0"
             izz="3.5e-05" />
  </inertial>
</link>
```

**Why this change:**
- **Mesh:** Uses `gripper_link_with_bracket.stl` - the gripper base portion with custom mounting bracket
- **Inertial properties:**
  - Mass: 0.065 kg (gripper base and bracket only)
  - COM: [0.045, 0, 0] (45mm forward, where gripper assembly is concentrated)
  - Inertia: Estimated values (TODO noted in commit)

**Purpose:** This link serves as the gripper mounting base and rotates with the wrist roll joint. The bracket design allows secure mounting of the gripper servos.

---

#### Change 4: Gripper Joint Parent Update
```xml
<!-- BEFORE -->
<joint name="${prefix}gripper_left_joint" type="prismatic">
  <parent link="${prefix}link5"/>
  <child link="${prefix}gripper_left_link"/>
  ...
</joint>

<joint name="${prefix}gripper_right_joint" type="prismatic">
  <parent link="${prefix}link5"/>
  <child link="${prefix}gripper_right_link"/>
  ...
</joint>

<!-- AFTER -->
<joint name="${prefix}gripper_left_joint" type="prismatic">
  <parent link="${prefix}wrist_roll_link"/>
  <child link="${prefix}gripper_left_link"/>
  ...
</joint>

<joint name="${prefix}gripper_right_joint" type="prismatic">
  <parent link="${prefix}wrist_roll_link"/>
  <child link="${prefix}gripper_right_link"/>
  ...
</joint>
```

**Why this change:**
- Gripper joints' parent changed from `link5` → `wrist_roll_link`
- **Origin remains [0.0817, ±0.021, 0]** - gripper base geometry unchanged

**Purpose:** Gripper fingers now attach to the rotating wrist_roll_link instead of the fixed link5, allowing them to rotate with the wrist roll joint.

---

#### Change 5: End Effector Parent Update
```xml
<!-- BEFORE -->
<joint name="${prefix}end_effector_joint" type="fixed">
  <parent link="${prefix}link5"/>
  <child link="${prefix}end_effector_link"/>
  <origin xyz="0.126 0.0 0.0" rpy="0 0 0"/>
</joint>

<!-- AFTER -->
<joint name="${prefix}end_effector_joint" type="fixed">
  <parent link="${prefix}wrist_roll_link"/>
  <child link="${prefix}end_effector_link"/>
  <origin xyz="0.126 0.0 0.0" rpy="0 0 0"/>
</joint>
```

**Why this change:**
- End effector frame parent changed from `link5` → `wrist_roll_link`
- **Origin remains [0.126, 0, 0]** - TCP (Tool Center Point) position unchanged relative to gripper

**Purpose:** The end effector frame (TCP) rotates with the gripper, maintaining the correct position for motion planning and task execution.

---

### 3. Gazebo Configuration: `open_manipulator_x.gazebo.xacro`

```xml
<!-- ADDED -->
<xacro:Link reference="${prefix}wrist_roll_link"/>

<!-- ADDED -->
<xacro:SimpleTransmission trans="${prefix}trans_roll" 
                         joint="${prefix}joint5_roll" 
                         actuator="${prefix}actuator_roll" />
```

**Why this change:**
- **Link reference:** Adds Gazebo physics properties for wrist_roll_link (friction, damping, collision)
- **Transmission:** Enables Gazebo to simulate the joint5_roll motor for position control

**Purpose:** Ensures the new joint and link work properly in Gazebo simulation with realistic physics.

---

## Why These Changes Are Required

### 1. **Kinematic Chain Modification**
Adding a new joint requires splitting the kinematic chain:
```
OLD: link4 → joint4 → link5 → [gripper joints]
NEW: link4 → joint4 → link5 → joint5_roll → wrist_roll_link → [gripper joints]
```

### 2. **Parent Link Updates**
All downstream elements (gripper joints, end effector) must update their parent references to maintain correct spatial relationships.

### 3. **Mesh Splitting**
A single rigid mesh cannot contain a joint. The original link5 had to be split at the planned motor location.

### 4. **Inertial Properties**
Each link needs accurate mass and inertia for:
- Gazebo physics simulation
- Gravity compensation
- Dynamics calculations
- (Note: Values are estimates, marked as TODO)

### 5. **Gazebo Integration**
Simulation requires:
- Link physics properties
- Joint transmission definition
- Collision geometry

---

## Technical Details

### Joint Origin Calculation
The `joint5_roll` origin `[0.0335, 0, 0]` represents:
- **33.5 mm** along X-axis from link5 origin
- Based on typical Dynamixel XM430 motor length (~34mm)
- This positions the output shaft at the correct location for gripper attachment

### Gripper Offset Preservation
The gripper joint offset `[0.0817, ±0.021, 0]` was **kept unchanged** because:
- The gripper base geometry in `gripper_link_with_bracket.stl` maintains the same mounting points
- Only the parent link changed, not the relative position

### Axis Selection
Roll about X-axis `[1, 0, 0]` because:
- X-axis is the arm's longitudinal axis (forward direction)
- Rolling about this axis rotates the gripper while maintaining approach direction
- Standard convention for wrist roll in serial manipulators

---

## Impact on Other Systems

✅ **Simulation:** Requires Gazebo transmission (added)  
✅ **Control:** Requires ros2_control interface (next commit)  
✅ **Planning:** Requires MoveIt configuration (commit 4)  
✅ **Visualization:** Requires RViz link display (automatically handled)  

---

## Testing Verification

After this commit, the URDF should:
1. ✅ Parse without errors: `xacro open_manipulator_x_arm.urdf.xacro`
2. ✅ Display in RViz with 6 links visible
3. ✅ Show joint5_roll in joint state publisher GUI
4. ⚠️ Not yet controllable (requires hardware config in next commit)

---

**Next:** [02_HARDWARE_CHANGES.md](./02_HARDWARE_CHANGES.md) - ros2_control and hardware interface configuration
