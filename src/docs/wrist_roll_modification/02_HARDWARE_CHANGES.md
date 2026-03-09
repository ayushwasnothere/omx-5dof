# Commit 2: Hardware and ros2_control Changes

**Commit:** `7527c79e` - "feat: wrist roll hardware changes"  
**Date:** November 19, 2025  
**Packages:** `open_manipulator_bringup`, `open_manipulator_description`

## Overview

This commit configures the ros2_control hardware interface to support the 6th Dynamixel motor, updates controller configurations, and modifies initialization parameters.

## Files Modified

### 1. ros2_control Configuration: `open_manipulator_x_position.ros2_control.xacro`

#### Change 1: Joint Count Update
```xml
<!-- BEFORE -->
<param name="number_of_joints">5</param>
<param name="number_of_transmissions">5</param>

<!-- AFTER -->
<param name="number_of_joints">6</param>
<param name="number_of_transmissions">6</param>
```

**Why this change:**
- Informs the `dynamixel_hardware_interface` that there are now 6 motors to manage
- Each joint has a corresponding transmission (motor → joint mapping)

**Purpose:** The hardware interface initializes communication with all 6 Dynamixel motors instead of 5.

---

#### Change 2: Transmission Matrix Expansion
```xml
<!-- BEFORE (5x5 identity matrix) -->
<param name="transmission_to_joint_matrix">
  1, 0, 0, 0, 0,
  0, 1, 0, 0, 0,
  0, 0, 1, 0, 0,
  0, 0, 0, 1, 0,
  0, 0, 0, 0, 1
</param>
<param name="joint_to_transmission_matrix">
  1, 0, 0, 0, 0,
  0, 1, 0, 0, 0,
  0, 0, 1, 0, 0,
  0, 0, 0, 1, 0,
  0, 0, 0, 0, 1
</param>

<!-- AFTER (6x6 identity matrix) -->
<param name="transmission_to_joint_matrix">
  1, 0, 0, 0, 0, 0,
  0, 1, 0, 0, 0, 0,
  0, 0, 1, 0, 0, 0,
  0, 0, 0, 1, 0, 0,
  0, 0, 0, 0, 1, 0,
  0, 0, 0, 0, 0, 1
</param>
<param name="joint_to_transmission_matrix">
  1, 0, 0, 0, 0, 0,
  0, 1, 0, 0, 0, 0,
  0, 0, 1, 0, 0, 0,
  0, 0, 0, 1, 0, 0,
  0, 0, 0, 0, 1, 0,
  0, 0, 0, 0, 0, 1
</param>
```

**Why this change:**
- Transmission matrices define the relationship between motor positions and joint positions
- Identity matrix = 1:1 direct coupling (motor angle = joint angle)
- Must match the number of joints (6x6 for 6 joints)

**Purpose:** Ensures each of the 6 motors maps directly to its corresponding joint with no coupling or gearing.

**Matrix Mapping:**
```
Row 1: dxl1 → joint1
Row 2: dxl2 → joint2
Row 3: dxl3 → joint3
Row 4: dxl4 → joint4
Row 5: dxl5 → joint5_roll  ← NEW
Row 6: dxl6 → gripper_left_joint (was dxl5)
```

---

#### Change 3: Gripper Motor Renumbering
```xml
<!-- BEFORE -->
<param name="revolute_to_prismatic_dxl">dxl5</param>

<!-- AFTER -->
<param name="revolute_to_prismatic_dxl">dxl6</param>
```

**Why this change:**
- The gripper uses a revolute Dynamixel motor but presents as a prismatic joint in URDF
- Hardware interface converts motor rotation → linear gripper position
- Motor ID changed: 15 → 16 (to make room for wrist roll at ID 15)

**Purpose:** Points the gripper conversion logic to the correct motor (now dxl6/ID 16 instead of dxl5/ID 15).

**Critical:** Physical gripper motor **MUST** be reconfigured to ID 16 for this to work!

---

#### Change 4: New Joint Interface
```xml
<!-- NEW JOINT ADDED -->
<joint name="${prefix}joint5_roll">
  <command_interface name="position"/>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
  <state_interface name="effort"/>
</joint>
```

**Why this change:**
- ros2_control needs explicit joint interface definitions
- Specifies what control commands the joint accepts (position)
- Specifies what state feedback is available (position, velocity, effort)

**Purpose:** Enables ros2_control to send position commands and read state from joint5_roll.

---

#### Change 5: New Dynamixel Motor Configuration (dxl5)
```xml
<!-- NEW MOTOR ADDED -->
<gpio name="dxl5">
  <param name="type">dxl</param>
  <param name="ID">15</param>
  <command_interface name="Goal Position"/>
  <state_interface name="Present Position"/>
  <state_interface name="Present Velocity"/>
  <state_interface name="Present Current"/>
  <param name="Operating Mode">3</param>
  <param name="Position P Gain">800</param>
  <param name="Position I Gain">100</param>
  <param name="Position D Gain">100</param>
  <param name="Profile Velocity">20</param>
  <param name="Profile Acceleration">10</param>
  <param name="Drive Mode">0</param>
  <param name="Return Delay Time">0</param>
</gpio>
```

**Why this change:**
- Defines the new wrist roll motor's Dynamixel-specific parameters
- Hardware interface uses this to configure the motor at startup

**Purpose:** Configures Dynamixel ID 15 for position control of the wrist roll joint.

**Parameter Breakdown:**

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `ID` | 15 | Dynamixel motor address on RS-485 bus |
| `Operating Mode` | 3 | Position Control (not velocity or current) |
| `Position P Gain` | 800 | Proportional gain for position control |
| `Position I Gain` | 100 | Integral gain for steady-state error |
| `Position D Gain` | 100 | Derivative gain for damping |
| `Profile Velocity` | 20 | Maximum velocity during motion |
| `Profile Acceleration` | 10 | Acceleration/deceleration rate |
| `Drive Mode` | 0 | Normal direction (not reversed) |
| `Return Delay Time` | 0 | No delay in status packet response |

**Why Operating Mode 3:**
- Mode 3 = Position Control (with profile)
- Allows precise position commands with velocity/acceleration limiting
- Same mode as joints 1-4 for consistent behavior
- Mode 5 (current-based) is only used for gripper

---

#### Change 6: Gripper Motor Renumbering (dxl6)
```xml
<!-- BEFORE: dxl5 (ID 15) -->
<gpio name="dxl5">
  <param name="ID">15</param>
  <param name="Operating Mode">5</param>
  ...
</gpio>

<!-- AFTER: dxl6 (ID 16) -->
<gpio name="dxl6">
  <param name="ID">16</param>
  <param name="Operating Mode">5</param>
  ...
</gpio>
```

**Why this change:**
- Gripper motor renamed: dxl5 → dxl6
- Motor ID changed: 15 → 16
- Makes room for wrist roll at ID 15

**Purpose:** Maintains gripper functionality while adding the new motor in the middle of the chain.

**Important:** This requires **physical reconfiguration** of the gripper Dynamixel motor to ID 16!

---

### 2. Controller Manager: `hardware_controller_manager.yaml`

```yaml
# BEFORE
arm_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
      - joint4

# AFTER
arm_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5_roll  # NEW
```

**Why this change:**
- The `arm_controller` (JointTrajectoryController) needs to know about all arm joints
- MoveIt and other planners send trajectories to this controller

**Purpose:** Enables the arm controller to accept and execute trajectories for 5 joints instead of 4.

**Note:** Gripper controller remains separate and unchanged.

---

### 3. Initial Positions: `initial_positions.yaml`

```yaml
# BEFORE
joint_trajectory_executor:
  ros__parameters:
    joint_names:
      - joint1
      - joint2
      - joint3
      - joint4
    
    home: [0.0, -1.0, 1.0, 0.0]  # 4 values

# AFTER
joint_trajectory_executor:
  ros__parameters:
    joint_names:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5_roll  # NEW
    
    home: [0.0, -1.0, 1.0, 0.0, 0.0]  # 5 values
```

**Why this change:**
- Joint trajectory executor sends robot to home position on startup
- Must include all arm joints
- Must provide position value for each joint

**Purpose:** Defines the home position for the 5-DOF arm with wrist roll at 0° (neutral position).

**Home Position:**
- joint1: 0.0 rad (base centered)
- joint2: -1.0 rad (shoulder down ~-57°)
- joint3: 1.0 rad (elbow bent ~57°)
- joint4: 0.0 rad (wrist level)
- joint5_roll: 0.0 rad (wrist neutral, no roll)

---

## Why These Changes Are Required

### 1. **Hardware Interface Awareness**
The `dynamixel_hardware_interface` plugin must know:
- How many motors exist (6)
- Each motor's ID and configuration
- How motors map to joints (transmission matrices)

Without these updates, the hardware interface would only initialize 5 motors and ignore the wrist roll.

### 2. **Motor ID Reassignment**
Dynamixel motors communicate via RS-485 with unique IDs:
```
OLD: [11, 12, 13, 14, 15]
NEW: [11, 12, 13, 14, 15, 16]
                      ↑new   ↑gripper moved here
```

The gripper **must** move from ID 15 to ID 16 to avoid conflict with the new wrist roll motor.

### 3. **Controller Configuration**
ros2_control controllers need explicit joint lists:
- `arm_controller`: Controls all arm joints for motion planning
- `gripper_controller`: Controls gripper independently

Missing joint5_roll from arm_controller would make it uncontrollable via MoveIt.

### 4. **Initialization Safety**
The home position must include all joints to prevent:
- Undefined joint states on startup
- Partial trajectory execution
- Controller errors

---

## Technical Deep Dive

### Transmission Matrix Explained
```
transmission_to_joint_matrix (6x6):
[motor1_pos]   [1 0 0 0 0 0]   [joint1_pos]
[motor2_pos]   [0 1 0 0 0 0]   [joint2_pos]
[motor3_pos] = [0 0 1 0 0 0] × [joint3_pos]
[motor4_pos]   [0 0 0 1 0 0]   [joint4_pos]
[motor5_pos]   [0 0 0 0 1 0]   [joint5_roll_pos]
[motor6_pos]   [0 0 0 0 0 1]   [gripper_pos]
```

Identity matrix = no gearing, no coupling, 1:1 mapping.

### Gripper Special Case
The gripper is special because:
- **Physical:** Rotary Dynamixel motor
- **URDF:** Prismatic joint (linear motion)
- **Conversion:** `revolute_to_prismatic_dxl` parameter handles this

The hardware interface converts:
```
Motor angle (radians) → Gripper opening (meters)
Using calibration: revolute_max, revolute_min, prismatic_max, prismatic_min
```

---

## Impact on Physical Hardware

⚠️ **CRITICAL PHYSICAL CHANGES REQUIRED:**

### Before Powering On:
1. ✅ Install new Dynamixel motor for wrist roll
2. ✅ Configure wrist roll motor to ID 15 using Dynamixel Wizard
3. ✅ **Reconfigure gripper motor from ID 15 → ID 16**
4. ✅ Connect motors in daisy chain
5. ✅ Verify USB2Dynamixel connection

### Motor Configuration Checklist:
```
ID 11: Base (joint1)          - Mode 3, existing
ID 12: Shoulder (joint2)      - Mode 3, existing
ID 13: Elbow (joint3)         - Mode 3, existing
ID 14: Wrist Pitch (joint4)   - Mode 3, existing
ID 15: Wrist Roll (NEW!)      - Mode 3, NEW MOTOR
ID 16: Gripper (changed!)     - Mode 5, RECONFIGURE FROM ID 15
```

### Testing After Changes:
```bash
# Scan for Dynamixel motors
ros2 run dynamixel_sdk_examples scan_dynamixel

# Expected output: IDs [11, 12, 13, 14, 15, 16]
# If you see [11, 12, 13, 14, 15, 15] → ID conflict!
```

---

## Error Prevention

### Common Issues:

**Issue 1:** Gripper motor still at ID 15
```
Error: Multiple motors with ID 15 detected
Solution: Reconfigure gripper to ID 16
```

**Issue 2:** Wrist roll motor not detected
```
Error: Expected 6 joints, found 5
Solution: Verify motor ID 15 is set and connected
```

**Issue 3:** Wrong operating mode
```
Error: Motor not responding to position commands
Solution: Set Operating Mode = 3 for wrist roll
```

---

## Testing Verification

After this commit, the system should:

```bash
# 1. Launch hardware interface
ros2 launch open_manipulator_bringup open_manipulator_x.launch.py

# 2. Check joint states (should show 6 joints)
ros2 topic echo /joint_states

# Expected joints: [joint1, joint2, joint3, joint4, joint5_roll, gripper_left_joint]

# 3. Test wrist roll motion
ros2 topic pub /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5_roll'],
  points: [{
    positions: [0.0, 0.0, 0.0, 0.0, 1.57],
    time_from_start: {sec: 2}
  }]
}" --once

# Expected: Wrist roll rotates 90°
```

---

**Next:** [03_LAUNCH_FIX.md](./03_LAUNCH_FIX.md) - ParameterValue type error fix
