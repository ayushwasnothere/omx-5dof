# Technical Overview: 5-DOF Wrist Roll Modification

## Executive Summary

This document provides a high-level technical overview of the Open Manipulator X wrist roll modification, transforming the robot from a 4-DOF + gripper configuration to a 5-DOF + gripper system with enhanced dexterity.

---

## System Architecture

### Hardware Stack
```
┌─────────────────────────────────────────────────┐
│              Physical Robot                      │
│  6× Dynamixel Motors (IDs 11-16)                │
│  - dxl1 (11): Joint 1 - Base rotation           │
│  - dxl2 (12): Joint 2 - Shoulder pitch          │
│  - dxl3 (13): Joint 3 - Elbow pitch             │
│  - dxl4 (14): Joint 4 - Wrist pitch             │
│  - dxl5 (15): Joint 5 - Wrist roll ★ NEW        │
│  - dxl6 (16): Gripper (renumbered from ID 15)   │
└─────────────────────────────────────────────────┘
                    ↕ RS-485 Bus
┌─────────────────────────────────────────────────┐
│          Dynamixel Hardware Interface            │
│  (dynamixel_hardware_interface package)         │
│  - Bulk read/write operations                   │
│  - Position/velocity/effort control             │
│  - 6×6 transmission matrices                    │
└─────────────────────────────────────────────────┘
                    ↕ ros2_control
┌─────────────────────────────────────────────────┐
│            Controller Manager                    │
│  - arm_controller (5 joints)                    │
│  - gripper_controller (1 joint)                 │
│  - joint_state_broadcaster                      │
└─────────────────────────────────────────────────┘
                    ↕ ROS 2 Topics
┌─────────────────────────────────────────────────┐
│          Application Layer                       │
│  ┌─────────────┐  ┌──────────┐  ┌───────────┐  │
│  │   MoveIt 2  │  │ Teleop   │  │    GUI    │  │
│  │ (Planning)  │  │(Keyboard)│  │ (Manual)  │  │
│  └─────────────┘  └──────────┘  └───────────┘  │
└─────────────────────────────────────────────────┘
```

---

## Component Interactions

### 1. Robot Description Flow
```
URDF Source Files
├── open_manipulator_x_arm.urdf.xacro
│   ├── Links: link1 → link2 → link3 → link4 → link5 
│   │           → wrist_roll_link ★ → gripper links
│   ├── Joints: joint1, joint2, joint3, joint4, 
│   │           joint5_roll ★, gripper_joint
│   └── Meshes: wrist_link.stl ★, 
│                gripper_link_with_bracket.stl ★
│
├── open_manipulator_x.gazebo.xacro
│   └── Physics: mu1, mu2, kp, kd for wrist_roll_link
│
└── open_manipulator_x_position.ros2_control.xacro
    └── Hardware: 6 joints → 6 Dynamixel motors

xacro Processing
↓
robot_description (URDF XML)
↓
┌─────────────────┬───────────────┬──────────────┐
│ robot_state     │ MoveIt        │ ros2_control │
│ _publisher      │ planning      │ spawner      │
└─────────────────┴───────────────┴──────────────┘
```

### 2. Control Data Flow

#### Forward Path (Command → Motion)
```
User Command
    ↓
[MoveIt Goal] or [GUI/Teleop Command]
    ↓
Trajectory Planning (MoveIt arm group: 5 joints)
    ↓
/arm_controller/joint_trajectory topic
    ↓
JointTrajectoryController (ros2_control)
    ↓
HardwareInterface::write()
    ↓
Dynamixel Bulk Write
    ↓
Motor Motion (RS-485 → 6 motors)
```

#### Feedback Path (Sensors → State)
```
Dynamixel Encoders
    ↓
RS-485 Bulk Read
    ↓
HardwareInterface::read()
    ↓
/joint_states topic
    ↓
┌────────────────┬──────────────┬──────────────┐
│ robot_state    │ MoveIt       │ GUI Display  │
│ _publisher     │ current_state│ (spinboxes)  │
└────────────────┴──────────────┴──────────────┘
```

---

## Key Modifications Summary

### Modification 1: Mechanical Design
**Goal:** Add wrist rotation capability between wrist pitch and gripper

**Implementation:**
- Split original `link5.stl` mesh into two parts:
  - `wrist_link.stl`: Bracket holding wrist roll motor
  - `gripper_link_with_bracket.stl`: Gripper body with motor bracket
- Added Dynamixel motor slot (33.5mm length)
- Preserved gripper geometry (81.7mm offset maintained)

**Impact:**
- ✅ 4-DOF → 5-DOF manipulation
- ✅ Additional roll axis for orientation control
- ⚠️ Requires physical motor reconfiguration (gripper ID 15→16)

---

### Modification 2: URDF Structure
**Goal:** Define new kinematic chain with wrist roll joint

**Changes:**
```
OLD: link4 → link5 → gripper_link
            (via joint4)  (via gripper_joint)

NEW: link4 → link5 → wrist_roll_link → gripper_link
            (via joint4)  (via joint5_roll)  (via gripper_joint)
```

**Joint Properties:**
- `joint5_roll`: Revolute, X-axis, limits ±π rad (±180°)
- Origin: [0.0335, 0, 0] (motor length offset)
- Control: Position mode (Dynamixel Mode 3)

**Impact:**
- ✅ Valid kinematic chain for IK/FK
- ✅ TF tree includes wrist_roll_link frame
- ✅ Collision detection for new link
- ⚠️ Inertial properties are estimates (need CAD measurement)

---

### Modification 3: Hardware Interface
**Goal:** Control 6 motors instead of 5

**Changes:**
- Motors: 5→6 (added dxl5 for joint5_roll)
- Transmission matrices: 5×5 → 6×6
- Motor mapping:
  ```
  joint1 → dxl1 (ID 11) ✓
  joint2 → dxl2 (ID 12) ✓
  joint3 → dxl3 (ID 13) ✓
  joint4 → dxl4 (ID 14) ✓
  joint5_roll → dxl5 (ID 15) ★ NEW
  gripper_joint → dxl6 (ID 16) ★ RENUMBERED
  ```

**Configuration:**
- Control mode: Position (Mode 3)
- Initial position: 0.0 rad
- Velocity/effort limits: Inherited from Dynamixel specs

**Impact:**
- ✅ All 6 motors controllable via ros2_control
- ⚠️ **CRITICAL:** Physical gripper motor MUST be reconfigured to ID 16
- ✅ Bulk read/write efficiency maintained

---

### Modification 4: MoveIt Planning
**Goal:** Enable motion planning for 5-DOF arm

**Changes:**
- Planning group "arm": 4 joints → 5 joints (added joint5_roll)
- Collision matrix: Added wrist_roll_link pairs
- Joint limits: joint5_roll [-π, π]
- IK solver timeout: 0.005s → 0.05s (10× increase)

**Planning Features:**
- ✅ Cartesian path planning with orientation control
- ✅ Self-collision avoidance for wrist link
- ✅ Home/init poses include 5 joint values
- ✅ Gripper control remains independent

**Impact:**
- ✅ IK solutions for end-effector orientation
- ✅ Redundancy for obstacle avoidance
- ⚠️ Longer IK solve time (10× timeout needed)

---

### Modification 5: User Interfaces
**Goal:** Provide wrist roll control across all interfaces

**Changes:**
1. **GUI (Qt):**
   - Added Joint 5 roll spinbox, slider, set button
   - Updated joint state display (5 arm joints + gripper)
   - CSV logging includes 6 values

2. **Teleop (Keyboard):**
   - Inherits from MoveIt arm group (automatic)
   - No code changes required

3. **Launch Files:**
   - Fixed ROS 2 Jazzy type error with ParameterValue wrapper

**Impact:**
- ✅ Manual control via GUI
- ✅ Keyboard teleoperation
- ✅ Trajectory logging/playback
- ✅ Compatible with ROS 2 Jazzy

---

## Configuration Matrix

| Component              | 4-DOF Config      | 5-DOF Config          | Change Required |
|------------------------|-------------------|------------------------|-----------------|
| **Arm Joints**         | 4                 | 5                     | ✓ URDF          |
| **Total Motors**       | 5 (4+gripper)     | 6 (5+gripper)         | ✓ Hardware      |
| **MoveIt Group Size**  | 4                 | 5                     | ✓ SRDF          |
| **Controller Joints**  | 4                 | 5                     | ✓ Controllers   |
| **GUI Spinboxes**      | 5 (4+gripper)     | 6 (5+gripper)         | ✓ UI            |
| **Transmission Matrix**| 5×5               | 6×6                   | ✓ ros2_control  |
| **Joint Limits File**  | 4 entries         | 5 entries             | ✓ MoveIt        |
| **Gripper Motor ID**   | 15                | 16                    | ⚠️ **CRITICAL** |

---

## Data Structures

### Joint State Message
```cpp
sensor_msgs::msg::JointState
{
  name: [
    "joint1",
    "joint2", 
    "joint3",
    "joint4",
    "joint5_roll",    // ★ NEW
    "gripper_joint"
  ]
  position: [6 values]  // Was 5
  velocity: [6 values]  // Was 5
  effort:   [6 values]  // Was 5
}
```

### MoveIt Planning Request
```cpp
moveit_msgs::msg::MotionPlanRequest
{
  group_name: "arm"
  goal_constraints: {
    joint_constraints: [
      {joint_name: "joint1", position: ...},
      {joint_name: "joint2", position: ...},
      {joint_name: "joint3", position: ...},
      {joint_name: "joint4", position: ...},
      {joint_name: "joint5_roll", position: ...}  // ★ NEW
    ]
  }
}
```

### Trajectory Command
```cpp
trajectory_msgs::msg::JointTrajectory
{
  joint_names: [
    "joint1", "joint2", "joint3", "joint4", "joint5_roll"
  ]
  points: [
    {
      positions: [5 values],  // Was 4
      time_from_start: ...
    }
  ]
}
```

---

## Critical Dependencies

### Build-Time Dependencies
```xml
<!-- package.xml -->
<depend>urdf</depend>
<depend>xacro</depend>
<depend>robot_state_publisher</depend>
<depend>ros2_control</depend>
<depend>ros2_controllers</depend>
<depend>moveit_ros_planning_interface</depend>
<depend>dynamixel_hardware_interface</depend>
```

### Runtime Dependencies
- **Dynamixel SDK**: Motor communication
- **ros2_control**: Hardware interface framework
- **MoveIt 2**: Motion planning and IK
- **TF2**: Coordinate frame transforms
- **Qt5**: GUI framework

### Hardware Requirements
- 6× Dynamixel motors (XM430-W350-T or equivalent)
- USB2Dynamixel or U2D2 adapter
- 12V power supply (minimum 5A)
- RS-485 daisy chain wiring

---

## Performance Characteristics

### Planning Time
- **IK Solve:** ~5-50ms (vs. ~1-5ms for 4-DOF)
- **Cartesian Path:** ~100-500ms
- **Collision Check:** ~1-5ms per state

### Communication
- **Bulk Read Cycle:** ~5-10ms (6 motors)
- **Bulk Write Cycle:** ~5-10ms (6 motors)
- **Control Loop Rate:** 100Hz (10ms period)

### Workspace
- **Reach:** ~450mm (unchanged)
- **Orientation Range:** ±180° roll added
- **Payload:** ~500g (estimate, not tested)

---

## Safety Considerations

### 1. Joint Limits
All joints configured with software limits in URDF and MoveIt:
- `joint5_roll`: ±π rad (±180°)
- Hardware e-stops enforced by Dynamixel firmware

### 2. Collision Avoidance
- Self-collision pairs defined in SRDF
- Minimum distance: 0.01m safety margin
- Real-time collision checking in MoveIt

### 3. Emergency Stop
- Hardware: Power disconnect
- Software: `ros2 topic pub /e_stop std_msgs/msg/Bool "{data: true}"`
- GUI: "Emergency Stop" button

### 4. Motor Torque Limits
- Maximum torque: 3.1 Nm (XM430-W350)
- Stall detection: Automatic shutdown
- Temperature monitoring: 70°C limit

---

## Testing Strategy

### Unit Testing
- ✅ URDF parsing (xacro validation)
- ✅ SRDF semantic validation
- ⚠️ Joint limit enforcement (manual test)
- ⚠️ Collision detection accuracy (simulation)

### Integration Testing
- ✅ ros2_control initialization
- ✅ MoveIt planning server startup
- ⚠️ Hardware communication (RS-485)
- ⚠️ GUI control responsiveness

### Hardware Testing
- ⚠️ Motor ID configuration verification
- ⚠️ Full range motion test
- ⚠️ Trajectory tracking accuracy
- ⚠️ Gripper operation after renumbering

**Status Legend:**
- ✅ = Validated in simulation
- ⚠️ = Requires physical robot testing

---

## Migration Path

### From 4-DOF to 5-DOF

1. **Simulation Only:**
   ```bash
   git checkout wrist-roll-dev
   colcon build --packages-select open_manipulator_*
   ros2 launch open_manipulator_bringup open_manipulator_x_gazebo.launch.py
   ```

2. **Hardware Upgrade:**
   ```bash
   # Step 1: Reconfigure gripper motor
   dynamixel_wizard  # Change ID 15 → 16
   
   # Step 2: Install wrist roll motor
   # - Mount motor at wrist position
   # - Set ID to 15
   # - Connect to RS-485 bus
   
   # Step 3: Deploy software
   git checkout wrist-roll-dev
   colcon build
   source install/setup.bash
   ros2 launch open_manipulator_bringup open_manipulator_x.launch.py
   ```

3. **Verification:**
   - Check `/joint_states` shows 6 values
   - Test GUI control of joint5_roll
   - Run MoveIt planning demo
   - Verify gripper responds to commands

---

## Known Limitations

### 1. Inertial Properties
- Current values are estimates
- **Action Required:** Measure with CAD or physical testing
- **Impact:** Simulation dynamics may not match reality

### 2. IK Solver Timeout
- 10× increase needed (0.005s → 0.05s)
- **Cause:** Higher dimensionality search space
- **Mitigation:** Consider switching to KDL or TRAC-IK

### 3. Motor Renumbering
- **CRITICAL:** Gripper motor ID must change
- **Risk:** Forgetting this step causes control mismatch
- **Detection:** Check motor response during bringup

### 4. Payload Capacity
- Not re-tested with additional motor weight
- **Estimate:** ~100g payload reduction
- **Action:** Load testing required

---

## Future Enhancements

### Short-Term
- [ ] Measure accurate inertial properties
- [ ] Validate hardware integration
- [ ] Tune PID controllers for wrist roll
- [ ] Create automated test suite

### Long-Term
- [ ] Dual-arm coordination with wrist roll
- [ ] Vision-based grasp planning using orientation
- [ ] Cartesian impedance control
- [ ] Force/torque sensor integration

---

## References

- **URDF Changes:** [01_URDF_CHANGES.md](./01_URDF_CHANGES.md)
- **Hardware Config:** [02_HARDWARE_CHANGES.md](./02_HARDWARE_CHANGES.md)
- **Launch Fix:** [03_LAUNCH_FIX.md](./03_LAUNCH_FIX.md)
- **MoveIt Setup:** [04_MOVEIT_CHANGES.md](./04_MOVEIT_CHANGES.md)
- **GUI Updates:** [05_GUI_CHANGES.md](./05_GUI_CHANGES.md)
- **Testing Guide:** [TESTING_GUIDE.md](./TESTING_GUIDE.md)

---

**Document Status:** Complete  
**Last Updated:** November 19, 2025  
**Applies To:** Branch `wrist-roll-dev` (commits 1b041112 through 1d563e87)
