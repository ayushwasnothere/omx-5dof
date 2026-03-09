# Testing Guide: 5-DOF Wrist Roll Modification

## Overview

This guide provides step-by-step testing procedures for validating the wrist roll modification in both simulation and hardware environments.

---

## Prerequisites

### Software Requirements
```bash
# ROS 2 Distribution
ROS_DISTRO=jazzy

# Required Packages
sudo apt install ros-${ROS_DISTRO}-ros2-control
sudo apt install ros-${ROS_DISTRO}-ros2-controllers
sudo apt install ros-${ROS_DISTRO}-moveit
sudo apt install ros-${ROS_DISTRO}-gazebo-ros-pkgs
sudo apt install ros-${ROS_DISTRO}-dynamixel-hardware-interface

# Build Tools
sudo apt install python3-colcon-common-extensions
```

### Workspace Setup
```bash
# Clone repository
cd ~/robotics/omx_vnc_ws/src
git clone https://github.com/iamak008/open_manipulator_x.git
cd open_manipulator_x
git checkout wrist-roll-dev

# Build workspace
cd ~/robotics/omx_vnc_ws
colcon build --symlink-install --packages-select \
  open_manipulator \
  open_manipulator_description \
  open_manipulator_bringup \
  open_manipulator_moveit_config \
  open_manipulator_gui \
  open_manipulator_teleop

# Source workspace
source install/setup.bash
```

---

## Test Suite

## Phase 1: Static Validation

### Test 1.1: URDF Syntax Check
**Purpose:** Verify URDF/xacro files parse without errors

```bash
# Check URDF processing
cd ~/robotics/omx_vnc_ws
xacro src/open_manipulator_x/open_manipulator_description/urdf/open_manipulator_x_arm.urdf.xacro > /tmp/robot.urdf

# Validate output
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat /tmp/robot.urdf)"
```

**Expected Output:**
```
[robot_state_publisher-1] [INFO] [1234567890.123]: got segment joint1
[robot_state_publisher-1] [INFO] [1234567890.124]: got segment joint2
[robot_state_publisher-1] [INFO] [1234567890.125]: got segment joint3
[robot_state_publisher-1] [INFO] [1234567890.126]: got segment joint4
[robot_state_publisher-1] [INFO] [1234567890.127]: got segment joint5_roll
[robot_state_publisher-1] [INFO] [1234567890.128]: got segment gripper_joint
```

**Pass Criteria:**
- ✅ No parsing errors
- ✅ 6 joints detected (joint1-4, joint5_roll, gripper_joint)
- ✅ No warnings about missing links

**Troubleshooting:**
- ❌ Error: "Could not parse URDF" → Check xacro syntax in `open_manipulator_x_arm.urdf.xacro`
- ❌ Missing joint5_roll → Verify commit 1b041112 applied

---

### Test 1.2: MoveIt SRDF Validation
**Purpose:** Verify semantic robot description is valid

```bash
# Check SRDF syntax
ros2 run moveit_configs_utils validate_srdf \
  --srdf src/open_manipulator_x/open_manipulator_moveit_config/config/open_manipulator_x.srdf \
  --urdf /tmp/robot.urdf
```

**Expected Output:**
```
Checking planning group 'arm'...
  - joint1 [OK]
  - joint2 [OK]
  - joint3 [OK]
  - joint4 [OK]
  - joint5_roll [OK]
Checking planning group 'gripper'...
  - gripper_joint [OK]
All checks passed!
```

**Pass Criteria:**
- ✅ "arm" group contains 5 joints
- ✅ "gripper" group contains 1 joint
- ✅ No undefined joint references

**Troubleshooting:**
- ❌ "Unknown joint: joint5_roll" → Check commit b1c9a874 applied
- ❌ Group "arm" has wrong size → Verify SRDF joint list

---

### Test 1.3: Joint Limits Verification
**Purpose:** Confirm joint5_roll limits are configured

```bash
# Extract joint limits
grep -A5 "joint5_roll" src/open_manipulator_x/open_manipulator_moveit_config/config/joint_limits.yaml
```

**Expected Output:**
```yaml
joint5_roll:
  has_velocity_limits: true
  max_velocity: 4.8
  has_acceleration_limits: false
  max_acceleration: 0
  has_position_limits: true
  min_position: -3.14159265359
  max_position: 3.14159265359
```

**Pass Criteria:**
- ✅ `min_position: -3.14159265359` (≈-π)
- ✅ `max_position: 3.14159265359` (≈π)
- ✅ Velocity limits defined

---

## Phase 2: Simulation Testing

### Test 2.1: Gazebo Launch
**Purpose:** Verify robot spawns correctly in Gazebo

```bash
# Launch Gazebo simulation
ros2 launch open_manipulator_bringup open_manipulator_x_gazebo.launch.py
```

**Expected Behavior:**
1. Gazebo window opens
2. Robot model loads with 6 links visible
3. No error messages in terminal
4. Robot in home position (all joints at 0.0)

**Pass Criteria:**
- ✅ Wrist roll link visible between link5 and gripper
- ✅ No "Failed to load model" errors
- ✅ `ros2 topic list` shows:
  ```
  /joint_states
  /arm_controller/joint_trajectory
  /gripper_controller/joint_trajectory
  ```

**Troubleshooting:**
- ❌ Missing wrist_roll_link → Check mesh files exist:
  ```bash
  ls src/open_manipulator_x/open_manipulator_description/meshes/chain_link_wrist/
  # Should show: wrist_link.stl, gripper_link_with_bracket.stl
  ```

---

### Test 2.2: Joint State Monitoring
**Purpose:** Verify /joint_states publishes 6 values

```bash
# In separate terminal, monitor joint states
ros2 topic echo /joint_states --once
```

**Expected Output:**
```yaml
name:
  - joint1
  - joint2
  - joint3
  - joint4
  - joint5_roll
  - gripper_joint
position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

**Pass Criteria:**
- ✅ 6 joint names present
- ✅ joint5_roll appears in list
- ✅ Arrays have length 6

**Troubleshooting:**
- ❌ Only 5 values → Check ros2_control config (commit 7527c79e)

---

### Test 2.3: Manual Joint Control (Gazebo)
**Purpose:** Test individual joint5_roll motion

```bash
# Send joint5_roll command
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5_roll'],
  points: [
    {
      positions: [0.0, 0.0, 0.0, 0.0, 1.57],
      time_from_start: {sec: 2}
    }
  ]
}"
```

**Expected Behavior:**
- Wrist rotates 90° (1.57 rad) over 2 seconds
- Other joints remain stationary
- No collision warnings

**Pass Criteria:**
- ✅ Smooth motion without jerking
- ✅ Final position matches command (check with `/joint_states`)
- ✅ No "Joint limit exceeded" warnings

**Test Variations:**
```bash
# Test full positive range
ros2 topic pub --once /arm_controller/joint_trajectory ... positions: [0, 0, 0, 0, 3.14]

# Test full negative range
ros2 topic pub --once /arm_controller/joint_trajectory ... positions: [0, 0, 0, 0, -3.14]

# Test gripper still works
ros2 topic pub --once /gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['gripper_joint'],
  points: [{positions: [0.01], time_from_start: {sec: 1}}]
}"
```

---

### Test 2.4: MoveIt Planning (Simulation)
**Purpose:** Verify MoveIt can plan with 5 joints

```bash
# Launch MoveIt + Gazebo
ros2 launch open_manipulator_moveit_config demo.launch.py use_sim:=true
```

**In RViz:**
1. **Planning Tab:**
   - Select "arm" planning group
   - Verify "Query Goal State" shows 5 sliders (joint1-4 + joint5_roll)

2. **Plan Random Valid:**
   - Click "Planning" → "Select Goal State" → "Random Valid"
   - Click "Plan"
   - Observe trajectory in RViz
   - Verify joint5_roll changes in planned path

3. **Execute:**
   - Click "Execute"
   - Watch Gazebo robot follow trajectory
   - Confirm wrist roll motion occurs

**Pass Criteria:**
- ✅ Planning succeeds (green path in RViz)
- ✅ Execution completes without errors
- ✅ joint5_roll actively participates in motion
- ✅ No collision warnings

**Troubleshooting:**
- ❌ "Planning failed" every time → Check IK timeout in `kinematics.yaml` (should be 0.05s)
- ❌ joint5_roll slider missing → Verify SRDF "arm" group

---

### Test 2.5: GUI Control (Simulation)
**Purpose:** Test Qt GUI with wrist roll spinbox

```bash
# Launch GUI + Gazebo
ros2 launch open_manipulator_gui open_manipulator_x_gui.launch.py use_sim:=true
```

**In GUI Window:**
1. **Visual Check:**
   - Verify 6 rows in joint control section:
     - Joint 1, 2, 3, 4
     - **Joint 5 roll** ← NEW
     - Gripper

2. **Spinbox Test:**
   - Set "Joint 5 roll" spinbox to 1.57
   - Click "Set" button
   - Observe wrist rotation in RViz/Gazebo
   - Verify spinbox updates with actual position

3. **Slider Test:**
   - Drag "Joint 5 roll" slider left/right
   - Confirm spinbox value changes
   - Click "Set" to apply

4. **Combined Motion:**
   - Set Joint 1: 0.5
   - Set Joint 2: 0.3
   - Set Joint 5 roll: 1.0
   - Click "Set"
   - Confirm all joints move together

**Pass Criteria:**
- ✅ GUI shows 6 joint controls
- ✅ Set button sends wrist roll commands
- ✅ Spinbox displays current joint5_roll state
- ✅ Slider syncs with spinbox

**Troubleshooting:**
- ❌ Only 5 rows visible → Check commit 1d563e87 applied
- ❌ Set button does nothing → Verify Qt signal-slot connection

---

## Phase 3: Hardware Testing

⚠️ **WARNING:** Requires physical robot with wrist roll motor installed

### Test 3.1: Motor ID Configuration
**Purpose:** Verify Dynamixel motors have correct IDs

```bash
# Install Dynamixel Wizard
sudo apt install dynamixel-workbench-toolbox

# Scan bus
ros2 run dynamixel_workbench_toolbox scan \
  --ros-args -p usb_port:=/dev/ttyUSB0 -p baud_rate:=1000000
```

**Expected Output:**
```
Scanning Dynamixel motors...
Found motor ID: 11 (Model: XM430-W350)
Found motor ID: 12 (Model: XM430-W350)
Found motor ID: 13 (Model: XM430-W350)
Found motor ID: 14 (Model: XM430-W350)
Found motor ID: 15 (Model: XM430-W350) ← Wrist roll
Found motor ID: 16 (Model: XM430-W350) ← Gripper (renumbered!)
```

**Pass Criteria:**
- ✅ 6 motors detected
- ✅ IDs 11-16 present
- ✅ No duplicate IDs

**Troubleshooting:**
- ❌ ID 15 missing → Wrist roll motor not configured
- ❌ ID 15 shows twice → Gripper motor not renumbered!
  ```bash
  # Reconfigure gripper motor
  ros2 run dynamixel_workbench_toolbox id_changer \
    --ros-args -p usb_port:=/dev/ttyUSB0 -p baud_rate:=1000000 \
    -p old_id:=15 -p new_id:=16
  ```

---

### Test 3.2: Hardware Bringup
**Purpose:** Launch ros2_control with real hardware

```bash
# Launch robot
ros2 launch open_manipulator_bringup open_manipulator_x.launch.py
```

**Expected Output:**
```
[controller_manager] Successfully loaded controller 'arm_controller'
[controller_manager] Successfully loaded controller 'gripper_controller'
[controller_manager] Successfully loaded controller 'joint_state_broadcaster'
[dynamixel_hardware] Connected to 6 motors: [11, 12, 13, 14, 15, 16]
[dynamixel_hardware] Initialized bulk read/write
```

**Pass Criteria:**
- ✅ No "Failed to connect" errors
- ✅ 6 motors reported
- ✅ `/joint_states` publishes at ~100Hz

**Troubleshooting:**
- ❌ "Timeout waiting for motor 15" → Check wiring
- ❌ "Motor 16 not found" → Gripper not renumbered
- ❌ Permission denied on /dev/ttyUSB0:
  ```bash
  sudo usermod -aG dialout $USER
  # Log out and back in
  ```

---

### Test 3.3: Safe Range of Motion Test
**Purpose:** Verify hardware joint limits are safe

```bash
# Test positive direction slowly
ros2 topic pub --rate 10 /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5_roll'],
  points: [{positions: [0, 0, 0, 0, 0.5], time_from_start: {sec: 3}}]
}"
```

**Monitoring:**
```bash
# In separate terminal, watch joint states
ros2 topic echo /joint_states | grep -A6 "name:"
```

**Test Sequence:**
1. Small positive: 0.5 rad (28°)
2. Medium positive: 1.57 rad (90°)
3. Large positive: 3.0 rad (172°)
4. Return to zero: 0.0 rad
5. Small negative: -0.5 rad
6. Medium negative: -1.57 rad
7. Large negative: -3.0 rad
8. Return to zero

**Pass Criteria:**
- ✅ No mechanical binding at any position
- ✅ Motor temperature stays below 50°C
- ✅ No unusual noise or vibration
- ✅ Position error < 0.05 rad

**Safety Notes:**
- ⚠️ Keep hand near emergency stop button
- ⚠️ Start with small motions (±0.5 rad)
- ⚠️ Check for cable interference at extremes
- ⚠️ Stop if motor makes grinding noise

---

### Test 3.4: MoveIt Hardware Integration
**Purpose:** Verify MoveIt planning works with real robot

```bash
# Launch MoveIt + Hardware
ros2 launch open_manipulator_moveit_config demo.launch.py use_sim:=false
```

**In RViz:**
1. **Current State:**
   - Move robot manually (if compliant mode supported)
   - Verify RViz model updates in real-time
   - Check joint5_roll value changes

2. **Plan to Goal:**
   - Set goal state with joint5_roll = 1.0
   - Click "Plan"
   - Review trajectory (should be smooth)

3. **Execute with Caution:**
   - Clear workspace around robot
   - Click "Execute"
   - **Be ready to emergency stop**
   - Verify motion is smooth and accurate

**Pass Criteria:**
- ✅ Planning succeeds
- ✅ Trajectory is collision-free
- ✅ Execution completes without jerking
- ✅ Final position within 0.1 rad of goal

---

### Test 3.5: GUI Hardware Test
**Purpose:** Manual control via GUI on real robot

```bash
# Launch GUI for hardware
ros2 launch open_manipulator_gui open_manipulator_x_gui.launch.py use_sim:=false
```

**Test Procedure:**
1. **Initial State:**
   - Verify all spinboxes show current positions
   - Joint 5 roll should display actual wrist angle

2. **Single Joint Test:**
   - Set only Joint 5 roll to 0.5
   - Click "Set"
   - Verify only wrist rotates (other joints stationary)

3. **Full Trajectory:**
   - Set all joints to safe positions:
     - Joint 1: 0.0
     - Joint 2: -0.5
     - Joint 3: 0.3
     - Joint 4: 0.2
     - Joint 5 roll: 1.0
   - Click "Set"
   - Observe coordinated motion

4. **CSV Logging:**
   - Enable CSV recording
   - Move through several poses
   - Stop recording
   - Verify CSV file has 6 columns

**Pass Criteria:**
- ✅ All joints respond to commands
- ✅ Wrist roll operates independently
- ✅ CSV log includes joint5_roll column
- ✅ No communication errors in terminal

---

### Test 3.6: Gripper Verification
**Purpose:** Confirm gripper works after renumbering to ID 16

```bash
# Open gripper
ros2 topic pub --once /gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['gripper_joint'],
  points: [{positions: [0.01], time_from_start: {sec: 1}}]
}"

# Close gripper
ros2 topic pub --once /gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['gripper_joint'],
  points: [{positions: [-0.01], time_from_start: {sec: 1}}]
}"
```

**Pass Criteria:**
- ✅ Gripper opens/closes smoothly
- ✅ No "Motor 15 not responding" errors
- ✅ Gripper state shown in GUI
- ✅ Gripper independent from arm motions

**Troubleshooting:**
- ❌ Gripper doesn't respond → Motor still at ID 15, needs reconfiguration

---

## Phase 4: Integration Testing

### Test 4.1: Combined Arm + Gripper Sequence
**Purpose:** Verify coordinated control of all 6 motors

```bash
# Test sequence script
ros2 run open_manipulator_teleop test_sequence.py
```

**Sequence:**
1. Move to home position [0, 0, 0, 0, 0]
2. Rotate base and wrist: [1.0, 0, 0, 0, 1.5]
3. Open gripper
4. Extend arm: [1.0, -0.5, 0.3, 0.2, 1.5]
5. Close gripper
6. Rotate wrist: [1.0, -0.5, 0.3, 0.2, -1.5]
7. Return home

**Pass Criteria:**
- ✅ All transitions smooth
- ✅ Gripper operates at any arm pose
- ✅ Wrist roll doesn't affect gripper
- ✅ No self-collisions

---

### Test 4.2: Stress Test
**Purpose:** Verify reliability under continuous operation

```bash
# Run continuous random motion for 100 cycles
ros2 launch open_manipulator_bringup stress_test.launch.py cycles:=100
```

**Monitoring:**
- Motor temperatures (should stay < 60°C)
- Communication errors (should be 0)
- Position tracking error (should be < 0.1 rad)
- Execution time per cycle (should be consistent)

**Pass Criteria:**
- ✅ Completes all cycles without errors
- ✅ No motor overheating
- ✅ Position error remains bounded
- ✅ No communication timeouts

---

## Test Results Template

```markdown
# Test Results - Wrist Roll Modification

**Date:** YYYY-MM-DD
**Tester:** [Your Name]
**Robot Serial:** [If applicable]

## Environment
- [ ] Simulation (Gazebo)
- [ ] Hardware

## Phase 1: Static Validation
- [ ] URDF syntax check: PASS / FAIL
- [ ] SRDF validation: PASS / FAIL
- [ ] Joint limits verification: PASS / FAIL

## Phase 2: Simulation
- [ ] Gazebo launch: PASS / FAIL
- [ ] Joint state monitoring: PASS / FAIL
- [ ] Manual joint control: PASS / FAIL
- [ ] MoveIt planning: PASS / FAIL
- [ ] GUI control: PASS / FAIL

## Phase 3: Hardware (if applicable)
- [ ] Motor ID configuration: PASS / FAIL
- [ ] Hardware bringup: PASS / FAIL
- [ ] Range of motion: PASS / FAIL
- [ ] MoveIt integration: PASS / FAIL
- [ ] GUI hardware test: PASS / FAIL
- [ ] Gripper verification: PASS / FAIL

## Phase 4: Integration
- [ ] Combined sequence: PASS / FAIL
- [ ] Stress test: PASS / FAIL

## Issues Found
1. [Description]
2. [Description]

## Notes
[Any observations or recommendations]
```

---

## Troubleshooting Reference

### Common Issues

#### Issue 1: "Parameter robot_description has wrong type"
**Symptoms:** Launch error about YAML parsing  
**Cause:** Missing ParameterValue wrapper (ROS 2 Jazzy)  
**Solution:** Verify commit df57608a applied  
**Reference:** [03_LAUNCH_FIX.md](./03_LAUNCH_FIX.md)

---

#### Issue 2: Gripper doesn't respond
**Symptoms:** `/gripper_controller` commands have no effect  
**Cause:** Motor still at ID 15 (not renumbered to 16)  
**Solution:**
```bash
ros2 run dynamixel_workbench_toolbox id_changer \
  --ros-args -p old_id:=15 -p new_id:=16
```
**Reference:** [02_HARDWARE_CHANGES.md](./02_HARDWARE_CHANGES.md)

---

#### Issue 3: MoveIt planning always fails
**Symptoms:** "Planning failed" in RViz  
**Cause:** IK timeout too short for 5-DOF  
**Solution:** Check `kinematics.yaml` has `timeout: 0.05` (not 0.005)  
**Reference:** [04_MOVEIT_CHANGES.md](./04_MOVEIT_CHANGES.md)

---

#### Issue 4: Joint 5 roll not in GUI
**Symptoms:** Only 5 spinboxes visible (should be 6)  
**Cause:** GUI not updated for wrist roll  
**Solution:** Verify commit 1d563e87 applied, rebuild workspace  
**Reference:** [05_GUI_CHANGES.md](./05_GUI_CHANGES.md)

---

#### Issue 5: Wrist link missing in RViz/Gazebo
**Symptoms:** Visual gap between link5 and gripper  
**Cause:** Missing mesh files  
**Solution:**
```bash
ls src/open_manipulator_x/open_manipulator_description/meshes/chain_link_wrist/
# Should show: wrist_link.stl, gripper_link_with_bracket.stl
```
**Reference:** [01_URDF_CHANGES.md](./01_URDF_CHANGES.md)

---

## Safety Checklist

Before hardware testing:
- [ ] Emergency stop button within reach
- [ ] Workspace clear of obstacles
- [ ] Motor temperatures monitored
- [ ] Power supply adequate (12V, 5A minimum)
- [ ] RS-485 wiring secure
- [ ] Motor IDs verified (scan test passed)
- [ ] Software limits configured (±π rad)
- [ ] Collision detection enabled in MoveIt
- [ ] Observer present for first tests

---

## Success Criteria Summary

### Simulation
✅ **Minimum Requirements:**
- URDF parses without errors
- 6 joints visible in `/joint_states`
- MoveIt can plan with joint5_roll
- GUI shows wrist roll control
- Gazebo simulation runs without crashes

### Hardware
✅ **Minimum Requirements:**
- 6 motors detected (IDs 11-16)
- Wrist roll responds to commands
- Gripper operates independently
- No communication errors
- Position tracking accurate (< 0.1 rad error)
- Motor temperatures safe (< 60°C)

---

**Last Updated:** November 19, 2025  
**Applies To:** Branch `wrist-roll-dev`  
**Related:** [TECHNICAL_OVERVIEW.md](./TECHNICAL_OVERVIEW.md)
