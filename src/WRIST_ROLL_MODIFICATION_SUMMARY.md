# Wrist Roll Modification Summary

## Overview
This document summarizes the modifications made to add a proper wrist roll mechanism with dedicated mesh files and a new Dynamixel motor to the Open Manipulator X robot.

## Changes Made

### 1. URDF Changes (`open_manipulator_description/urdf/open_manipulator_x/open_manipulator_x_arm.urdf.xacro`)

#### Link5 (Wrist Base)
- **Old mesh**: `link5.stl` (combined wrist and gripper)
- **New mesh**: `wrist_link.stl` (separated wrist base with motor housing)
- **Purpose**: Houses the wrist roll Dynamixel motor
- **Updated inertial properties**:
  - Mass: 0.085 kg
  - COM: [0.025, 0, 0] m

#### Joint5_roll (Wrist Roll Joint)
- **Type**: Revolute joint
- **Axis**: X-axis (roll about arm longitudinal axis)
- **Range**: ±150° (±2.618 rad)
- **Parent**: link5
- **Child**: wrist_roll_link
- **Origin**: [0.0335, 0, 0] m from link5
- **Velocity limit**: 4.8 rad/s
- **Effort limit**: 1000 (scaled for position control)

#### Wrist_roll_link (Gripper Base with Bracket)
- **Old geometry**: Simple cylinder placeholder
- **New mesh**: `gripper_link_with_bracket.stl` 
- **Purpose**: Gripper mounting base with custom bracket design
- **Updated inertial properties**:
  - Mass: 0.065 kg
  - COM: [0.045, 0, 0] m

#### Gripper Joint Origins
- **Old X-offset**: 0.0817 m (from old link5)
- **New X-offset**: 0.0482 m (from wrist_roll_link)
- **Reason**: Accounting for the split between wrist_link and gripper_link_with_bracket

#### End Effector Frame
- **Old X-offset**: 0.126 m
- **New X-offset**: 0.0925 m
- **Total TCP distance from link4**: ~0.250 m (unchanged in practice)

### 2. ROS2 Control Changes (`open_manipulator_description/ros2_control/open_manipulator_x_position.ros2_control.xacro`)

#### Hardware Interface Configuration
- **Number of joints**: 5 → **6**
- **Number of transmissions**: 5 → **6**
- **Transmission matrices**: Updated from 5x5 to 6x6 identity matrices

#### Dynamixel Motor Mappings
- **dxl1** (ID 11): joint1 (base rotation)
- **dxl2** (ID 12): joint2 (shoulder)
- **dxl3** (ID 13): joint3 (elbow)
- **dxl4** (ID 14): joint4 (wrist pitch)
- **dxl5** (ID 15): **joint5_roll** (wrist roll) - **NEW**
  - Operating Mode: 3 (Position Control)
  - P Gain: 800, I Gain: 100, D Gain: 100
  - Profile Velocity: 20, Profile Acceleration: 10
  - Drive Mode: 0 (normal direction)
- **dxl6** (ID 16): gripper_left_joint (previously dxl5, ID 15)
  - Operating Mode: 5 (Current-based position control)
  - Goal Current: 200

#### Gripper Control
- **Revolute-to-prismatic conversion**: Now uses dxl6 (was dxl5)
- Gripper parameters unchanged

### 3. MoveIt Configuration Changes

#### SRDF (`open_manipulator_moveit_config/config/open_manipulator_x/open_manipulator_x.srdf`)
- **Arm group**: Now includes joint5_roll
- **Init pose**: Added joint5_roll = 0
- **Home pose**: Added joint5_roll = 0
- **Collision matrix updates**:
  - Added wrist_roll_link adjacency with link5
  - Updated gripper links to reference wrist_roll_link instead of link5
  - Added wrist_roll_link to end_effector_link adjacency

#### Joint Limits (`open_manipulator_moveit_config/config/open_manipulator_x/joint_limits.yaml`)
- Already configured with joint5_roll limits:
  - Position: -2.618 to 2.618 rad
  - Velocity: 5.0 rad/s (with 0.1 scaling factor)
  - Acceleration: 5.0 rad/s² (with 0.1 scaling factor)

### 4. Controller Configuration (`open_manipulator_bringup/config/open_manipulator_x/`)

#### hardware_controller_manager.yaml
- **arm_controller joints**: Includes joint5_roll (5 joints total)
  - joint1, joint2, joint3, joint4, joint5_roll
- **gripper_controller**: Unchanged (gripper_left_joint)

#### initial_positions.yaml
- **joint_names**: Includes joint5_roll
- **home position**: [0.0, -1.0, 1.0, 0.0, 0.0] (joint5_roll = 0.0)

### 5. Gazebo Simulation (`open_manipulator_description/gazebo/open_manipulator_x.gazebo.xacro`)
- Already configured with:
  - wrist_roll_link physics properties
  - joint5_roll transmission (trans_roll, actuator_roll)
  - No changes needed

### 6. GUI (`open_manipulator_gui/src/open_manipulator_x/`)
- Already handles 6 DOF:
  - Joint indices 0-4 for arm (including joint5_roll at index 4)
  - Joint index 5 for gripper
- Column headers already include "Joint 5 roll"
- No changes needed

## Mesh Files Required

The following STL files must be present in `open_manipulator_description/meshes/open_manipulator_x/`:

1. **wrist_link.stl** - Wrist base with Dynamixel motor housing
2. **gripper_link_with_bracket.stl** - Gripper base with custom mounting bracket
3. **gripper_left_palm.stl** - Left gripper finger (unchanged)
4. **gripper_right_palm.stl** - Right gripper finger (unchanged)

## Physical Implementation Notes

### Dynamixel Motor Configuration
- **Motor Model**: Recommend XM430-W210 or XM430-W350
- **ID Assignment**: Set to 15 for wrist roll
- **Baud Rate**: 1000000
- **Operating Mode**: Position Control (Mode 3)
- **Mounting**: Between wrist_link and gripper_link_with_bracket

### Hardware Assembly
1. Link5 (wrist_link) attaches to link4 end
2. Dynamixel motor (ID 15) mounts to wrist_link
3. Motor shaft connects to wrist_roll_link (gripper base with bracket)
4. Gripper servos attach to wrist_roll_link
5. Gripper fingers attach to gripper servos

### Calibration
- Ensure joint5_roll zero position aligns gripper fingers with link4 orientation
- Verify gripper joint offsets (0.0482 m) match physical assembly
- Confirm end effector TCP position is accurate for your application

## Testing Checklist

- [ ] Build workspace: `colcon build`
- [ ] Source workspace: `source install/setup.bash`
- [ ] Launch robot: `ros2 launch open_manipulator_bringup open_manipulator_x.launch.py`
- [ ] Verify 6 joints appear in joint_states
- [ ] Test joint5_roll motion range (±150°)
- [ ] Verify gripper open/close operation
- [ ] Test MoveIt motion planning with new DOF
- [ ] Verify end effector pose accuracy
- [ ] Check collision detection with wrist_roll_link

## Files Modified

1. `/open_manipulator_description/urdf/open_manipulator_x/open_manipulator_x_arm.urdf.xacro`
2. `/open_manipulator_description/ros2_control/open_manipulator_x_position.ros2_control.xacro`
3. `/open_manipulator_moveit_config/config/open_manipulator_x/open_manipulator_x.srdf`

## Files Already Configured (No Changes Needed)

1. `/open_manipulator_bringup/config/open_manipulator_x/hardware_controller_manager.yaml`
2. `/open_manipulator_bringup/config/open_manipulator_x/initial_positions.yaml`
3. `/open_manipulator_moveit_config/config/open_manipulator_x/joint_limits.yaml`
4. `/open_manipulator_description/gazebo/open_manipulator_x.gazebo.xacro`
5. GUI source files (already support 6 DOF)

## Additional Recommendations

### Inertial Properties
The inertial values for wrist_link and wrist_roll_link are estimates. For better simulation accuracy:
1. Measure actual component masses
2. Use CAD software to calculate accurate inertia tensors
3. Update URDF inertial blocks accordingly

### Joint Origins
The joint origin offsets (0.0335 m for joint5_roll, 0.0482 m for grippers) are based on typical Dynamixel dimensions. Measure your actual assembly and update if needed.

### Drive Mode
dxl5 uses Drive Mode 0 (normal). If your motor is mounted in reverse orientation, change to Drive Mode 1.

## Conclusion

All necessary changes have been implemented to support the new wrist roll mechanism with proper mesh files. The system now has:
- 6 Dynamixel motors (IDs 11-16)
- 6 controllable joints (4 arm + 1 wrist roll + 1 gripper)
- Proper mesh visualization for wrist_link and gripper_link_with_bracket
- Full MoveIt integration with the new DOF
- Gazebo simulation support

The robot is ready for testing once the Dynamixel motor (ID 15) is physically installed and the mesh files are confirmed to be in place.
