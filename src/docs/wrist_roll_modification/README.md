# Open Manipulator X - Wrist Roll Joint Addition

## Overview

This documentation covers the complete modification process of adding a **6th degree of freedom (wrist roll joint)** to the Open Manipulator X robot arm, upgrading it from a **4-DOF + gripper** to a **5-DOF + gripper** configuration.

**Branch:** `wrist-roll-dev`  
**Base Commit:** `8be8e2f55fcf42a6ddaf7eab2e6302ff93749795`  
**Date:** November 19, 2025

## Modification Summary

### Hardware Changes
- Split original `link5.stl` into two separate mesh files
- Added new Dynamixel motor (ID 15) for wrist roll capability
- Created custom bracket design to mount the roll motor

### Software Changes
- **5 commits** spanning multiple ROS 2 packages
- Modified URDF, ros2_control, MoveIt, and GUI configurations
- Added proper collision detection and joint limits

## Repository Structure

```
docs/wrist_roll_modification/
├── README.md                          # This file - Overview
├── 01_URDF_CHANGES.md                 # Commit 1b041112 - Description package
├── 02_HARDWARE_CHANGES.md             # Commit 7527c79e - Bringup & ros2_control
├── 03_LAUNCH_FIX.md                   # Commit df57608a - Launch file fix
├── 04_MOVEIT_CHANGES.md               # Commit b1c9a874 - MoveIt configuration
├── 05_GUI_CHANGES.md                  # Commit 1d563e87 - GUI additions
├── TECHNICAL_OVERVIEW.md              # Technical deep-dive
└── TESTING_GUIDE.md                   # Testing procedures
```

## Commit History

| Commit | Date | Description | Files Changed |
|--------|------|-------------|---------------|
| `1b041112` | Nov 19, 2025 | feat: wrist roll urdf (todo: correct inertial properties) | 6 files |
| `7527c79e` | Nov 19, 2025 | feat: wrist roll hardware changes | 3 files |
| `df57608a` | Nov 19, 2025 | fix: type errors in launch | 1 file |
| `b1c9a874` | Nov 19, 2025 | feat: wrist roll move it config changes | 4 files |
| `1d563e87` | Nov 19, 2025 | feat: wrist roll gui additions | 4 files |

**Total Changes:** 18 files modified, 2 new STL meshes added

## Key Features Added

✅ **New Joint:** `joint5_roll` - Revolute joint about X-axis  
✅ **Range:** ±180° (full rotation capability)  
✅ **Motor:** Dynamixel ID 15 (position control mode)  
✅ **Gripper Motor:** Renumbered from ID 15 → ID 16  
✅ **Simulation:** Full Gazebo support  
✅ **Motion Planning:** MoveIt integration with 5-DOF planning  
✅ **GUI Control:** Interactive control in Qt GUI  

## Quick Links

- [Detailed URDF Changes](./01_URDF_CHANGES.md)
- [Hardware Integration](./02_HARDWARE_CHANGES.md)
- [Launch File Fix](./03_LAUNCH_FIX.md)
- [MoveIt Configuration](./04_MOVEIT_CHANGES.md)
- [GUI Updates](./05_GUI_CHANGES.md)
- [Technical Overview](./TECHNICAL_OVERVIEW.md)
- [Testing Guide](./TESTING_GUIDE.md)

## Building and Testing

```bash
# Build the modified packages
cd ~/robotics/omx_vnc_ws
colcon build --packages-select \
  open_manipulator_description \
  open_manipulator_bringup \
  open_manipulator_moveit_config \
  open_manipulator_gui

# Source the workspace
source install/setup.bash

# Launch (simulation)
ros2 launch open_manipulator_bringup open_manipulator_x_gazebo.launch.py

# Launch (hardware - requires physical robot with 6 motors)
ros2 launch open_manipulator_bringup open_manipulator_x.launch.py
```

## Hardware Requirements

### For Physical Robot
- **6 Dynamixel Motors** (previously 5):
  - ID 11: Base (joint1)
  - ID 12: Shoulder (joint2)
  - ID 13: Elbow (joint3)
  - ID 14: Wrist Pitch (joint4)
  - **ID 15: Wrist Roll (joint5_roll)** ← NEW
  - **ID 16: Gripper** ← Changed from ID 15

### Mesh Files
- `wrist_link.stl` - Wrist base with motor housing
- `gripper_link_with_bracket.stl` - Gripper base with custom bracket

## Authors

- **Akash Konda** - Wrist roll modification
- Original Open Manipulator X by ROBOTIS

## License

Apache License 2.0 (same as Open Manipulator X)

---

**Next:** Read [01_URDF_CHANGES.md](./01_URDF_CHANGES.md) for detailed URDF modifications
