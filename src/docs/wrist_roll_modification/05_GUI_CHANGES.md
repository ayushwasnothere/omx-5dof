# Commit 5: GUI Changes for Wrist Roll Control

**Commit:** `1d563e87` - "feat: wrist roll gui additions"  
**Date:** November 19, 2025  
**Package:** `open_manipulator_gui`

## Overview

This commit adds interactive wrist roll control to the Qt-based GUI, including spinbox input, slider control, and position display for the new joint5_roll.

## Files Modified

### 1. UI File: `open_manipulator_x_main_window.ui`
- Added SpinBox widget for joint5_roll input
- Added horizontal slider for visual control
- Inserted into existing joint control grid layout
- Row added between Joint 4 and Gripper rows

**New UI Elements:**
- `label_joint_5_roll`: "Joint 5 roll"
- `doubleSpinBox_joint5_roll`: Value input (-3.14 to 3.14 rad)
- `horizontalSlider_joint_5_roll`: Slider control
- `btn_set_joint5_roll`: "Set" button to send command

### 2. Header: `main_window.hpp`
Added Qt slot declarations:
```cpp
void on_btn_set_joint5_roll_clicked(void);
void on_horizontalSlider_joint_5_roll_valueChanged(int value);
```

### 3. Implementation: `main_window.cpp`

#### Change 1: Table Header Update
```cpp
// BEFORE
headers << "Joint 1" << "Joint 2" << "Joint 3" << "Joint 4" << "Gripper" << "Status";

// AFTER  
headers << "Joint 1" << "Joint 2" << "Joint 3" << "Joint 4" << "Joint 5 roll" << "Gripper" << "Status";
```

**Why:** Adds column header for wrist roll position display in joint status table.

---

#### Change 2: Joint Value Display Update
```cpp
// Updates display with current joint values
void MainWindow::updateJointSpinbox(std::vector<double> joint_angle)
{
  if (joint_angle.size() >= 6) {  // Was 5, now 6
    ui.doubleSpinBox_joint1->setValue(joint_angle.at(0));
    ui.doubleSpinBox_joint2->setValue(joint_angle.at(1));
    ui.doubleSpinBox_joint3->setValue(joint_angle.at(2));
    ui.doubleSpinBox_joint4->setValue(joint_angle.at(3));
    ui.doubleSpinBox_joint5_roll->setValue(joint_angle.at(4));  // NEW
    ui.doubleSpinBox_gripper->setValue(joint_angle.at(5));      // Was index 4
  }
}
```

**Why:** Displays current wrist roll position from `/joint_states` topic.

---

#### Change 3: Set Joint5 Roll Button Handler
```cpp
void MainWindow::on_btn_set_joint5_roll_clicked(void)
{
  std::vector<double> joint_angle;
  // Get current values for joints 1-4
  joint_angle.push_back(ui.doubleSpinBox_joint1->value());
  joint_angle.push_back(ui.doubleSpinBox_joint2->value());
  joint_angle.push_back(ui.doubleSpinBox_joint3->value());
  joint_angle.push_back(ui.doubleSpinBox_joint4->value());
  // Add new joint5_roll value
  joint_angle.push_back(ui.doubleSpinBox_joint5_roll->value());
  
  qnode.setJointSpacePath(joint_angle);
  writeLog("Send joint 5 roll value");
}
```

**Why:** Sends 5-joint trajectory when wrist roll "Set" button clicked.

---

#### Change 4: Slider Sync
```cpp
void MainWindow::on_horizontalSlider_joint_5_roll_valueChanged(int value)
{
  double rad_value = value * 0.01;  // Convert slider (int) to radians
  ui.doubleSpinBox_joint5_roll->setValue(rad_value);
}
```

**Why:** Synchronizes slider movement with spinbox value (slider range: -314 to 314 → ±3.14 rad).

---

#### Change 5: CSV Logging Update
```cpp
// When saving joint values to CSV
if (joint_angle.size() >= 6) {  // Was 5, now 6
  csv_file << joint_angle[0] << ","
           << joint_angle[1] << ","
           << joint_angle[2] << ","
           << joint_angle[3] << ","
           << joint_angle[4] << ","  // joint5_roll - NEW
           << joint_angle[5] << "\n"; // gripper - moved from index 4
}
```

**Why:** Logs all 6 joint values to CSV for trajectory playback/analysis.

---

### 4. QNode: `qnode.cpp`

```cpp
// Update state reading
void QNode::updateRobotState()
{
  std::vector<double> jointValues = move_group_->getCurrentJointValues();
  std::vector<double> jointValues2 = move_group2_->getCurrentJointValues();
  
  if (jointValues.size() >= 5) {  // Was 4, now 5 (arm joints)
    temp_angle.push_back(jointValues.at(0));  // joint1
    temp_angle.push_back(jointValues.at(1));  // joint2
    temp_angle.push_back(jointValues.at(2));  // joint3
    temp_angle.push_back(jointValues.at(3));  // joint4
    temp_angle.push_back(jointValues.at(4));  // joint5_roll - NEW
  }
  
  if (jointValues2.size() >= 1) {
    temp_angle.push_back(jointValues2.at(0));  // gripper
  }
}
```

**Why:** Reads 5 arm joint values from MoveIt move_group instead of 4.

---

## Why These Changes Are Required

### 1. User Interface Control
GUI must provide control for all arm joints:
- Input methods: spinbox (precise) + slider (intuitive)
- Feedback: Display current position
- Logging: Save trajectories for playback

Without GUI control, users would need command-line tools or code to control wrist roll.

### 2. Joint State Synchronization
```
MoveIt arm group (5 joints) → GUI display (5 spinboxes)
/joint_states topic (6 values) → CSV log (6 columns)
```

Mismatch in indices causes:
- ❌ Wrong values displayed
- ❌ Gripper shown in wrist roll box
- ❌ Array out-of-bounds errors

### 3. Consistent Data Flow
```
User Input → GUI → QNode → MoveIt → ros2_control → Hardware
Hardware → joint_states → QNode → GUI Display
```

Each stage must handle 5 arm joints + 1 gripper = 6 total.

---

## GUI Layout

### Before (4-DOF):
```
┌──────────────────────────────┐
│ Joint 1:  [0.00] ▓▓░░░░ [Set]│
│ Joint 2:  [0.00] ▓▓░░░░ [Set]│
│ Joint 3:  [0.00] ▓▓░░░░ [Set]│
│ Joint 4:  [0.00] ▓▓░░░░ [Set]│
│ Gripper:  [0.00]        [Set]│
└──────────────────────────────┘
```

### After (5-DOF):
```
┌──────────────────────────────┐
│ Joint 1:      [0.00] ▓▓░░░░ [Set]│
│ Joint 2:      [0.00] ▓▓░░░░ [Set]│
│ Joint 3:      [0.00] ▓▓░░░░ [Set]│
│ Joint 4:      [0.00] ▓▓░░░░ [Set]│
│ Joint 5 roll: [0.00] ▓▓░░░░ [Set]│ ← NEW
│ Gripper:      [0.00]        [Set]│
└──────────────────────────────┘
```

---

## Testing Verification

### 1. Launch GUI
```bash
ros2 launch open_manipulator_gui open_manipulator_x_gui.launch.py
```

### 2. Check Display
- ✅ 6 rows visible (5 joints + gripper)
- ✅ Joint 5 roll spinbox shows current value
- ✅ Slider moves with joint motion
- ✅ Table header shows "Joint 5 roll" column

### 3. Test Control
```
1. Adjust Joint 5 roll spinbox to 1.57 (90°)
2. Click "Set" button
3. Watch robot wrist rotate
4. Verify spinbox updates with actual position
5. Move slider → spinbox follows
```

### 4. CSV Logging
```
1. Enable CSV logging
2. Move joints
3. Check CSV file has 6 columns:
   joint1, joint2, joint3, joint4, joint5_roll, gripper
```

---

## Implementation Details

### Slider-to-Radian Conversion
```cpp
// Slider range: -314 to +314 (integer)
// Radian range: -3.14 to +3.14
double rad = slider_value * 0.01;

// Example:
// Slider = 157 → rad = 1.57 (90°)
// Slider = -314 → rad = -3.14 (-180°)
```

### Joint Angle Vector
```cpp
std::vector<double> joint_angle = [
  joint1_val,      // Index 0
  joint2_val,      // Index 1
  joint3_val,      // Index 2
  joint4_val,      // Index 3
  joint5_roll_val, // Index 4 - NEW
  gripper_val      // Index 5 (was 4)
];
```

---

## Common Issues

### Issue 1: Gripper Value in Wrong Box
```
Symptom: Gripper position shows in Joint 5 roll box
Cause: Index mismatch in updateJointSpinbox()
Solution: joint_angle.at(4) for roll, at(5) for gripper ✓
```

### Issue 2: "Set" Button Does Nothing
```
Symptom: Clicking Set button doesn't move robot
Cause: Signal-slot connection missing in .ui file
Solution: Check btn_set_joint5_roll connected to slot ✓
```

### Issue 3: Slider Out of Sync
```
Symptom: Moving slider doesn't update spinbox
Cause: valueChanged signal not connected
Solution: Connect horizontalSlider_joint_5_roll to slot ✓
```

---

**Next:** See [TECHNICAL_OVERVIEW.md](./TECHNICAL_OVERVIEW.md) for complete system architecture
