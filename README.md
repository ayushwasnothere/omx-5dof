# Open Manipulator X — Service-Based MTC Manipulator

## Build

```bash
cd ~/omx_ws
colcon build --packages-select omx_interfaces open_manipulator_mtc
source install/setup.bash
```

## Launch (3 terminals)

### Terminal 1 — Gazebo

```bash
cd ~/omx_ws && source install/setup.bash
ros2 launch open_manipulator_bringup open_manipulator_x_gazebo.launch.py
```

### Terminal 2 — MoveIt

```bash
cd ~/omx_ws && source install/setup.bash
ros2 launch open_manipulator_moveit_config open_manipulator_x_moveit.launch.py use_sim:=True
```

### Terminal 3 — Manipulator Node

```bash
cd ~/omx_ws && source install/setup.bash
ros2 launch open_manipulator_mtc manipulator.launch.py use_sim:=true
```

### Terminal 4 — Test Commands

```bash
cd ~/omx_ws && source install/setup.bash
```

Run the commands below in this terminal.

---

## Test Commands

### 1. Add Objects to Scene

Add a cylinder ("cup1"):

```bash
ros2 service call /manipulator_node/add_objects omx_interfaces/srv/AddObjects \
  "{objects: [
    {id: 'cup1', type: 'cylinder',
     pose: {header: {frame_id: 'world'}, pose: {position: {x: 0.20, y: 0.0, z: 0.04}, orientation: {w: 1.0}}},
     dimensions: [0.15, 0.015]}
  ]}"
```

> Cylinder dimensions: `[height, radius]`
> Box dimensions: `[dx, dy, dz]`
> Sphere dimensions: `[radius]`

Add multiple objects at once:

```bash
ros2 service call /manipulator_node/add_objects omx_interfaces/srv/AddObjects \
  "{objects: [
    {id: 'cup1', type: 'cylinder',
     pose: {header: {frame_id: 'world'}, pose: {position: {x: 0.35, y: 0.15, z: 0.04}, orientation: {w: 1.0}}},
     dimensions: [0.2, 0.015]},
    {id: 'bowl1', type: 'cylinder',
     pose: {header: {frame_id: 'world'}, pose: {position: {x: 0.3, y: -0.15, z: 0.03}, orientation: {w: 1.0}}},
     dimensions: [0.06, 0.025]}
  ]}"
```

### 2. Get Objects (verify scene)

```bash
ros2 service call /manipulator_node/get_objects omx_interfaces/srv/GetObjects "{}"
```

### 3. Pick an Object

```bash
ros2 service call /manipulator_node/pick omx_interfaces/srv/Pick "{object_id: 'cup1'}"
```

### 4. Place at a Specific Pose

```bash
ros2 service call /manipulator_node/place omx_interfaces/srv/Place \
  "{use_target_pose: true,
    target_pose: {header: {frame_id: 'world'},
                  pose: {position: {x: 0.15, y: -0.10, z: 0.04},
                         orientation: {w: 1.0}}}}"
```

### 5. Place at a Random Position

```bash
ros2 service call /manipulator_node/place omx_interfaces/srv/Place \
  "{use_target_pose: false}"
```

### 6. Rotate / Mix (wrist roll oscillation)

Single rotation (π rad = 180°):

```bash
ros2 service call /manipulator_node/rotate omx_interfaces/srv/Rotate \
  "{angle: 3.14, cycles: 0}"
```

Oscillate ±30° for 3 cycles (mix motion):

```bash
ros2 service call /manipulator_node/rotate omx_interfaces/srv/Rotate \
  "{angle: 0.25, cycles: 3}"
```

### 7. Pour (WIP — may fail on 5-DOF IK)

Pick an object first, then pour into another:

` +[[]]``bash
ros2 service call /manipulator_node/pick omx_interfaces/srv/Pick "{object_id: 'cup1'}"
ros2 service call /manipulator_node/pour omx_interfaces/srv/Pour \
  "{target_object_id: 'bowl1', pour_angle: 2.0}"
```

### 8. Go Home

```bash
ros2 service call /manipulator_node/go_home omx_interfaces/srv/GoHome "{state_name: 'home'}"
```

Go to init (all joints zero):

```bash
ros2 service call /manipulator_node/go_home omx_interfaces/srv/GoHome "{state_name: 'init'}"
```

### 9. Remove Objects

Remove specific objects:

```bash
 
```

Clear all:

```bash
ros2 service call /manipulator_node/remove_objects omx_interfaces/srv/RemoveObjects \
  "{clear_all: true}"
```

---

## Full Pick-Place Test Sequence

Run these in order:

```bash
# 1. Add objects
ros2 service call /manipulator_node/add_objects omx_interfaces/srv/AddObjects \
  "{objects: [
    {id: 'cup1', type: 'cylinder',
     pose: {header: {frame_id: 'world'}, pose: {position: {x: 0.20, y: 0.0, z: 0.04}, orientation: {w: 1.0}}},
     dimensions: [0.08, 0.015]}
  ]}"

# 2. Pick
ros2 service call /manipulator_node/pick omx_interfaces/srv/Pick "{object_id: 'cup1'}"

# 3. Rotate while holding
ros2 service call /manipulator_node/rotate omx_interfaces/srv/Rotate "{angle: 0.5, cycles: 2}"

# 4. Place
ros2 service call /manipulator_node/place omx_interfaces/srv/Place \
  "{use_target_pose: true,
    target_pose: {header: {frame_id: 'world'},
                  pose: {position: {x: 0.15, y: 0.12, z: 0.04}, orientation: {w: 1.0}}}}"

# 5. Go home
ros2 service call /manipulator_node/go_home omx_interfaces/srv/GoHome "{state_name: 'home'}"

# 6. Cleanup
ros2 service call /manipulator_node/remove_objects omx_interfaces/srv/RemoveObjects "{clear_all: true}"
```

---

## Services Reference

| Service | Type | Description |
|---------|------|-------------|
| `~/add_objects` | `omx_interfaces/srv/AddObjects` | Add collision objects to scene |
| `~/remove_objects` | `omx_interfaces/srv/RemoveObjects` | Remove objects or clear all |
| `~/get_objects` | `omx_interfaces/srv/GetObjects` | List tracked objects |
| `~/pick` | `omx_interfaces/srv/Pick` | Pick an object by ID |
| `~/place` | `omx_interfaces/srv/Place` | Place held object at pose or random |
| `~/pour` | `omx_interfaces/srv/Pour` | Pour into target object (WIP) |
| `~/rotate` | `omx_interfaces/srv/Rotate` | Rotate wrist roll joint |
| `~/go_home` | `omx_interfaces/srv/GoHome` | Move arm to named state |

---

## Redis Bridge (External API)

The Redis bridge exposes all manipulator services over Redis Streams, allowing external apps (any language) to control the robot without ROS2.

### Prerequisites

```bash
# Redis server (already installed system-wide)
redis-server --daemonize yes
redis-cli ping   # should return PONG

# Python redis client (already installed system-wide)
python3 -c "import redis; print(redis.__version__)"
```

### Terminal 5 — Redis Bridge

```bash
cd ~/omx_ws && source install/setup.bash
python3 bridge/redis_bridge.py
```

### Sending Tasks via redis-cli

Tasks go to the `robot.tasks` stream. Results appear on `robot.events`.

**Pick:**
```bash
redis-cli XADD robot.tasks '*' task_id pick-001 skill pick params '{"object_id":"cup1"}'
```

**Place at target pose:**
```bash
redis-cli XADD robot.tasks '*' task_id place-001 skill place params \
  '{"use_target_pose":true,"target_pose":{"frame_id":"world","position":{"x":0.15,"y":-0.10,"z":0.04},"orientation":{"w":1.0}}}'
```

**Place random:**
```bash
redis-cli XADD robot.tasks '*' task_id place-002 skill place params '{"use_target_pose":false}'
```

**Add objects:**
```bash
redis-cli XADD robot.tasks '*' task_id add-001 skill add_objects params \
  '{"objects":[{"id":"cup1","type":"cylinder","pose":{"frame_id":"world","position":{"x":0.20,"y":0.0,"z":0.04},"orientation":{"w":1.0}},"dimensions":[0.15,0.015]}]}'
```

**Go home:**
```bash
redis-cli XADD robot.tasks '*' task_id home-001 skill go_home params '{"state_name":"home"}'
```

**Rotate:**
```bash
redis-cli XADD robot.tasks '*' task_id rot-001 skill rotate params '{"angle":0.5,"cycles":3}'
```

**Pour:**
```bash
redis-cli XADD robot.tasks '*' task_id pour-001 skill pour params '{"target_object_id":"bowl1","pour_angle":2.0}'
```

**Remove objects:**
```bash
redis-cli XADD robot.tasks '*' task_id rm-001 skill remove_objects params '{"clear_all":true}'
```

**Get objects:**
```bash
redis-cli XADD robot.tasks '*' task_id get-001 skill get_objects params '{}'
```

**Ping / health check:**
```bash
redis-cli XADD robot.tasks '*' task_id ping-001 skill ping params '{}'
```

### Reading Events

```bash
# Read all events
redis-cli XRANGE robot.events - +

# Follow events in real-time
redis-cli XREAD BLOCK 0 STREAMS robot.events $
```

### Event Format

Each event contains:
| Field | Description |
|-------|-------------|
| `task_id` | Matches the task_id from the request |
| `status` | `running`, `success`, or `error` |
| `data` | JSON with result details (`message`, `success`, or `objects` for get_objects) |

### Python Client Test

```bash
python3 bridge/test_bridge.py              # run default sequence
python3 bridge/test_bridge.py pick place   # run specific tests
python3 bridge/test_bridge.py add_camera   # test camera-frame add
python3 bridge/test_bridge.py --list       # list available tests
```

---

## Camera Frame Support

The manipulator node can accept object positions in **any TF frame** (e.g. `camera_color_optical_frame`). It automatically transforms the **position only** to the world frame via TF2.

> **Important:** Only the position (x, y, z) from the camera frame is used. The orientation is **always** determined by the object's dimensions and the robot's grasp logic — not by the camera frame. This means you never need to worry about camera rotation when sending object coordinates.

### Camera TF Publisher

The `camera_tf_node.py` publishes a static transform from `world` → `camera_color_optical_frame`. It is included in the launch file automatically.

The transform is split into **base** (measured values) and **error** (calibration correction) — the effective transform is `base + error`.

### Setting the Camera Transform

The default values come from the physical camera calibration:

```
translation: [0.87, 0.0, 0.10] (meters)
quaternion:  [-0.5, -0.5, 0.5, 0.5] (qx, qy, qz, qw)
```

**Set new base transform values:**
```bash
ros2 param set /camera_tf_publisher tx 0.87
ros2 param set /camera_tf_publisher ty 0.0
ros2 param set /camera_tf_publisher tz 0.10
ros2 param set /camera_tf_publisher qx -0.5
ros2 param set /camera_tf_publisher qy -0.5
ros2 param set /camera_tf_publisher qz 0.5
ros2 param set /camera_tf_publisher qw 0.5
```

The transform republishes automatically after each change and saves to `~/.config/camera_tf_params.yaml` so it **persists across restarts**.

### Correcting Calibration Errors

Use the `error_*` parameters for fine-tuning without modifying the base values:

```bash
# Nudge position by 1cm in X and -0.5cm in Y
ros2 param set /camera_tf_publisher error_tx 0.01
ros2 param set /camera_tf_publisher error_ty -0.005

# Correct rotation by small quaternion offset
ros2 param set /camera_tf_publisher error_qz 0.01

# Reset an error offset
ros2 param set /camera_tf_publisher error_tx 0.0
```

### Parameters Reference

| Parameter | Default | Description |
|-----------|---------|-------------|
| `tx`, `ty`, `tz` | 0.87, 0.0, 0.10 | Base translation (meters) |
| `qx`, `qy`, `qz`, `qw` | -0.5, -0.5, 0.5, 0.5 | Base rotation quaternion |
| `error_tx`, `error_ty`, `error_tz` | 0.0, 0.0, 0.0 | Translation error offset (meters) |
| `error_qx`, `error_qy`, `error_qz`, `error_qw` | 0.0, 0.0, 0.0, 0.0 | Quaternion error offset (added to base, then normalized) |
| `parent_frame` | world | Parent TF frame |
| `child_frame` | camera_color_optical_frame | Child TF frame |

**View all current values:**
```bash
ros2 param dump /camera_tf_publisher
```

**Config file location:** `~/.config/camera_tf_params.yaml` (auto-saved on every change, loaded on startup)

### Add Objects in Camera Frame

Set `use_camera_frame: true` on the object — the manipulator node transforms the position to world automatically:

**ROS2 Service:**
```bash
ros2 service call /manipulator_node/add_objects omx_interfaces/srv/AddObjects \
  "{objects: [
    {id: 'cam_cup1', type: 'cylinder', use_camera_frame: true,
     pose: {pose: {position: {x: -0.0720, y: 0.25, z: 0.557}, orientation: {w: 1.0}}},
     dimensions: [0.2, 0.025]}
  ]}"
```

**Redis Bridge:**
```bash
redis-cli XADD robot.tasks '*' task_id cam-001 skill add_objects params \
  '{"objects":[{"id":"cam_cup1","type":"cylinder","use_camera_frame":true,"pose":{"position":{"x":0.0,"y":0.0,"z":0.5}},"dimensions":[0.08,0.015]}]}'
```

**Place in camera frame:**
```bash
ros2 service call /manipulator_node/place omx_interfaces/srv/Place \
  "{use_target_pose: true, use_camera_frame: true,
    target_pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.5}, orientation: {w: 1.0}}}}"
```

```bash
redis-cli XADD robot.tasks '*' task_id cam-place-001 skill place params \
  '{"use_target_pose":true,"use_camera_frame":true,"target_pose":{"position":{"x":0.0,"y":0.0,"z":0.5}}}'
```

**Robot frame (default):** When `use_camera_frame` is `false` (or omitted), coordinates are in the robot/world frame as before.

