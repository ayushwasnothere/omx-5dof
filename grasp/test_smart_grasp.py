#!/usr/bin/env python3
# test_smart_grasp.py
#
# Test suite for CollisionAwareGrasp.smart_grasp()
#
# Structure
# ─────────
# Unit tests  (no ROS, no hardware — all collaborators are mocked)
#   TestSmartGraspHappyPath      — full success path, verifies call sequence
#   TestSmartGraspFailurePoints  — each step failing in isolation returns False
#   TestSmartGraspSceneContent   — correct objects/obstacles reach the scene
#   TestSmartGraspPoseCalculation — pre-grasp and grasp Z offsets are correct
#
# Integration test  (requires live ROS2 + MoveIt + Gazebo)
#   TestSmartGraspIntegration    — end-to-end on a real node
#
# Run unit tests only (no ROS):
#   pytest test_smart_grasp.py -v -k "not Integration"
#
# Run integration tests (requires running stack):
#   pytest test_smart_grasp.py -v -k "Integration"

import copy
import sys
import types
import unittest
from unittest.mock import MagicMock, call, PropertyMock

# ─────────────────────────────────────────────────────────────────────────────
# ROS mock setup — must happen before any grasp_node import.
# rclpy.node.Node must be a real class (not MagicMock) so that
# CollisionAwareGrasp can subclass it and object.__new__ works correctly.
# ─────────────────────────────────────────────────────────────────────────────

class _FakeNode:
    """Minimal stand-in for rclpy.node.Node — no ROS internals."""
    def __init__(self, *args, **kwargs):
        pass

_node_mod = types.ModuleType("rclpy.node")
_node_mod.Node = _FakeNode
sys.modules["rclpy.node"] = _node_mod

for _m in [
    "rclpy", "rclpy.action", "rclpy.executors",
    "rclpy.parameter", "rclpy.duration",
    "tf2_ros", "tf2_geometry_msgs", "tf2_geometry_msgs.tf2_geometry_msgs",
    "geometry_msgs", "geometry_msgs.msg",
    "control_msgs", "control_msgs.action",
    "moveit_msgs", "moveit_msgs.action", "moveit_msgs.msg", "moveit_msgs.srv",
    "shape_msgs", "shape_msgs.msg",
    "scene_manager", "motion_client",
]:
    if _m not in sys.modules:
        sys.modules[_m] = MagicMock()

# Minimal geometry_msgs.msg stubs with real position arithmetic
class _Point:
    def __init__(self): self.x = self.y = self.z = 0.0
class _Quaternion:
    def __init__(self): self.x = self.y = self.z = 0.0; self.w = 1.0
class _Pose:
    def __init__(self): self.position = _Point(); self.orientation = _Quaternion()
class _Header:
    def __init__(self): self.frame_id = "world"; self.stamp = MagicMock()
class _PoseStamped:
    def __init__(self): self.header = _Header(); self.pose = _Pose()

import geometry_msgs.msg as _gm
_gm.PoseStamped = _PoseStamped

# Now we can safely import the module under test
from grasp_node import CollisionAwareGrasp  # noqa: E402


def make_pose(x=0.35, y=-0.10, z=0.10, frame="world"):
    pose = _PoseStamped()
    pose.header.frame_id = frame
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.w = 1.0
    return pose

# ── Constants mirrored from grasp_node so tests don't hardcode magic numbers ─
PRE_GRASP_OFFSET_Z = 0.10
RETREAT_HEIGHT     = 0.15
GRIPPER_CLOSE_POS  = -0.01
GRIPPER_LINKS = [
    "end_effector_link",
    "gripper_link",
    "gripper_left_finger_link",
    "gripper_right_finger_link",
]


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────


def make_node(
    planner_result="x",
    move_returns=True,
    cartesian_returns=True,
    gripper_returns=True,
):
    """
    Build a CollisionAwareGrasp instance with all ROS collaborators mocked.
    Uses object.__new__ to bypass __init__ entirely — no ROS runtime needed.
    All ROS imports are already mocked at module level.
    """
    node = object.__new__(CollisionAwareGrasp)

    node.get_logger = MagicMock(return_value=MagicMock(
        info=MagicMock(), error=MagicMock(), warn=MagicMock()
    ))

    node.scene = MagicMock()
    node.scene.clear.return_value = True
    node.scene.add_obstacles.return_value = True
    node.scene.attach_object.return_value = True
    node.scene.remove_object.return_value = True

    node.motion = MagicMock()
    node.motion.move_to_pose.return_value = move_returns
    node.motion.cartesian_path.return_value = cartesian_returns

    node.planner = MagicMock()
    if planner_result is None:
        node.planner.evaluate.return_value = None
    else:
        plan = MagicMock()
        plan.axis = planner_result
        plan.grasp_width = 0.008
        node.planner.evaluate.return_value = plan

    node._control_gripper = MagicMock(return_value=gripper_returns)

    return node



# ─────────────────────────────────────────────────────────────────────────────
# 1. Happy Path — verify the full call sequence
# ─────────────────────────────────────────────────────────────────────────────

class TestSmartGraspHappyPath(unittest.TestCase):

    def setUp(self):
        self.node = make_node()
        self.pose = make_pose()
        self.dims = [0.02, 0.03, 0.05]
        self.obstacles = [
            {"id": "wall", "type": "box",
             "pose": make_pose(0.5, 0.0, 0.1), "size": [0.05, 0.3, 0.3]}
        ]

    def test_returns_true_on_full_success(self):
        result = self.node.smart_grasp("cup", self.pose, self.dims, self.obstacles)
        self.assertTrue(result)

    def test_planner_called_with_dimensions(self):
        self.node.smart_grasp("cup", self.pose, self.dims)
        self.node.planner.evaluate.assert_called_once_with(self.dims)

    def test_scene_cleared_before_adding_objects(self):
        self.node.smart_grasp("cup", self.pose, self.dims)
        # clear() must be called before add_obstacles()
        call_order = self.node.scene.method_calls
        clear_idx = next(i for i, c in enumerate(call_order) if c[0] == "clear")
        add_idx   = next(i for i, c in enumerate(call_order) if c[0] == "add_obstacles")
        self.assertLess(clear_idx, add_idx)

    def test_target_object_added_to_scene(self):
        self.node.smart_grasp("cup", self.pose, self.dims)
        all_calls = self.node.scene.add_obstacles.call_args_list
        # Flatten all objects passed across all add_obstacles() calls
        added_ids = [
            obj["id"]
            for c in all_calls
            for obj in c[0][0]
        ]
        self.assertIn("cup", added_ids)

    def test_obstacles_added_to_scene(self):
        self.node.smart_grasp("cup", self.pose, self.dims, self.obstacles)
        all_calls = self.node.scene.add_obstacles.call_args_list
        added_ids = [
            obj["id"]
            for c in all_calls
            for obj in c[0][0]
        ]
        self.assertIn("wall", added_ids)

    def test_target_object_and_obstacles_added_together(self):
        """Both must be in the scene before pre-grasp so OMPL avoids everything."""
        self.node.smart_grasp("cup", self.pose, self.dims, self.obstacles)
        first_add_call_objects = self.node.scene.add_obstacles.call_args_list[0][0][0]
        ids = [o["id"] for o in first_add_call_objects]
        self.assertIn("cup",  ids)
        self.assertIn("wall", ids)

    def test_pre_grasp_is_above_target(self):
        self.node.smart_grasp("cup", self.pose, self.dims)
        pre_grasp_call = self.node.motion.move_to_pose.call_args_list[0]
        pre_pose = pre_grasp_call[0][0]
        expected_z = self.pose.pose.position.z + PRE_GRASP_OFFSET_Z
        self.assertAlmostEqual(pre_pose.pose.position.z, expected_z, places=4)

    def test_pre_grasp_xy_unchanged(self):
        self.node.smart_grasp("cup", self.pose, self.dims)
        pre_pose = self.node.motion.move_to_pose.call_args_list[0][0][0]
        self.assertAlmostEqual(pre_pose.pose.position.x, self.pose.pose.position.x, places=4)
        self.assertAlmostEqual(pre_pose.pose.position.y, self.pose.pose.position.y, places=4)

    def test_grasp_pose_z_is_top_surface(self):
        """Grasp Z should be object centre Z + half the object height (dz/2).
        waypoints has a single entry (index 0) — the target grasp pose only.
        GetCartesianPath starts from current robot state implicitly."""
        self.node.smart_grasp("cup", self.pose, self.dims)
        waypoints = self.node.motion.cartesian_path.call_args[0][0]
        grasp_z = waypoints[0].position.z
        expected_z = self.pose.pose.position.z + self.dims[2] / 2.0
        self.assertAlmostEqual(grasp_z, expected_z, places=4)

    def test_cartesian_descent_uses_avoid_collisions_false(self):
        """Final approach must use avoid_collisions=False — not a global ACM change."""
        self.node.smart_grasp("cup", self.pose, self.dims)
        kwargs = self.node.motion.cartesian_path.call_args[1]
        self.assertFalse(
            kwargs.get("avoid_collisions", True),
            "cartesian_path must be called with avoid_collisions=False for the descent"
        )

    def test_attach_called_before_close_gripper(self):
        """attach_object() must precede _control_gripper() so close doesn't collide."""
        call_log = []
        self.node.scene.attach_object.side_effect  = lambda *a, **kw: call_log.append("attach")
        self.node._control_gripper.side_effect     = lambda *a, **kw: call_log.append("gripper") or True

        self.node.smart_grasp("cup", self.pose, self.dims)

        attach_idx  = call_log.index("attach")
        gripper_idx = call_log.index("gripper")
        self.assertLess(attach_idx, gripper_idx,
                        "attach_object() must be called before _control_gripper()")

    def test_attach_called_with_correct_object_id(self):
        self.node.smart_grasp("cup", self.pose, self.dims)
        attach_args = self.node.scene.attach_object.call_args
        self.assertEqual(attach_args[0][0], "cup")

    def test_attach_passes_touch_links(self):
        self.node.smart_grasp("cup", self.pose, self.dims)
        kwargs = self.node.scene.attach_object.call_args[1]
        touch = kwargs.get("touch_links", [])
        for link in GRIPPER_LINKS:
            self.assertIn(link, touch,
                          f"touch_links must include '{link}' to prevent self-collision")

    def test_gripper_closed_with_correct_position(self):
        self.node.smart_grasp("cup", self.pose, self.dims)
        gripper_call = self.node._control_gripper.call_args
        self.assertAlmostEqual(gripper_call[0][0], GRIPPER_CLOSE_POS, places=4)

    def test_retreat_is_above_grasp_pose(self):
        self.node.smart_grasp("cup", self.pose, self.dims)
        # move_to_pose is called twice: pre-grasp and retreat
        retreat_call = self.node.motion.move_to_pose.call_args_list[1]
        retreat_pose = retreat_call[0][0]
        grasp_z = self.pose.pose.position.z + self.dims[2] / 2.0
        expected_retreat_z = grasp_z + RETREAT_HEIGHT
        self.assertAlmostEqual(retreat_pose.pose.position.z, expected_retreat_z, places=4)

    def test_move_to_pose_called_exactly_twice(self):
        """Once for pre-grasp, once for retreat — no extra moves."""
        self.node.smart_grasp("cup", self.pose, self.dims)
        self.assertEqual(self.node.motion.move_to_pose.call_count, 2)

    def test_cartesian_path_called_exactly_once(self):
        self.node.smart_grasp("cup", self.pose, self.dims)
        self.assertEqual(self.node.motion.cartesian_path.call_count, 1)

    def test_target_object_not_removed_from_scene(self):
        """Object must stay in scene during descent — remove_object() is the old wrong pattern."""
        self.node.smart_grasp("cup", self.pose, self.dims)
        remove_calls = [
            c for c in self.node.scene.method_calls
            if c[0] == "remove_object" and "cup" in c[1]
        ]
        self.assertEqual(len(remove_calls), 0,
                         "remove_object() must NOT be called on the target during smart_grasp")

    def test_returns_true_with_no_obstacles(self):
        result = self.node.smart_grasp("cup", self.pose, self.dims, obstacles=None)
        self.assertTrue(result)


# ─────────────────────────────────────────────────────────────────────────────
# 2. Failure Points — each failing step must return False and stop immediately
# ─────────────────────────────────────────────────────────────────────────────

class TestSmartGraspFailurePoints(unittest.TestCase):

    def setUp(self):
        self.pose = make_pose()
        self.dims = [0.02, 0.03, 0.05]

    def test_returns_false_when_not_graspable(self):
        node = make_node(planner_result=None)
        result = node.smart_grasp("cup", self.pose, self.dims)
        self.assertFalse(result)

    def test_no_motion_attempted_when_not_graspable(self):
        node = make_node(planner_result=None)
        node.smart_grasp("cup", self.pose, self.dims)
        node.motion.move_to_pose.assert_not_called()
        node.motion.cartesian_path.assert_not_called()

    def test_no_scene_populated_when_not_graspable(self):
        node = make_node(planner_result=None)
        node.smart_grasp("cup", self.pose, self.dims)
        node.scene.add_obstacles.assert_not_called()

    def test_returns_false_when_pre_grasp_fails(self):
        node = make_node()
        node.motion.move_to_pose.return_value = False
        result = node.smart_grasp("cup", self.pose, self.dims)
        self.assertFalse(result)

    def test_cartesian_not_called_when_pre_grasp_fails(self):
        node = make_node()
        node.motion.move_to_pose.return_value = False
        node.smart_grasp("cup", self.pose, self.dims)
        node.motion.cartesian_path.assert_not_called()

    def test_returns_false_when_cartesian_descent_fails(self):
        node = make_node()
        node.motion.cartesian_path.return_value = False
        result = node.smart_grasp("cup", self.pose, self.dims)
        self.assertFalse(result)

    def test_attach_not_called_when_descent_fails(self):
        node = make_node()
        node.motion.cartesian_path.return_value = False
        node.smart_grasp("cup", self.pose, self.dims)
        node.scene.attach_object.assert_not_called()

    def test_gripper_not_closed_when_descent_fails(self):
        node = make_node()
        node.motion.cartesian_path.return_value = False
        node.smart_grasp("cup", self.pose, self.dims)
        node._control_gripper.assert_not_called()

    def test_returns_false_when_gripper_fails(self):
        node = make_node(gripper_returns=False)
        result = node.smart_grasp("cup", self.pose, self.dims)
        self.assertFalse(result)

    def test_retreat_not_called_when_gripper_fails(self):
        node = make_node(gripper_returns=False)
        node.smart_grasp("cup", self.pose, self.dims)
        # Only pre-grasp move should have happened — retreat must not be called
        self.assertEqual(node.motion.move_to_pose.call_count, 1)

    def test_returns_false_when_retreat_fails(self):
        node = make_node()
        # First move (pre-grasp) succeeds, second (retreat) fails
        node.motion.move_to_pose.side_effect = [True, False]
        result = node.smart_grasp("cup", self.pose, self.dims)
        self.assertFalse(result)

    def test_all_steps_complete_before_retreat_fails(self):
        """Even if retreat fails, attach and gripper-close should have run."""
        node = make_node()
        node.motion.move_to_pose.side_effect = [True, False]
        node.smart_grasp("cup", self.pose, self.dims)
        node.scene.attach_object.assert_called_once()
        node._control_gripper.assert_called_once()


# ─────────────────────────────────────────────────────────────────────────────
# 3. Scene Content — what exactly gets put in the planning scene
# ─────────────────────────────────────────────────────────────────────────────

class TestSmartGraspSceneContent(unittest.TestCase):

    def setUp(self):
        self.node = make_node()
        self.pose = make_pose()
        self.dims = [0.02, 0.03, 0.05]

    def _all_added_objects(self):
        """Flatten all objects passed to add_obstacles() across all calls."""
        return [
            obj
            for c in self.node.scene.add_obstacles.call_args_list
            for obj in c[0][0]
        ]

    def test_target_object_has_correct_type(self):
        self.node.smart_grasp("cup", self.pose, self.dims)
        target = next(o for o in self._all_added_objects() if o["id"] == "cup")
        self.assertEqual(target["type"], "box")

    def test_target_object_has_correct_dimensions(self):
        self.node.smart_grasp("cup", self.pose, self.dims)
        target = next(o for o in self._all_added_objects() if o["id"] == "cup")
        self.assertEqual(target["size"], self.dims)

    def test_target_object_has_correct_pose(self):
        self.node.smart_grasp("cup", self.pose, self.dims)
        target = next(o for o in self._all_added_objects() if o["id"] == "cup")
        self.assertEqual(target["pose"], self.pose)

    def test_multiple_obstacles_all_added(self):
        obstacles = [
            {"id": "wall1", "type": "box",      "pose": make_pose(0.5, 0.0, 0.1), "size": [0.05, 0.3, 0.3]},
            {"id": "wall2", "type": "box",      "pose": make_pose(0.0, 0.5, 0.1), "size": [0.3, 0.05, 0.3]},
            {"id": "shelf", "type": "cylinder", "pose": make_pose(0.4, 0.2, 0.05), "size": [0.1, 0.02]},
        ]
        self.node.smart_grasp("cup", self.pose, self.dims, obstacles)
        added_ids = [o["id"] for o in self._all_added_objects()]
        for obs in obstacles:
            self.assertIn(obs["id"], added_ids)

    def test_scene_cleared_exactly_once(self):
        self.node.smart_grasp("cup", self.pose, self.dims)
        self.assertEqual(self.node.scene.clear.call_count, 1)

    def test_no_extra_objects_injected(self):
        """Only the target + provided obstacles should appear — no phantom objects."""
        obstacles = [{"id": "box1", "type": "box", "pose": make_pose(), "size": [0.1, 0.1, 0.1]}]
        self.node.smart_grasp("cup", self.pose, self.dims, obstacles)
        added_ids = set(o["id"] for o in self._all_added_objects())
        self.assertEqual(added_ids, {"cup", "box1"})


# ─────────────────────────────────────────────────────────────────────────────
# 4. Pose Calculation — Z offsets are geometrically correct
# ─────────────────────────────────────────────────────────────────────────────

class TestSmartGraspPoseCalculation(unittest.TestCase):

    def setUp(self):
        self.node = make_node()

    def _run(self, z, dims):
        pose = make_pose(z=z)
        self.node.smart_grasp("obj", pose, dims)
        pre_pose    = self.node.motion.move_to_pose.call_args_list[0][0][0]
        waypoints   = self.node.motion.cartesian_path.call_args[0][0]
        grasp_pose  = waypoints[-1]  # last waypoint is the target grasp Pose
        retreat_pose = self.node.motion.move_to_pose.call_args_list[1][0][0]
        return pre_pose, grasp_pose, retreat_pose

    def test_pre_grasp_offset_is_10cm(self):
        pre, _, _ = self._run(z=0.10, dims=[0.01, 0.01, 0.05])
        self.assertAlmostEqual(pre.pose.position.z, 0.10 + PRE_GRASP_OFFSET_Z, places=4)

    def test_grasp_z_equals_centre_plus_half_height(self):
        dz = 0.06
        _, grasp, _ = self._run(z=0.10, dims=[0.01, 0.01, dz])
        self.assertAlmostEqual(grasp.position.z, 0.10 + dz / 2.0, places=4)

    def test_retreat_z_equals_grasp_plus_retreat_height(self):
        dz = 0.06
        _, grasp, retreat = self._run(z=0.10, dims=[0.01, 0.01, dz])
        self.assertAlmostEqual(
            retreat.pose.position.z, grasp.position.z + RETREAT_HEIGHT, places=4
        )

    def test_tall_object_grasp_z_higher(self):
        _, grasp_tall, _  = self._run(z=0.10, dims=[0.01, 0.01, 0.10])
        _, grasp_short, _ = self._run(z=0.10, dims=[0.01, 0.01, 0.02])
        self.assertGreater(grasp_tall.position.z, grasp_short.position.z)

    def test_cartesian_has_single_target_waypoint(self):
        """
        GetCartesianPath starts from current robot state implicitly.
        Only the target grasp pose should be passed — NOT the start/pre-grasp pose.
        Passing start+end gives near-zero fraction because the robot is already
        at the start pose and MoveIt plans from its actual current state.
        """
        pose = make_pose(z=0.10)
        dims = [0.01, 0.01, 0.05]
        self.node.smart_grasp("obj", pose, dims)
        waypoints = self.node.motion.cartesian_path.call_args[0][0]
        # Must be exactly one waypoint: the grasp target
        self.assertEqual(len(waypoints), 1,
                         "cartesian_path must receive only the target waypoint, not [start, target]")
        # That single waypoint must be the top surface of the object
        expected_z = pose.pose.position.z + dims[2] / 2.0
        self.assertAlmostEqual(waypoints[0].position.z, expected_z, places=4)

# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    unittest.main(verbosity=2)
