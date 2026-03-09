#!/usr/bin/env python3
# grasp_node.py
#
# Main ROS2 node — orchestrates scene setup, planning, and motion execution.
#
# Public API:
#   node.grasp_world(target_pose_world, dimensions, obstacles)
#   node.grasp_camera(target_pose_camera, dimensions, obstacles)
#   node.smart_grasp(object_id, target_pose_world, dimensions, obstacles)

import argparse
import copy
import sys

import rclpy
import rclpy.duration
import rclpy.parameter
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

import tf2_ros
import tf2_geometry_msgs.tf2_geometry_msgs

from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped

from grasp_planner import GraspPlanner
from motion_client import MotionClient
from scene_manager import SceneManager


# ─────────────────────────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────────────────────────

PRE_GRASP_OFFSET_Z = 0.10   # metres above target for approach waypoint
RETREAT_HEIGHT     = 0.15   # metres to lift after grasping
GRIPPER_CLOSE_POS  = -0.01  # joint position for closed gripper
GRIPPER_OPEN_POS   =  0.019 # joint position for open gripper
GRIPPER_EFFORT     =  8.0   # Nm


# ─────────────────────────────────────────────────────────────────────────────

class CollisionAwareGrasp(Node):
    """
    Collision-aware pick-and-place node.

    Separates concerns into three injected modules:
      SceneManager  — MoveIt planning scene (obstacle injection / attachment)
      MotionClient  — MoveGroup action client (free-space & Cartesian paths)
      GraspPlanner  — pure-Python graspability heuristic (no ROS)
    """

    def __init__(self, use_sim_time: bool = False):
        super().__init__(
            "collision_aware_grasp",
            parameter_overrides=[
                rclpy.parameter.Parameter(
                    "use_sim_time",
                    rclpy.parameter.Parameter.Type.BOOL,
                    use_sim_time,
                )
            ],
        )

        self.sim_mode = use_sim_time
        mode = "SIMULATION (Gazebo)" if use_sim_time else "REAL HARDWARE"
        self.get_logger().info(f"CollisionAwareGrasp starting — mode: {mode}")

        # ── Injected modules ──────────────────────────────────────────────
        self.scene   = SceneManager(self)
        self.motion  = MotionClient(self, sim_mode=use_sim_time)
        self.planner = GraspPlanner()

        # ── TF2 (camera → world transforms) ──────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Gripper action client ─────────────────────────────────────────
        # Gazebo typically exposes a different gripper controller name than real hw
        # ── Gripper action client ─────────────────────────────────────────
        gripper_topic = "/gripper_controller/gripper_cmd"
        self.get_logger().info(f"Gripper action topic: {gripper_topic}")
        self._gripper_client = ActionClient(self, GripperCommand, gripper_topic)
        self.get_logger().info("Waiting for gripper action server...")
        self._gripper_client.wait_for_server()
        self.get_logger().info("CollisionAwareGrasp: ready.")

    # =========================================================================
    # Part A — Dynamic Collision Mapping
    # =========================================================================

    def populate_scene(self, obstacles: list) -> None:
        """
        Clear the planning scene and inject all obstacle objects.

        `obstacles` format (each entry):
            {
                "id":   str,
                "type": "box" | "sphere" | "cylinder",
                "pose": PoseStamped,
                "size": list[float]
            }
        """
        self.scene.clear()
        if obstacles:
            self.scene.add_obstacles(obstacles)
            self.get_logger().info(
                f"Scene populated with {len(obstacles)} obstacle(s)."
            )

    # =========================================================================
    # Part B — Grasp Sequence (world frame)
    # =========================================================================

    def grasp_world(
        self,
        target_pose_world: PoseStamped,
        dimensions: list[float],
        obstacles: list | None = None,
    ) -> bool:
        """
        Full grasp sequence from a world-frame target pose.

        Steps:
          1. Populate scene with obstacles
          2. Evaluate graspability (GraspPlanner)
          3. Move to pre-grasp (10 cm above, free-space OMPL)
          4. Linear Cartesian descent to grasp pose
          5. Close gripper
          6. Attach object in scene
          7. Vertical retreat

        Args:
            target_pose_world: PoseStamped of the object centre (world frame).
            dimensions:        [dx, dy, dz] of the object bounding box (metres).
            obstacles:         Optional list of nearby collision objects to add.

        Returns:
            True on complete success, False on any failure.
        """
        self.get_logger().info("── grasp_world: starting ──")

        # ── 1. Scene setup ────────────────────────────────────────────────
        self.populate_scene(obstacles or [])

        # ── 2. Graspability check ─────────────────────────────────────────
        plan = self.planner.evaluate(dimensions)
        if plan is None:
            self.get_logger().error(
                f"Object not graspable with dimensions {dimensions}."
            )
            return False
        self.get_logger().info(
            f"Grasp plan: axis={plan.axis}, width={plan.grasp_width:.4f}m"
        )

        # ── 3. Pre-grasp approach (free-space) ────────────────────────────
        pre_pose = copy.deepcopy(target_pose_world)
        pre_pose.pose.position.z += (dimensions[2] / 2.0) + PRE_GRASP_OFFSET_Z

        self.get_logger().info("Moving to pre-grasp position...")
        if not self.motion.move_to_pose(pre_pose):
            self.get_logger().error("Pre-grasp motion failed.")
            return False

        # ── 4. Linear Cartesian descent ───────────────────────────────────
        self.get_logger().info("Executing Cartesian descent to grasp pose...")
        waypoints = [
            pre_pose.pose,
            target_pose_world.pose,
        ]
        if not self.motion.cartesian_path(waypoints):
            self.get_logger().error("Cartesian descent failed.")
            return False

        # ── 5. Close gripper ──────────────────────────────────────────────
        self.get_logger().info("Closing gripper...")
        if not self._control_gripper(GRIPPER_CLOSE_POS):
            self.get_logger().error("Gripper close failed.")
            return False

        # ── 6. Attach object ──────────────────────────────────────────────
        # Use the first obstacle that matches the target pose as the grasped
        # object, if any. Caller may pass object_id explicitly via smart_grasp.
        self.get_logger().info("grasp_world: no object_id for attachment (use smart_grasp for full pipeline).")

        # ── 7. Retreat ────────────────────────────────────────────────────
        retreat_pose = copy.deepcopy(target_pose_world)
        retreat_pose.pose.position.z += RETREAT_HEIGHT

        self.get_logger().info("Retreating...")
        if not self.motion.move_to_pose(retreat_pose):
            self.get_logger().error("Retreat motion failed.")
            return False

        self.get_logger().info("── grasp_world: complete ✅ ──")
        return True

    # =========================================================================
    # Camera-frame grasp (wraps grasp_world with TF transform)
    # =========================================================================

    def grasp_camera(
        self,
        target_pose_camera: PoseStamped,
        dimensions: list[float],
        obstacles: list | None = None,
    ) -> bool:
        """
        Transform `target_pose_camera` from the camera optical frame to the
        world frame, then execute grasp_world().
        """
        self.get_logger().info("Transforming camera-frame pose to world frame...")
        try:
            world_pose = self.tf_buffer.transform(
                target_pose_camera,
                "world",
                timeout=rclpy.duration.Duration(seconds=2.0),
            )
        except Exception as exc:
            self.get_logger().error(f"TF transform failed: {exc}")
            return False

        self.get_logger().info(
            f"World pose: "
            f"x={world_pose.pose.position.x:.3f}, "
            f"y={world_pose.pose.position.y:.3f}, "
            f"z={world_pose.pose.position.z:.3f}"
        )
        return self.grasp_world(world_pose, dimensions, obstacles)

    # =========================================================================
    # Smart grasp — full pipeline with object attachment / detachment
    # =========================================================================

    def smart_grasp(
        self,
        object_id: str,
        target_pose_world: PoseStamped,
        dimensions: list[float],
        obstacles: list | None = None,
    ) -> bool:
        """
        Full collision-aware grasp pipeline.

        Sequence:
          1. Graspability check (GraspPlanner)
          2. Add target object + all obstacles to the planning scene
          3. Move to pre-grasp pose (10 cm above) — OMPL avoids all objects
          4. Disable collision between EEF/gripper and target object
             (object stays in scene so its geometry is still known to MoveIt,
              but the arm is now allowed to descend into it)
          5. Final approach using OMPL
          6. Close gripper
          7. Attach object to EEF (object becomes part of robot model;
             re-enables self-collision checking for the combined body)
          8. Retreat vertically (MoveIt plans with the attached object)

        Args:
            object_id:          Unique ID for the target collision object.
            target_pose_world:  PoseStamped of the object centre (world frame).
            dimensions:         [dx, dy, dz] bounding box (metres).
            obstacles:          Additional nearby collision objects.

        Returns:
            True on complete success, False on any failure.
        """
        self.get_logger().info("── smart_grasp: starting ──")

        GRIPPER_LINKS = [
            "end_effector_link",
            "gripper_link",
            "gripper_left_finger_link",
            "gripper_right_finger_link",
        ]

        # ── 1. Graspability check ─────────────────────────────────────────
        plan = self.planner.evaluate(dimensions)
        if plan is None:
            self.get_logger().error(
                f"Object not graspable with dimensions {dimensions}."
            )
            return False
        self.get_logger().info(
            f"Grasp plan: axis={plan.axis}, width={plan.grasp_width:.4f}m"
        )

        # ── 2. Populate scene: target object + all obstacles ──────────────
        # Object is present as a full collision body so OMPL avoids it
        # during the pre-grasp approach.
        all_objects = list(obstacles or []) + [{
            "id":   object_id,
            "type": "box",
            "pose": target_pose_world,
            "size": dimensions,
        }]
        self.populate_scene(all_objects)


        # ── 3. Pre-grasp — approach along the calculated axis ───
        pre_pose = copy.deepcopy(target_pose_world)
        grasp_pose = copy.deepcopy(target_pose_world)
        
        approach_dist = PRE_GRASP_OFFSET_Z
        self.get_logger().info(f"Using dynamically selected approach axis: {plan.approach_axis}")
        
        if plan.approach_axis == "z":
            # Top-down descent (stops at top surface)
            pre_pose.pose.position.z += (dimensions[2] / 2.0) + approach_dist
            grasp_pose.pose.position.z += (dimensions[2] / 2.0)
            
        elif plan.approach_axis == "x":
            # Frontal slide - pull back towards the robot base (origin)
            dir_x = -1.0 if target_pose_world.pose.position.x > 0 else 1.0
            pre_pose.pose.position.x += dir_x * ((dimensions[0] / 2.0) + approach_dist)
            # grasp_pose stays exactly at the center for a deep slide!
            
        elif plan.approach_axis == "y":
            # Side slide - pull back towards the robot base (origin)
            dir_y = -1.0 if target_pose_world.pose.position.y > 0 else 1.0
            pre_pose.pose.position.y += dir_y * ((dimensions[1] / 2.0) + approach_dist)
            # grasp_pose stays exactly at the center for a deep slide!

        self.get_logger().info("Moving to pre-grasp position...")
        if not self.motion.move_to_pose(pre_pose):
            self.get_logger().error("Pre-grasp motion failed.")
            return False

        # ── 4. Allow gripper to enter object bounding box ───────────
        # We temporarily disable collision between the cylinder and the gripper links
        # so OMPL allows the arm to reach into the object.
        self.get_logger().info("Disabling collision checks for final approach...")
        for link in GRIPPER_LINKS:
            self.scene.set_collision_allowed(object_id, link, allow=True)

        # ── 5. Final approach using OMPL (just like RViz!) ──────────
        self.get_logger().info("Moving to target grasp pose...")
        if not self.motion.move_to_pose(grasp_pose):
            self.get_logger().error("Final approach failed.")
            return False

        # ── 6. Attach object BEFORE closing gripper ───────────────────────
        # attach_object() is the standard suppression mechanism: it moves the
        # object from world.collision_objects into
        # robot_state.attached_collision_objects.  touch_links define which
        # gripper links may remain in contact without triggering self-collision.
        # Attaching first means the gripper close cannot register as a
        # collision error.
        self.get_logger().info(f"Attaching '{object_id}' to end-effector...")
        self.scene.attach_object(
            object_id,
            link_name="end_effector_link",
            touch_links=GRIPPER_LINKS,
        )

        # ── 7. Close gripper ──────────────────────────────────────────────
        self.get_logger().info("Closing gripper...")
        if not self._control_gripper(GRIPPER_CLOSE_POS):
            self.get_logger().error("Gripper close failed.")
            return False

        # ── 8. Retreat with attached object ───────────────────────────────
        lift_pose = copy.deepcopy(grasp_pose)
        lift_pose.pose.position.z += RETREAT_HEIGHT

        self.get_logger().info("Retreating with object attached...")
        if not self.motion.move_to_pose(lift_pose):
            self.get_logger().error("Retreat failed.")
            return False

        self.get_logger().info("── smart_grasp: complete ✅ ──")
        return True

    # =========================================================================
    # Gripper control
    # =========================================================================

    def _control_gripper(self, position: float, effort: float = GRIPPER_EFFORT) -> bool:
        """Send a GripperCommand action goal and wait for completion."""
        goal = GripperCommand.Goal()
        goal.command.position   = position
        goal.command.max_effort = effort

        future = self._gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("Gripper goal rejected.")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info(
            f"Gripper moved to position={position:.4f}."
        )
        return True

    def open_gripper(self) -> bool:
        return self._control_gripper(GRIPPER_OPEN_POS)

    def close_gripper(self) -> bool:
        return self._control_gripper(GRIPPER_CLOSE_POS)


# ─────────────────────────────────────────────────────────────────────────────
# Standalone entry point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Collision-aware grasp node")
    parser.add_argument("--sim", action="store_true",
                        help="Enable simulation mode (use_sim_time=True)")
    args = parser.parse_args(sys.argv[1:])

    rclpy.init()

    node = CollisionAwareGrasp(use_sim_time=args.sim)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
