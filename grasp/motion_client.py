# motion_client.py
#
# Thin wrapper around the MoveGroup action server.
# Handles pose goals and Cartesian path execution.
# No scene logic lives here — that belongs to SceneManager.

import copy

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    Constraints,
    MotionPlanRequest,
    PositionConstraint,
    OrientationConstraint,
    MoveItErrorCodes,
    RobotState,
)
from moveit_msgs.srv import GetCartesianPath
from shape_msgs.msg import SolidPrimitive


class MotionClient:
    """
    Sends motion goals to MoveIt's MoveGroup action server.

    Two movement modes:
      move_to_pose()      — free-space planning (OMPL)
      cartesian_path()    — linear interpolated Cartesian trajectory
    """

    PLANNING_GROUP = "arm"
    EEF_LINK = "end_effector_link"

    def __init__(self, node: Node, planning_time: float = 5.0, sim_mode: bool = False):
        self.node = node
        self.sim_mode = sim_mode

        # In simulation we allow higher velocity — no risk to real hardware.
        # On real hardware keep conservative defaults for safety.
        if sim_mode:
            self.vel_scale  = 0.5
            self.acc_scale  = 0.5
            self.planning_time = planning_time or 10.0  # Gazebo clock can lag
        else:
            self.vel_scale  = 0.2
            self.acc_scale  = 0.2
            self.planning_time = planning_time or 5.0

        self._move_client = ActionClient(node, MoveGroup, "/move_action")
        self._exec_client = ActionClient(node, ExecuteTrajectory, "/execute_trajectory")
        self._cartesian_client = node.create_client(
            GetCartesianPath, "/compute_cartesian_path"
        )

        self.node.get_logger().info("MotionClient: waiting for MoveGroup action server...")
        self._move_client.wait_for_server()
        self._exec_client.wait_for_server()
        self._cartesian_client.wait_for_service()
        self.node.get_logger().info("MotionClient: ready.")

    # ------------------------------------------------------------------
    # Free-space motion (OMPL)
    # ------------------------------------------------------------------

    def move_to_pose(self, pose: PoseStamped) -> bool:
        """
        Plan and execute a collision-free trajectory to `pose`.
        Returns True on success.
        """
        goal = self._build_pose_goal(pose)

        future = self._move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)

        handle = future.result()
        if not handle.accepted:
            self.node.get_logger().error("MotionClient: move_to_pose goal rejected.")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)

        result = result_future.result().result
        success = result.error_code.val == MoveItErrorCodes.SUCCESS
        self.node.get_logger().info(
            f"MotionClient: move_to_pose {'OK' if success else 'FAILED'} "
            f"(code={result.error_code.val})"
        )
        return success

    # ------------------------------------------------------------------
    # Cartesian path (linear interpolation)
    # ------------------------------------------------------------------

    def cartesian_path(
        self,
        waypoints: list,
        eef_step: float = 0.005,
        jump_threshold: float = 0.0,
        min_fraction: float = 0.95,
        avoid_collisions: bool = True,
    ) -> bool:
        """
        Execute a Cartesian trajectory through `waypoints` (list of Pose).
        Returns True if the achieved fraction ≥ min_fraction.

        avoid_collisions:
            True  — standard collision-checked path (pre-grasp, retreat).
            False — final approach only: allows the gripper to enter the
                    object bounding box. Scoped to this trajectory only;
                    does NOT modify the global ACM.
        """
        req = GetCartesianPath.Request()
        req.group_name = self.PLANNING_GROUP
        req.link_name = self.EEF_LINK
        req.waypoints = waypoints
        req.max_step = eef_step
        req.jump_threshold = jump_threshold
        req.avoid_collisions = avoid_collisions

        future = self._cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

        res = future.result()
        fraction = res.fraction
        self.node.get_logger().info(
            f"MotionClient: Cartesian path fraction={fraction:.2f}"
        )

        if fraction < min_fraction:
            self.node.get_logger().error(
                f"MotionClient: Cartesian path below threshold "
                f"({fraction:.2f} < {min_fraction})"
            )
            return False

        # Execute the planned trajectory
        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = res.solution

        exec_future = self._exec_client.send_goal_async(exec_goal)
        rclpy.spin_until_future_complete(self.node, exec_future)

        exec_handle = exec_future.result()
        if not exec_handle.accepted:
            self.node.get_logger().error("MotionClient: execute trajectory rejected.")
            return False

        exec_result_future = exec_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, exec_result_future)

        exec_result = exec_result_future.result().result
        success = exec_result.error_code.val == MoveItErrorCodes.SUCCESS
        self.node.get_logger().info(
            f"MotionClient: execute trajectory {'OK' if success else 'FAILED'}"
        )
        return success

    # ------------------------------------------------------------------
    # Private builders
    # ------------------------------------------------------------------

    def _build_pose_goal(self, pose: PoseStamped) -> MoveGroup.Goal:
        """Construct a MoveGroup goal from a target PoseStamped."""
        # Position constraint — small tolerance box around target
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.01, 0.01, 0.01]

        pos_con = PositionConstraint()
        pos_con.header = pose.header
        pos_con.link_name = self.EEF_LINK
        pos_con.constraint_region.primitives.append(box)
        pos_con.constraint_region.primitive_poses.append(pose.pose)
        pos_con.weight = 1.0

        # NOTE: OrientationConstraint intentionally omitted.
        # Locking orientation with a generic w=1.0 quaternion causes
        # PLANNING_FAILED (code=99999) on most real/sim configurations because
        # OMPL cannot find an IK solution satisfying both position and a fixed
        # orientation simultaneously.  For a pick-and-place the planner should
        # be free to choose the best EEF orientation — orientation is enforced
        # implicitly by the Cartesian descent, not the free-space approach.
        constraints = Constraints()
        constraints.position_constraints.append(pos_con)

        req = MotionPlanRequest()
        req.group_name = self.PLANNING_GROUP
        req.allowed_planning_time = self.planning_time
        req.num_planning_attempts = 5
        req.max_velocity_scaling_factor = self.vel_scale
        req.max_acceleration_scaling_factor = self.acc_scale
        req.goal_constraints.append(constraints)

        goal = MoveGroup.Goal()
        goal.request = req
        return goal
