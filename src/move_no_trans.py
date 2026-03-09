#!/usr/bin/env python3
"""
Interactive Move-To-Grasp console
Input is in CENTIMETRES.

Commands:
  x y z       → move gripper to position (cm)
  open        → open gripper
  close       → close gripper
  h           → help
  q           → quit
"""
import argparse
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, MotionPlanRequest, MoveItErrorCodes, PositionConstraint,
)
from control_msgs.action import GripperCommand
from shape_msgs.msg import SolidPrimitive


# ── tune these to match your robot ──────────────────────
# Check your actual root frame: ros2 run tf2_tools view_frames
BASE_FRAME     = "world"
EEF_LINK       = "end_effector_link"
PLANNING_GROUP = "arm"
MOVE_ACTION    = "/move_action"
GRIPPER_ACTION = "/gripper_controller/gripper_cmd"

GRIPPER_OPEN   =  0.018
GRIPPER_CLOSE  = -0.010
GRIPPER_EFFORT =  8.0
# ────────────────────────────────────────────────────────


class MoveToGrasp(Node):
    def __init__(self, use_sim_time: bool = False):
        super().__init__(
            "move_to_grasp",
            parameter_overrides=[
                rclpy.parameter.Parameter(
                    "use_sim_time",
                    rclpy.parameter.Parameter.Type.BOOL,
                    use_sim_time
                )
            ]
        )
        mode = "SIMULATION (Gazebo)" if use_sim_time else "REAL HARDWARE"
        self.get_logger().info(f"Mode: {mode}")

        self.moveit_client  = ActionClient(self, MoveGroup,      MOVE_ACTION)
        self.gripper_client = ActionClient(self, GripperCommand, GRIPPER_ACTION)

        self.get_logger().info("Waiting for MoveIt...")
        self.moveit_client.wait_for_server()

        self.get_logger().info("Waiting for gripper...")
        self.gripper_client.wait_for_server()

        self.get_logger().info("Ready ✅")
        threading.Thread(target=self.console_loop, daemon=True).start()

    # ── thread-safe future wait (same as reference) ──────
    def wait_future(self, future):
        while not future.done():
            time.sleep(0.01)

    # ── move: position-only, exactly like your reference ─
    def move_to(self, x, y, z) -> bool:
        """x, y, z in metres."""
        self.get_logger().info(
            f"-> EEF target  x={x*100:.1f}cm  y={y*100:.1f}cm  z={z*100:.1f}cm"
        )

        pose = PoseStamped()
        pose.header.frame_id    = BASE_FRAME
        pose.pose.position.x    = x
        pose.pose.position.y    = y
        pose.pose.position.z    = z
        pose.pose.orientation.w = 1.0   # fixed — same as working reference

        box = SolidPrimitive()
        box.type       = SolidPrimitive.BOX
        box.dimensions = [0.01, 0.01, 0.01]

        pos_con = PositionConstraint()
        pos_con.header    = pose.header
        pos_con.link_name = EEF_LINK
        pos_con.constraint_region.primitives.append(box)
        pos_con.constraint_region.primitive_poses.append(pose.pose)
        pos_con.weight    = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pos_con)

        req = MotionPlanRequest()
        req.group_name                      = PLANNING_GROUP
        req.allowed_planning_time           = 5.0
        req.num_planning_attempts           = 5
        req.max_velocity_scaling_factor     = 0.15
        req.max_acceleration_scaling_factor = 0.15
        req.goal_constraints.append(constraints)

        goal         = MoveGroup.Goal()
        goal.request = req

        future = self.moveit_client.send_goal_async(goal)
        self.wait_future(future)

        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().error("Goal rejected by MoveIt")
            return False

        result_future = handle.get_result_async()
        self.wait_future(result_future)

        ok = result_future.result().result.error_code.val == MoveItErrorCodes.SUCCESS
        self.get_logger().info("Move succeeded" if ok else "Move FAILED")
        return ok

    # ── gripper ──────────────────────────────────────────
    def control_gripper(self, open_gripper: bool) -> bool:
        goal = GripperCommand.Goal()
        goal.command.position   = GRIPPER_OPEN if open_gripper else GRIPPER_CLOSE
        goal.command.max_effort = GRIPPER_EFFORT

        future = self.gripper_client.send_goal_async(goal)
        self.wait_future(future)

        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().error("Gripper goal rejected")
            return False

        result_future = handle.get_result_async()
        self.wait_future(result_future)

        self.get_logger().info(f"Gripper {'opened' if open_gripper else 'closed'}")
        return True

    # ── console ───────────────────────────────────────────
    def console_loop(self):
        HELP = """
+------------------------------------------+
|      Move-To-Grasp  (input in cm)        |
+------------------------------------------+
|  x y z    ->  move to position           |
|  open     ->  open gripper               |
|  close    ->  close gripper              |
|  h        ->  show this help             |
|  q        ->  quit                       |
+------------------------------------------+
Example:  28.5 13 10   (= 0.285m 0.13m 0.10m)
"""
        print(HELP)

        while rclpy.ok():
            try:
                raw = input(">> ").strip()
            except (EOFError, KeyboardInterrupt):
                rclpy.shutdown()
                break

            if not raw:
                continue

            tokens = raw.split()
            cmd    = tokens[0].lower()

            if cmd == "q":
                print("Bye!")
                rclpy.shutdown()
                break

            elif cmd == "h":
                print(HELP)

            elif cmd == "open":
                self.control_gripper(open_gripper=True)

            elif cmd == "close":
                self.control_gripper(open_gripper=False)

            elif len(tokens) == 3:
                try:
                    # input in cm -> convert to metres internally
                    x = float(tokens[0]) / 100.0
                    y = float(tokens[1]) / 100.0
                    z = float(tokens[2]) / 100.0
                except ValueError:
                    print("Invalid input. Example:  28.5 13 10")
                    continue

                self.move_to(x, y, z)

            else:
                print("Unknown command. Type 'h' for help.")


def main():
    parser = argparse.ArgumentParser(description="Move-To-Grasp console")
    parser.add_argument(
        "--sim", action="store_true",
        help="Use simulation time (Gazebo). Omit this flag for real hardware."
    )
    # strip ROS args before parsing
    import sys
    args = parser.parse_args(sys.argv[1:])

    rclpy.init()
    node = MoveToGrasp(use_sim_time=args.sim)

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
