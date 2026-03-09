#!/usr/bin/env python3
"""
Interactive Move-To-Grasp console — camera frame input
Input is in CENTIMETRES, in the CAMERA frame.
tf2 handles the transform to world frame automatically.

Setup (run once, outside this script):
  ros2 run tf2_ros static_transform_publisher \
    --x 0.5 --y 0.0 --z 0.8 \
    --roll 0 --pitch 0 --yaw 0 \
    --frame-id world --child-frame-id camera_frame

  Replace the numbers with your actual camera position/orientation.
  Once TF is broadcasting, this script does the rest.

Commands at prompt:
  x y z    → move gripper to position (cm, camera frame)
  open     → open gripper
  close    → close gripper
  h        → help
  q        → quit
"""
import argparse
import sys
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

# tf2
import tf2_ros
import tf2_geometry_msgs  # registers PoseStamped transform support

from geometry_msgs.msg import PoseStamped, PointStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, MotionPlanRequest, MoveItErrorCodes, PositionConstraint,
)
from control_msgs.action import GripperCommand
from shape_msgs.msg import SolidPrimitive


# ── config ───────────────────────────────────────────────
BASE_FRAME     = "world"         # root of your TF tree
CAMERA_FRAME   = "camera_color_optical_frame"  # the frame your camera detects objects in
EEF_LINK       = "end_effector_link"
PLANNING_GROUP = "arm"
MOVE_ACTION    = "/move_action"
GRIPPER_ACTION = "/gripper_controller/gripper_cmd"

GRIPPER_OPEN   =  0.018
GRIPPER_CLOSE  = -0.010
GRIPPER_EFFORT =  8.0

TF_TIMEOUT_SEC = 2.0   # how long to wait for a TF lookup
# ─────────────────────────────────────────────────────────


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

        # tf2 buffer + listener
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.moveit_client  = ActionClient(self, MoveGroup,      MOVE_ACTION)
        self.gripper_client = ActionClient(self, GripperCommand, GRIPPER_ACTION)

        self.get_logger().info("Waiting for MoveIt...")
        self.moveit_client.wait_for_server()

        self.get_logger().info("Waiting for gripper...")
        self.gripper_client.wait_for_server()

        self.get_logger().info("Ready ✅")
        threading.Thread(target=self.console_loop, daemon=True).start()

    # ── thread-safe future wait ───────────────────────────
    def wait_future(self, future):
        while not future.done():
            time.sleep(0.01)

    # ── camera → world via tf2 ────────────────────────────
    def cam_to_world(self, cx: float, cy: float, cz: float):
        """
        Transform a point from CAMERA_FRAME to BASE_FRAME using live TF.
        cx, cy, cz in metres. Returns (wx, wy, wz) in metres, or None on failure.
        """
        point_in_cam = PointStamped()
        point_in_cam.header.frame_id    = CAMERA_FRAME
        point_in_cam.header.stamp       = self.get_clock().now().to_msg()
        point_in_cam.point.x            = cx
        point_in_cam.point.y            = cy
        point_in_cam.point.z            = cz

        try:
            point_in_world = self.tf_buffer.transform(
                point_in_cam,
                BASE_FRAME,
                timeout=rclpy.duration.Duration(seconds=TF_TIMEOUT_SEC)
            )
            return (
                point_in_world.point.x,
                point_in_world.point.y,
                point_in_world.point.z,
            )
        except tf2_ros.LookupException:
            self.get_logger().error(
                f"TF lookup failed: '{CAMERA_FRAME}' → '{BASE_FRAME}' not found.\n"
                f"  Run: ros2 run tf2_ros static_transform_publisher "
                f"--x X --y Y --z Z --roll R --pitch P --yaw Y "
                f"--frame-id {BASE_FRAME} --child-frame-id {CAMERA_FRAME}"
            )
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().error(f"TF extrapolation error: {e}")
        except Exception as e:
            self.get_logger().error(f"TF error: {e}")
        return None

    # ── MoveIt move (position only) ───────────────────────
    def move_to(self, x, y, z) -> bool:
        """x, y, z in metres, world frame."""
        self.get_logger().info(
            f"-> EEF target  x={x*100:.1f}cm  y={y*100:.1f}cm  z={z*100:.1f}cm  (world)"
        )

        pose = PoseStamped()
        pose.header.frame_id    = BASE_FRAME
        pose.pose.position.x    = x
        pose.pose.position.y    = y
        pose.pose.position.z    = z
        pose.pose.orientation.w = 1.0

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

    # ── gripper ───────────────────────────────────────────
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
        HELP = f"""
+----------------------------------------------------+
|      Move-To-Grasp  (camera frame input, cm)       |
+----------------------------------------------------+
|  x y z   ->  move gripper (coords in camera frame) |
|  open    ->  open gripper                          |
|  close   ->  close gripper                         |
|  h       ->  this help                             |
|  q       ->  quit                                  |
+----------------------------------------------------+
  Camera frame : {CAMERA_FRAME}
  Robot frame  : {BASE_FRAME}
  TF does the transform automatically.

  Example:  15 -5 30
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
                    cx = float(tokens[0]) / 100.0  # cm → m
                    cy = float(tokens[1]) / 100.0
                    cz = float(tokens[2]) / 100.0
                except ValueError:
                    print("Invalid input. Example:  15 -5 30")
                    continue

                result = self.cam_to_world(cx, cy, cz)
                if result is None:
                    print("Transform failed — check TF is broadcasting (see error above)")
                    continue

                wx, wy, wz = result
                self.get_logger().info(
                    f"  cam=({cx*100:.1f}, {cy*100:.1f}, {cz*100:.1f})cm"
                    f"  ->  world=({wx*100:.1f}, {wy*100:.1f}, {wz*100:.1f})cm"
                )
                self.move_to(wx, wy, wz)

            else:
                print("Unknown command. Type 'h' for help.")


def main():
    parser = argparse.ArgumentParser(description="Move-To-Grasp (camera frame input)")
    parser.add_argument("--sim", action="store_true",
                        help="Use sim time (Gazebo). Omit for real hardware.")
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
