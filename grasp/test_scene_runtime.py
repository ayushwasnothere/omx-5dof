#!/usr/bin/env python3

import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from scene_manager import SceneManager
from motion_client import MotionClient


class SceneRuntimeTest(Node):

    def __init__(self):
        super().__init__("scene_runtime_test")

        self.scene = SceneManager(self)
        self.motion = MotionClient(self)

        self.get_logger().info("Scene runtime test initialized.")


def create_pose(x=0.3, y=0.0, z=0.1, frame="world"):
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.pose.orientation.w = 1.0
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    return pose


def main():

    rclpy.init()

    node = SceneRuntimeTest()

    # -----------------------------
    # 1️⃣ Clear Scene
    # -----------------------------
    node.get_logger().info("Clearing scene...")
    node.scene.clear()
    time.sleep(1)

    # -----------------------------
    # 2️⃣ Add Obstacle
    # -----------------------------
    obstacle_pose = create_pose(0.25, 0.0, 0.05)

    obstacles = [{
        "id": "test_box",
        "type": "box",
        "pose": obstacle_pose,
        "size": [0.1, 0.1, 0.1]
    }]

    node.get_logger().info("Adding obstacle...")
    node.scene.add_obstacles(obstacles)
    time.sleep(1)

    # -----------------------------
    # 3️⃣ Verify Obstacle Exists
    # -----------------------------
    names = node.scene.get_known_objects()

    if "test_box" in names:
        node.get_logger().info("Obstacle successfully added.")
    else:
        node.get_logger().error("Obstacle NOT found in scene.")

    # -----------------------------
    # 4️⃣ Move Robot
    # -----------------------------
    node.get_logger().info("Sending motion goal...")

    target_pose = create_pose(0.3, 0.1, 0.2)

    success = node.motion.move_to_pose(target_pose)

    if success:
        node.get_logger().info("Motion succeeded.")
    else:
        node.get_logger().error("Motion failed.")

    # -----------------------------
    # Done
    # -----------------------------
    node.get_logger().info("Test complete. Shutting down.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
