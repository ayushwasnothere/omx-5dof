#!/usr/bin/env python3

import rclpy
import time
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped

from grasp_node import CollisionAwareGrasp


def main():

    rclpy.init()

    node = CollisionAwareGrasp(use_sim_time=True)

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # -------------------------------------------------
    # Allow TF buffer to receive /tf and /tf_static
    # -------------------------------------------------
    node.get_logger().info("Waiting for TF to populate...")
    for _ in range(20):  # ~2 seconds
        executor.spin_once(timeout_sec=0.1)

    # -------------------------------------------------
    # Create camera-frame pose
    # -------------------------------------------------
    camera_pose = PoseStamped()
    camera_pose.header.frame_id = "camera_color_optical_frame"
    camera_pose.header.stamp = node.get_clock().now().to_msg()

    # -10 cm, 0 cm, 46 cm  (converted to meters)
    camera_pose.pose.position.x = -0.10
    camera_pose.pose.position.y = 0.0
    camera_pose.pose.position.z = 0.46
    camera_pose.pose.orientation.w = 1.0

    dimensions = [0.02, 0.03, 0.05]  # dummy object size

    node.get_logger().info("Testing grasp_camera()")

    success = node.grasp_camera(camera_pose, dimensions)

    if success:
        node.get_logger().info("Camera-frame grasp succeeded.")
    else:
        node.get_logger().error("Camera-frame grasp failed.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
