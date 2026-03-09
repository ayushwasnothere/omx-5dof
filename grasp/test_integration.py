#!/usr/bin/env python3
# test_smart_grasp_integration.py

import os
import time
import unittest
import pytest

# ── Camera-frame pose from your RealSense (-5 cm depth correction applied) ───
CAMERA_FRAME = "camera_color_optical_frame"
CAMERA_X     = -0.10   # m (-10 cm)
CAMERA_Y     =  0.00   # m
CAMERA_Z     =  0.46   # m (46 cm)

# SIM=1 → Gazebo  |  (unset) → real hardware
USE_SIM      = os.environ.get("SIM", "0") == "1"


def _ros_available() -> bool:
    try:
        import rclpy  # noqa: F401
        return True
    except ImportError:
        return False


@pytest.mark.skipif(not _ros_available(), reason="ROS2 not available")
class TestSmartGraspIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        print("\n[TEST LOG] --------------------------------------------------")
        print(f"[TEST LOG] Starting Test Suite. SIM Mode: {USE_SIM}")
        print("[TEST LOG] Importing ROS 2 modules...")
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from grasp_node import CollisionAwareGrasp

        print("[TEST LOG] Initializing rclpy...")
        rclpy.init()
        
        print("[TEST LOG] Instantiating CollisionAwareGrasp Node...")
        print("[TEST LOG] ⚠️ IF IT HANGS HERE: Your MoveIt/Gripper Action Servers are NOT running!")
        cls.node = CollisionAwareGrasp(use_sim_time=USE_SIM)
        print("[TEST LOG] Node instantiated successfully!")

        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)

        # Spin until TF buffer has data and MoveIt publishes joint state
        settle = 3.0 if USE_SIM else 2.0
        print(f"[TEST LOG] Spinning executor for {settle} seconds to settle TF/Joint States...")
        t0 = time.time()
        while time.time() - t0 < settle:
            cls.executor.spin_once(timeout_sec=0.1)
        print("[TEST LOG] Setup complete. Ready to run tests.")
        print("[TEST LOG] --------------------------------------------------\n")

    @classmethod
    def tearDownClass(cls):
        print("\n[TEST LOG] Tearing down test suite...")
        import rclpy
        cls.node.destroy_node()
        rclpy.shutdown()
        print("[TEST LOG] Teardown complete.")

    # ── helpers ───────────────────────────────────────────────────────────────

    def _camera_pose(self, x=CAMERA_X, y=CAMERA_Y, z=CAMERA_Z):
        from geometry_msgs.msg import PoseStamped
        p = PoseStamped()
        p.header.frame_id = CAMERA_FRAME
        p.header.stamp    = self.node.get_clock().now().to_msg()
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = z
        p.pose.orientation.w = 1.0
        return p

    def _spin(self, secs=0.5):
        t0 = time.time()
        while time.time() - t0 < secs:
            self.executor.spin_once(timeout_sec=0.05)

    # ── tests ─────────────────────────────────────────────────────────────────

    def test_smart_grasp_cylinder(self):
        """
        Tests the smart_grasp pipeline for a cylindrical object.
        Dimensions: h = 20 cm, w = 5 cm
        Camera Position: x = -10 cm, y = 0 cm, z = 46 cm
        """
        import rclpy.duration
        print("\n[TEST LOG] >>> RUNNING test_smart_grasp_cylinder")

        # 1. Define dimensions in meters
        cylinder_dims = [0.028, 0.028, 0.20]  # dx, dy, dz
        print(f"[TEST LOG] Cylinder dimensions set to: {cylinder_dims} meters")

        # 2. Define the camera-frame pose (in meters)
        cam_pose = self._camera_pose(x=-0.10, y=0.00, z=0.46)
        print(f"[TEST LOG] Camera pose defined at x={cam_pose.pose.position.x}, y={cam_pose.pose.position.y}, z={cam_pose.pose.position.z}")

        # 3. Transform to world frame
        print("[TEST LOG] Waiting for TF transform from camera frame to world frame...")
        try:
            world_pose = self.node.tf_buffer.transform(
                cam_pose,
                "world",
                timeout=rclpy.duration.Duration(seconds=3.0)
            )
            # This ONLY changes this specific object's bounding box rotation
            # Your TF tree and position remain completely unaffected!
            world_pose.pose.orientation.x = 0.0
            world_pose.pose.orientation.y = 0.0
            world_pose.pose.orientation.z = 0.0
            world_pose.pose.orientation.w = 1.0
            print(f"[TEST LOG] Transform SUCCESS! World pose: x={world_pose.pose.position.x:.3f}, y={world_pose.pose.position.y:.3f}, z={world_pose.pose.position.z:.3f}")
        except Exception as exc:
            self.fail(f"[TEST LOG] TF transform failed! Make sure your static TF broadcaster is running. Error: {exc}")

        # 3.5 ADJUST GRIPPER LIMITS FOR THIS TEST TO MATCH REAL HARDWARE
        print("[TEST LOG] Adjusting GraspPlanner limits for the 20x5cm cylinder...")
        self.node.planner.max_width = 0.07     # hardware max is 7.5cm, capped safely at 7cm
        self.node.planner.max_height = 0.25    # allow up to 25 cm height (object is 20cm)
        self.node.planner.finger_depth = 0.06  # safe buffer for the OpenManipulator fingers

        # 4. Invoke the full smart_grasp pipeline
        print("[TEST LOG] Calling self.node.smart_grasp()...")
        result = self.node.smart_grasp(
            object_id="tall_cylinder_test",
            target_pose_world=world_pose,
            dimensions=cylinder_dims,
        )
        
        # 5. Assert it completes successfully
        print(f"[TEST LOG] smart_grasp returned: {result}")
        self.assertTrue(result, "smart_grasp failed to pick up the 20x5cm cylinder.")
        print("[TEST LOG] <<< FINISHED test_smart_grasp_cylinder SUCCESS\n")

if __name__ == "__main__":
    unittest.main(verbosity=2)
