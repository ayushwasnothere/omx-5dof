#!/usr/bin/env python3

import unittest
from unittest.mock import patch
import rclpy
from geometry_msgs.msg import PoseStamped
from mtc_grasp_node import MTCSmartGrasp


class TestMTCGraspUnit(unittest.TestCase):

    def setUp(self):

        if not rclpy.ok():
            rclpy.init()

        self.node = MTCSmartGrasp()

    def tearDown(self):
        self.node.destroy_node()

    @patch("mtc_grasp_node.core.Task")
    @patch("mtc_grasp_node.core.PipelinePlanner")
    @patch("mtc_grasp_node.core.CartesianPath")
    @patch("mtc_grasp_node.core.SerialContainer")
    @patch("mtc_grasp_node.stages.CurrentState")
    @patch("mtc_grasp_node.stages.Connect")
    @patch("mtc_grasp_node.stages.MoveRelative")
    @patch("mtc_grasp_node.stages.MoveTo")
    @patch("mtc_grasp_node.stages.ModifyPlanningScene")
    def test_task_composition(
        self,
        mock_scene,
        mock_moveto,
        mock_moverel,
        mock_connect,
        mock_state,
        mock_container,
        mock_cartesian,
        mock_pipeline,
        mock_task,
    ):

        pose = PoseStamped()
        pose.pose.position.x = 0.3
        dims = [0.02, 0.02, 0.2]

        self.node.create_pick_task("test_obj", pose, dims)

        # Verify stages were constructed
        mock_state.assert_called_once()
        mock_connect.assert_called_once()
        mock_container.assert_called_once()

        # Verify task.add() was used
        task_instance = mock_task.return_value
        assert task_instance.add.call_count == 3

class TestMTCGraspIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

        cls.node = MTCSmartGrasp()

    def test_execute_grasp(self):

        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = 0.28
        pose.pose.position.y = -0.1
        pose.pose.position.z = 0.1
        pose.pose.orientation.w = 1.0

        dims = [0.028, 0.028, 0.20]

        result = self.node.execute_grasp("test_cylinder", pose, dims)

        self.assertTrue(result)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
