#!/usr/bin/env python3

import pytest
import rclpy
import time

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from scene_manager import SceneManager


# -------------------------------------------------------
# Pytest ROS2 Fixture
# -------------------------------------------------------
@pytest.fixture(scope="module")
def ros_node():

    rclpy.init()
    node = Node("test_scene_manager_node")

    yield node

    node.destroy_node()
    rclpy.shutdown()


@pytest.fixture
def scene_manager(ros_node):
    return SceneManager(ros_node)


# -------------------------------------------------------
# Helper
# -------------------------------------------------------
def create_pose(frame="world"):
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.pose.orientation.w = 1.0
    return pose


# -------------------------------------------------------
# 1️⃣ Test Add Single Box
# -------------------------------------------------------
def test_add_box(scene_manager):

    scene_manager.clear()

    obstacles = [{
        "id": "box1",
        "type": "box",
        "pose": create_pose(),
        "size": [0.1, 0.1, 0.1]
    }]

    scene_manager.add_obstacles(obstacles)

    time.sleep(1)

    names = scene_manager.scene.get_known_object_names()

    assert "box1" in names


# -------------------------------------------------------
# 2️⃣ Test Add Multiple Objects
# -------------------------------------------------------
def test_add_multiple_objects(scene_manager):

    scene_manager.clear()

    obstacles = [
        {
            "id": "boxA",
            "type": "box",
            "pose": create_pose(),
            "size": [0.1, 0.1, 0.1]
        },
        {
            "id": "sphereB",
            "type": "sphere",
            "pose": create_pose(),
            "size": [0.05]
        },
        {
            "id": "cylC",
            "type": "cylinder",
            "pose": create_pose(),
            "size": [0.2, 0.03]
        }
    ]

    scene_manager.add_obstacles(obstacles)

    time.sleep(1)

    names = scene_manager.scene.get_known_object_names()

    assert "boxA" in names
    assert "sphereB" in names
    assert "cylC" in names


# -------------------------------------------------------
# 3️⃣ Test Clear Scene
# -------------------------------------------------------
def test_clear_scene(scene_manager):

    scene_manager.clear()
    time.sleep(1)

    names = scene_manager.scene.get_known_object_names()

    assert len(names) == 0


# -------------------------------------------------------
# 4️⃣ Test Re-Add After Clear
# -------------------------------------------------------
def test_re_add_after_clear(scene_manager):

    scene_manager.clear()

    obstacles = [{
        "id": "box_readd",
        "type": "box",
        "pose": create_pose(),
        "size": [0.1, 0.1, 0.1]
    }]

    scene_manager.add_obstacles(obstacles)
    time.sleep(1)

    names = scene_manager.scene.get_known_object_names()

    assert "box_readd" in names
