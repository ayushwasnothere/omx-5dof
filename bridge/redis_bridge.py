#!/usr/bin/env python3
"""
Redis ↔ ROS2 Bridge for Open Manipulator X.

Listens on a Redis stream (robot.tasks) for incoming task requests,
maps each skill to a ROS2 service call, and publishes results back
on another Redis stream (robot.events).

Usage:
    # Terminal must have ROS2 environment sourced
    cd ~/omx_ws && source install/setup.bash
    python3 bridge/redis_bridge.py

Task format (Redis XADD):
    XADD robot.tasks * task_id <uuid> skill <name> params <json>

Event format (published on robot.events):
    {task_id, status: "success"|"error", data: <json>}

Supported skills:
    pick, place, pour, rotate, go_home, add_objects, remove_objects, get_objects
"""

import json
import signal
import sys
import time
import uuid

import rclpy
from rclpy.node import Node

import redis

from geometry_msgs.msg import PoseStamped
from omx_interfaces.msg import SceneObject
from omx_interfaces.srv import (
    AddObjects,
    GetObjects,
    GoHome,
    Pick,
    Place,
    Pour,
    RemoveObjects,
    Rotate,
)

# ── Redis config ──────────────────────────────────────────────────────────
REDIS_HOST = "localhost"
REDIS_PORT = 6379
TASK_STREAM = "robot.tasks"
EVENT_STREAM = "robot.events"
TASK_GROUP = "ros_bridge"
CONSUMER_NAME = f"bridge-{uuid.uuid4().hex[:8]}"
BLOCK_MS = 1000  # poll interval


class RedisBridge(Node):
    """ROS2 node that bridges Redis streams to manipulator services."""

    def __init__(self):
        super().__init__("redis_bridge")
        self.get_logger().info(f"Redis bridge starting (consumer={CONSUMER_NAME})")

        # ── Redis connection ──────────────────────────────────────────────
        self.r = redis.Redis(host=REDIS_HOST, port=REDIS_PORT, decode_responses=True)
        self.r.ping()
        self.get_logger().info("Connected to Redis")

        # Create consumer group (ignore if already exists)
        try:
            self.r.xgroup_create(TASK_STREAM, TASK_GROUP, id="0", mkstream=True)
            self.get_logger().info(f"Created consumer group '{TASK_GROUP}' on '{TASK_STREAM}'")
        except redis.exceptions.ResponseError as e:
            if "BUSYGROUP" in str(e):
                self.get_logger().info(f"Consumer group '{TASK_GROUP}' already exists")
            else:
                raise

        # Remove stale consumers from previous runs
        self._cleanup_stale_consumers()

        # ── ROS2 service clients ─────────────────────────────────────────
        self._srv_clients = {
            "pick": self.create_client(Pick, "/manipulator_node/pick"),
            "place": self.create_client(Place, "/manipulator_node/place"),
            "pour": self.create_client(Pour, "/manipulator_node/pour"),
            "rotate": self.create_client(Rotate, "/manipulator_node/rotate"),
            "go_home": self.create_client(GoHome, "/manipulator_node/go_home"),
            "add_objects": self.create_client(AddObjects, "/manipulator_node/add_objects"),
            "remove_objects": self.create_client(RemoveObjects, "/manipulator_node/remove_objects"),
            "get_objects": self.create_client(GetObjects, "/manipulator_node/get_objects"),
        }

        # ── Skill → request builder mapping ──────────────────────────────
        self._builders = {
            "pick": self._build_pick,
            "place": self._build_place,
            "pour": self._build_pour,
            "rotate": self._build_rotate,
            "go_home": self._build_go_home,
            "add_objects": self._build_add_objects,
            "remove_objects": self._build_remove_objects,
            "get_objects": self._build_get_objects,
        }

        self._running = True
        self.get_logger().info(f"Redis bridge ready — listening on '{TASK_STREAM}'")

    # =====================================================================
    # Main loop
    # =====================================================================

    def spin_bridge(self):
        """Block-read from Redis stream and dispatch to ROS2 services."""
        while self._running and rclpy.ok():
            try:
                entries = self.r.xreadgroup(
                    TASK_GROUP,
                    CONSUMER_NAME,
                    {TASK_STREAM: ">"},
                    count=1,
                    block=BLOCK_MS,
                )
            except redis.exceptions.ConnectionError:
                self.get_logger().warn("Redis connection lost, retrying in 2s...")
                time.sleep(2)
                continue

            if not entries:
                continue

            for _stream, messages in entries:
                for msg_id, fields in messages:
                    self._handle_task(msg_id, fields)

    def _handle_task(self, msg_id: str, fields: dict):
        """Process a single task from the stream."""
        task_id = fields.get("task_id", msg_id)
        skill = fields.get("skill", "")
        params_raw = fields.get("params", "{}")

        self.get_logger().info(f"Task {task_id}: skill={skill}")

        try:
            params = json.loads(params_raw)
        except json.JSONDecodeError as e:
            self._emit_event(task_id, "error", {"reason": f"Invalid JSON params: {e}"})
            self.r.xack(TASK_STREAM, TASK_GROUP, msg_id)
            return

        # Ping / health check — respond immediately, no ROS2 call
        if skill == "ping":
            self._emit_event(task_id, "success", {"status": "alive", "consumer": CONSUMER_NAME})
            self.r.xack(TASK_STREAM, TASK_GROUP, msg_id)
            return

        if skill not in self._srv_clients:
            self._emit_event(task_id, "error", {"reason": f"Unknown skill: {skill}"})
            self.r.xack(TASK_STREAM, TASK_GROUP, msg_id)
            return

        # Emit "running" event
        self._emit_event(task_id, "running", {"skill": skill})

        try:
            # Build ROS2 request
            request = self._builders[skill](params)

            # Call service (synchronous wait)
            client = self._srv_clients[skill]
            if not client.wait_for_service(timeout_sec=5.0):
                self._emit_event(task_id, "error", {"reason": f"Service for '{skill}' not available"})
                self.r.xack(TASK_STREAM, TASK_GROUP, msg_id)
                return

            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=500.0)

            if future.result() is None:
                self._emit_event(task_id, "error", {"reason": "Service call timed out or failed"})
            else:
                response = future.result()
                result_data = self._serialize_response(skill, response)
                status = "success" if getattr(response, "success", True) else "error"
                self._emit_event(task_id, status, result_data)

        except Exception as e:
            self.get_logger().error(f"Task {task_id} exception: {e}")
            self._emit_event(task_id, "error", {"reason": str(e)})

        # Acknowledge message
        self.r.xack(TASK_STREAM, TASK_GROUP, msg_id)

    # =====================================================================
    # Event publishing
    # =====================================================================

    def _emit_event(self, task_id: str, status: str, data: dict):
        """Publish an event to the robot.events stream."""
        self.r.xadd(EVENT_STREAM, {
            "task_id": task_id,
            "status": status,
            "data": json.dumps(data),
        })
        self.get_logger().info(f"Event: task_id={task_id} status={status}")

    # =====================================================================
    # Request builders — convert JSON params to ROS2 service requests
    # =====================================================================

    def _build_pick(self, params: dict) -> Pick.Request:
        req = Pick.Request()
        req.object_id = params["object_id"]
        return req

    def _build_place(self, params: dict) -> Place.Request:
        req = Place.Request()
        req.use_target_pose = params.get("use_target_pose", False)
        req.use_camera_frame = params.get("use_camera_frame", False)
        if req.use_target_pose and "target_pose" in params:
            req.target_pose = self._make_pose_stamped(params["target_pose"])
        return req

    def _build_pour(self, params: dict) -> Pour.Request:
        req = Pour.Request()
        req.target_object_id = params["target_object_id"]
        req.pour_angle = float(params.get("pour_angle", 2.0))
        return req

    def _build_rotate(self, params: dict) -> Rotate.Request:
        req = Rotate.Request()
        req.angle = float(params.get("angle", 3.14))
        req.cycles = int(params.get("cycles", 0))
        return req

    def _build_go_home(self, params: dict) -> GoHome.Request:
        req = GoHome.Request()
        req.state_name = params.get("state_name", "home")
        return req

    def _build_add_objects(self, params: dict) -> AddObjects.Request:
        req = AddObjects.Request()
        for obj in params.get("objects", []):
            so = SceneObject()
            so.id = obj["id"]
            so.type = obj.get("type", "box")
            so.pose = self._make_pose_stamped(obj.get("pose", {}))
            so.dimensions = [float(d) for d in obj.get("dimensions", [])]
            so.use_camera_frame = obj.get("use_camera_frame", False)
            req.objects.append(so)
        return req

    def _build_remove_objects(self, params: dict) -> RemoveObjects.Request:
        req = RemoveObjects.Request()
        req.clear_all = params.get("clear_all", False)
        req.object_ids = params.get("object_ids", [])
        return req

    def _build_get_objects(self, params: dict) -> GetObjects.Request:
        return GetObjects.Request()

    # =====================================================================
    # Response serializer — convert ROS2 responses to JSON-friendly dicts
    # =====================================================================

    def _serialize_response(self, skill: str, response) -> dict:
        """Convert a ROS2 service response to a JSON-serializable dict."""
        if skill == "get_objects":
            objects = []
            for i, oid in enumerate(response.object_ids):
                obj = {"id": oid, "type": response.object_types[i] if i < len(response.object_types) else ""}
                if i < len(response.poses):
                    p = response.poses[i]
                    obj["pose"] = {
                        "frame_id": p.header.frame_id,
                        "position": {"x": p.pose.position.x, "y": p.pose.position.y, "z": p.pose.position.z},
                        "orientation": {"x": p.pose.orientation.x, "y": p.pose.orientation.y,
                                        "z": p.pose.orientation.z, "w": p.pose.orientation.w},
                    }
                objects.append(obj)
            return {"objects": objects}

        # Standard success/message response
        data = {}
        if hasattr(response, "success"):
            data["success"] = response.success
        if hasattr(response, "message"):
            data["message"] = response.message
        return data

    # =====================================================================
    # Helpers
    # =====================================================================

    def _make_pose_stamped(self, d: dict) -> PoseStamped:
        """Build a PoseStamped from a flat dict."""
        ps = PoseStamped()
        ps.header.frame_id = d.get("frame_id", "world")
        pos = d.get("position", {})
        ps.pose.position.x = float(pos.get("x", 0.0))
        ps.pose.position.y = float(pos.get("y", 0.0))
        ps.pose.position.z = float(pos.get("z", 0.0))
        ori = d.get("orientation", {})
        ps.pose.orientation.x = float(ori.get("x", 0.0))
        ps.pose.orientation.y = float(ori.get("y", 0.0))
        ps.pose.orientation.z = float(ori.get("z", 0.0))
        ps.pose.orientation.w = float(ori.get("w", 1.0))
        return ps

    def _cleanup_stale_consumers(self):
        """Remove all other consumers from the group (stale from previous runs)."""
        try:
            consumers = self.r.xinfo_consumers(TASK_STREAM, TASK_GROUP)
            for c in consumers:
                name = c["name"]
                if name != CONSUMER_NAME:
                    pending = self.r.xgroup_delconsumer(TASK_STREAM, TASK_GROUP, name)
                    self.get_logger().info(
                        f"Removed stale consumer '{name}' ({pending} pending msgs dropped)"
                    )
        except redis.exceptions.ResponseError as e:
            self.get_logger().warn(f"Could not clean stale consumers: {e}")

    def shutdown(self):
        self._running = False


def main():
    rclpy.init()
    node = RedisBridge()

    # Graceful shutdown on SIGINT/SIGTERM
    def _signal_handler(sig, frame):
        node.get_logger().info("Shutting down...")
        node.shutdown()

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    try:
        node.spin_bridge()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
