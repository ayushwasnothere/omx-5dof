#!/usr/bin/env python3
"""
Client-side test for the Redis ↔ ROS2 bridge.

This module provides a RobotClient class that sends tasks to the bridge
via Redis streams and waits for results. It also includes a CLI test
runner that exercises all supported skills.

Usage:
    # Make sure redis-server, ROS2 stack, and redis_bridge.py are running.
    python3 bridge/test_bridge.py                  # run all tests
    python3 bridge/test_bridge.py pick place       # run specific tests
    python3 bridge/test_bridge.py --list            # list available tests
"""

import json
import sys
import time
import uuid
from typing import Any, Optional

import redis

# ── Redis config (must match redis_bridge.py) ────────────────────────────
REDIS_HOST = "localhost"
REDIS_PORT = 6379
TASK_STREAM = "robot.tasks"
EVENT_STREAM = "robot.events"


class RobotClient:
    """Client that sends tasks to the robot via Redis streams."""

    def __init__(self, host: str = REDIS_HOST, port: int = REDIS_PORT):
        self.r = redis.Redis(host=host, port=port, decode_responses=True)
        self.r.ping()
        # Track the last-read event ID for this client instance
        self._last_event_id = "$"

    def send_task(self, skill: str, params: Optional[dict] = None, timeout: float = 120.0) -> dict:
        """Send a task and wait for the result event.

        Args:
            skill: Skill name (pick, place, pour, rotate, go_home, etc.)
            params: Skill parameters as a dict.
            timeout: Max seconds to wait for a result.

        Returns:
            dict with keys: task_id, status, data
        """
        task_id = uuid.uuid4().hex[:12]
        params = params or {}

        # Remember where the event stream is *before* we send the task
        # so we only read events that arrive after our send.
        last_id = self._get_stream_last_id()

        # Publish task
        self.r.xadd(TASK_STREAM, {
            "task_id": task_id,
            "skill": skill,
            "params": json.dumps(params),
        })
        print(f"  → Sent task {task_id}: skill={skill} params={json.dumps(params, separators=(',', ':'))}")

        # Wait for matching event(s)
        deadline = time.time() + timeout
        final_result = None

        while time.time() < deadline:
            remaining_ms = int((deadline - time.time()) * 1000)
            if remaining_ms <= 0:
                break

            entries = self.r.xread({EVENT_STREAM: last_id}, count=10, block=min(remaining_ms, 2000))
            if not entries:
                continue

            for _stream, messages in entries:
                for msg_id, fields in messages:
                    last_id = msg_id
                    if fields.get("task_id") != task_id:
                        continue

                    status = fields.get("status", "")
                    data = json.loads(fields.get("data", "{}"))

                    if status == "running":
                        print(f"  ⏳ Task {task_id}: running...")
                        continue

                    # Terminal status
                    final_result = {"task_id": task_id, "status": status, "data": data}
                    print(f"  ← Result: status={status} data={json.dumps(data, separators=(',', ':'))}")
                    return final_result

        return {"task_id": task_id, "status": "timeout", "data": {"reason": "No response within timeout"}}

    def _get_stream_last_id(self) -> str:
        """Get the last ID in the event stream (or '0' if empty/missing)."""
        try:
            info = self.r.xinfo_stream(EVENT_STREAM)
            return info.get("last-generated-id", "0")
        except redis.exceptions.ResponseError:
            return "0"


# ── Test definitions ─────────────────────────────────────────────────────

def test_add_objects(client: RobotClient) -> dict:
    """Add a cup and a bowl to the scene."""
    return client.send_task("add_objects", {
        "objects": [
            {
                "id": "cup1",
                "type": "cylinder",
                "pose": {"frame_id": "world", "position": {"x": 0.20, "y": 0.0, "z": 0.04}},
                "dimensions": [0.08, 0.015],
            },
            {
                "id": "bowl1",
                "type": "cylinder",
                "pose": {"frame_id": "world", "position": {"x": 0.15, "y": 0.15, "z": 0.03}},
                "dimensions": [0.06, 0.025],
            },
        ]
    })


def test_get_objects(client: RobotClient) -> dict:
    """Query tracked objects."""
    return client.send_task("get_objects", {})


def test_pick(client: RobotClient) -> dict:
    """Pick cup1."""
    return client.send_task("pick", {"object_id": "cup1"})


def test_rotate(client: RobotClient) -> dict:
    """Rotate wrist ±0.3 rad for 2 cycles."""
    return client.send_task("rotate", {"angle": 0.3, "cycles": 2})


def test_place(client: RobotClient) -> dict:
    """Place at a specific pose."""
    return client.send_task("place", {
        "use_target_pose": False,
    })



def test_pour(client: RobotClient) -> dict:
    """Pour into bowl1 (WIP — may fail on 5-DOF IK)."""
    return client.send_task("pour", {"target_object_id": "bowl1", "pour_angle": 2.0})


def test_go_home(client: RobotClient) -> dict:
    """Go to home position."""
    return client.send_task("go_home", {"state_name": "home"})


def test_go_init(client: RobotClient) -> dict:
    """Go to init position (all joints zero)."""
    return client.send_task("go_home", {"state_name": "init"})


def test_remove_objects(client: RobotClient) -> dict:
    """Clear all objects from scene."""
    return client.send_task("remove_objects", {"clear_all": True})


def test_add_objects_camera_frame(client: RobotClient) -> dict:
    """Add an object using camera frame coordinates.
    The manipulator node transforms position to world frame via TF2."""
    return client.send_task("add_objects", {
        "objects": [
            {
                "id": "cam_cup1",
                "type": "cylinder",
                "use_camera_frame": True,
                "pose": {
                    "position": {"x": 0.0, "y": 0.0, "z": 0.5},
                },
                "dimensions": [0.08, 0.015],
            },
        ]
    })


# ── Ordered test sequence ────────────────────────────────────────────────

TESTS = [
    ("add_objects",    test_add_objects),
    ("get_objects",    test_get_objects),
    ("pick",           test_pick),
    ("rotate",         test_rotate),
    ("place",          test_place),
    ("go_home",        test_go_home),
    ("remove_objects", test_remove_objects),
]

# Extra tests not in the default sequence
EXTRA_TESTS = {
    "pour":    test_pour,
    "go_init": test_go_init,
    "add_camera": test_add_objects_camera_frame,
}

ALL_TESTS = {name: fn for name, fn in TESTS}
ALL_TESTS.update(EXTRA_TESTS)


def print_header(title: str):
    width = 60
    print(f"\n{'=' * width}")
    print(f"  {title}")
    print(f"{'=' * width}")


def run_tests(names: list[str]):
    """Run the specified tests in order."""
    client = RobotClient()
    print(f"Connected to Redis at {REDIS_HOST}:{REDIS_PORT}")

    passed, failed, errors = 0, 0, []

    for name in names:
        fn = ALL_TESTS.get(name)
        if fn is None:
            print(f"\n⚠  Unknown test: {name}")
            continue

        print_header(f"TEST: {name}")
        try:
            result = fn(client)
            status = result.get("status", "")
            if status == "success":
                print(f"  ✅ PASS")
                passed += 1
            elif status == "timeout":
                print(f"  ⏰ TIMEOUT")
                failed += 1
                errors.append((name, "timeout"))
            else:
                print(f"  ❌ FAIL ({status})")
                failed += 1
                errors.append((name, result.get("data", {})))
        except Exception as e:
            print(f"  💥 ERROR: {e}")
            failed += 1
            errors.append((name, str(e)))

    # ── Summary ──────────────────────────────────────────────────────
    print_header("SUMMARY")
    print(f"  Passed: {passed}/{passed + failed}")
    print(f"  Failed: {failed}/{passed + failed}")
    if errors:
        print(f"\n  Failures:")
        for name, reason in errors:
            print(f"    - {name}: {reason}")
    print()

    return failed == 0


def main():
    if "--list" in sys.argv:
        print("Available tests:")
        print("  Default sequence:", ", ".join(name for name, _ in TESTS))
        print("  Extra:           ", ", ".join(EXTRA_TESTS.keys()))
        return

    if "--help" in sys.argv or "-h" in sys.argv:
        print(__doc__)
        return

    # Pick which tests to run
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    if args:
        names = args
    else:
        names = [name for name, _ in TESTS]

    success = run_tests(names)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
