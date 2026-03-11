#!/usr/bin/python3
"""
Camera TF Publisher — publishes a static transform from world to camera frame.

Parameters are persistent: once set via `ros2 param set`, they stay until changed.
Has separate error/offset parameters for calibration correction.

Effective transform = base (translation/rotation) + error offsets.

Usage:
    ros2 run open_manipulator_mtc camera_tf_node.py
    ros2 launch open_manipulator_mtc manipulator.launch.py  # includes this node

Set parameters at runtime:
    ros2 param set /camera_tf_publisher tx 0.85
    ros2 param set /camera_tf_publisher error_tx 0.01

Parameters:
    Base transform (world → camera_color_optical_frame):
        tx, ty, tz           — translation (meters)
        qx, qy, qz, qw      — rotation quaternion
    Error offsets (added to base):
        error_tx, error_ty, error_tz  — translation error (meters)
        error_qx, error_qy, error_qz, error_qw — rotation error (quaternion offsets, added to base)

Config file:
    Saved to ~/.config/camera_tf_params.yaml on every parameter change.
    Loaded on startup if it exists.
"""

import os
import yaml
import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor, FloatingPointRange
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import tf2_ros


CONFIG_PATH = os.path.expanduser("~/.config/camera_tf_params.yaml")
PARENT_FRAME = "world"
CHILD_FRAME = "camera_color_optical_frame"


class CameraTFPublisher(Node):
    """Publishes a configurable static transform with persistent parameters."""

    PARAM_NAMES = [
        "tx", "ty", "tz", "qx", "qy", "qz", "qw",
        "error_tx", "error_ty", "error_tz",
        "error_qx", "error_qy", "error_qz", "error_qw",
    ]

    # Default values: camera at (0.87, 0, 0.10) looking along -world_X
    #   optical frame: X=right(+Y_w), Y=down(-Z_w), Z=forward(-X_w)
    DEFAULTS = {
        "tx": 0.87, "ty": 0.0, "tz": 0.10,
        "qx": -0.5, "qy": -0.5, "qz": 0.5, "qw": 0.5,
        "error_tx": 0.0, "error_ty": 0.0, "error_tz": 0.0,
        "error_qx": 0.0, "error_qy": 0.0, "error_qz": 0.0, "error_qw": 0.0,
    }

    def __init__(self):
        super().__init__("camera_tf_publisher")

        self._broadcaster = StaticTransformBroadcaster(self)

        # Load saved config if it exists
        saved = self._load_config()

        # Declare parameters with descriptors
        for name in self.PARAM_NAMES:
            default = saved.get(name, self.DEFAULTS[name])
            desc = ParameterDescriptor()
            desc.description = self._param_description(name)
            self.declare_parameter(name, default, desc)

        # Declare frame names (also configurable)
        self.declare_parameter("parent_frame", PARENT_FRAME,
                               ParameterDescriptor(description="Parent TF frame"))
        self.declare_parameter("child_frame", CHILD_FRAME,
                               ParameterDescriptor(description="Child TF frame"))

        # Register parameter change callback
        self._pending_republish = False
        self.add_on_set_parameters_callback(self._on_param_change)

        # Publish initial transform and save config
        self._publish_transform()
        self._save_config()

        self.get_logger().info(
            f"Camera TF: {self._get('parent_frame')} → {self._get('child_frame')} "
            f"t=[{self._eff('tx'):.4f}, {self._eff('ty'):.4f}, {self._eff('tz'):.4f}] "
            f"q=[{self._eff('qx'):.4f}, {self._eff('qy'):.4f}, {self._eff('qz'):.4f}, {self._eff('qw'):.4f}]"
        )

    def _param_description(self, name: str) -> str:
        parts = name.split("_")
        if name.startswith("error_"):
            axis = parts[-1]
            kind = "translation" if parts[1] == "t" else "quaternion"
            return f"Calibration error offset for {kind} {axis}"
        axis = name[-1]
        kind = "translation" if name[0] == "t" else "quaternion"
        unit = "meters" if name[0] == "t" else ""
        return f"Base {kind} {axis}" + (f" ({unit})" if unit else "")

    def _get(self, name: str):
        """Get a parameter value."""
        return self.get_parameter(name).value

    def _eff(self, base_name: str) -> float:
        """Get effective value = base + error."""
        base = self._get(base_name)
        error = self._get(f"error_{base_name}")
        return base + error

    def _on_param_change(self, params: list) -> SetParametersResult:
        """Called when any parameter changes. Re-publishes transform and saves config."""
        result = SetParametersResult(successful=True)
        if not self._pending_republish:
            self._pending_republish = True
            self._republish_timer = self.create_timer(0.1, self._republish_once)
        return result

    def _republish_once(self):
        """One-shot republish (destroys the timer after first call)."""
        self._republish_timer.cancel()
        self.destroy_timer(self._republish_timer)
        self._pending_republish = False
        self._publish_transform()
        self._save_config()
        # Log the effective transform
        self.get_logger().info(
            f"Updated TF: t=[{self._eff('tx'):.4f}, {self._eff('ty'):.4f}, {self._eff('tz'):.4f}] "
            f"q=[{self._eff('qx'):.4f}, {self._eff('qy'):.4f}, {self._eff('qz'):.4f}, {self._eff('qw'):.4f}]"
        )

    def _publish_transform(self):
        """Build and broadcast the static transform."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._get("parent_frame")
        t.child_frame_id = self._get("child_frame")

        # Effective = base + error
        t.transform.translation.x = self._eff("tx")
        t.transform.translation.y = self._eff("ty")
        t.transform.translation.z = self._eff("tz")

        # Quaternion (base + error offset, then normalize)
        qx = self._eff("qx")
        qy = self._eff("qy")
        qz = self._eff("qz")
        qw = self._eff("qw")
        norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if norm > 0:
            qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self._broadcaster.sendTransform(t)

    def _save_config(self):
        """Save current parameter values to a YAML file."""
        data = {}
        for name in self.PARAM_NAMES:
            data[name] = self._get(name)
        data["parent_frame"] = self._get("parent_frame")
        data["child_frame"] = self._get("child_frame")

        os.makedirs(os.path.dirname(CONFIG_PATH), exist_ok=True)
        with open(CONFIG_PATH, "w") as f:
            yaml.safe_dump(data, f, default_flow_style=False)
        self.get_logger().info(f"Saved config to {CONFIG_PATH}")

    def _load_config(self) -> dict:
        """Load parameter values from saved YAML file."""
        if not os.path.exists(CONFIG_PATH):
            return {}
        try:
            with open(CONFIG_PATH, "r") as f:
                data = yaml.safe_load(f) or {}
            self.get_logger().info(f"Loaded config from {CONFIG_PATH}")
            return data
        except Exception as e:
            self.get_logger().warn(f"Failed to load config: {e}")
            return {}


def main(args=None):
    rclpy.init(args=args)
    node = CameraTFPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
