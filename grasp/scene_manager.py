# scene_manager.py
#
# Manages the MoveIt Planning Scene: add, remove, attach, detach, clear.
# Fully decoupled from motion logic — accepts a ROS2 Node for service calls.

import rclpy
from rclpy.node import Node

from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from moveit_msgs.msg import (
    PlanningScene,
    CollisionObject,
    AttachedCollisionObject,
    AllowedCollisionEntry,
)
from shape_msgs.msg import SolidPrimitive


class SceneManager:
    """
    Handles all MoveIt Planning Scene operations.

    Usage:
        scene = SceneManager(node)
        scene.add_obstacles([{"id": "box1", "type": "box",
                              "pose": pose_stamped, "size": [0.1, 0.1, 0.1]}])
        scene.attach_object("box1")
        scene.detach_object("box1")
        scene.clear()
    """

    def __init__(self, node: Node):
        self.node = node

        self._apply_client = node.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )
        self._get_client = node.create_client(
            GetPlanningScene, "/get_planning_scene"
        )

        self.node.get_logger().info("SceneManager: waiting for planning scene services...")
        self._apply_client.wait_for_service()
        self._get_client.wait_for_service()
        self.node.get_logger().info("SceneManager: ready.")

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _apply(self, scene: PlanningScene) -> bool:
        """Send a PlanningScene diff and block until complete."""
        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self._apply_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result().success

    def _build_primitive(self, obj_type: str, size) -> SolidPrimitive:
        """Create a SolidPrimitive from type string and size list."""
        prim = SolidPrimitive()
        if obj_type == "box":
            # size = [dx, dy, dz]
            prim.type = SolidPrimitive.BOX
            prim.dimensions = list(size)
        elif obj_type == "sphere":
            # size = [radius]
            prim.type = SolidPrimitive.SPHERE
            prim.dimensions = [size[0]]
        elif obj_type == "cylinder":
            # size = [height, radius]  ← MoveIt cylinder convention
            prim.type = SolidPrimitive.CYLINDER
            prim.dimensions = list(size)
        else:
            raise ValueError(f"Unknown collision shape type: '{obj_type}'")
        return prim

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def add_obstacles(self, obstacles: list) -> bool:
        """
        Add one or more collision objects to the planning scene.

        Each entry in `obstacles` must be a dict:
            {
                "id":   str,
                "type": "box" | "sphere" | "cylinder",
                "pose": PoseStamped,
                "size": list[float]   # see _build_primitive for convention
            }
        """
        collision_objects = []

        for obj in obstacles:
            co = CollisionObject()
            co.id = obj["id"]
            co.header = obj["pose"].header
            co.operation = CollisionObject.ADD
            co.primitives.append(self._build_primitive(obj["type"], obj["size"]))
            co.primitive_poses.append(obj["pose"].pose)
            collision_objects.append(co)

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = collision_objects

        success = self._apply(scene)
        self.node.get_logger().info(
            f"SceneManager: added {len(collision_objects)} object(s). success={success}"
        )
        return success

    def remove_object(self, object_id: str) -> bool:
        """Remove a single named collision object from the world."""
        co = CollisionObject()
        co.id = object_id
        co.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [co]

        success = self._apply(scene)
        self.node.get_logger().info(
            f"SceneManager: removed '{object_id}'. success={success}"
        )
        return success

    def get_known_objects(self) -> list[str]:
        """Return a list of all collision object IDs currently in the scene."""
        req = GetPlanningScene.Request()
        future = self._get_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        scene = future.result().scene
        return [obj.id for obj in scene.world.collision_objects]

    def clear(self) -> bool:
        """
        Remove ALL collision objects from the world.

        Fetches the current list and issues individual REMOVE operations so
        that MoveIt actually deletes existing objects (an empty list diff does
        not remove previously added objects).
        """
        names = self.get_known_objects()

        if not names:
            self.node.get_logger().info("SceneManager: scene already empty.")
            return True

        collision_objects = []
        for name in names:
            co = CollisionObject()
            co.id = name
            co.operation = CollisionObject.REMOVE
            collision_objects.append(co)

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = collision_objects

        success = self._apply(scene)
        self.node.get_logger().info(
            f"SceneManager: cleared {len(names)} object(s). success={success}"
        )
        return success

    def attach_object(
        self,
        object_id: str,
        link_name: str = "end_effector_link",
        touch_links: list[str] | None = None,
    ) -> bool:
        """
        Attach a world collision object to the robot end-effector.

        After attachment MoveIt treats the object as part of the robot, so it
        won't trigger self-collision checks during the retreat motion.

        `touch_links`: gripper link names allowed to touch the object.
        """
        aco = AttachedCollisionObject()
        aco.link_name = link_name
        aco.object.id = object_id
        aco.object.operation = CollisionObject.ADD
        if touch_links:
            aco.touch_links = touch_links

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.attached_collision_objects.append(aco)

        success = self._apply(scene)
        self.node.get_logger().info(
            f"SceneManager: attached '{object_id}' to '{link_name}'. success={success}"
        )
        return success

    def detach_object(self, object_id: str) -> bool:
        """Detach a previously attached object and return it to the world."""
        aco = AttachedCollisionObject()
        aco.object.id = object_id
        aco.object.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.attached_collision_objects.append(aco)

        success = self._apply(scene)
        self.node.get_logger().info(
            f"SceneManager: detached '{object_id}'. success={success}"
        )
        return success

    def set_collision_allowed(
        self,
        object_id: str,
        link_name: str = "end_effector_link",
        allow: bool = True,
    ) -> bool:
        """Temporarily allow or disallow collision between an object and a link."""
        scene = PlanningScene()
        scene.is_diff = True
        
        # We are modifying rules for 2 entities
        scene.allowed_collision_matrix.entry_names = [object_id, link_name]

        # MoveIt strictly requires a square N x N matrix.
        # So for 2 names, we need 2 rows, each containing 2 booleans.
        
        # Row 1: object_id vs [object_id, link_name]
        entry1 = AllowedCollisionEntry()
        entry1.enabled = [False, allow]  

        # Row 2: link_name vs [object_id, link_name]
        entry2 = AllowedCollisionEntry()
        entry2.enabled = [allow, False]  

        scene.allowed_collision_matrix.entry_values = [entry1, entry2]

        success = self._apply(scene)
        self.node.get_logger().info(
            f"SceneManager: collision {object_id}↔{link_name} allowed={allow}. success={success}"
        )
        return success
