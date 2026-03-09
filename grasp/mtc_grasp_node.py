#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Vector3Stamped
from moveit.task_constructor import core, stages

class MTCSmartGrasp(Node):
    def __init__(self):
        super().__init__("mtc_smart_grasp")
        self.arm_group = "arm"
        self.gripper_group = "gripper"
        self.eef_link = "end_effector_link"
        self.gripper_links = [
            "gripper_link",
            "gripper_left_finger_link",
            "gripper_right_finger_link"
        ]
        self.get_logger().info("MTC Smart Grasp Node Initialized.")

    def create_pick_task(self, object_id: str, target_pose: PoseStamped, dimensions: list):
        task = core.Task(f"pick_{object_id}")

        # Load the robot model — ROS 2 requires passing the node
        task.loadRobotModel(self)

        pipeline_planner = core.PipelinePlanner()
        cartesian_planner = core.CartesianPath()

        # ... rest of your stages unchanged ...
        task.add(stages.CurrentState("current_state"))
        task.add(stages.Connect("connect_to_pregrasp", [(self.arm_group, pipeline_planner)]))
        # serial container + approach/allowCollision/etc ...
        # (unchanged from your implementation)

    # execute_grasp unchanged
        pick = core.SerialContainer("pick_object")

        approach = stages.MoveRelative("approach", cartesian_planner)
        approach.group = self.arm_group
        approach.ik_frame = self.eef_link

        direction = Vector3Stamped()
        direction.header.frame_id = "world"
        direction.vector.x = 1.0 if target_pose.pose.position.x > 0 else -1.0

        approach.setDirection(direction)
        approach.setMinMaxDistance(0.05, 0.10)

        pick.insert(approach)

        allow_collision = stages.ModifyPlanningScene("allow_collision")
        allow_collision.allowCollisions(object_id, self.gripper_links, True)

        pick.insert(allow_collision)

        close_gripper = stages.MoveTo("close_gripper", pipeline_planner)
        close_gripper.group = self.gripper_group
        close_gripper.setGoal("close")

        pick.insert(close_gripper)

        attach_object = stages.ModifyPlanningScene("attach_object")
        attach_object.attachObject(object_id, self.eef_link)

        pick.insert(attach_object)

        lift = stages.MoveRelative("lift", cartesian_planner)
        lift.group = self.arm_group
        lift.ik_frame = self.eef_link

        lift_dir = Vector3Stamped()
        lift_dir.header.frame_id = "world"
        lift_dir.vector.z = 1.0

        lift.setDirection(lift_dir)
        lift.setMinMaxDistance(0.1, 0.15)

        pick.insert(lift)

        task.add(pick)

        return task

    def execute_grasp(self, object_id, target_pose, dimensions):

        task = self.create_pick_task(object_id, target_pose, dimensions)

        self.get_logger().info(f"Planning pick task for {object_id}...")

        if task.plan():
            self.get_logger().info("Plan found! Executing grasp sequence...")
            task.execute()
            return True
        else:
            self.get_logger().error("MTC failed to find a valid solution.")
            return False


def main():
    rclpy.init()
    node = MTCSmartGrasp()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
