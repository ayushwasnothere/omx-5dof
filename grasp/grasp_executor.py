# grasp_executor.py

import copy
from moveit_commander import MoveGroupCommander


class GraspExecutor:

    def __init__(self, node):
        self.node = node
        self.arm = MoveGroupCommander("arm")
        self.gripper = MoveGroupCommander("gripper")

        self.arm.set_max_velocity_scaling_factor(0.2)
        self.arm.set_max_acceleration_scaling_factor(0.2)

    def compute_pregrasp(self, target_pose, distance=0.10):
        pre = copy.deepcopy(target_pose)
        pre.position.z += distance
        return pre

    def go_to_pose(self, pose):
        self.arm.set_pose_target(pose)
        success = self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()
        return success

    def cartesian_approach(self, start_pose, target_pose):
        waypoints = [copy.deepcopy(start_pose),
                     copy.deepcopy(target_pose)]

        plan, fraction = self.arm.compute_cartesian_path(
            waypoints,
            0.01,
            0.0
        )

        self.arm.execute(plan, wait=True)
        return fraction > 0.9

    def open_gripper(self):
        self.gripper.set_named_target("open")
        self.gripper.go(wait=True)

    def close_gripper(self):
        self.gripper.set_named_target("close")
        self.gripper.go(wait=True)

    def retreat(self, lift=0.10):
        current = self.arm.get_current_pose().pose
        current.position.z += lift
        return self.go_to_pose(current)
