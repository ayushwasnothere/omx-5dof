# grasp_planner.py
#
# Pure Python — zero ROS dependency.
# Decides whether an object is graspable given its bounding box dimensions
# and returns the best grasp axis.

from dataclasses import dataclass


@dataclass
class GraspPlan:
    axis: str          # "x", "y", or "z"
    grasp_width: float # dimension along the chosen axis (meters)
    approach_axis: str # perpendicular axis used for the linear approach


class GraspPlanner:
    """
    Evaluates graspability from an OBB dimension triple [dx, dy, dz].

    Constraints (all tunable in __init__):
    - grasp_width  must be in (min_width, max_width)
    - the finger depth must not exceed `finger_depth`
    - the object height must not exceed `max_height`

    Returns a GraspPlan (or None if not graspable).
    """

    def __init__(
        self,
        max_width: float = 0.018,   # max gripper opening  [m]
        min_width: float = 0.002,   # min meaningful gap   [m]
        finger_depth: float = 0.030, # max finger reach     [m]
        max_height: float = 0.150,  # max object height    [m]
    ):
        self.max_width = max_width
        self.min_width = min_width
        self.finger_depth = finger_depth
        self.max_height = max_height

    # ------------------------------------------------------------------

    def evaluate(self, dimensions: list[float]) -> GraspPlan | None:
        """
        dimensions = [dx, dy, dz]  (meters, from PCA OBB)
        """
        dx, dy, dz = dimensions

        # Define potential grasps: (approach_axis, grip_axis, grasp_width, reach_dim, is_horizontal)
        candidates = [
            # Priority 1: Horizontal grasps (Frontal / Side slides)
            ("x", "y", dy, dx, True),  # Approach along X, Pinch the Y sides
            ("y", "x", dx, dy, True),  # Approach along Y, Pinch the X sides
            
            # Priority 2: Vertical grasps (Top-down descent)
            ("z", "x", dx, dz, False), # Approach along Z, Pinch the X sides
            ("z", "y", dy, dz, False), # Approach along Z, Pinch the Y sides
        ]

        viable = []

        for approach, grip, width, reach, is_horizontal in candidates:
            # 1. Can the gripper open wide enough?
            if not (self.min_width < width < self.max_width):
                continue
            
            # 2. Can the fingers reach the center of the object?
            if (reach / 2.0) > self.finger_depth:
                continue
                
            # 3. If top-down, is the object short enough to clear the arm?
            if not is_horizontal and dz > self.max_height:
                continue
                
            viable.append({
                "approach_axis": approach,
                "grip_axis": grip,
                "grasp_width": width,
                "is_horizontal": is_horizontal
            })

        if not viable:
            return None

        # Sort by priority: Horizontal approaches first, then smallest grasp width
        viable.sort(key=lambda c: (not c["is_horizontal"], c["grasp_width"]))
        best = viable[0]

        return GraspPlan(
            axis=best["grip_axis"],
            grasp_width=best["grasp_width"],
            approach_axis=best["approach_axis"],
        )
