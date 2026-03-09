"""
pick_place.launch.py - MTC node for pick and place

Launch order:
  Terminal 1: ros2 launch open_manipulator_bringup open_manipulator_x_gazebo.launch.py
  Terminal 2: ros2 launch open_manipulator_moveit_config open_manipulator_x_moveit.launch.py use_sim:=True
  Terminal 3: ros2 launch open_manipulator_mtc pick_place.launch.py use_sim:=true
"""

from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Arguments
    use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        description="Use simulation time (true) or real robot (false)",
    )
    use_sim = LaunchConfiguration("use_sim")

    # MoveIt config
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="open_manipulator_x",
            package_name="open_manipulator_moveit_config",
        )
        .robot_description_semantic(
            str(Path("config") / "open_manipulator_x" / "open_manipulator_x.srdf")
        )
        .joint_limits(
            str(Path("config") / "open_manipulator_x" / "joint_limits.yaml")
        )
        .trajectory_execution(
            str(Path("config") / "open_manipulator_x" / "moveit_controllers.yaml")
        )
        .robot_description_kinematics(
            str(Path("config") / "open_manipulator_x" / "kinematics.yaml")
        )
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # MTC node with delay to ensure MoveGroup is ready
    mtc_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="open_manipulator_mtc",
                executable="mtc_node",
                name="mtc_node",
                output="screen",
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                    moveit_config.joint_limits,
                    moveit_config.trajectory_execution,
                    {"use_sim_time": use_sim},
                ],
            )
        ],
    )

    return LaunchDescription([
        use_sim_arg,
        mtc_node,
    ])
