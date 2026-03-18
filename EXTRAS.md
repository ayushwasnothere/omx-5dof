sudo apt install ros-$ROS_DISTRO-moveit-task-constructor-visualization

cd ~/omx_ws/src
git clone https://github.com/moveit/moveit_task_constructor.git -b jazzy

cd ~/omx_ws
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install --packages-select \
    moveit_task_constructor_msgs \
    moveit_task_constructor_core \
    moveit_task_constructor_visualization

source install/setup.bash