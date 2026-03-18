#!/bin/bash

# Array to keep track of the Process IDs (PIDs) we start
PIDS=()
# Variable to track if we are using simulation (defaults to false)
USE_SIM=false

# --- Parse Command Line Arguments ---
while getopts "s" opt; do
  case $opt in
    s)
      USE_SIM=true
      ;;
    \?)
      echo "Usage: $0 [-s]"
      echo "  -s    Run in simulation mode (Gazebo)"
      exit 1
      ;;
  esac
done

# --- Cleanup Function ---
cleanup() {
    echo -e "\nCaught termination signal! Shutting down all ROS 2 nodes cleanly..."
    for pid in "${PIDS[@]}"; do
        # Sending SIGINT (Ctrl+C equivalent) so ROS 2 nodes shut down gracefully
        kill -INT "$pid" 2>/dev/null
    done
    
    # Wait for the processes to actually finish closing
    wait "${PIDS[@]}" 2>/dev/null
    echo "All processes stopped. Exiting."
    exit 0
}

# Trap Ctrl+C (SIGINT) and standard kill (SIGTERM) to trigger the cleanup function
trap cleanup SIGINT SIGTERM

# --- Workspace Setup ---
cd /home/ubuntu/omx_ws || exit
source install/setup.bash

# --- Launch Logic ---
if [ "$USE_SIM" = true ]; then
    echo "=== RUNNING IN SIMULATION MODE ==="
    
    echo "Starting Gazebo..."
    ros2 launch open_manipulator_bringup open_manipulator_x_gazebo.launch.py &
    PIDS+=($!)
    sleep 5 

    echo "Starting MoveIt (Sim)..."
    ros2 launch open_manipulator_moveit_config open_manipulator_x_moveit.launch.py use_sim:=true &
    PIDS+=($!)
    sleep 3

    echo "Starting MTC (Sim)..."
    ros2 launch open_manipulator_mtc manipulator.launch.py use_sim:=true &
    PIDS+=($!)

else
    echo "=== RUNNING IN REAL ROBOT MODE ==="
    
    echo "Starting Real Hardware Bringup..."
    ros2 launch open_manipulator_bringup open_manipulator_x.launch.py &
    PIDS+=($!)
    sleep 3 

    echo "Starting MoveIt (Real)..."
    ros2 launch open_manipulator_moveit_config open_manipulator_x_moveit.launch.py &
    PIDS+=($!)
    sleep 3

    echo "Starting MTC (Real)..."
    ros2 launch open_manipulator_mtc manipulator.launch.py &
    PIDS+=($!)
fi

echo "All nodes are running. Press Ctrl+C to stop everything."

# Keep the script running and waiting for the background processes
wait