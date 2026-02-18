#!/bin/bash
# Launches the PCT planner node + RViz2 together.
# Usage: ./launch_ros2.sh [--skip-tomo]

set -e
source /opt/ros/humble/setup.bash

ROOT=$(cd $(dirname "$0"); pwd)
ARGS="$@"

# Launch the planner node in the background
python3 "$ROOT/run_ros2_interactive.py" $ARGS &
NODE_PID=$!

# Wait for the node to be ready (it logs "Interactive ROS2 mode" when done)
echo "Waiting for planner node to initialize..."
for i in $(seq 1 60); do
    sleep 1
    if ! kill -0 $NODE_PID 2>/dev/null; then
        echo "ERROR: planner node exited unexpectedly."
        exit 1
    fi
done &
WATCHDOG=$!

# Open RViz2 with the PCT config
rviz2 -d "$ROOT/rsc/rviz/pct_ros2.rviz"

# Cleanup on RViz2 exit
kill $WATCHDOG 2>/dev/null || true
kill $NODE_PID 2>/dev/null || true
wait $NODE_PID 2>/dev/null || true
