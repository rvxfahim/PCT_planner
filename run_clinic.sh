#!/bin/bash
# End-to-end script: tomography + planning for clinic.pcd
#
# Usage:
#   ./run_clinic.sh                          # standalone (no ROS, default start/end)
#   ./run_clinic.sh 0 0 10 5                 # standalone, custom start/end
#   ./run_clinic.sh --ros2                   # ROS2 mode (publishes to RViz2)
#   ./run_clinic.sh --ros2 0 0 10 5         # ROS2 mode, custom start/end

set -e
ROOT=$(cd $(dirname "$0"); pwd)

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ROOT/planner/lib/3rdparty/gtsam-4.1.1/install/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ROOT/planner/lib/build/src/common/smoothing
export PYTHONPATH=$PYTHONPATH:$ROOT/planner/lib

ROS2_MODE=0
if [[ "$1" == "--ros2" ]]; then
    ROS2_MODE=1
    shift
    source /opt/ros/humble/setup.bash 2>/dev/null || true
fi

START_X=${1:-0.0}
START_Y=${2:-0.0}
END_X=${3:-10.0}
END_Y=${4:-5.0}

echo "=== Step 1: Tomography (clinic.pcd -> clinic.pickle) ==="
cd "$ROOT/tomography/scripts"
if [[ "$ROS2_MODE" == "1" ]]; then
    python3 run_standalone.py --scene Clinic  # generate tomogram first
else
    python3 run_standalone.py --scene Clinic
fi

echo ""
echo "=== Step 2: Planning (start=[${START_X}, ${START_Y}] -> end=[${END_X}, ${END_Y}]) ==="
cd "$ROOT/planner/scripts"
if [[ "$ROS2_MODE" == "1" ]]; then
    echo "(ROS2 mode: publishing path to /pct_path topic)"
    python3 plan_ros2.py --tomo clinic \
        --start "$START_X" "$START_Y" \
        --end   "$END_X"   "$END_Y"
else
    python3 plan_standalone.py --tomo clinic \
        --start "$START_X" "$START_Y" \
        --end   "$END_X"   "$END_Y"
fi

echo ""
echo "=== Done! Trajectory saved to rsc/clinic_traj.npy ==="
