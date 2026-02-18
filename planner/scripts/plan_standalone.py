#!/usr/bin/env python3
"""Standalone planner runner (no ROS required)."""
import sys
import os
import argparse
import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + '/../')
from config import Config
from planner_wrapper import TomogramPlanner

parser = argparse.ArgumentParser()
parser.add_argument('--tomo', type=str, default='clinic', help='Tomogram file stem in rsc/tomogram/')
parser.add_argument('--start', type=float, nargs=2, default=[0.0, 0.0], metavar=('X', 'Y'))
parser.add_argument('--end',   type=float, nargs=2, default=[10.0, 5.0], metavar=('X', 'Y'))
args = parser.parse_args()

# Set LD_LIBRARY_PATH programmatically
root = os.path.dirname(os.path.abspath(__file__)) + '/..'
gtsam_lib = root + '/lib/3rdparty/gtsam-4.1.1/install/lib'
smoothing_lib = root + '/lib/build/src/common/smoothing'
sys.path.insert(0, root + '/lib')

for path in [gtsam_lib, smoothing_lib]:
    ld = os.environ.get('LD_LIBRARY_PATH', '')
    if path not in ld:
        os.environ['LD_LIBRARY_PATH'] = ld + ':' + path

cfg = Config()
planner = TomogramPlanner(cfg)
planner.loadTomogram(args.tomo)

start_pos = np.array(args.start, dtype=np.float32)
end_pos   = np.array(args.end,   dtype=np.float32)

print(f"[INFO] Planning from {start_pos} to {end_pos}")
traj_3d = planner.plan(start_pos, end_pos)

if traj_3d is None:
    print("[WARN] No path found between the given start and end positions.")
else:
    print(f"[INFO] Trajectory found: {traj_3d.shape[0]} waypoints")
    print("[INFO] First waypoint:", traj_3d[0])
    print("[INFO] Last  waypoint:", traj_3d[-1])
    out = os.path.dirname(os.path.abspath(__file__)) + '/../../rsc/clinic_traj.npy'
    np.save(out, traj_3d)
    print(f"[INFO] Trajectory saved to {out}")
