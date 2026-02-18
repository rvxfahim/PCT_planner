#!/usr/bin/env python3
"""
Interactive ROS2 PCT Planner for clinic.pcd

Workflow:
  1. Runs tomography on clinic.pcd (or loads existing tomogram)
  2. Publishes the point cloud + tomogram to RViz2
  3. Waits for two /clicked_point messages (RViz2 "Publish Point" tool)
     - 1st click → start position
     - 2nd click → end position
  4. Plans a path and publishes it as /pct_path (nav_msgs/Path)
     + a LineStrip Marker on /pct_marker for easy visualization

Usage:
  source /opt/ros/humble/setup.bash
  python3 run_ros2_interactive.py [--skip-tomo]
"""

import os, sys, argparse, pickle, time
import ctypes
import numpy as np, open3d as o3d

ROOT = os.path.dirname(os.path.abspath(__file__))

# Must preload GTSAM and smoothing libs via ctypes *before* importing the
# pybind11 extension modules; os.environ changes don't affect the current
# process's dynamic linker cache.
_LIB_ROOT = ROOT + '/planner/lib'
for _lib in [
    _LIB_ROOT + '/3rdparty/gtsam-4.1.1/install/lib/libmetis-gtsam.so',
    _LIB_ROOT + '/3rdparty/gtsam-4.1.1/install/lib/libgtsam.so.4',
    _LIB_ROOT + '/build/src/common/smoothing/libcommon_smoothing.so',
]:
    ctypes.CDLL(_lib, mode=ctypes.RTLD_GLOBAL)

# planner_wrapper.py does `from lib import a_star, ...` which needs planner/
# (not planner/lib/) on sys.path so Python can find the `lib` sub-package.
sys.path.insert(0, ROOT + '/planner')

import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

import importlib.util as _ilu

def _load(path, module_name):
    spec = _ilu.spec_from_file_location(module_name, path)
    mod = _ilu.module_from_spec(spec)
    sys.modules[module_name] = mod
    spec.loader.exec_module(mod)
    return mod

# Load tomography modules under unique names to avoid collision with planner's
# identically-named 'config' package.
sys.path.insert(0, ROOT + '/tomography/scripts')
sys.path.insert(0, ROOT + '/tomography')
from tomogram import Tomogram

_tomo_cfg = _load(ROOT + '/tomography/config/__init__.py', 'tomo_config')
TomoCfg   = _tomo_cfg.Config
SceneClinic = _tomo_cfg.SceneClinic

# Planner modules
sys.path.insert(0, ROOT + '/planner/scripts')
sys.path.insert(0, ROOT + '/planner')
_plan_cfg = _load(ROOT + '/planner/config/__init__.py', 'plan_config')
PlanCfg   = _plan_cfg.Config
from planner_wrapper import TomogramPlanner


# ─── helpers ────────────────────────────────────────────────────────────────

def make_pc2(node, points_f32, fields_xyz=True, frame='map'):
    msg = PointCloud2()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = frame
    msg.height = 1
    msg.width = len(points_f32)
    if fields_xyz:
        msg.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12
    else:  # XYZI
        msg.fields = [
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 16
    msg.row_step = msg.point_step * len(points_f32)
    msg.is_bigendian = False
    msg.is_dense = True
    msg.data = np.ascontiguousarray(points_f32).tobytes()
    return msg


def traj_to_path(traj_3d, node, frame='map'):
    msg = Path()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = frame
    for pt in traj_3d:
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose.position.x = float(pt[0])
        ps.pose.position.y = float(pt[1])
        ps.pose.position.z = float(pt[2])
        ps.pose.orientation.w = 1.0
        msg.poses.append(ps)
    return msg


def traj_to_marker(traj_3d, node, frame='map', marker_id=0):
    m = Marker()
    m.header.stamp = node.get_clock().now().to_msg()
    m.header.frame_id = frame
    m.ns = 'pct'
    m.id = marker_id
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.scale.x = 0.1
    m.color = ColorRGBA(r=0.0, g=1.0, b=0.3, a=1.0)
    for pt in traj_3d:
        from geometry_msgs.msg import Point
        p = Point(x=float(pt[0]), y=float(pt[1]), z=float(pt[2]))
        m.points.append(p)
    return m


def sphere_marker(pos, node, marker_id, color, frame='map'):
    m = Marker()
    m.header.stamp = node.get_clock().now().to_msg()
    m.header.frame_id = frame
    m.ns = 'pct_clicks'
    m.id = marker_id
    m.type = Marker.SPHERE
    m.action = Marker.ADD
    m.pose.position.x = float(pos[0])
    m.pose.position.y = float(pos[1])
    m.pose.position.z = float(pos[2]) if len(pos) > 2 else 0.0
    m.scale.x = m.scale.y = m.scale.z = 0.5
    m.color = color
    return m


# ─── main node ──────────────────────────────────────────────────────────────

class PCTNode(Node):
    def __init__(self, skip_tomo: bool):
        super().__init__('pct_planner_ros2')

        # Publishers
        self.pc_pub     = self.create_publisher(PointCloud2, '/global_points', 1)
        self.tomo_pub   = self.create_publisher(PointCloud2, '/tomogram',       1)
        self.path_pub   = self.create_publisher(Path,        '/pct_path',       1)
        self.marker_pub = self.create_publisher(Marker,      '/pct_marker',     1)

        # Click state
        self._clicks = []
        self._start  = None
        self._end    = None

        # ── Step 1: tomography ──
        tomo_pickle = ROOT + '/rsc/tomogram/clinic.pickle'
        if skip_tomo and os.path.exists(tomo_pickle):
            self.get_logger().info('Skipping tomography, loading existing pickle.')
        else:
            self.get_logger().info('Running tomography on clinic.pcd …')
            self._run_tomography(tomo_pickle)

        # ── Step 2: load planner ──
        self.get_logger().info('Loading planner …')
        plan_cfg = PlanCfg()
        self.planner = TomogramPlanner(plan_cfg)
        self.planner.loadTomogram('clinic')
        self.get_logger().info('Planner ready.')

        # ── Publish point cloud + tomogram ──
        self._publish_pcd()
        self._publish_tomo()

        # ── Subscribe to RViz2 clicked point ──
        self.create_subscription(PointStamped, '/clicked_point',
                                 self._on_click, 10)

        self.get_logger().info(
            '\n'
            '══════════════════════════════════════════════\n'
            ' PCT Planner – Interactive ROS2 mode\n'
            '──────────────────────────────────────────────\n'
            ' In RViz2:\n'
            '   1. Click "Publish Point" toolbar button\n'
            '   2. Click on the map → START  (green sphere)\n'
            '   3. Click again      → END    (red sphere)\n'
            '   → Path published on /pct_path + /pct_marker\n'
            '   4. Next two clicks start a new query.\n'
            '══════════════════════════════════════════════'
        )

    # ── tomography ──────────────────────────────────────────────────────────

    def _run_tomography(self, out_path):
        import sys as _sys
        tomo_cfg = TomoCfg()
        scene_cfg = SceneClinic

        tomo = Tomogram(scene_cfg)
        pcd_path = ROOT + '/rsc/pcd/clinic.pcd'
        pcd = o3d.io.read_point_cloud(pcd_path)
        pts = np.asarray(pcd.points).astype(np.float32)
        self.get_logger().info(f'PCD points: {pts.shape[0]}')

        pts_max, pts_min = pts.max(0), pts.min(0)
        pts_min[-1] = scene_cfg.map.ground_h
        res   = scene_cfg.map.resolution
        dh    = scene_cfg.map.slice_dh
        dim_x = int(np.ceil((pts_max[0]-pts_min[0])/res)) + 4
        dim_y = int(np.ceil((pts_max[1]-pts_min[1])/res)) + 4
        n_sl  = int(np.ceil((pts_max[2]-pts_min[2])/dh))
        ctr   = (pts_max[:2]+pts_min[:2])/2
        h0    = pts_min[-1] + dh

        tomo.initMappingEnv(ctr, dim_x, dim_y, n_sl, h0)
        self.get_logger().info(f'Map {dim_x}×{dim_y}, {n_sl} slices init')

        lt, gx, gy, lg, lc, tg = tomo.point2map(pts)
        self.get_logger().info(
            f'Done: {lg.shape[0]} slices, '
            f't={tg["t_map"]+tg["t_trav"]+tg["t_simp"]:.0f} ms'
        )

        data = {
            'data': np.stack((lt, gx, gy, lg, lc)).astype(np.float16),
            'resolution': res, 'center': ctr,
            'slice_h0': h0, 'slice_dh': dh,
        }
        with open(out_path, 'wb') as f:
            pickle.dump(data, f, protocol=pickle.HIGHEST_PROTOCOL)
        self.get_logger().info(f'Tomogram saved: {out_path}')
        self._pts_raw = pts
        self._tomo_data = data

    # ── publishers ──────────────────────────────────────────────────────────

    def _publish_pcd(self):
        if not hasattr(self, '_pts_raw'):
            pcd = o3d.io.read_point_cloud(ROOT + '/rsc/pcd/clinic.pcd')
            self._pts_raw = np.asarray(pcd.points).astype(np.float32)
        pts = self._pts_raw
        # Subsample for publishing (every 10th point)
        pts_sub = pts[::10]
        msg = make_pc2(self, pts_sub)
        self.pc_pub.publish(msg)
        self.get_logger().info(f'Published {len(pts_sub)} raw points')

    def _publish_tomo(self):
        if not hasattr(self, '_tomo_data'):
            with open(ROOT + '/rsc/tomogram/clinic.pickle', 'rb') as f:
                self._tomo_data = pickle.load(f)

        d     = self._tomo_data
        tomo  = np.asarray(d['data'], dtype=np.float32)
        trav  = tomo[0]
        elev  = tomo[3]
        res   = float(d['resolution'])
        ctr   = np.asarray(d['center'], dtype=np.float32)
        n_sl, dim_x, dim_y = trav.shape
        ox, oy = dim_x//2, dim_y//2

        all_pts = []
        for s in range(n_sl):
            g = elev[s]; t = trav[s]
            mask = ~np.isnan(g)
            ix, iy = np.where(mask)
            wx = (ix - ox) * res + ctr[0]
            wy = (iy - oy) * res + ctr[1]
            wz = g[mask]
            wt = t[mask]
            layer = np.stack([wx, wy, wz, wt], axis=1).astype(np.float32)
            all_pts.append(layer)

        pts4 = np.concatenate(all_pts, 0)
        msg = make_pc2(self, pts4, fields_xyz=False)
        self.tomo_pub.publish(msg)
        self.get_logger().info(f'Published {len(pts4)} tomogram points')

    # ── click handler ────────────────────────────────────────────────────────

    def _z_to_slice(self, x, y, z):
        """Map a 3D world point to the best-matching simplified slice index.

        For each simplified slice, the tomogram stores the floor height at
        every grid cell.  We find the slice whose floor is closest to (and at
        or below) the clicked z, which corresponds to the floor the user is
        standing on.
        """
        if not hasattr(self, '_tomo_data'):
            with open(ROOT + '/rsc/tomogram/clinic.pickle', 'rb') as f:
                self._tomo_data = pickle.load(f)

        d      = self._tomo_data
        tomo   = np.asarray(d['data'], dtype=np.float32)
        elev_g = tomo[3]                             # (n_slice, dim_x, dim_y)
        res    = float(d['resolution'])
        ctr    = np.asarray(d['center'], dtype=np.float32)
        n_slice, dim_x, dim_y = elev_g.shape
        ox, oy = dim_x // 2, dim_y // 2

        ix = int(round((x - float(ctr[0])) / res)) + ox
        iy = int(round((y - float(ctr[1])) / res)) + oy
        ix = int(np.clip(ix, 0, dim_x - 1))
        iy = int(np.clip(iy, 0, dim_y - 1))

        # Floor heights at the clicked cell for every simplified slice.
        # NaN was replaced by -100 during planner load; treat those as missing.
        heights = elev_g[:, ix, iy]

        best_slice = 0
        best_diff  = float('inf')
        for s in range(n_slice):
            h = float(heights[s])
            if h < -50:          # no floor data at this cell for this slice
                continue
            if h <= z + 0.3:     # floor is at or just below the clicked point
                diff = z - h
                if diff < best_diff:
                    best_diff  = diff
                    best_slice = s

        return best_slice

    def _on_click(self, msg: PointStamped):
        x, y, z = msg.point.x, msg.point.y, msg.point.z
        click = (np.array([x, y], dtype=np.float32), z)
        self._clicks.append(click)
        n = len(self._clicks)
        self.get_logger().info(f'Click #{n}: ({x:.2f}, {y:.2f}, z={z:.2f})')

        if n % 2 == 1:
            # First of a pair → start
            self._start = click
            col = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            self.marker_pub.publish(sphere_marker([x, y, z], self, 100, col))
            self.get_logger().info(f'  → START ({x:.2f}, {y:.2f}, z={z:.2f})')
        else:
            # Second of a pair → end, then plan
            self._end = click
            col = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            self.marker_pub.publish(sphere_marker([x, y, z], self, 101, col))
            self.get_logger().info(f'  → END ({x:.2f}, {y:.2f}, z={z:.2f}), planning …')
            self._plan()

    def _plan(self):
        start_pos, start_z = self._start
        end_pos,   end_z   = self._end

        start_slice = self._z_to_slice(float(start_pos[0]), float(start_pos[1]), start_z)
        end_slice   = self._z_to_slice(float(end_pos[0]),   float(end_pos[1]),   end_z)
        self.get_logger().info(f'  Slices: start={start_slice}, end={end_slice}')

        # TomogramPlanner.plan() only writes idx[1:] (row, col); we set idx[0]
        # (the floor/slice index) here so the planner routes on the right floor.
        self.planner.start_idx[0] = start_slice
        self.planner.end_idx[0]   = end_slice

        traj = self.planner.plan(start_pos, end_pos)
        if traj is None:
            self.get_logger().warn('No path found! Try different start/end points.')
            return

        self.get_logger().info(f'Path found: {traj.shape[0]} waypoints')
        self.path_pub.publish(traj_to_path(traj, self))
        self.marker_pub.publish(traj_to_marker(traj, self))

        out = ROOT + '/rsc/clinic_traj.npy'
        np.save(out, traj)
        self.get_logger().info(f'Saved trajectory to {out}')


# ─── entry point ─────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--skip-tomo', action='store_true',
                        help='Skip tomography if clinic.pickle already exists')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = PCTNode(skip_tomo=args.skip_tomo)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
