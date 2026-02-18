#!/usr/bin/python3
"""Standalone tomography runner (no ROS required)."""
import os
import sys
import time
import pickle
import argparse
import numpy as np
import open3d as o3d

from tomogram import Tomogram

sys.path.append('../')
from config import Config
from config import SceneClinic, SceneBuilding, ScenePlaza

rsg_root = os.path.dirname(os.path.abspath(__file__)) + '/../..'

SCENES = {
    'Clinic': SceneClinic,
    'Building': SceneBuilding,
    'Plaza': ScenePlaza,
}


def log(msg):
    print(f"[INFO] {msg}")


def run(scene_name):
    cfg = Config()
    scene_cfg = SCENES[scene_name]

    export_dir = rsg_root + cfg.map.export_dir
    pcd_file = scene_cfg.pcd.file_name
    resolution = scene_cfg.map.resolution
    ground_h = scene_cfg.map.ground_h
    slice_dh = scene_cfg.map.slice_dh

    tomogram = Tomogram(scene_cfg)

    # Load PCD
    pcd_path = rsg_root + "/rsc/pcd/" + pcd_file
    log(f"Loading PCD: {pcd_path}")
    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points).astype(np.float32)
    log(f"PCD points: {points.shape[0]}")

    if points.shape[1] > 3:
        points = points[:, :3]

    points_max = np.max(points, axis=0)
    points_min = np.min(points, axis=0)
    points_min[-1] = ground_h
    map_dim_x = int(np.ceil((points_max[0] - points_min[0]) / resolution)) + 4
    map_dim_y = int(np.ceil((points_max[1] - points_min[1]) / resolution)) + 4
    n_slice_init = int(np.ceil((points_max[2] - points_min[2]) / slice_dh))
    center = (points_max[:2] + points_min[:2]) / 2
    slice_h0 = points_min[-1] + slice_dh

    tomogram.initMappingEnv(center, map_dim_x, map_dim_y, n_slice_init, slice_h0)

    log(f"Map center: [{center[0]:.2f}, {center[1]:.2f}]")
    log(f"Dim_x: {map_dim_x}")
    log(f"Dim_y: {map_dim_y}")
    log(f"Num slices init: {n_slice_init}")

    # Benchmark
    t_map = t_trav = t_simp = t_all = 0.0
    n_repeat = 3

    for i in range(n_repeat + 1):
        t_start = time.time()
        layers_t, trav_grad_x, trav_grad_y, layers_g, layers_c, t_gpu = tomogram.point2map(points)
        if i > 0:
            t_map += t_gpu['t_map']
            t_trav += t_gpu['t_trav']
            t_simp += t_gpu['t_simp']
            t_all += (time.time() - t_start) * 1e3

    log(f"Num slices simplified: {layers_g.shape[0]}")
    log(f"Avg t_map  (ms): {t_map / n_repeat:.3f}")
    log(f"Avg t_trav (ms): {t_trav / n_repeat:.3f}")
    log(f"Avg t_simp (ms): {t_simp / n_repeat:.3f}")
    log(f"Avg t_all  (ms): {t_all / n_repeat:.3f}")

    # Export
    map_file = os.path.splitext(pcd_file)[0]
    data_dict = {
        'data': np.stack((layers_t, trav_grad_x, trav_grad_y, layers_g, layers_c)).astype(np.float16),
        'resolution': resolution,
        'center': center,
        'slice_h0': slice_h0,
        'slice_dh': slice_dh,
    }
    out_path = export_dir + map_file + '.pickle'
    with open(out_path, 'wb') as handle:
        pickle.dump(data_dict, handle, protocol=pickle.HIGHEST_PROTOCOL)
    log(f"Tomogram exported: {out_path}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--scene', type=str, default='Clinic',
                        help='Scene name: Clinic, Building, Plaza')
    args = parser.parse_args()
    run(args.scene)
