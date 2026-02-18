#!/usr/bin/python3
"""ROS2 tomography runner - generates tomogram and publishes to RViz2."""
import os
import sys
import time
import pickle
import argparse
import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from builtin_interfaces.msg import Time

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


def make_pointcloud2(node, points, frame_id='map'):
    """Create a PointCloud2 message from Nx3 float32 numpy array."""
    msg = PointCloud2()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.data = points.astype(np.float32).tobytes()
    msg.is_dense = True
    return msg


class TomographyNode(Node):
    def __init__(self, scene_name):
        super().__init__('pointcloud_tomography')
        self.map_frame = 'map'

        cfg = Config()
        scene_cfg = SCENES[scene_name]

        self.export_dir = rsg_root + cfg.map.export_dir
        pcd_file = scene_cfg.pcd.file_name
        resolution = scene_cfg.map.resolution
        ground_h = scene_cfg.map.ground_h
        slice_dh = scene_cfg.map.slice_dh

        tomo = Tomogram(scene_cfg)

        # Load PCD
        pcd_path = rsg_root + '/rsc/pcd/' + pcd_file
        self.get_logger().info(f'Loading PCD: {pcd_path}')
        pcd = o3d.io.read_point_cloud(pcd_path)
        points = np.asarray(pcd.points).astype(np.float32)
        self.get_logger().info(f'PCD points: {points.shape[0]}')

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

        tomo.initMappingEnv(center, map_dim_x, map_dim_y, n_slice_init, slice_h0)
        self.get_logger().info(f'Map center: {center}, Dim: {map_dim_x}x{map_dim_y}, Slices: {n_slice_init}')

        # Process
        self.get_logger().info('Running tomography...')
        layers_t, trav_grad_x, trav_grad_y, layers_g, layers_c, t_gpu = tomo.point2map(points)
        self.get_logger().info(f'Slices simplified: {layers_g.shape[0]}, t_all: {t_gpu["t_map"]+t_gpu["t_trav"]+t_gpu["t_simp"]:.1f} ms')

        # Export
        map_file = os.path.splitext(pcd_file)[0]
        data_dict = {
            'data': np.stack((layers_t, trav_grad_x, trav_grad_y, layers_g, layers_c)).astype(np.float16),
            'resolution': resolution,
            'center': center,
            'slice_h0': slice_h0,
            'slice_dh': slice_dh,
        }
        out_path = self.export_dir + map_file + '.pickle'
        with open(out_path, 'wb') as f:
            pickle.dump(data_dict, f, protocol=pickle.HIGHEST_PROTOCOL)
        self.get_logger().info(f'Tomogram exported: {out_path}')

        # Publish pointcloud
        self.pc_pub = self.create_publisher(PointCloud2, '/global_points', 1)
        self.tomo_pub = self.create_publisher(PointCloud2, '/tomogram', 1)

        # Publish original pointcloud
        pc_msg = make_pointcloud2(self, points)
        self.pc_pub.publish(pc_msg)

        # Publish simplified tomogram (top surface only)
        tomo_pts = []
        n_slice = layers_g.shape[0]
        for i in range(n_slice):
            layer = layers_g[i]
            trav = layers_t[i]
            mask = ~np.isnan(layer)
            xs = (np.where(mask)[0] - map_dim_x // 2) * resolution + center[0]
            ys = (np.where(mask)[1] - map_dim_y // 2) * resolution + center[1]
            zs = layer[mask]
            tv = trav[mask]
            if len(xs) > 0:
                layer_pts = np.stack([xs, ys, tv], axis=1).astype(np.float32)
                tomo_pts.append(layer_pts)

        if tomo_pts:
            all_tomo = np.concatenate(tomo_pts, axis=0)
            tomo_msg = make_pointcloud2(self, all_tomo)
            self.tomo_pub.publish(tomo_msg)
            self.get_logger().info(f'Published {len(all_tomo)} tomogram points')

        self.get_logger().info('Published. Spinning (Ctrl-C to exit)...')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scene', type=str, default='Clinic',
                        help='Scene name: Clinic, Building, Plaza')
    args = parser.parse_args()

    rclpy.init()
    node = TomographyNode(args.scene)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
