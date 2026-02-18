#!/usr/bin/env python3
"""ROS2 planner runner - loads tomogram, plans path, publishes to RViz2."""
import sys
import os
import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + '/../')
from config import Config
from planner_wrapper import TomogramPlanner

rsg_root = os.path.dirname(os.path.abspath(__file__)) + '/../..'


def traj_to_ros2_path(traj_3d, frame_id='map', node=None):
    msg = Path()
    msg.header.frame_id = frame_id
    msg.header.stamp = node.get_clock().now().to_msg() if node else rclpy.clock.Clock().now().to_msg()
    for pt in traj_3d:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = float(pt[0])
        pose.pose.position.y = float(pt[1])
        pose.pose.position.z = float(pt[2])
        pose.pose.orientation.w = 1.0
        msg.poses.append(pose)
    return msg


class PlannerNode(Node):
    def __init__(self, tomo_file, start_pos, end_pos):
        super().__init__('pct_planner')
        self.path_pub = self.create_publisher(Path, '/pct_path', 1)

        cfg = Config()
        planner = TomogramPlanner(cfg)
        self.get_logger().info(f'Loading tomogram: {tomo_file}')
        planner.loadTomogram(tomo_file)

        self.get_logger().info(f'Planning {start_pos} -> {end_pos}')
        traj_3d = planner.plan(start_pos, end_pos)

        if traj_3d is None:
            self.get_logger().warn('No path found!')
        else:
            self.get_logger().info(f'Trajectory: {traj_3d.shape[0]} waypoints')
            path_msg = traj_to_ros2_path(traj_3d, node=self)
            self.path_pub.publish(path_msg)
            out = rsg_root + '/rsc/clinic_traj.npy'
            np.save(out, traj_3d)
            self.get_logger().info(f'Saved to {out}')

        self.get_logger().info('Done. Spinning (Ctrl-C to exit)...')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--tomo', type=str, default='clinic')
    parser.add_argument('--start', type=float, nargs=2, default=[0.0, 0.0])
    parser.add_argument('--end',   type=float, nargs=2, default=[10.0, 5.0])
    args = parser.parse_args()

    ROOT = os.path.dirname(os.path.abspath(__file__)) + '/..'
    sys.path.insert(0, ROOT + '/lib')
    os.environ['LD_LIBRARY_PATH'] = (
        os.environ.get('LD_LIBRARY_PATH', '') +
        ':' + ROOT + '/lib/3rdparty/gtsam-4.1.1/install/lib' +
        ':' + ROOT + '/lib/build/src/common/smoothing'
    )

    rclpy.init()
    node = PlannerNode(
        args.tomo,
        np.array(args.start, dtype=np.float32),
        np.array(args.end,   dtype=np.float32),
    )
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
