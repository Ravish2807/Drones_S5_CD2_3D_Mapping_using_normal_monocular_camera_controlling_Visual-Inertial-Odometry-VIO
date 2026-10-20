#!/usr/bin/env python3
"""Report whether the dense mapper is producing depth and obstacle markers."""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray


class ObstacleChecker(Node):
    def __init__(self):
        super().__init__('obstacle_output_checker')
        self.last_camera = None
        self.last_depth = None
        self.last_obstacles = None
        self.camera_count = 0
        self.depth_count = 0
        self.array_count = 0
        self.obstacle_count = 0
        self.create_subscription(Image, '/camera/image_raw', self.camera_cb, 10)
        self.create_subscription(Image, '/camera/depth/image_raw', self.depth_cb, 10)
        self.create_subscription(MarkerArray, '/drone/obstacles', self.obstacle_cb, 10)
        self.create_timer(2.0, self.report)
        self.started = time.monotonic()
        print('Watching /camera/image_raw, /camera/depth/image_raw, /drone/obstacles')
        print('Keep this running while the mapper and RViz are open. Ctrl+C exits.')

    def camera_cb(self, msg):
        self.last_camera = time.monotonic()
        self.camera_count += 1

    def depth_cb(self, msg):
        self.last_depth = time.monotonic()
        self.depth_count += 1

    def obstacle_cb(self, msg):
        self.last_obstacles = time.monotonic()
        self.array_count += 1
        cubes = [m for m in msg.markers if m.type == Marker.CUBE and m.action == Marker.ADD]
        labels = [m.text for m in msg.markers if m.type == Marker.TEXT_VIEW_FACING and m.action == Marker.ADD]
        self.obstacle_count = len(cubes)
        if cubes:
            print(f'\nDETECTED {len(cubes)} obstacle box(es):')
            for box, label in zip(cubes, labels):
                p, s = box.pose.position, box.scale
                print(f'  {label} | center=({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) m '
                      f'size=({s.x:.2f}, {s.y:.2f}, {s.z:.2f}) m')

    @staticmethod
    def age(last_time):
        return 'never' if last_time is None else f'{time.monotonic() - last_time:.1f}s ago'

    def report(self):
        elapsed = time.monotonic() - self.started
        print(
            f'[{elapsed:5.0f}s] camera={self.camera_count} (last {self.age(self.last_camera)}), '
            f'depth={self.depth_count} (last {self.age(self.last_depth)}), '
            f'obstacle arrays={self.array_count} (last {self.age(self.last_obstacles)}), '
            f'boxes in latest array={self.obstacle_count}'
        )
        if self.last_camera is None:
            print('  CHECK: no camera images reach /camera/image_raw; start/fix the camera bridge.')
        elif self.last_depth is None:
            print('  CHECK: camera works, but the dense mapper has not published depth yet. '
                  'Check dense_mapping_node logs/model startup and confirm it is running.')
        elif self.last_obstacles is None:
            print('  CHECK: depth works, but /drone/obstacles has no publisher data yet.')
        elif self.obstacle_count == 0:
            print('  CHECK: mapper is publishing, but found no clusters. Put a textured object '
                  '0.5-1.5 m ahead, well inside the view, and wait a few seconds.')


def main():
    rclpy.init()
    node = ObstacleChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
