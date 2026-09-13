#!/usr/bin/env python3
"""
Publish a color image as a 3D point-cloud "floor" for RViz.

The point cloud is placed in the XY plane using the same
resolution/origin convention as a nav2 map.yaml file, so it lines up
with your existing map/costmap if you have one.

Usage (ROS 2):
    python3 image_to_pointcloud.py \
        --image /path/to/isaac_top_view.png \
        --resolution 0.05 \
        --origin_x -10.0 --origin_y -10.0 \
        --frame map \
        --topic /background_map \
        --stride 2
"""

import argparse
import struct

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def image_to_pointcloud2(image_path, resolution, origin_x, origin_y, z, frame_id, stride):
    img = cv2.imread(image_path, cv2.IMREAD_COLOR)  # BGR
    if img is None:
        raise FileNotFoundError(image_path)
    h, w, _ = img.shape

    points = []
    for row in range(0, h, stride):
        for col in range(0, w, stride):
            b, g, r = img[row, col]
            # Flip row so image "up" matches +Y, matching map.yaml convention
            x = origin_x + col * resolution
            y = origin_y + (h - row) * resolution
            rgb = struct.unpack('f', struct.pack('I', (int(r) << 16) | (int(g) << 8) | int(b)))[0]
            points.append((x, y, z, rgb))

    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
    ]

    header = Header()
    header.frame_id = frame_id

    cloud = PointCloud2()
    cloud.header = header
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = fields
    cloud.is_bigendian = False
    cloud.point_step = 16
    cloud.row_step = cloud.point_step * len(points)
    cloud.is_dense = True

    buf = bytearray()
    for x, y, zc, rgb in points:
        buf += struct.pack('ffff', x, y, zc, rgb)
    cloud.data = bytes(buf)

    return cloud


class BackgroundMapPublisher(Node):
    def __init__(self, args):
        super().__init__('background_map_publisher')
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL  # latched, like a static map
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.pub = self.create_publisher(PointCloud2, args.topic, qos)
        self.cloud = image_to_pointcloud2(
            args.image, args.resolution, args.origin_x, args.origin_y,
            args.z, args.frame, args.stride,
        )
        # Republish periodically too, in case a late subscriber's QoS doesn't match
        self.timer = self.create_timer(2.0, self.publish_cloud)

    def publish_cloud(self):
        self.cloud.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.cloud)
        self.get_logger().info(f'Published background map ({self.cloud.width} points)')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--image', required=True)
    parser.add_argument('--resolution', type=float, required=True,
                         help='meters per pixel, same as your map.yaml')
    parser.add_argument('--origin_x', type=float, default=0.0)
    parser.add_argument('--origin_y', type=float, default=0.0)
    parser.add_argument('--z', type=float, default=-0.01,
                         help='height to draw the floor at; slightly negative avoids z-fighting with the robot/costmap')
    parser.add_argument('--frame', default='map')
    parser.add_argument('--topic', default='/background_map')
    parser.add_argument('--stride', type=int, default=2,
                         help='pixel skip factor to keep point count manageable')
    args = parser.parse_args()

    rclpy.init()
    node = BackgroundMapPublisher(args)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()