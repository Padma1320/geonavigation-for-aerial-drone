#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from sensor_msgs.msg import LaserScan

from gz.transport13 import Node as GzNode
from gz.msgs10.laserscan_pb2 import LaserScan as GzLaserScan


class GeoNavLidarInterface(Node):

    def __init__(self):
        super().__init__("geonav_lidar_interface")

        self.gz_topic = (
            "/world/default/model/x500_lidar_front_0/"
            "link/lidar_sensor_link/sensor/lidar/scan"
        )

        self.ros_topic = "/geonav/lidar_scan"

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.publisher = self.create_publisher(
            LaserScan,
            self.ros_topic,
            qos,
        )

        self.gz_node = GzNode()

        self.scan_count = 0

        success = self.gz_node.subscribe(
            GzLaserScan,
            self.gz_topic,
            self.gz_scan_callback,
        )

        if not success:
            raise RuntimeError(
                f"Failed to subscribe to Gazebo topic: {self.gz_topic}"
            )

        self.get_logger().info(
            "GeoNav LiDAR interface started"
        )

        self.get_logger().info(
            f"Gazebo: {self.gz_topic}"
        )

        self.get_logger().info(
            f"ROS 2:   {self.ros_topic}"
        )

        self.get_logger().info(
            "Native Gazebo Transport -> ROS 2 enabled"
        )

    def gz_scan_callback(self, gz_msg: GzLaserScan):

        ros_msg = LaserScan()

        # ROS timestamp
        ros_msg.header.stamp = self.get_clock().now().to_msg()

        ros_msg.header.frame_id = "lidar_sensor_link"

        ros_msg.angle_min = float(gz_msg.angle_min)
        ros_msg.angle_max = float(gz_msg.angle_max)
        ros_msg.angle_increment = float(gz_msg.angle_step)

        ros_msg.range_min = float(gz_msg.range_min)
        ros_msg.range_max = float(gz_msg.range_max)

        # Gazebo sensor configured at 20 Hz.
        ros_msg.scan_time = 1.0 / 20.0

        if gz_msg.count > 1:
            ros_msg.time_increment = (
                ros_msg.scan_time /
                float(gz_msg.count - 1)
            )
        else:
            ros_msg.time_increment = 0.0

        ros_msg.ranges = [
            float(r)
            for r in gz_msg.ranges
        ]

        # Gazebo LiDAR currently provides no useful intensity data.
        ros_msg.intensities = []

        self.publisher.publish(ros_msg)

        self.scan_count += 1

        # Status message roughly once per second.
        if self.scan_count % 20 == 0:

            valid_ranges = [
                r for r in ros_msg.ranges
                if math.isfinite(r)
                and ros_msg.range_min <= r <= ros_msg.range_max
            ]

            if valid_ranges:
                closest = min(valid_ranges)

                self.get_logger().info(
                    f"LiDAR OK | "
                    f"{len(ros_msg.ranges)} rays | "
                    f"{len(valid_ranges)} returns | "
                    f"closest {closest:.2f} m"
                )

            else:
                self.get_logger().info(
                    f"LiDAR OK | "
                    f"{len(ros_msg.ranges)} rays | "
                    "no obstacle returns"
                )


def main(args=None):

    rclpy.init(args=args)

    node = GeoNavLidarInterface()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
