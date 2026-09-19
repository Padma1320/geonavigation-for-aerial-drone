#!/usr/bin/env python3

import math
import time

from gz.transport13 import Node
from gz.msgs10.laserscan_pb2 import LaserScan


TOPIC = (
    "/world/default/model/x500_lidar_front_0/"
    "link/lidar_sensor_link/sensor/lidar/scan"
)

message_count = 0


def lidar_callback(msg: LaserScan):
    global message_count
    message_count += 1

    ranges = list(msg.ranges)

    valid = [
        (i, r)
        for i, r in enumerate(ranges)
        if math.isfinite(r)
        and msg.range_min <= r <= msg.range_max
    ]

    # Print only every 20th scan so the terminal stays readable.
    if message_count % 20 != 0:
        return

    print("\n--- GeoNav LiDAR ---")
    print(f"scan: {message_count}")
    print(f"total rays: {len(ranges)}")
    print(f"valid obstacle returns: {len(valid)}")

    if valid:
        closest_index, closest_range = min(
            valid,
            key=lambda item: item[1]
        )

        closest_angle = (
            msg.angle_min +
            closest_index * msg.angle_step
        )

        print(f"closest obstacle: {closest_range:.2f} m")
        print(
            f"closest angle: "
            f"{math.degrees(closest_angle):.1f} deg"
        )

        print("\nSample detections:")

        for index, distance in valid[:10]:
            angle = (
                msg.angle_min +
                index * msg.angle_step
            )

            print(
                f"  ray {index:3d}: "
                f"{distance:6.2f} m "
                f"@ {math.degrees(angle):6.1f} deg"
            )

    else:
        print("No obstacle inside LiDAR range.")


def main():
    node = Node()

    print("GeoNav native Gazebo LiDAR test")
    print(f"Topic: {TOPIC}")

    success = node.subscribe(
        LaserScan,
        TOPIC,
        lidar_callback
    )

    if not success:
        print("ERROR: Gazebo LiDAR subscription failed.")
        return

    print("Subscription successful.")
    print("Waiting for LiDAR scans...")
    print("Ctrl+C to stop.")

    try:
        while True:
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
