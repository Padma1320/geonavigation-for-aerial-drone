#!/usr/bin/env python3

import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from px4_msgs.msg import VehicleOdometry


class LidarOccupancyGrid(Node):

    def __init__(self):
        super().__init__("geonav_lidar_occupancy_grid")

        # ==========================================================
        # MAP CONFIGURATION
        # ==========================================================
        self.width = 30
        self.height = 30
        self.resolution = 1.0

        self.origin_north = -10.0
        self.origin_east = -10.0

        # ==========================================================
        # SENSOR CONFIGURATION
        # ==========================================================
        self.lidar_forward_offset = 0.15
        self.min_cluster_points = 3

        # ==========================================================
        # TEMPORAL STABILIZATION
        # ==========================================================
        self.cluster_history = deque(maxlen=5)

        self.clear_after_misses = 5
        self.missed_scans = 0

        self.dynamic_cell = None

        self.inflation_radius = 1

        # ==========================================================
        # VEHICLE STATE
        # ==========================================================
        self.vehicle_north = None
        self.vehicle_east = None
        self.vehicle_yaw = None

        # Used to avoid republishing identical maps
        self.last_grid_signature = None

        # ==========================================================
        # QOS
        # ==========================================================
        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ==========================================================
        # ROS INTERFACES
        # ==========================================================
        self.create_subscription(
            LaserScan,
            "/geonav/lidar_scan",
            self.scan_callback,
            lidar_qos,
        )

        self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self.odom_callback,
            odom_qos,
        )

        self.map_pub = self.create_publisher(
            OccupancyGrid,
            "/geonav/occupancy_grid",
            map_qos,
        )

        # ==========================================================
        # STATIC MAP
        # ==========================================================
        self.static_cells = set()
        self.build_static_map()

        # ==========================================================
        # STARTUP
        # ==========================================================
        self.get_logger().info(
            "GeoNav LiDAR occupancy mapper started"
        )

        self.get_logger().info(
            "LiDAR cluster-based obstacle stabilization enabled"
        )

        self.get_logger().info(
            "Input: /geonav/lidar_scan"
        )

        self.get_logger().info(
            "Pose:  /fmu/out/vehicle_odometry"
        )

        self.get_logger().info(
            "Output: /geonav/occupancy_grid"
        )

        # IMPORTANT:
        # Publish the initial static map immediately,
        # even when there is no LiDAR obstacle.
        self.publish_map(
            valid_returns=0,
            closest_range=None,
            cluster_center=None,
            force_publish=True,
        )

    # ==============================================================
    # STATIC MAP
    # ==============================================================

    def build_static_map(self):

        self.add_static_rectangle(
            2.0, 7.0,
            3.0, 4.0,
        )

        self.add_static_rectangle(
            7.0, 8.0,
            4.0, 9.0,
        )

        self.add_static_rectangle(
            3.0, 5.0,
            8.0, 9.0,
        )

    def add_static_rectangle(
        self,
        north_min,
        north_max,
        east_min,
        east_max,
    ):

        north = north_min

        while north <= north_max:

            east = east_min

            while east <= east_max:

                cell = self.world_to_grid(
                    north,
                    east,
                )

                if cell is not None:
                    self.static_cells.add(cell)

                east += self.resolution

            north += self.resolution

    # ==============================================================
    # PX4 ODOMETRY
    # ==============================================================

    def odom_callback(self, msg):

        self.vehicle_north = float(
            msg.position[0]
        )

        self.vehicle_east = float(
            msg.position[1]
        )

        q = msg.q

        w = float(q[0])
        x = float(q[1])
        y = float(q[2])
        z = float(q[3])

        siny_cosp = 2.0 * (
            w * z + x * y
        )

        cosy_cosp = 1.0 - 2.0 * (
            y * y + z * z
        )

        self.vehicle_yaw = math.atan2(
            siny_cosp,
            cosy_cosp,
        )

    # ==============================================================
    # LIDAR CALLBACK
    # ==============================================================

    def scan_callback(self, scan):

        if self.vehicle_north is None:
            return

        if self.vehicle_east is None:
            return

        if self.vehicle_yaw is None:
            return

        yaw = self.vehicle_yaw

        sensor_north = (
            self.vehicle_north
            + self.lidar_forward_offset
            * math.cos(yaw)
        )

        sensor_east = (
            self.vehicle_east
            + self.lidar_forward_offset
            * math.sin(yaw)
        )

        world_points = []

        closest_range = float("inf")

        for i, distance in enumerate(scan.ranges):

            if not math.isfinite(distance):
                continue

            if distance < scan.range_min:
                continue

            if distance > scan.range_max:
                continue

            if distance < closest_range:
                closest_range = distance

            scan_angle = (
                scan.angle_min
                + i * scan.angle_increment
            )

            # Gazebo LaserScan:
            # positive scan angle = CCW
            #
            # PX4 NED yaw:
            # positive yaw = clockwise
            #
            # Therefore:
            global_bearing = (
                yaw - scan_angle
            )

            obstacle_north = (
                sensor_north
                + distance
                * math.cos(global_bearing)
            )

            obstacle_east = (
                sensor_east
                + distance
                * math.sin(global_bearing)
            )

            world_points.append(
                (
                    obstacle_north,
                    obstacle_east,
                )
            )

        # ==========================================================
        # NO OBSTACLE
        # ==========================================================
        if len(world_points) < self.min_cluster_points:

            self.missed_scans += 1

            if (
                self.missed_scans
                >= self.clear_after_misses
            ):

                self.cluster_history.clear()

                if self.dynamic_cell is not None:

                    self.dynamic_cell = None

                    self.publish_map(
                        valid_returns=0,
                        closest_range=None,
                        cluster_center=None,
                    )

            return

        # ==========================================================
        # VALID OBSTACLE
        # ==========================================================
        self.missed_scans = 0

        cluster_north = sum(
            point[0]
            for point in world_points
        ) / len(world_points)

        cluster_east = sum(
            point[1]
            for point in world_points
        ) / len(world_points)

        self.cluster_history.append(
            (
                cluster_north,
                cluster_east,
            )
        )

        # ==========================================================
        # TEMPORAL AVERAGING
        # ==========================================================
        stable_north = sum(
            point[0]
            for point in self.cluster_history
        ) / len(self.cluster_history)

        stable_east = sum(
            point[1]
            for point in self.cluster_history
        ) / len(self.cluster_history)

        stable_cell = self.world_to_grid(
            stable_north,
            stable_east,
        )

        if stable_cell is None:
            return

        # ==========================================================
        # CELL HYSTERESIS
        # ==========================================================
        if self.dynamic_cell is None:

            self.dynamic_cell = stable_cell

        else:

            old_center_north, old_center_east = (
                self.grid_to_world_center(
                    self.dynamic_cell
                )
            )

            movement = math.hypot(
                stable_north
                - old_center_north,
                stable_east
                - old_center_east,
            )

            if (
                stable_cell != self.dynamic_cell
                and movement > 0.75
            ):

                self.dynamic_cell = stable_cell

        self.publish_map(
            valid_returns=len(world_points),
            closest_range=closest_range,
            cluster_center=(
                stable_north,
                stable_east,
            ),
        )

    # ==============================================================
    # GRID CONVERSION
    # ==============================================================

    def world_to_grid(
        self,
        north,
        east,
    ):

        gx = math.floor(
            (
                north
                - self.origin_north
            )
            / self.resolution
        )

        gy = math.floor(
            (
                east
                - self.origin_east
            )
            / self.resolution
        )

        if gx < 0 or gx >= self.width:
            return None

        if gy < 0 or gy >= self.height:
            return None

        return (
            gx,
            gy,
        )

    def grid_to_world_center(
        self,
        cell,
    ):

        gx, gy = cell

        north = (
            self.origin_north
            + (
                gx + 0.5
            )
            * self.resolution
        )

        east = (
            self.origin_east
            + (
                gy + 0.5
            )
            * self.resolution
        )

        return (
            north,
            east,
        )

    def grid_index(
        self,
        gx,
        gy,
    ):

        return (
            gy * self.width
            + gx
        )

    # ==============================================================
    # OBSTACLE INFLATION
    # ==============================================================

    def inflate_cell(
        self,
        cell,
    ):

        inflated = set()

        if cell is None:
            return inflated

        gx, gy = cell

        for dx in range(
            -self.inflation_radius,
            self.inflation_radius + 1,
        ):

            for dy in range(
                -self.inflation_radius,
                self.inflation_radius + 1,
            ):

                nx = gx + dx
                ny = gy + dy

                if (
                    0 <= nx < self.width
                    and 0 <= ny < self.height
                ):

                    inflated.add(
                        (
                            nx,
                            ny,
                        )
                    )

        return inflated

    # ==============================================================
    # MAP PUBLISHER
    # ==============================================================

    def publish_map(
        self,
        valid_returns,
        closest_range,
        cluster_center,
        force_publish=False,
    ):

        grid = [
            0
            for _ in range(
                self.width
                * self.height
            )
        ]

        # Static obstacles
        for gx, gy in self.static_cells:

            grid[
                self.grid_index(
                    gx,
                    gy,
                )
            ] = 100

        # Dynamic LiDAR obstacle
        dynamic_cells = self.inflate_cell(
            self.dynamic_cell
        )

        for gx, gy in dynamic_cells:

            grid[
                self.grid_index(
                    gx,
                    gy,
                )
            ] = 100

        signature = tuple(grid)

        if (
            not force_publish
            and signature
            == self.last_grid_signature
        ):
            return

        self.last_grid_signature = signature

        msg = OccupancyGrid()

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = "map"

        msg.info.resolution = (
            self.resolution
        )

        msg.info.width = (
            self.width
        )

        msg.info.height = (
            self.height
        )

        msg.info.origin.position.x = (
            self.origin_north
        )

        msg.info.origin.position.y = (
            self.origin_east
        )

        msg.info.origin.position.z = 0.0

        msg.info.origin.orientation.w = 1.0

        msg.data = grid

        self.map_pub.publish(msg)

        if force_publish:

            self.get_logger().info(
                "INITIAL MAP PUBLISHED | "
                "static map available | "
                "no dynamic LiDAR obstacle"
            )

        elif (
            valid_returns > 0
            and cluster_center is not None
        ):

            north, east = cluster_center

            self.get_logger().info(
                "MAP CHANGED | "
                f"{valid_returns} LiDAR returns | "
                f"cluster N={north:.2f} m "
                f"E={east:.2f} m | "
                f"cell={self.dynamic_cell} | "
                f"closest {closest_range:.2f} m"
            )

        else:

            self.get_logger().info(
                "MAP CHANGED | "
                "dynamic LiDAR obstacle cleared"
            )


def main(args=None):

    rclpy.init(args=args)

    node = LidarOccupancyGrid()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
