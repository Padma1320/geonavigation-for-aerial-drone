import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid


class OccupancyGridPublisher(Node):

    def __init__(self):
        super().__init__('geonav_occupancy_grid_publisher')

        self.publisher_ = self.create_publisher(
            OccupancyGrid,
            '/geonav/occupancy_grid',
            10,
        )

        # ==================================================
        # Map parameters
        # ==================================================

        self.width = 30
        self.height = 30
        self.resolution = 1.0

        self.origin_x = -10.0
        self.origin_y = -10.0

        # ==================================================
        # Dynamic obstacle state
        # ==================================================

        self.dynamic_obstacle_enabled = False

        # Publish map at 1 Hz
        self.map_timer = self.create_timer(
            1.0,
            self.publish_map,
        )

        # Activate dynamic obstacle once after 8 seconds
        self.dynamic_timer = self.create_timer(
            8.0,
            self.enable_dynamic_obstacle,
        )

        self.get_logger().info(
            'GeoNav occupancy grid publisher started'
        )

        self.get_logger().info(
            'STATIC OBSTACLES ENABLED'
        )

        self.get_logger().info(
            'Dynamic obstacle scheduled after 8 seconds'
        )

    # ======================================================
    # World -> grid
    # ======================================================

    def world_to_grid(
        self,
        north,
        east,
    ):

        row = int(
            (
                north
                -
                self.origin_x
            )
            /
            self.resolution
        )

        col = int(
            (
                east
                -
                self.origin_y
            )
            /
            self.resolution
        )

        return row, col

    # ======================================================
    # Rectangular obstacle helper
    # ======================================================

    def add_rect_obstacle(
        self,
        data,
        north_min,
        north_max,
        east_min,
        east_max,
    ):

        row_min, col_min = self.world_to_grid(
            north_min,
            east_min,
        )

        row_max, col_max = self.world_to_grid(
            north_max,
            east_max,
        )

        row_min = max(
            0,
            min(
                self.height - 1,
                row_min,
            ),
        )

        row_max = max(
            0,
            min(
                self.height - 1,
                row_max,
            ),
        )

        col_min = max(
            0,
            min(
                self.width - 1,
                col_min,
            ),
        )

        col_max = max(
            0,
            min(
                self.width - 1,
                col_max,
            ),
        )

        for row in range(
            row_min,
            row_max + 1,
        ):

            for col in range(
                col_min,
                col_max + 1,
            ):

                index = (
                    row
                    *
                    self.width
                    +
                    col
                )

                data[index] = 100

    # ======================================================
    # Enable dynamic obstacle
    # ======================================================

    def enable_dynamic_obstacle(self):

        if self.dynamic_obstacle_enabled:
            return

        self.dynamic_obstacle_enabled = True

        self.get_logger().warning(
            'DYNAMIC OBSTACLE ACTIVATED'
        )

        self.get_logger().warning(
            'Occupancy grid changed - planner should replan'
        )

        # Fire only once
        self.dynamic_timer.cancel()

        # Immediately publish changed map
        self.publish_map()

    # ======================================================
    # Publish map
    # ======================================================

    def publish_map(self):

        msg = OccupancyGrid()

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = 'map'

        msg.info.resolution = float(
            self.resolution
        )

        msg.info.width = int(
            self.width
        )

        msg.info.height = int(
            self.height
        )

        msg.info.origin.position.x = float(
            self.origin_x
        )

        msg.info.origin.position.y = float(
            self.origin_y
        )

        msg.info.origin.position.z = 0.0

        msg.info.origin.orientation.w = 1.0

        data = [
            0
            for _ in range(
                self.width
                *
                self.height
            )
        ]

        # ==================================================
        # Static obstacles
        # ==================================================

        self.add_rect_obstacle(
            data,
            2.0,
            7.0,
            3.0,
            4.0,
        )

        self.add_rect_obstacle(
            data,
            7.0,
            8.0,
            4.0,
            9.0,
        )

        self.add_rect_obstacle(
            data,
            3.0,
            5.0,
            8.0,
            9.0,
        )

        # ==================================================
        # Dynamic obstacle
        # ==================================================

        if self.dynamic_obstacle_enabled:

            self.add_rect_obstacle(
                data,
                -1.0,
                2.0,
                0.0,
                2.0,
            )

        msg.data = data

        self.publisher_.publish(
            msg
        )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = OccupancyGridPublisher()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
