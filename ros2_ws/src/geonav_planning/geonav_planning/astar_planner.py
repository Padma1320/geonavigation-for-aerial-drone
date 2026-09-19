# ~/geonav_ai/ros2_ws/src/geonav_planning/geonav_planning/astar_planner.py

import heapq
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleOdometry


class AStarPlanner(Node):

    def __init__(self):
        super().__init__('geonav_astar_planner')

        # ==================================================
        # PX4 QoS
        # ==================================================

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ==================================================
        # Planned-path QoS
        #
        # TRANSIENT_LOCAL means the latest path is retained.
        # A controller started later can immediately receive it.
        # ==================================================

        path_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ==================================================
        # Subscribers
        # ==================================================

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/geonav/occupancy_grid',
            self.map_callback,
            10,
        )

        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            px4_qos,
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/geonav/goal',
            self.goal_callback,
            10,
        )

        # ==================================================
        # Publisher
        # ==================================================

        self.path_pub = self.create_publisher(
            Path,
            '/geonav/planned_path',
            path_qos,
        )

        # ==================================================
        # Map state
        # ==================================================

        self.grid = None

        self.width = None
        self.height = None
        self.resolution = None

        self.origin_x = 0.0
        self.origin_y = 0.0

        self.last_map_signature = None

        # ==================================================
        # UAV state
        # ==================================================

        self.uav_x = None
        self.uav_y = None

        # ==================================================
        # Goal state
        # ==================================================

        self.goal_x = None
        self.goal_y = None

        # ==================================================
        # Planner parameters
        # ==================================================

        self.inflation_radius = 1

        self.get_logger().info(
            'GeoNav A* planner started'
        )

        self.get_logger().info(
            'EVENT-DRIVEN REPLANNING ENABLED'
        )

        self.get_logger().info(
            'Replan only on NEW GOAL or CHANGED MAP'
        )

        self.get_logger().info(
            'TRANSIENT_LOCAL PATH QoS ENABLED'
        )

    # ======================================================
    # ODOMETRY
    # ======================================================

    def odom_callback(self, msg):

        self.uav_x = float(
            msg.position[0]
        )

        self.uav_y = float(
            msg.position[1]
        )

    # ======================================================
    # GOAL
    # ======================================================

    def goal_callback(self, msg):

        self.goal_x = float(
            msg.pose.position.x
        )

        self.goal_y = float(
            msg.pose.position.y
        )

        self.get_logger().info(
            f'NEW GOAL received: '
            f'North={self.goal_x:.2f}, '
            f'East={self.goal_y:.2f}'
        )

        self.plan_path(
            reason='NEW GOAL'
        )

    # ======================================================
    # MAP
    # ======================================================

    def map_callback(self, msg):

        map_signature = (
            msg.info.width,
            msg.info.height,
            round(
                float(msg.info.resolution),
                6
            ),
            round(
                float(msg.info.origin.position.x),
                6
            ),
            round(
                float(msg.info.origin.position.y),
                6
            ),
            tuple(msg.data),
        )

        # Same exact map -> ignore.
        if (
            self.last_map_signature
            ==
            map_signature
        ):
            return

        first_map = (
            self.last_map_signature is None
        )

        self.last_map_signature = (
            map_signature
        )

        self.width = int(
            msg.info.width
        )

        self.height = int(
            msg.info.height
        )

        self.resolution = float(
            msg.info.resolution
        )

        self.origin_x = float(
            msg.info.origin.position.x
        )

        self.origin_y = float(
            msg.info.origin.position.y
        )

        self.grid = []

        for row in range(
            self.height
        ):

            current_row = []

            for col in range(
                self.width
            ):

                index = (
                    row
                    *
                    self.width
                    +
                    col
                )

                value = (
                    msg.data[index]
                )

                occupied = (
                    1
                    if value >= 50
                    else 0
                )

                current_row.append(
                    occupied
                )

            self.grid.append(
                current_row
            )

        if first_map:

            self.get_logger().info(
                f'Initial occupancy map received: '
                f'{self.width}x{self.height}, '
                f'resolution={self.resolution:.2f} m'
            )

        else:

            self.get_logger().warning(
                'OCCUPANCY MAP CHANGED'
            )

        if self.goal_x is None:

            self.get_logger().info(
                'Waiting for /geonav/goal'
            )

            return

        if first_map:

            self.plan_path(
                reason='INITIAL MAP'
            )

        else:

            self.plan_path(
                reason='MAP CHANGED'
            )

    # ======================================================
    # WORLD <-> GRID
    # ======================================================

    def world_to_grid(
        self,
        north,
        east
    ):

        row = math.floor(
            (
                north
                -
                self.origin_x
            )
            /
            self.resolution
        )

        col = math.floor(
            (
                east
                -
                self.origin_y
            )
            /
            self.resolution
        )

        return (
            int(row),
            int(col)
        )

    def grid_to_world(
        self,
        row,
        col
    ):

        north = (
            self.origin_x
            +
            row
            *
            self.resolution
        )

        east = (
            self.origin_y
            +
            col
            *
            self.resolution
        )

        return (
            north,
            east
        )

    # ======================================================
    # BOUNDS
    # ======================================================

    def in_bounds(
        self,
        cell
    ):

        if (
            self.width is None
            or
            self.height is None
        ):

            return False

        row, col = cell

        return (
            0 <= row < self.height
            and
            0 <= col < self.width
        )

    # ======================================================
    # OBSTACLE INFLATION
    # ======================================================

    def inflate_obstacles(
        self,
        grid,
        radius
    ):

        inflated = [
            row[:]
            for row in grid
        ]

        for row in range(
            self.height
        ):

            for col in range(
                self.width
            ):

                if (
                    grid[row][col]
                    != 1
                ):
                    continue

                for dr in range(
                    -radius,
                    radius + 1
                ):

                    for dc in range(
                        -radius,
                        radius + 1
                    ):

                        nr = row + dr
                        nc = col + dc

                        if (
                            0 <= nr < self.height
                            and
                            0 <= nc < self.width
                        ):

                            inflated[nr][nc] = 1

        return inflated

    # ======================================================
    # SAFE START RECOVERY
    # ======================================================

    def nearest_free_cell(
        self,
        grid,
        start,
        max_radius=4
    ):

        if not self.in_bounds(
            start
        ):

            return None

        sr, sc = start

        if (
            grid[sr][sc]
            == 0
        ):

            return start

        self.get_logger().warning(
            f'Start cell {start} '
            f'is occupied after inflation'
        )

        best_cell = None
        best_distance = float(
            'inf'
        )

        for radius in range(
            1,
            max_radius + 1
        ):

            for dr in range(
                -radius,
                radius + 1
            ):

                for dc in range(
                    -radius,
                    radius + 1
                ):

                    if (
                        abs(dr) != radius
                        and
                        abs(dc) != radius
                    ):
                        continue

                    nr = sr + dr
                    nc = sc + dc

                    candidate = (
                        nr,
                        nc
                    )

                    if not self.in_bounds(
                        candidate
                    ):

                        continue

                    if (
                        grid[nr][nc]
                        != 0
                    ):

                        continue

                    distance = math.sqrt(
                        dr * dr
                        +
                        dc * dc
                    )

                    if (
                        distance
                        <
                        best_distance
                    ):

                        best_distance = (
                            distance
                        )

                        best_cell = (
                            candidate
                        )

            if (
                best_cell
                is not None
            ):
                break

        if (
            best_cell
            is None
        ):

            self.get_logger().error(
                'No nearby free start cell found'
            )

            return None

        self.get_logger().warning(
            f'SAFE START RECOVERY: '
            f'{start} -> {best_cell}'
        )

        return best_cell

    # ======================================================
    # NEIGHBORS
    # ======================================================

    def get_neighbors(
        self,
        current,
        grid
    ):

        row, col = current

        motions = [
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),

            (
                -1,
                -1,
                math.sqrt(2)
            ),
            (
                -1,
                1,
                math.sqrt(2)
            ),
            (
                1,
                -1,
                math.sqrt(2)
            ),
            (
                1,
                1,
                math.sqrt(2)
            ),
        ]

        neighbors = []

        for (
            dr,
            dc,
            cost
        ) in motions:

            nr = row + dr
            nc = col + dc

            candidate = (
                nr,
                nc
            )

            if not self.in_bounds(
                candidate
            ):
                continue

            if (
                grid[nr][nc]
                == 1
            ):
                continue

            # Prevent diagonal corner cutting.
            if (
                dr != 0
                and
                dc != 0
            ):

                side1 = (
                    row + dr,
                    col
                )

                side2 = (
                    row,
                    col + dc
                )

                if (
                    not self.in_bounds(
                        side1
                    )
                    or
                    not self.in_bounds(
                        side2
                    )
                ):
                    continue

                if (
                    grid[
                        side1[0]
                    ][
                        side1[1]
                    ]
                    == 1
                    or
                    grid[
                        side2[0]
                    ][
                        side2[1]
                    ]
                    == 1
                ):
                    continue

            neighbors.append(
                (
                    candidate,
                    cost
                )
            )

        return neighbors

    # ======================================================
    # HEURISTIC
    # ======================================================

    def heuristic(
        self,
        a,
        b
    ):

        return math.sqrt(
            (
                a[0]
                -
                b[0]
            ) ** 2
            +
            (
                a[1]
                -
                b[1]
            ) ** 2
        )

    # ======================================================
    # A*
    # ======================================================

    def astar(
        self,
        grid,
        start,
        goal
    ):

        open_heap = []

        heapq.heappush(
            open_heap,
            (
                0.0,
                start
            )
        )

        came_from = {}

        g_score = {
            start: 0.0
        }

        closed = set()

        while open_heap:

            _, current = (
                heapq.heappop(
                    open_heap
                )
            )

            if (
                current
                in closed
            ):
                continue

            closed.add(
                current
            )

            if (
                current
                ==
                goal
            ):

                path = [
                    current
                ]

                while (
                    current
                    in came_from
                ):

                    current = (
                        came_from[
                            current
                        ]
                    )

                    path.append(
                        current
                    )

                path.reverse()

                return path

            for (
                neighbor,
                move_cost
            ) in self.get_neighbors(
                current,
                grid
            ):

                tentative_g = (
                    g_score[current]
                    +
                    move_cost
                )

                if (
                    neighbor
                    not in g_score
                    or
                    tentative_g
                    <
                    g_score[
                        neighbor
                    ]
                ):

                    came_from[
                        neighbor
                    ] = current

                    g_score[
                        neighbor
                    ] = tentative_g

                    f_score = (
                        tentative_g
                        +
                        self.heuristic(
                            neighbor,
                            goal
                        )
                    )

                    heapq.heappush(
                        open_heap,
                        (
                            f_score,
                            neighbor
                        )
                    )

        return None

    # ======================================================
    # PATH SIMPLIFICATION
    # ======================================================

    def simplify_path(
        self,
        path
    ):

        if (
            path is None
        ):
            return None

        if (
            len(path)
            <= 2
        ):
            return path

        simplified = [
            path[0]
        ]

        previous_direction = None

        for i in range(
            1,
            len(path)
        ):

            direction = (
                path[i][0]
                -
                path[i - 1][0],

                path[i][1]
                -
                path[i - 1][1],
            )

            if (
                previous_direction
                is None
            ):

                previous_direction = (
                    direction
                )

                continue

            if (
                direction
                !=
                previous_direction
            ):

                simplified.append(
                    path[i - 1]
                )

            previous_direction = (
                direction
            )

        simplified.append(
            path[-1]
        )

        return simplified

    # ======================================================
    # PLAN PATH
    # ======================================================

    def plan_path(
        self,
        reason='UNKNOWN'
    ):

        if (
            self.grid
            is None
        ):

            self.get_logger().info(
                'Waiting for occupancy grid'
            )

            return

        if (
            self.uav_x is None
            or
            self.uav_y is None
        ):

            self.get_logger().info(
                'Waiting for UAV odometry'
            )

            return

        if (
            self.goal_x is None
            or
            self.goal_y is None
        ):

            self.get_logger().info(
                'Waiting for /geonav/goal'
            )

            return

        self.get_logger().info(
            f'Planning triggered by: '
            f'{reason}'
        )

        self.get_logger().info(
            f'UAV local position: '
            f'x={self.uav_x:.2f}, '
            f'y={self.uav_y:.2f}'
        )

        start = self.world_to_grid(
            self.uav_x,
            self.uav_y
        )

        goal = self.world_to_grid(
            self.goal_x,
            self.goal_y
        )

        self.get_logger().info(
            f'Dynamic start cell: '
            f'{start}'
        )

        self.get_logger().info(
            f'Dynamic goal cell: '
            f'{goal}'
        )

        if not self.in_bounds(
            start
        ):

            self.get_logger().error(
                f'Start cell '
                f'{start} outside map'
            )

            return

        if not self.in_bounds(
            goal
        ):

            self.get_logger().error(
                f'Goal cell '
                f'{goal} outside map'
            )

            return

        inflated_grid = (
            self.inflate_obstacles(
                self.grid,
                self.inflation_radius
            )
        )

        safe_start = (
            self.nearest_free_cell(
                inflated_grid,
                start
            )
        )

        if (
            safe_start
            is None
        ):

            self.get_logger().error(
                'Planning failed: '
                'no safe start'
            )

            return

        if (
            inflated_grid[
                goal[0]
            ][
                goal[1]
            ]
            == 1
        ):

            self.get_logger().warning(
                f'Goal cell {goal} '
                f'is occupied after inflation'
            )

            return

        path = self.astar(
            inflated_grid,
            safe_start,
            goal
        )

        if (
            path is None
        ):

            self.get_logger().warning(
                'No valid A* path found'
            )

            return

        self.get_logger().info(
            f'Raw A* path: '
            f'{len(path)} cells'
        )

        simplified = (
            self.simplify_path(
                path
            )
        )

        self.get_logger().info(
            f'Simplified path: '
            f'{len(simplified)} waypoints'
        )

        self.publish_path(
            simplified
        )

    # ======================================================
    # PUBLISH PATH
    # ======================================================

    def publish_path(
        self,
        path
    ):

        msg = Path()

        now = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.stamp = now
        msg.header.frame_id = 'map'

        for (
            row,
            col
        ) in path:

            (
                north,
                east
            ) = self.grid_to_world(
                row,
                col
            )

            pose = PoseStamped()

            pose.header.stamp = now
            pose.header.frame_id = 'map'

            pose.pose.position.x = float(
                north
            )

            pose.pose.position.y = float(
                east
            )

            pose.pose.position.z = 2.0

            pose.pose.orientation.w = 1.0

            msg.poses.append(
                pose
            )

        self.path_pub.publish(
            msg
        )

        self.get_logger().info(
            f'Published dynamic path '
            f'with {len(msg.poses)} poses'
        )


def main(args=None):

    rclpy.init(args=args)

    node = AStarPlanner()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
