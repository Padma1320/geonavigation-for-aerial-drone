# ~/geonav_ai/ros2_ws/src/geonav_control/geonav_control/planned_mission.py

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from nav_msgs.msg import Path

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleOdometry,
    VehicleLandDetected,
)


class GeoNavPlannedMission(Node):

    def __init__(self):
        super().__init__('geonav_planned_mission')

        # ==========================================================
        # PX4 QoS
        # ==========================================================

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ==========================================================
        # Planner path QoS
        # ==========================================================

        path_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ==========================================================
        # PX4 Publishers
        # ==========================================================

        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10,
        )

        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10,
        )

        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10,
        )

        # ==========================================================
        # Subscribers
        # ==========================================================

        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            px4_qos,
        )

        self.land_sub = self.create_subscription(
            VehicleLandDetected,
            '/fmu/out/vehicle_land_detected',
            self.land_callback,
            px4_qos,
        )

        self.path_sub = self.create_subscription(
            Path,
            '/geonav/planned_path',
            self.path_callback,
            path_qos,
        )

        # ==========================================================
        # Vehicle state
        # ==========================================================

        self.x = None
        self.y = None
        self.z = None

        self.landed = None
        self.at_rest = None

        # ==========================================================
        # Mission state
        # ==========================================================

        self.waypoints = []

        self.current_wp = 0

        self.path_signature = None

        self.arm_requested = False

        self.airborne_once = False

        self.land_requested = False

        self.mission_complete = False

        # ==========================================================
        # Parameters
        # ==========================================================

        self.timer_count = 0

        self.prearm_stream_cycles = 20

        self.waypoint_tolerance = 0.45

        # ==========================================================
        # Control timer
        # ==========================================================

        self.timer = self.create_timer(
            0.1,
            self.control_loop,
        )

        # ==========================================================
        # Startup logs
        # ==========================================================

        self.get_logger().info(
            'GeoNav mission controller started'
        )

        self.get_logger().info(
            'MID-FLIGHT REPLANNING ENABLED'
        )

        self.get_logger().info(
            'LANDING REPLAN GUARD ENABLED'
        )

        self.get_logger().info(
            'TRANSIENT_LOCAL PATH QoS ENABLED'
        )

        self.get_logger().info(
            'Waiting for odometry + landed state + planner path'
        )

    # ==============================================================
    # Timestamp
    # ==============================================================

    def timestamp_us(self):

        return int(
            self.get_clock().now().nanoseconds
            /
            1000
        )

    # ==============================================================
    # Odometry callback
    # ==============================================================

    def odom_callback(self, msg):

        self.x = float(
            msg.position[0]
        )

        self.y = float(
            msg.position[1]
        )

        self.z = float(
            msg.position[2]
        )

    # ==============================================================
    # Land detector callback
    # ==============================================================

    def land_callback(self, msg):

        new_landed = bool(
            msg.landed
        )

        new_at_rest = bool(
            msg.at_rest
        )

        state_changed = (
            self.landed != new_landed
            or
            self.at_rest != new_at_rest
        )

        self.landed = new_landed

        self.at_rest = new_at_rest

        if state_changed:

            self.get_logger().info(
                f'PX4 landed state -> '
                f'landed={self.landed}, '
                f'at_rest={self.at_rest}'
            )

        if self.landed is False:

            self.airborne_once = True

        if (
            self.land_requested
            and
            self.airborne_once
            and
            self.landed is True
            and
            self.at_rest is True
        ):

            if not self.mission_complete:

                self.mission_complete = True

                self.get_logger().info(
                    'LANDING COMPLETE: '
                    'PX4 confirms LANDED + AT REST'
                )

    # ==============================================================
    # Path callback
    # ==============================================================

    def path_callback(self, msg):

        # ----------------------------------------------------------
        # Ignore paths after landing has started
        # ----------------------------------------------------------

        if self.land_requested:

            self.get_logger().warning(
                'REPLAN IGNORED: LAND already requested'
            )

            return

        if self.mission_complete:

            self.get_logger().warning(
                'REPLAN IGNORED: mission already complete'
            )

            return

        if len(msg.poses) == 0:

            self.get_logger().warning(
                'Planner path ignored: no valid waypoints'
            )

            return

        # ----------------------------------------------------------
        # Convert path to PX4 NED coordinates
        # ----------------------------------------------------------

        new_waypoints = []

        for pose_stamped in msg.poses:

            x = float(
                pose_stamped.pose.position.x
            )

            y = float(
                pose_stamped.pose.position.y
            )

            altitude_up = float(
                pose_stamped.pose.position.z
            )

            z = -abs(
                altitude_up
            )

            new_waypoints.append(
                (
                    x,
                    y,
                    z,
                )
            )

        if len(new_waypoints) == 0:

            self.get_logger().warning(
                'Planner path ignored: no usable waypoints'
            )

            return

        # ----------------------------------------------------------
        # Generate path signature
        # ----------------------------------------------------------

        new_signature = tuple(
            (
                round(
                    wp[0],
                    3
                ),
                round(
                    wp[1],
                    3
                ),
                round(
                    wp[2],
                    3
                ),
            )
            for wp in new_waypoints
        )

        # ----------------------------------------------------------
        # Ignore identical path
        # ----------------------------------------------------------

        if (
            self.path_signature is not None
            and
            new_signature == self.path_signature
        ):

            return

        # ----------------------------------------------------------
        # Initial path
        # ----------------------------------------------------------

        if self.path_signature is None:

            self.waypoints = new_waypoints

            self.path_signature = new_signature

            self.current_wp = 0

            self.get_logger().info(
                f'Initial planner path received: '
                f'{len(self.waypoints)} waypoints'
            )

            for i, wp in enumerate(
                self.waypoints,
                start=1,
            ):

                self.get_logger().info(
                    f'WP{i}: '
                    f'x={wp[0]:.2f}, '
                    f'y={wp[1]:.2f}, '
                    f'z={wp[2]:.2f}'
                )

            return

        # ----------------------------------------------------------
        # Updated path while still landed
        # ----------------------------------------------------------

        if self.landed is True:

            self.waypoints = new_waypoints

            self.path_signature = new_signature

            self.current_wp = 0

            self.get_logger().info(
                'Planner path updated before takeoff'
            )

            for i, wp in enumerate(
                self.waypoints,
                start=1,
            ):

                self.get_logger().info(
                    f'WP{i}: '
                    f'x={wp[0]:.2f}, '
                    f'y={wp[1]:.2f}, '
                    f'z={wp[2]:.2f}'
                )

            return

        # ----------------------------------------------------------
        # Mid-flight replanning
        # ----------------------------------------------------------

        self.waypoints = new_waypoints

        self.path_signature = new_signature

        self.current_wp = 0

        self.get_logger().info(
            'REPLAN ACCEPTED'
        )

        self.get_logger().info(
            f'New path contains '
            f'{len(self.waypoints)} waypoints'
        )

        for i, wp in enumerate(
            self.waypoints,
            start=1,
        ):

            self.get_logger().info(
                f'NEW WP{i}: '
                f'x={wp[0]:.2f}, '
                f'y={wp[1]:.2f}, '
                f'z={wp[2]:.2f}'
            )

    # ==============================================================
    # Publish OffboardControlMode
    # ==============================================================

    def publish_offboard_control_mode(self):

        msg = OffboardControlMode()

        msg.timestamp = self.timestamp_us()

        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.offboard_pub.publish(
            msg
        )

    # ==============================================================
    # Publish trajectory setpoint
    # ==============================================================

    def publish_trajectory_setpoint(
        self,
        x,
        y,
        z,
    ):

        msg = TrajectorySetpoint()

        msg.timestamp = self.timestamp_us()

        msg.position = [
            float(x),
            float(y),
            float(z),
        ]

        msg.velocity = [
            math.nan,
            math.nan,
            math.nan,
        ]

        msg.acceleration = [
            math.nan,
            math.nan,
            math.nan,
        ]

        msg.yaw = math.nan

        msg.yawspeed = math.nan

        self.trajectory_pub.publish(
            msg
        )

    # ==============================================================
    # Publish PX4 vehicle command
    # ==============================================================

    def publish_vehicle_command(
        self,
        command,
        param1=0.0,
        param2=0.0,
        param3=0.0,
        param4=0.0,
        param5=0.0,
        param6=0.0,
        param7=0.0,
    ):

        msg = VehicleCommand()

        msg.timestamp = self.timestamp_us()

        msg.param1 = float(
            param1
        )

        msg.param2 = float(
            param2
        )

        msg.param3 = float(
            param3
        )

        msg.param4 = float(
            param4
        )

        msg.param5 = float(
            param5
        )

        msg.param6 = float(
            param6
        )

        msg.param7 = float(
            param7
        )

        msg.command = int(
            command
        )

        msg.target_system = 1
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

        msg.confirmation = 0

        msg.from_external = True

        self.vehicle_command_pub.publish(
            msg
        )

    # ==============================================================
    # Arm
    # ==============================================================

    def arm(self):

        self.publish_vehicle_command(
            command=400,
            param1=1.0,
        )

    # ==============================================================
    # Set Offboard mode
    # ==============================================================

    def set_offboard_mode(self):

        self.publish_vehicle_command(
            command=176,
            param1=1.0,
            param2=6.0,
        )

    # ==============================================================
    # Request land
    # ==============================================================

    def request_land(self):

        if self.land_requested:
            return

        self.land_requested = True

        self.get_logger().info(
            'Mission path complete. Requesting LAND'
        )

        self.publish_vehicle_command(
            command=21
        )

    # ==============================================================
    # Waypoint distance error
    # ==============================================================

    def waypoint_error(
        self,
        waypoint,
    ):

        if (
            self.x is None
            or
            self.y is None
            or
            self.z is None
        ):

            return float(
                'inf'
            )

        dx = (
            waypoint[0]
            -
            self.x
        )

        dy = (
            waypoint[1]
            -
            self.y
        )

        dz = (
            waypoint[2]
            -
            self.z
        )

        return math.sqrt(
            dx * dx
            +
            dy * dy
            +
            dz * dz
        )

    # ==============================================================
    # Main control loop
    # ==============================================================

    def control_loop(self):

        if self.mission_complete:
            return

        # Once LAND begins, PX4 handles landing.
        if self.land_requested:
            return

        if (
            self.x is None
            or
            self.y is None
            or
            self.z is None
        ):

            return

        if len(self.waypoints) == 0:
            return

        # ----------------------------------------------------------
        # Always stream OffboardControlMode
        # ----------------------------------------------------------

        self.publish_offboard_control_mode()

        # ----------------------------------------------------------
        # Pre-arm stage
        # ----------------------------------------------------------

        if not self.arm_requested:

            first_wp = self.waypoints[0]

            self.publish_trajectory_setpoint(
                first_wp[0],
                first_wp[1],
                first_wp[2],
            )

            # Wait for land detector
            if (
                self.landed is None
                or
                self.at_rest is None
            ):

                if (
                    self.timer_count
                    %
                    10
                    ==
                    0
                ):

                    self.get_logger().warning(
                        'ARM BLOCKED: waiting for PX4 land detector'
                    )

                self.timer_count += 1

                return

            # Require landed + at rest
            if not (
                self.landed is True
                and
                self.at_rest is True
            ):

                if (
                    self.timer_count
                    %
                    10
                    ==
                    0
                ):

                    self.get_logger().warning(
                        'ARM BLOCKED: '
                        'PX4 does not confirm LANDED + AT REST'
                    )

                self.timer_count += 1

                return

            # Stream setpoints before enabling Offboard
            self.timer_count += 1

            if (
                self.timer_count
                <
                self.prearm_stream_cycles
            ):

                return

            self.get_logger().info(
                'PRE-ARM SAFETY PASSED'
            )

            self.get_logger().info(
                'Requesting OFFBOARD + ARM'
            )

            self.set_offboard_mode()

            self.arm()

            self.arm_requested = True

            return

        # ----------------------------------------------------------
        # Active mission
        # ----------------------------------------------------------

        if (
            self.current_wp
            >=
            len(self.waypoints)
        ):

            self.request_land()

            return

        waypoint = self.waypoints[
            self.current_wp
        ]

        self.publish_trajectory_setpoint(
            waypoint[0],
            waypoint[1],
            waypoint[2],
        )

        error = self.waypoint_error(
            waypoint
        )

        if (
            error
            <=
            self.waypoint_tolerance
        ):

            reached_number = (
                self.current_wp
                +
                1
            )

            total = len(
                self.waypoints
            )

            self.get_logger().info(
                f'Waypoint '
                f'{reached_number}/{total} reached '
                f'| error={error:.2f} m'
            )

            self.current_wp += 1

            if (
                self.current_wp
                >=
                len(self.waypoints)
            ):

                self.request_land()


def main(args=None):

    rclpy.init(
        args=args
    )

    node = GeoNavPlannedMission()

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
