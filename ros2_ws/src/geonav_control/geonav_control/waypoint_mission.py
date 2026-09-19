import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleOdometry
)


class WaypointMission(Node):

    def __init__(self):
        super().__init__('geonav_waypoint_mission')

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos
        )

        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos
        )

        self.command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            qos
        )

        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            qos
        )

        self.position = None

        # Captured automatically from live odometry
        self.origin = None

        # Relative mission coordinates:
        # [North offset, East offset, Up altitude]
        self.relative_waypoints = [
            [0.0, 0.0, 2.0],
            [3.0, 0.0, 2.0],
            [3.0, 3.0, 2.0],
            [0.0, 0.0, 2.0]
        ]

        self.waypoints = []

        self.current_waypoint = 0
        self.counter = 0
        self.waypoint_tolerance = 0.35
        self.mission_started = False
        self.land_requested = False

        self.timer = self.create_timer(
            0.1,
            self.timer_callback
        )

        self.get_logger().info(
            'GeoNav relative waypoint mission started'
        )

    def odom_callback(self, msg):

        self.position = [
            float(msg.position[0]),
            float(msg.position[1]),
            float(msg.position[2])
        ]

        # Capture mission origin only once
        if self.origin is None:

            self.origin = self.position.copy()

            self.get_logger().info(
                f'Mission origin captured: '
                f'x={self.origin[0]:.2f}, '
                f'y={self.origin[1]:.2f}, '
                f'z={self.origin[2]:.2f}'
            )

            self.generate_absolute_waypoints()

    def generate_absolute_waypoints(self):

        ox, oy, oz = self.origin

        self.waypoints = []

        for north, east, altitude_up in self.relative_waypoints:

            # PX4 local coordinates are NED:
            # x = North
            # y = East
            # z = Down
            #
            # altitude_up = +2 m
            # therefore PX4 z target = origin_z - 2 m

            waypoint = [
                ox + north,
                oy + east,
                oz - altitude_up
            ]

            self.waypoints.append(waypoint)

        self.get_logger().info(
            f'Generated mission waypoints: {self.waypoints}'
        )

    def publish_vehicle_command(
        self,
        command,
        param1=0.0,
        param2=0.0
    ):

        msg = VehicleCommand()

        msg.timestamp = (
            self.get_clock().now().nanoseconds // 1000
        )

        msg.param1 = param1
        msg.param2 = param2

        msg.command = command

        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.command_pub.publish(msg)

    def distance_to_waypoint(self):

        if self.position is None:
            return None

        if len(self.waypoints) == 0:
            return None

        target = self.waypoints[
            self.current_waypoint
        ]

        dx = target[0] - self.position[0]
        dy = target[1] - self.position[1]
        dz = target[2] - self.position[2]

        return (
            dx**2 +
            dy**2 +
            dz**2
        ) ** 0.5

    def timer_callback(self):

        if self.origin is None:
            return

        now = (
            self.get_clock().now().nanoseconds // 1000
        )

        # OFFBOARD heartbeat
        offboard = OffboardControlMode()

        offboard.timestamp = now
        offboard.position = True
        offboard.velocity = False
        offboard.acceleration = False
        offboard.attitude = False
        offboard.body_rate = False
        offboard.thrust_and_torque = False
        offboard.direct_actuator = False

        self.offboard_pub.publish(offboard)

        # Current target
        target = self.waypoints[
            self.current_waypoint
        ]

        setpoint = TrajectorySetpoint()

        setpoint.timestamp = now
        setpoint.position = target

        self.trajectory_pub.publish(setpoint)

        # Give PX4 heartbeat before switching to OFFBOARD
        if self.counter == 20:

            self.get_logger().info(
                'Requesting OFFBOARD + ARM'
            )

            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                1.0,
                6.0
            )

            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0
            )

            self.mission_started = True

        # Check actual position against target
        if (
            self.mission_started
            and self.counter > 30
            and not self.land_requested
        ):

            distance = self.distance_to_waypoint()

            if (
                distance is not None
                and distance < self.waypoint_tolerance
            ):

                self.get_logger().info(
                    f'Waypoint '
                    f'{self.current_waypoint + 1} reached '
                    f'| error={distance:.2f} m'
                )

                if (
                    self.current_waypoint
                    < len(self.waypoints) - 1
                ):

                    self.current_waypoint += 1

                    target = self.waypoints[
                        self.current_waypoint
                    ]

                    self.get_logger().info(
                        f'Next target: '
                        f'x={target[0]:.2f}, '
                        f'y={target[1]:.2f}, '
                        f'z={target[2]:.2f}'
                    )

                else:

                    self.get_logger().info(
                        'Mission complete. Requesting LAND'
                    )

                    self.publish_vehicle_command(
                        VehicleCommand.VEHICLE_CMD_NAV_LAND
                    )

                    self.land_requested = True

        self.counter += 1


def main(args=None):

    rclpy.init(args=args)

    node = WaypointMission()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
