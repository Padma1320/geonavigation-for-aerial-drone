import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand
)


class OffboardControl(Node):

    def __init__(self):
        super().__init__('geonav_offboard_control')

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

        self.counter = 0

        # 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('GeoNav takeoff-hold-land node started')

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):

        msg = VehicleCommand()

        msg.timestamp = self.get_clock().now().nanoseconds // 1000

        msg.param1 = param1
        msg.param2 = param2

        msg.command = command

        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.command_pub.publish(msg)

    def timer_callback(self):

        now = self.get_clock().now().nanoseconds // 1000

        # Keep sending OFFBOARD heartbeat
        offboard_msg = OffboardControlMode()

        offboard_msg.timestamp = now
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.thrust_and_torque = False
        offboard_msg.direct_actuator = False

        self.offboard_pub.publish(offboard_msg)

        # Hold 2 m above takeoff point
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = now

        # NED frame
        setpoint.position = [0.0, 0.0, -2.0]

        self.trajectory_pub.publish(setpoint)

        # After 2 seconds
        if self.counter == 20:

            self.get_logger().info('Requesting OFFBOARD mode')

            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                1.0,
                6.0
            )

            self.get_logger().info('Requesting ARM')

            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0
            )

        # After 12 seconds total
        if self.counter == 120:

            self.get_logger().info('Requesting LAND')

            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_NAV_LAND
            )

        self.counter += 1


def main(args=None):

    rclpy.init(args=args)

    node = OffboardControl()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
