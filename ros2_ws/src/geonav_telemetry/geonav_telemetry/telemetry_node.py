import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleOdometry, SensorGps


class GeoNavTelemetry(Node):

    def __init__(self):
        super().__init__('geonav_telemetry')

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.latest_odom = None
        self.latest_gps = None

        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            qos
        )

        self.gps_sub = self.create_subscription(
            SensorGps,
            '/fmu/out/vehicle_gps_position',
            self.gps_callback,
            qos
        )

        self.timer = self.create_timer(1.0, self.print_state)

        self.get_logger().info('GeoNav telemetry node started')

    def odom_callback(self, msg):
        self.latest_odom = msg

    def gps_callback(self, msg):
        self.latest_gps = msg

    def print_state(self):
        if self.latest_odom is None or self.latest_gps is None:
            self.get_logger().info('Waiting for PX4 telemetry...')
            return

        x = self.latest_odom.position[0]
        y = self.latest_odom.position[1]
        z = self.latest_odom.position[2]

        vx = self.latest_odom.velocity[0]
        vy = self.latest_odom.velocity[1]
        vz = self.latest_odom.velocity[2]

        gps = self.latest_gps

        self.get_logger().info(
            '\n'
            f'POSITION  : x={x:.2f}, y={y:.2f}, z={z:.2f} m\n'
            f'VELOCITY  : vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f} m/s\n'
            f'GPS       : lat={gps.latitude_deg:.7f}, '
            f'lon={gps.longitude_deg:.7f}, alt={gps.altitude_msl_m:.2f} m\n'
            f'GPS FIX   : {gps.fix_type} | Satellites={gps.satellites_used}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = GeoNavTelemetry()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
