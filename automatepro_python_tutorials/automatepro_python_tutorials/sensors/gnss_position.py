import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class GNSSPositionSubscriber(Node):

    def __init__(self):
        super().__init__('gnss_position_subscriber')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/sensor/gnss/position/fix',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info('Latitude: %f' % msg.latitude)
        self.get_logger().info('Longitude: %f' % msg.longitude)
        self.get_logger().info('Altitude: %f' % msg.altitude)


def main(args=None):
    rclpy.init(args=args)
    node = GNSSPositionSubscriber()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
