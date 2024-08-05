import rclpy
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
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()