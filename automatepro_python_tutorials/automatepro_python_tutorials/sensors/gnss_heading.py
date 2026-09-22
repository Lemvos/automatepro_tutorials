import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu

# The GNSS driver sets this yaw variance while the heading is invalid.
INVALID_HEADING_COVARIANCE = 1000.0


class GNSSHeadingSubscriber(Node):

    def __init__(self):
        super().__init__('gnss_heading_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            '/sensor/gnss/heading/true_heading',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        # orientation_covariance[8] is the yaw variance in rad^2.
        yaw_variance = msg.orientation_covariance[8]
        if yaw_variance >= INVALID_HEADING_COVARIANCE:
            self.get_logger().info('Heading not valid')
            return
        q = msg.orientation
        # Yaw in the ROS ENU convention: counterclockwise from east.
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        # Compass heading: clockwise from north, 0 to 360 degrees.
        heading_deg = (90.0 - math.degrees(yaw)) % 360.0
        self.get_logger().info('Heading: %.2f deg, accuracy: %.2f deg' % (
            heading_deg, math.degrees(math.sqrt(yaw_variance))))


def main(args=None):
    rclpy.init(args=args)
    node = GNSSHeadingSubscriber()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
