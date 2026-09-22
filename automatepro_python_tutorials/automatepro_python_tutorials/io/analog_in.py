import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from automatepro_interfaces.msg import AnalogIn

LOG_PERIOD_S = 1.0


class AnalogInSubscriber(Node):

    def __init__(self):
        super().__init__('analog_in_subscriber')
        self.subscription = self.create_subscription(
            AnalogIn,
            '/io/ain',
            self.subscriber_callback,
            10)

    def subscriber_callback(self, msg):
        self.get_logger().info(
            f'Received AnalogIn message:\n'
            f'AIN_01: {msg.ain_01}\nAIN_02: {msg.ain_02}\nAIN_03: {msg.ain_03}\n'
            f'AIN_04: {msg.ain_04}\nAIN_05: {msg.ain_05}\nAIN_06: {msg.ain_06}\n'
            f'AIN_07: {msg.ain_07}\nAIN_08: {msg.ain_08}\nAIN_09: {msg.ain_09}\n'
            f'AIN_10: {msg.ain_10}\nAIN_11: {msg.ain_11}\nAIN_12: {msg.ain_12}\n'
            f'AIN_13: {msg.ain_13}\nAIN_14: {msg.ain_14}',
            throttle_duration_sec=LOG_PERIOD_S)


def main(args=None):
    rclpy.init(args=args)
    node = AnalogInSubscriber()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
