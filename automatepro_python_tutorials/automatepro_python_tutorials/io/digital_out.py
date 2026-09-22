import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from automatepro_interfaces.msg import DigitalOut


class DigitalOutPublisher(Node):

    def __init__(self):
        super().__init__('digital_out_publisher')
        self.publisher_ = self.create_publisher(DigitalOut, '/io/digital_out', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1s
        self.duty_cycle_sequence = [0, 50, 100, 50]
        self.sequence_index = 0

    def timer_callback(self):
        msg = DigitalOut()
        msg.d_out_pin_id = DigitalOut.DIGITAL_OUT_H_01  # Digital Out Pin 01
        # Duty Cycle: 0%, 50%, 100%, 50%
        # 0% - OFF, 100% - ON
        msg.duty_cycle_percent = self.duty_cycle_sequence[self.sequence_index]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)
        self.sequence_index = (self.sequence_index + 1) % len(self.duty_cycle_sequence)


def main(args=None):
    rclpy.init(args=args)
    node = DigitalOutPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
