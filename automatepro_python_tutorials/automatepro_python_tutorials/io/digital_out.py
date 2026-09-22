import signal

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
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

    def switch_off(self):
        msg = DigitalOut()
        msg.d_out_pin_id = DigitalOut.DIGITAL_OUT_H_01
        msg.duty_cycle_percent = 0
        self.publisher_.publish(msg)
        # Fast DDS acknowledges on the writer heartbeat, sent every 3 s by default.
        if self.publisher_.wait_for_all_acked(Duration(seconds=4)):
            self.get_logger().info('Switched DIGITAL_OUT_H_01 off')
        else:
            self.get_logger().warn('DIGITAL_OUT_H_01 off command not acknowledged')


def main(args=None):
    # The IO controller keeps the last command, so switch the output off before exiting.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = DigitalOutPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.switch_off()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
