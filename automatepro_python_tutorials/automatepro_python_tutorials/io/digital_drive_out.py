import signal

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from automatepro_interfaces.msg import DigitalDriveOut


class DigitalDriveOutPublisher(Node):

    def __init__(self):
        super().__init__('digital_drive_out_publisher')
        self.publisher_ = self.create_publisher(DigitalDriveOut, '/io/digital_drive_out', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1s
        self.duty_cycle = 0

    def timer_callback(self):
        msg = DigitalDriveOut()
        msg.d_drive_pin_id = DigitalDriveOut.HALF_BRIDGE_DRIVE_01
        msg.direction = DigitalDriveOut.FORWARD
        msg.duty_cycle_percent = self.duty_cycle
        self.publisher_.publish(msg)
        self.get_logger().info(
            'Publishing DigitalDriveOut: d_drive_pin_id=%d, direction=%d, '
            'duty_cycle_percent=%d' %
            (msg.d_drive_pin_id, msg.direction, msg.duty_cycle_percent))
        # Toggle duty cycle between 0 (OFF) and 100 (ON)
        self.duty_cycle = 100 if self.duty_cycle == 0 else 0

    def switch_off(self):
        msg = DigitalDriveOut()
        msg.d_drive_pin_id = DigitalDriveOut.HALF_BRIDGE_DRIVE_01
        msg.direction = DigitalDriveOut.FORWARD
        msg.duty_cycle_percent = 0
        self.publisher_.publish(msg)
        # With no matched subscriber, wait_for_all_acked() succeeds without delivering anything.
        if self.publisher_.get_subscription_count() == 0:
            self.get_logger().warn(
                'HALF_BRIDGE_DRIVE_01 off command not delivered: no subscriber on %s'
                % self.publisher_.topic_name)
            return
        # Fast DDS acknowledges on the writer heartbeat, sent every 3 s by default.
        if self.publisher_.wait_for_all_acked(Duration(seconds=4)):
            self.get_logger().info('Switched HALF_BRIDGE_DRIVE_01 off')
        else:
            self.get_logger().warn('HALF_BRIDGE_DRIVE_01 off command not acknowledged')


def main(args=None):
    # The IO controller keeps the last command, so switch the output off before exiting.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = DigitalDriveOutPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.switch_off()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
