import signal

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from automatepro_interfaces.msg import WarningSystems


class WarningSystemsPublisher(Node):

    def __init__(self):
        super().__init__('warning_systems_publisher')
        self.publisher_ = self.create_publisher(WarningSystems, '/io/warning_system_out', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1s
        self.state = False

    def timer_callback(self):
        msg = WarningSystems()
        msg.warning_system_id = WarningSystems.WARNING_BUZZER
        msg.state = self.state
        self.publisher_.publish(msg)
        self.get_logger().info(
            'Publishing WarningSystems: warning_system_id=%d, state=%d' %
            (msg.warning_system_id, msg.state))
        self.state = not self.state  # Toggle state

    def switch_off(self):
        msg = WarningSystems()
        msg.warning_system_id = WarningSystems.WARNING_BUZZER
        msg.state = WarningSystems.OFF
        self.publisher_.publish(msg)
        # With no matched subscriber, wait_for_all_acked() succeeds without delivering anything.
        if self.publisher_.get_subscription_count() == 0:
            self.get_logger().warn(
                'WARNING_BUZZER off command not delivered: no subscriber on %s'
                % self.publisher_.topic_name)
            return
        # Fast DDS acknowledges on the writer heartbeat, sent every 3 s by default.
        if self.publisher_.wait_for_all_acked(Duration(seconds=4)):
            self.get_logger().info('Switched WARNING_BUZZER off')
        else:
            self.get_logger().warn('WARNING_BUZZER off command not acknowledged')


def main(args=None):
    # The IO controller keeps the last command, so switch the buzzer off before exiting.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = WarningSystemsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # A second Ctrl+C must not cut the off command short; the wait is bounded.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        node.switch_off()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
