import rclpy
from rclpy.node import Node
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
        self.state = not self.state # Toggle state

def main(args=None):
    rclpy.init(args=args)
    node = WarningSystemsPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()