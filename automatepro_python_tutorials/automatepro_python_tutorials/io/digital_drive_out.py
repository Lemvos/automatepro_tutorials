import rclpy
from rclpy.node import Node
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
            'Publishing DigitalDriveOut: d_out_pin_id=%d, direction=%d, duty_cycle_percent=%d' %
            (msg.d_drive_pin_id, msg.direction, msg.duty_cycle_percent))
        self.duty_cycle = 100 if self.duty_cycle == 0 else 0 # Toggle duty cycle between 0(ON) and 100(OFF)

def main(args=None):
    rclpy.init(args=args)
    node = DigitalDriveOutPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()