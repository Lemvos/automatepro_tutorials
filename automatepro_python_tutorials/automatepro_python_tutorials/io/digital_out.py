import rclpy
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
        msg.d_out_pin_id = DigitalOut.DIGITAL_OUT_H_01 # Digital Out Pin 01
        msg.duty_cycle_percent = self.duty_cycle_sequence[self.sequence_index] # Duty Cycle:  0%, 50%, 100%, 50%
                                                                               # 0% - OFF, 100% - ON 
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)
        self.sequence_index = (self.sequence_index + 1) % len(self.duty_cycle_sequence)

def main(args=None):
    rclpy.init(args=args)
    node = DigitalOutPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
