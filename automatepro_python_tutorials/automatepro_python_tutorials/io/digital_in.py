import rclpy
from rclpy.node import Node
from automatepro_interfaces.msg import DigitalIn
from automatepro_interfaces.srv import ReqDigitalIn

class DigitalInSubscriber(Node):

    def __init__(self):
        super().__init__('digital_in_subscriber')
        self.subscription = self.create_subscription(
            DigitalIn,
            '/io/din',
            self.listener_callback,
            10)
        self.client = self.create_client(ReqDigitalIn, '/io/din/request')
        self.request_state() # Request the current state of the digital inputs

    def listener_callback(self, msg):
        self.get_logger().info('Received DigitalIn message: %s' % [
            msg.din_01, msg.din_02, msg.din_03, msg.din_04, msg.din_05,
            msg.din_06, msg.din_07, msg.din_08, msg.din_09, msg.din_10])

    def request_state(self):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        request = ReqDigitalIn.Request()
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.service_callback)

    def service_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Service response received: success=%s' % response.success)
        except Exception as e:
            self.get_logger().info('Service call failed %r' % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = DigitalInSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
