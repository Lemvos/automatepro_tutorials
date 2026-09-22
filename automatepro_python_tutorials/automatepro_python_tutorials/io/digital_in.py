# Copyright 2024 Lemvos Robotics GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
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

    def listener_callback(self, msg):
        self.get_logger().info('Received DigitalIn message: %s' % str([
            msg.din_01, msg.din_02, msg.din_03, msg.din_04, msg.din_05,
            msg.din_06, msg.din_07, msg.din_08, msg.din_09, msg.din_10]))

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
    # SIGINT only: the rclpy handler wakes spin() while /io/din is idle and raises
    # KeyboardInterrupt out of the service wait. SIGTERM keeps its default action,
    # because a context shut down during wait_for_service() raises RCLError.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.SIGINT)
    node = DigitalInSubscriber()
    try:
        node.request_state()  # Request the current state of the digital inputs
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
