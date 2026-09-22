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

import signal

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from automatepro_interfaces.msg import DigitalDriveOut


class DigitalDriveOutPublisher(Node):

    def __init__(self):
        super().__init__('digital_drive_out_publisher')
        self.publisher = self.create_publisher(DigitalDriveOut, '/io/digital_drive_out', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1s
        self.duty_cycle = 0

    def timer_callback(self):
        msg = DigitalDriveOut()
        msg.d_drive_pin_id = DigitalDriveOut.HALF_BRIDGE_DRIVE_01
        msg.direction = DigitalDriveOut.FORWARD
        msg.duty_cycle_percent = self.duty_cycle
        self.publisher.publish(msg)
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
        self.publisher.publish(msg)
        # With no matched subscriber, wait_for_all_acked() succeeds without delivering anything.
        if self.publisher.get_subscription_count() == 0:
            self.get_logger().warn(
                'HALF_BRIDGE_DRIVE_01 off command not delivered: no subscriber on %s'
                % self.publisher.topic_name)
            return
        # Fast DDS acknowledges on the writer heartbeat, sent every 3 s by default.
        if self.publisher.wait_for_all_acked(Duration(seconds=4)):
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
        # A second Ctrl+C must not cut the off command short; the wait is bounded.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        node.switch_off()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
