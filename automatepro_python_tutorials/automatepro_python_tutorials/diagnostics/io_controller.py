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
from automatepro_interfaces.msg import IOControllerDiagnostic


class IOControllerDiagnosticSubscriber(Node):

    def __init__(self):
        super().__init__('io_controller_diagnostic_subscriber')
        self.subscription = self.create_subscription(
            IOControllerDiagnostic,
            '/diagnostic/io_controller_hw',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info('Received IOControllerDiagnostic message:')
        self.get_logger().info(f'Battery Voltage: {msg.vbatt_voltage_monitor:.3f} V')
        self.get_logger().info(f'12V IO SMPS Current: {msg.v12_io_smps_current_monitor:.3f} A')
        self.get_logger().info(f'12V IO SMPS Voltage: {msg.v12_io_smps_voltage_monitor:.3f} V')
        self.get_logger().info(f'Main SBC SMPS Current: {msg.main_sbc_smps_current_monitor:.3f} A')
        self.get_logger().info(f'Main SBC SMPS Voltage: {msg.main_sbc_smps_voltage_monitor:.3f} V')
        self.get_logger().info(f'5V IO SMPS Voltage: {msg.v5_io_smps_voltage_monitor:.3f} V')
        self.get_logger().info(f'5V IO SMPS Current: {msg.v5_io_smps_current_monitor:.3f} A')
        self.get_logger().info(f'Total Power: {msg.amp_total_power_monitor:.3f} W')
        self.get_logger().info(f'12V IO Power Good: {msg.v12_io_power_good}')
        self.get_logger().info(f'5V IO Power Good: {msg.v5_io_power_good}')
        self.get_logger().info(f'3.3V IO Power Good: {msg.v3_3_io_power_good}')
        self.get_logger().info(f'Main SBC Power Good: {msg.main_sbc_power_good}')
        self.get_logger().info(f'5V SBC Power Good: {msg.v5_sbc_power_good}')
        self.get_logger().info(f'3.3V SBC Power Good: {msg.v3_3_sbc_power_good}')
        self.get_logger().info(f'1.8V SBC Power Good: {msg.v1_8_sbc_power_good}')
        self.get_logger().info(f'Motor Drive Fault 1: {msg.digital_drive_fault_1}')
        self.get_logger().info(f'Motor Drive Fault 2: {msg.digital_drive_fault_2}')
        self.get_logger().info(f'Motor Drive Fault 3: {msg.digital_drive_fault_3}')
        self.get_logger().info(f'Motor Drive Fault 4: {msg.digital_drive_fault_4}')
        self.get_logger().info(f'Board Temperature: {msg.board_temp} °C')


def main(args=None):
    rclpy.init(args=args)
    node = IOControllerDiagnosticSubscriber()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
