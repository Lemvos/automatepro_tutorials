import rclpy
from rclpy.node import Node
from automatepro_interfaces.msg import IOControllerDiagnostic

class IOControllerDiagnosticSubscriber(Node):

    def __init__(self):
        super().__init__('io_controller_diagnostic_subscriber')
        self.subscription = self.create_subscription(
            IOControllerDiagnostic,
            '/diagnostics/io_controller',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        # Convert mA to A
        self.get_logger().info(f'Battery Current: {msg.vbatt_board_current_monitor / 1000} A')
        self.get_logger().info(f'12V Rail Current: {msg.v12_current_monitor / 1000} A')
        self.get_logger().info(f'5V Rail Current: {msg.v5_current_monitor / 1000} A')
        # Convert mV to V
        self.get_logger().info(f'Battery Voltage: {msg.vbatt_voltage_monitor / 1000} V') 
        self.get_logger().info(f'12V Rail Voltage: {msg.v12_voltage_monitor / 1000} V')
        self.get_logger().info(f'5V Rail Voltage: {msg.v5_voltage_monitor / 1000} V')
        self.get_logger().info(f'12V Power Good: {msg.v12_power_good}')
        self.get_logger().info(f'5V Power Good: {msg.v5_power_good}')
        self.get_logger().info(f'3.3V Power Good: {msg.v3_3_power_good}')
        self.get_logger().info(f'Motor Drive Fault 1: {msg.motor_drive_fault_1}')
        self.get_logger().info(f'Motor Drive Fault 2: {msg.motor_drive_fault_2}')
        self.get_logger().info(f'Motor Drive Fault 3: {msg.motor_drive_fault_3}')
        self.get_logger().info(f'Motor Drive Fault 4: {msg.motor_drive_fault_4}')
        self.get_logger().info(f'Board Temperature: {msg.board_temp} °C')

def main(args=None):
    rclpy.init(args=args)
    node = IOControllerDiagnosticSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()