import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

class ImuSubscriber(Node):

    def __init__(self):
        super().__init__('imu_subscriber')
        self.imu_subscription = self.create_subscription(
            Imu,
            '/sensor/imu/data',
            self.imu_callback,
            10)
        self.magnetic_field_subscription = self.create_subscription(
            MagneticField,
            '/sensor/imu/magnetic_field',
            self.magnetic_field_callback,
            10)

    def imu_callback(self, msg):
        self.get_logger().info('Received IMU message: orientation=%s, angular_velocity=%s, linear_acceleration=%s' % (
          msg.orientation, msg.angular_velocity, msg.linear_acceleration))

    def magnetic_field_callback(self, msg):
        self.get_logger().info('Received MagneticField message: magnetic_field=%s' % msg.magnetic_field)

def main(args=None):
    rclpy.init(args=args)
    node = ImuSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()