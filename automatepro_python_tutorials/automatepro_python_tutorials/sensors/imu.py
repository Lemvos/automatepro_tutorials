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
from sensor_msgs.msg import Imu, MagneticField

LOG_PERIOD_S = 1.0


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
        self.get_logger().info(
            'Received IMU message: orientation=%s, angular_velocity=%s, '
            'linear_acceleration=%s' % (
                msg.orientation, msg.angular_velocity, msg.linear_acceleration),
            throttle_duration_sec=LOG_PERIOD_S)

    def magnetic_field_callback(self, msg):
        self.get_logger().info(
            'Received MagneticField message: magnetic_field=%s' % msg.magnetic_field,
            throttle_duration_sec=LOG_PERIOD_S)


def main(args=None):
    rclpy.init(args=args)
    node = ImuSubscriber()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
