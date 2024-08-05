#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

class ImuSubscriber : public rclcpp::Node
{
public:
  ImuSubscriber()
      : Node("imu_subscriber")
  {
      imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
          "/sensor/imu/data", 10, std::bind(&ImuSubscriber::imu_callback, this, std::placeholders::_1));

      magnetic_field_subscription_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
          "/sensor/imu/magnetic_field", 10, std::bind(&ImuSubscriber::magnetic_field_callback, this, std::placeholders::_1));
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const
  {
      RCLCPP_INFO(this->get_logger(),
                  "Received IMU message: orientation=[x: %f, y: %f, z: %f, w: %f], angular_velocity=[x: %f, y: %f, z: %f], linear_acceleration=[x: %f, y: %f, z: %f]",
                  msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w,
                  msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
                  msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  }

  void magnetic_field_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg) const
  {
      RCLCPP_INFO(this->get_logger(),
                  "Received MagneticField message: magnetic_field=[x: %f, y: %f, z: %f]",
                  msg->magnetic_field.x, msg->magnetic_field.y, msg->magnetic_field.z);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr magnetic_field_subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuSubscriber>());
  rclcpp::shutdown();
  return 0;
}