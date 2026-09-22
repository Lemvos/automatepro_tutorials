#include <cmath>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class GNSSHeadingSubscriber : public rclcpp::Node
{
public:
  GNSSHeadingSubscriber()
  : Node("gnss_heading_subscriber")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/sensor/gnss/heading/true_heading",
      10,
      std::bind(&GNSSHeadingSubscriber::listener_callback, this, std::placeholders::_1));
  }

private:
  // The GNSS driver sets this yaw variance while the heading is invalid.
  static constexpr double kInvalidHeadingCovariance = 1000.0;
  static constexpr double kRadToDeg = 180.0 / M_PI;

  void listener_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const
  {
    // orientation_covariance[8] is the yaw variance in rad^2.
    const double yaw_variance = msg->orientation_covariance[8];
    if (yaw_variance >= kInvalidHeadingCovariance) {
      RCLCPP_INFO(this->get_logger(), "Heading not valid");
      return;
    }
    const auto & q = msg->orientation;
    // Yaw in the ROS ENU convention: counterclockwise from east.
    const double yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    // Compass heading: clockwise from north, 0 to 360 degrees.
    double heading_deg = std::fmod(90.0 - yaw * kRadToDeg, 360.0);
    if (heading_deg < 0.0) {
      heading_deg += 360.0;
    }
    RCLCPP_INFO(
      this->get_logger(), "Heading: %.2f deg, accuracy: %.2f deg",
      heading_deg, std::sqrt(yaw_variance) * kRadToDeg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GNSSHeadingSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
