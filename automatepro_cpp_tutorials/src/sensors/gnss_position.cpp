// Copyright 2024 Lemvos Robotics GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

class GNSSPositionSubscriber : public rclcpp::Node
{
public:
  GNSSPositionSubscriber()
  : Node("gnss_position_subscriber")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/sensor/gnss/position/fix",
      10,
      std::bind(&GNSSPositionSubscriber::listener_callback, this, std::placeholders::_1));
  }

private:
  void listener_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Latitude: %f", msg->latitude);
    RCLCPP_INFO(this->get_logger(), "Longitude: %f", msg->longitude);
    RCLCPP_INFO(this->get_logger(), "Altitude: %f", msg->altitude);
  }
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GNSSPositionSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
