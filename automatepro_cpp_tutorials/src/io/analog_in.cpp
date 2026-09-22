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
#include <automatepro_interfaces/msg/analog_in.hpp>

class AnalogInSubscriber : public rclcpp::Node
{
public:
  AnalogInSubscriber()
  : Node("analog_in_subscriber")
  {
    subscription_ = this->create_subscription<automatepro_interfaces::msg::AnalogIn>(
      "/io/ain", 10,
      std::bind(&AnalogInSubscriber::listener_callback, this, std::placeholders::_1));
  }

private:
  static constexpr int kLogPeriodMs = 1000;

  void listener_callback(const automatepro_interfaces::msg::AnalogIn::SharedPtr msg)
  {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), kLogPeriodMs,
      "Received AnalogIn message:\n"
      "AIN_01: %d\nAIN_02: %d\nAIN_03: %d\n"
      "AIN_04: %d\nAIN_05: %d\nAIN_06: %d\n"
      "AIN_07: %d\nAIN_08: %d\nAIN_09: %d\n"
      "AIN_10: %d\nAIN_11: %d\nAIN_12: %d\n"
      "AIN_13: %d\nAIN_14: %d",
      msg->ain_01, msg->ain_02, msg->ain_03,
      msg->ain_04, msg->ain_05, msg->ain_06,
      msg->ain_07, msg->ain_08, msg->ain_09,
      msg->ain_10, msg->ain_11, msg->ain_12,
      msg->ain_13, msg->ain_14);
  }

  rclcpp::Subscription<automatepro_interfaces::msg::AnalogIn>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AnalogInSubscriber>());
  rclcpp::shutdown();
  return 0;
}
