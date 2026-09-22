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

#include <chrono>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <automatepro_interfaces/msg/warning_systems.hpp>

class WarningSystemsPublisher : public rclcpp::Node
{
public:
  WarningSystemsPublisher()
  : Node("warning_systems_publisher"), state_(false)
  {
    publisher_ = this->create_publisher<automatepro_interfaces::msg::WarningSystems>(
      "/io/warning_system_out", 10);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&WarningSystemsPublisher::timer_callback, this));
  }

  void switch_off()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopped_ = true;
    timer_->cancel();
    auto msg = automatepro_interfaces::msg::WarningSystems();
    msg.warning_system_id = automatepro_interfaces::msg::WarningSystems::WARNING_BUZZER;
    msg.state = automatepro_interfaces::msg::WarningSystems::OFF;
    publisher_->publish(msg);
    // With no matched subscriber, wait_for_all_acked() succeeds without delivering anything.
    if (publisher_->get_subscription_count() == 0) {
      RCLCPP_WARN(
        this->get_logger(), "WARNING_BUZZER off command not delivered: no subscriber on %s",
        publisher_->get_topic_name());
      return;
    }
    // Fast DDS acknowledges on the writer heartbeat, sent every 3 s by default.
    if (publisher_->wait_for_all_acked(std::chrono::seconds(4))) {
      RCLCPP_INFO(this->get_logger(), "Switched WARNING_BUZZER off");
    } else {
      RCLCPP_WARN(this->get_logger(), "WARNING_BUZZER off command not acknowledged");
    }
  }

private:
  void timer_callback()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopped_) {
      return;
    }
    auto msg = automatepro_interfaces::msg::WarningSystems();
    // Change to WARNING_LIGHT_01 or WARNING_LIGHT_02 as needed
    msg.warning_system_id = automatepro_interfaces::msg::WarningSystems::WARNING_BUZZER;
    msg.state = state_;
    publisher_->publish(msg);
    RCLCPP_INFO(
      this->get_logger(), "Publishing WarningSystems: warning_system_id=%d, state=%d",
      msg.warning_system_id, msg.state);
    state_ = !state_;      // Toggle state
  }

  rclcpp::Publisher<automatepro_interfaces::msg::WarningSystems>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool state_;
  std::mutex mutex_;
  bool stopped_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WarningSystemsPublisher>();
  // The IO controller keeps the last command, so switch the buzzer off before shutdown.
  std::weak_ptr<WarningSystemsPublisher> weak_node = node;
  rclcpp::contexts::get_global_default_context()->add_pre_shutdown_callback(
    [weak_node]() {
      if (auto locked_node = weak_node.lock()) {
        locked_node->switch_off();
      }
    });
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
