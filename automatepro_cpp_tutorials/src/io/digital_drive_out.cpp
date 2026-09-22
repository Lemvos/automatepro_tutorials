#include <chrono>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <automatepro_interfaces/msg/digital_drive_out.hpp>

class DigitalDriveOutPublisher : public rclcpp::Node
{
public:
  DigitalDriveOutPublisher()
  : Node("digital_drive_out_publisher"), duty_cycle_(0)
  {
    publisher_ = this->create_publisher<automatepro_interfaces::msg::DigitalDriveOut>(
      "/io/digital_drive_out", 10);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&DigitalDriveOutPublisher::timer_callback, this));
  }

  void switch_off()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopped_ = true;
    timer_->cancel();
    auto msg = automatepro_interfaces::msg::DigitalDriveOut();
    msg.d_drive_pin_id = automatepro_interfaces::msg::DigitalDriveOut::HALF_BRIDGE_DRIVE_01;
    msg.direction = automatepro_interfaces::msg::DigitalDriveOut::FORWARD;
    msg.duty_cycle_percent = 0;
    publisher_->publish(msg);
    // Fast DDS acknowledges on the writer heartbeat, sent every 3 s by default.
    if (publisher_->wait_for_all_acked(std::chrono::seconds(4))) {
      RCLCPP_INFO(this->get_logger(), "Switched HALF_BRIDGE_DRIVE_01 off");
    } else {
      RCLCPP_WARN(this->get_logger(), "HALF_BRIDGE_DRIVE_01 off command not acknowledged");
    }
  }

private:
  void timer_callback()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopped_) {
      return;
    }
    auto msg = automatepro_interfaces::msg::DigitalDriveOut();
    msg.d_drive_pin_id = automatepro_interfaces::msg::DigitalDriveOut::HALF_BRIDGE_DRIVE_01;
    msg.direction = automatepro_interfaces::msg::DigitalDriveOut::FORWARD;
    msg.duty_cycle_percent = duty_cycle_;
    publisher_->publish(msg);
    RCLCPP_INFO(
      this->get_logger(), "Publishing DigitalDriveOut: d_out_pin_id=%d, direction=%d, duty_cycle_percent=%d",
      msg.d_drive_pin_id, msg.direction, msg.duty_cycle_percent);
    duty_cycle_ = (duty_cycle_ == 0) ? 100 : 0;     // Toggle duty cycle between 0(ON) and 100(OFF)
  }

  rclcpp::Publisher<automatepro_interfaces::msg::DigitalDriveOut>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int duty_cycle_;
  std::mutex mutex_;
  bool stopped_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DigitalDriveOutPublisher>();
  // The IO controller keeps the last command, so switch the output off before shutdown.
  std::weak_ptr<DigitalDriveOutPublisher> weak_node = node;
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
