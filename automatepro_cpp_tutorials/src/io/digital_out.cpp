#include <chrono>
#include <memory>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <automatepro_interfaces/msg/digital_out.hpp>

class DigitalOutPublisher : public rclcpp::Node
{
public:
  DigitalOutPublisher()
  : Node("digital_out_publisher"),
    duty_cycle_sequence_{0, 50, 100, 50},
    sequence_index_(0)
  {
    publisher_ = this->create_publisher<automatepro_interfaces::msg::DigitalOut>(
      "/io/digital_out",
      10);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&DigitalOutPublisher::timer_callback, this));
  }

  void switch_off()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopped_ = true;
    timer_->cancel();
    auto msg = automatepro_interfaces::msg::DigitalOut();
    msg.d_out_pin_id = automatepro_interfaces::msg::DigitalOut::DIGITAL_OUT_H_01;
    msg.duty_cycle_percent = 0;
    publisher_->publish(msg);
    // Fast DDS acknowledges on the writer heartbeat, sent every 3 s by default.
    if (publisher_->wait_for_all_acked(std::chrono::seconds(4))) {
      RCLCPP_INFO(this->get_logger(), "Switched DIGITAL_OUT_H_01 off");
    } else {
      RCLCPP_WARN(this->get_logger(), "DIGITAL_OUT_H_01 off command not acknowledged");
    }
  }

private:
  void timer_callback()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopped_) {
      return;
    }
    auto msg = automatepro_interfaces::msg::DigitalOut();
    msg.d_out_pin_id = automatepro_interfaces::msg::DigitalOut::DIGITAL_OUT_H_01;     // Digital Out Pin 01
    msg.duty_cycle_percent = duty_cycle_sequence_[sequence_index_];     // Duty Cycle: 0%, 50%, 100%, 50%
                                                                        // 0% - OFF, 100% - ON
    publisher_->publish(msg);
    RCLCPP_INFO(
      this->get_logger(), "Publishing: d_out_pin_id=%d, duty_cycle_percent=%d", msg.d_out_pin_id,
      msg.duty_cycle_percent);

    // Update the sequence index
    sequence_index_ = (sequence_index_ + 1) % duty_cycle_sequence_.size();
  }

  rclcpp::Publisher<automatepro_interfaces::msg::DigitalOut>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<int> duty_cycle_sequence_;
  size_t sequence_index_;
  std::mutex mutex_;
  bool stopped_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DigitalOutPublisher>();
  // The IO controller keeps the last command, so switch the output off before shutdown.
  std::weak_ptr<DigitalOutPublisher> weak_node = node;
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
