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
        publisher_ = this->create_publisher<automatepro_interfaces::msg::DigitalOut>("/io/digital_out", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&DigitalOutPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto msg = automatepro_interfaces::msg::DigitalOut();
        msg.d_out_pin_id = automatepro_interfaces::msg::DigitalOut::DIGITAL_OUT_01; // Digital Out Pin 01
        msg.duty_cycle_percent = duty_cycle_sequence_[sequence_index_]; // Duty Cycle: 0%, 50%, 100%, 50%
                                                                        // 0% - OFF, 100% - ON
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publishing: d_out_pin_id=%d, duty_cycle_percent=%d", msg.d_out_pin_id, msg.duty_cycle_percent);

        // Update the sequence index
        sequence_index_ = (sequence_index_ + 1) % duty_cycle_sequence_.size();
    }

    rclcpp::Publisher<automatepro_interfaces::msg::DigitalOut>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<int> duty_cycle_sequence_;
    size_t sequence_index_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DigitalOutPublisher>());
    rclcpp::shutdown();
    return 0;
}
