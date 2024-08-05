#include <rclcpp/rclcpp.hpp>
#include <automatepro_interfaces/msg/digital_drive_out.hpp>

class DigitalDriveOutPublisher : public rclcpp::Node
{
public:
    DigitalDriveOutPublisher()
        : Node("digital_drive_out_publisher"), duty_cycle_(0)
    {
        publisher_ = this->create_publisher<automatepro_interfaces::msg::DigitalDriveOut>("/io/digital_drive_out", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&DigitalDriveOutPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto msg = automatepro_interfaces::msg::DigitalDriveOut();
        msg.d_drive_pin_id = automatepro_interfaces::msg::DigitalDriveOut::HALF_BRIDGE_DRIVE_01;
        msg.direction = automatepro_interfaces::msg::DigitalDriveOut::FORWARD;
        msg.percent_duty_cycle = duty_cycle_;
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publishing DigitalDriveOut: d_out_pin_id=%d, direction=%d, duty_cycle_percent=%d",
                    msg.d_drive_pin_id, msg.direction, msg.percent_duty_cycle);
        duty_cycle_ = (duty_cycle_ == 0) ? 100 : 0; // Toggle duty cycle between 0(ON) and 100(OFF)
    }

    rclcpp::Publisher<automatepro_interfaces::msg::DigitalDriveOut>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int duty_cycle_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DigitalDriveOutPublisher>());
    rclcpp::shutdown();
    return 0;
}
