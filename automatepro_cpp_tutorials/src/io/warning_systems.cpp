#include <rclcpp/rclcpp.hpp>
#include <automatepro_interfaces/msg/warning_systems.hpp>

class WarningSystemsPublisher : public rclcpp::Node
{
public:
    WarningSystemsPublisher()
        : Node("warning_systems_publisher"), state_(false)
    {
        publisher_ = this->create_publisher<automatepro_interfaces::msg::WarningSystems>("/io/warning_systems", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&WarningSystemsPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto msg = automatepro_interfaces::msg::WarningSystems();
        msg.warning_system_id = automatepro_interfaces::msg::WarningSystems::WARNING_BUZZER;  // Change to WARNING_LIGHT1 or WARNING_LIGHT2 as needed
        msg.state = state_;
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publishing WarningSystems: warning_system_id=%d, state=%d",
                    msg.warning_system_id, msg.state);
        state_ = !state_;  // Toggle state
    }

    rclcpp::Publisher<automatepro_interfaces::msg::WarningSystems>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool state_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WarningSystemsPublisher>());
    rclcpp::shutdown();
    return 0;
}
