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
    void listener_callback(const automatepro_interfaces::msg::AnalogIn::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), 
            "Received AnalogIn message:\n"
            "AIN_01: %u\nAIN_02: %u\nAIN_03: %u\n"
            "AIN_04: %u\nAIN_05: %u\nAIN_06: %u\n"
            "AIN_07: %u\nAIN_08: %u\nAIN_09: %u\n"
            "AIN_10: %u\nAIN_11: %u\nAIN_12: %u\n"
            "AIN_13: %u\nAIN_14: %u",
            msg->ain_01, msg->ain_02, msg->ain_03,
            msg->ain_04, msg->ain_05, msg->ain_06,
            msg->ain_07, msg->ain_08, msg->ain_09,
            msg->ain_10, msg->ain_11, msg->ain_12,
            msg->ain_13, msg->ain_14);
    }

    rclcpp::Subscription<automatepro_interfaces::msg::AnalogIn>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AnalogInSubscriber>());
    rclcpp::shutdown();
    return 0;
}