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

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GNSSPositionSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}