#include <rclcpp/rclcpp.hpp>

class GNSSHeadingSubscriber : public rclcpp::Node
{
public:
    GNSSHeadingSubscriber()
        : Node("gnss_heading_subscriber")
    {
        RCLCPP_INFO(this->get_logger(), "GNSS Heading Subscriber Node Not Implemented Yet");
    }

private:
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GNSSHeadingSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

