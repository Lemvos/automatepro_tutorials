#include <rclcpp/rclcpp.hpp>
#include <automatepro_interfaces/msg/digital_in.hpp>
#include <automatepro_interfaces/srv/req_digital_in.hpp>

class DigitalInSubscriber : public rclcpp::Node
{
public:
    DigitalInSubscriber()
        : Node("digital_in_subscriber")
    {
        subscription_ = this->create_subscription<automatepro_interfaces::msg::DigitalIn>(
            "/io/din", 10,
            std::bind(&DigitalInSubscriber::listener_callback, this, std::placeholders::_1));

        client_ = this->create_client<automatepro_interfaces::srv::ReqDigitalIn>("/io/din/request");
        request_state(); // Request the current state of the digital inputs
    }

private:
    /*
    * Callback function for the subscription
    */
    void listener_callback(const automatepro_interfaces::msg::DigitalIn::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Received DigitalIn message: %d %d %d %d %d %d %d %d %d %d",
                    msg->din_01, msg->din_02, msg->din_03, msg->din_04, msg->din_05,
                    msg->din_06, msg->din_07, msg->din_08, msg->din_09, msg->din_10);
    }

    /*
    * Request the current state of the digital inputs
    */
    void request_state()
    {
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        auto request = std::make_shared<automatepro_interfaces::srv::ReqDigitalIn::Request>();
        using ServiceResponseFuture = rclcpp::Client<automatepro_interfaces::srv::ReqDigitalIn>::SharedFuture;
        auto response_received_callback = [this](ServiceResponseFuture future) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Service response received: success= %d", response->success);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Service call failed");
            }
        };

        auto result = client_->async_send_request(request, response_received_callback);
    }

    rclcpp::Subscription<automatepro_interfaces::msg::DigitalIn>::SharedPtr subscription_;
    rclcpp::Client<automatepro_interfaces::srv::ReqDigitalIn>::SharedPtr client_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DigitalInSubscriber>());
    rclcpp::shutdown();
    return 0;
}