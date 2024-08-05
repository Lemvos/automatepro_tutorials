#include <rclcpp/rclcpp.hpp>
#include <automatepro_interfaces/msg/io_controller_diagnostic.hpp>

class IOControllerDiagnosticSubscriber : public rclcpp::Node
{
public:
    IOControllerDiagnosticSubscriber()
        : Node("io_controller_diagnostic_subscriber")
    {
        subscription_ = this->create_subscription<automatepro_interfaces::msg::IOControllerDiagnostic>(
            "/diagnostics/io_controller", 10,
            std::bind(&IOControllerDiagnosticSubscriber::listener_callback, this, std::placeholders::_1));
    }

private:
    void listener_callback(const automatepro_interfaces::msg::IOControllerDiagnostic::SharedPtr msg) const
    {   
        // Convert mA to A
        RCLCPP_INFO(this->get_logger(), "Battery Current: %.3f A", msg->vbatt_board_current_monitor / 1000.0); 
        RCLCPP_INFO(this->get_logger(), "12V Rail Current: %.3f A", msg->v12_current_monitor / 1000.0); 
        RCLCPP_INFO(this->get_logger(), "5V Rail Current: %.3f A", msg->v5_current_monitor / 1000.0);
        // Convert mV to V
        RCLCPP_INFO(this->get_logger(), "Battery Voltage: %.3f V", msg->vbatt_voltage_monitor / 1000.0); 
        RCLCPP_INFO(this->get_logger(), "12V Rail Voltage: %.3f V", msg->v12_voltage_monitor / 1000.0);
        RCLCPP_INFO(this->get_logger(), "5V Rail Voltage: %.3f V", msg->v5_voltage_monitor / 1000.0);
        RCLCPP_INFO(this->get_logger(), "12V Power Good: %d", msg->v12_power_good);
        RCLCPP_INFO(this->get_logger(), "5V Power Good: %d", msg->v5_power_good);
        RCLCPP_INFO(this->get_logger(), "3.3V Power Good: %d", msg->v3_3_power_good);
        RCLCPP_INFO(this->get_logger(), "Motor Drive Fault 1: %d", msg->motor_drive_fault_1);
        RCLCPP_INFO(this->get_logger(), "Motor Drive Fault 2: %d", msg->motor_drive_fault_2);
        RCLCPP_INFO(this->get_logger(), "Motor Drive Fault 3: %d", msg->motor_drive_fault_3);
        RCLCPP_INFO(this->get_logger(), "Motor Drive Fault 4: %d", msg->motor_drive_fault_4);
        RCLCPP_INFO(this->get_logger(), "Board Temperature: %.2d °C", msg->board_temp);
    }

    rclcpp::Subscription<automatepro_interfaces::msg::IOControllerDiagnostic>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IOControllerDiagnosticSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}