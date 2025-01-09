#include <rclcpp/rclcpp.hpp>
#include <automatepro_interfaces/msg/io_controller_diagnostic.hpp>

class IOControllerDiagnosticSubscriber : public rclcpp::Node
{
public:
    IOControllerDiagnosticSubscriber()
        : Node("io_controller_diagnostic_subscriber")
    {
        subscription_ = this->create_subscription<automatepro_interfaces::msg::IOControllerDiagnostic>(
            "/diagnostic/io_controller_hw", 10,
            std::bind(&IOControllerDiagnosticSubscriber::listener_callback, this, std::placeholders::_1));
    }

private:
    void listener_callback(const automatepro_interfaces::msg::IOControllerDiagnostic::SharedPtr msg) const
    {   
        RCLCPP_INFO(this->get_logger(), "Received IOControllerDiagnostic message");
        RCLCPP_INFO(this->get_logger(), "Battery Voltage: %.3f V", msg->vbatt_voltage_monitor); 
        RCLCPP_INFO(this->get_logger(), "12V IO SMPS Current: %.3f A", msg->v12_io_smps_current_monitor); 
        RCLCPP_INFO(this->get_logger(), "12V IO SMPS Voltage: %.3f V", msg->v12_io_smps_voltage_monitor);
        RCLCPP_INFO(this->get_logger(), "Main SBC SMPS Current: %.3f A", msg->main_sbc_smps_current_monitor); 
        RCLCPP_INFO(this->get_logger(), "Main SBC SMPS Voltage: %.3f V", msg->main_sbc_smps_voltage_monitor);
        RCLCPP_INFO(this->get_logger(), "5V IO SMPS Voltage: %.3f V", msg->v5_io_smps_voltage_monitor);
        RCLCPP_INFO(this->get_logger(), "5V IO SMPS Current: %.3f A", msg->v5_io_smps_current_monitor);
        RCLCPP_INFO(this->get_logger(), "Total Power: %.3f W", msg->amp_total_power_monitor);
        RCLCPP_INFO(this->get_logger(), "12V IO Power Good: %d", msg->v12_io_power_good);
        RCLCPP_INFO(this->get_logger(), "5V IO Power Good: %d", msg->v5_io_power_good);
        RCLCPP_INFO(this->get_logger(), "3.3V IO Power Good: %d", msg->v3_3_io_power_good);
        RCLCPP_INFO(this->get_logger(), "Main SBC Power Good: %d", msg->main_sbc_power_good);
        RCLCPP_INFO(this->get_logger(), "5V SBC Power Good: %d", msg->v5_sbc_power_good);
        RCLCPP_INFO(this->get_logger(), "3.3V SBC Power Good: %d", msg->v3_3_sbc_power_good);
        RCLCPP_INFO(this->get_logger(), "1.8V SBC Power Good: %d", msg->v1_8_sbc_power_good);
        RCLCPP_INFO(this->get_logger(), "Motor Drive Fault 1: %d", msg->digital_drive_fault_1);
        RCLCPP_INFO(this->get_logger(), "Motor Drive Fault 2: %d", msg->digital_drive_fault_2);
        RCLCPP_INFO(this->get_logger(), "Motor Drive Fault 3: %d", msg->digital_drive_fault_3);
        RCLCPP_INFO(this->get_logger(), "Motor Drive Fault 4: %d", msg->digital_drive_fault_4);
        RCLCPP_INFO(this->get_logger(), "Board Temperature: %d °C", msg->board_temp);
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