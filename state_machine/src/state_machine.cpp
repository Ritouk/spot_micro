#include "rclcpp/rclcpp.hpp"
#include "spot_interfaces/srv/StateMachineInfo.srv"
#include "spot_interfaces/srv/StateMachineSet.srv"

class StateMachineNode : public rclcpp::Node // MODIFY NAME
{
public:
    StateMachineNode() : Node("state_machine") // MODIFY NAME
    {
        server_ = this->create_service<spot_interfaces::srv::StateMachineInfo>("state_machine_info");
    }

private:

    rclcpp::Service<spot_interfaces::srv::StateMachineInfo>::SharedPtr server_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StateMachineNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
