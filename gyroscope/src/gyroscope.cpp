#include "rclcpp/rclcpp.hpp"
//#include "example_interfaces/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp" 

class GyroscopeNode: public rclcpp::Node
{
public:
    GyroscopeNode(): Node("gyroscope_node")
    {
        RCLCPP_INFO(this->get_logger(), "Gyroscope Node Activated");
    }

};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GyroscopeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
