#include <rclcpp/rclcpp.hpp>
#include <action_servers/pick_action_server.hpp>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<action_servers::PickActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}