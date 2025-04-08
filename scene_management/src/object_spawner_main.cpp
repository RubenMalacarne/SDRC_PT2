#include "scene_management/object_spawner.hpp"
#include <rclcpp/rclcpp.hpp>

using namespace scene_management;

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectSpawner>();
    node->initPlanningSceneMonitor();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}