#ifndef OBJECT_SPAWNER_HPP_
#define OBJECT_SPAWNER_HPP_

#include <interfaces/msg/object_info.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/allowed_collision_matrix.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>

namespace scene_management
{

    class ObjectSpawner : public rclcpp::Node
    {
    public:

        explicit ObjectSpawner(
            const rclcpp::NodeOptions &options = rclcpp::NodeOptions(),
            const std::string &robot_description = "robot_description",
            const std::string &psm_name = "planning_scene_monitor"
        );

        void initPlanningSceneMonitor();

    private:

        void spawnObject(const interfaces::msg::ObjectInfo object_info);

        planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

        rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr object_publisher_;
        rclcpp::Subscription<interfaces::msg::ObjectInfo>::SharedPtr object_info_subscriber_;
        
        std::string psm_name_;
        std::string robot_description_;
    };

}

#endif
