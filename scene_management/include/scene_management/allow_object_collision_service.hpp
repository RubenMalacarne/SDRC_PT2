#ifndef ALLOW_OBJECT_COLLISION_SERVICE_HPP_
#define ALLOW_OBJECT_COLLISION_SERVICE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <interfaces/srv/allow_object_collision.hpp>

namespace scene_management
{
    class AllowObjectCollisionService : public rclcpp::Node
    {
    public: 

        explicit AllowObjectCollisionService(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    
    private:

        void allowCollisionCallback(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<interfaces::srv::AllowObjectCollision::Request> request,
            std::shared_ptr<interfaces::srv::AllowObjectCollision::Response> response
        );
    
        const std::vector<std::string> gripper_links = {
            "robotiq_85_left_knuckle_link",
            "robotiq_85_right_knuckle_link",
            "robotiq_85_left_finger_link",
            "robotiq_85_right_finger_link",
            "robotiq_85_left_inner_knuckle_link",
            "robotiq_85_right_inner_knuckle_link",
            "robotiq_85_left_finger_tip_link",
            "robotiq_85_right_finger_tip_link"
        };

        planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
        rclcpp::Service<interfaces::srv::AllowObjectCollision>::SharedPtr allow_object_collision_srv_;    

    };
}

#endif
