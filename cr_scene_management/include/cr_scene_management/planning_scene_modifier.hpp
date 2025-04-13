#ifndef CR_SCENE_MANAGEMENT_PLANNING_SCENE_MODIFIER_HPP
#define CR_SCENE_MANAGEMENT_PLANNING_SCENE_MODIFIER_HPP

#include <rclcpp/rclcpp.hpp>
#include <interfaces/msg/object_info.hpp>
#include <interfaces/srv/allow_collision.hpp>
#include <interfaces/srv/attach_object.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

namespace cr {
namespace scene_management {

    class PlanningSceneModifier : public rclcpp::Node
    {
    public:
        explicit PlanningSceneModifier(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    private:
        // Sottoscrizione per spawn object
        void onObjectInfoReceived(const interfaces::msg::ObjectInfo::SharedPtr object_info);
        // Callback service per allow collision
        void allowCollision(
            const std::shared_ptr<interfaces::srv::AllowCollision::Request> request,
            std::shared_ptr<interfaces::srv::AllowCollision::Response> response
        );
        // Callback service per attach object
        void attachObject(
            const std::shared_ptr<interfaces::srv::AttachObject::Request> request,
            std::shared_ptr<interfaces::srv::AttachObject::Response> response
        );

        // Publisher, subscriber, service
        rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub_;
        rclcpp::Subscription<interfaces::msg::ObjectInfo>::SharedPtr object_info_sub_;
        rclcpp::Service<interfaces::srv::AllowCollision>::SharedPtr allow_collision_srv_;
        rclcpp::Service<interfaces::srv::AttachObject>::SharedPtr attach_object_srv_;
    };

} // namespace scene_management
} // namespace cr

#endif // CR_SCENE_MANAGEMENT_PLANNING_SCENE_MODIFIER_HPP
