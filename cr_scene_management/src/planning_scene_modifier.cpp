#include "cr_scene_management/planning_scene_modifier.hpp"
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit/collision_detection/collision_matrix.h>
#include <cr_scene_management/scene_manager.hpp>

namespace cr {
namespace scene_management {

    PlanningSceneModifier::PlanningSceneModifier(const rclcpp::NodeOptions& options)
    : Node("planning_scene_modifier", options)
    {
        // Publisher su "planning_scene" se vuoi inviare diff a mano
        planning_scene_pub_ = create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);

        // Subscriber su /object_info
        object_info_sub_ = create_subscription<interfaces::msg::ObjectInfo>(
            "/object_info", 10,
            std::bind(&PlanningSceneModifier::onObjectInfoReceived, this, std::placeholders::_1));

        // Servizio per allow collision
        allow_collision_srv_ = create_service<interfaces::srv::AllowCollision>(
            "/allow_collision",
            std::bind(&PlanningSceneModifier::allowCollision, this,
                        std::placeholders::_1, std::placeholders::_2));

        // Servizio per attach
        attach_object_srv_ = create_service<interfaces::srv::AttachObject>(
            "/attach_object",
            std::bind(&PlanningSceneModifier::attachObject, this,
                        std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(), "PlanningSceneModifier avviato.");
    }

    void PlanningSceneModifier::onObjectInfoReceived(const interfaces::msg::ObjectInfo::SharedPtr object_info)
    {
        auto& manager = SceneManager::instance(shared_from_this());
        auto psm = manager.getPlanningSceneMonitor();

        moveit_msgs::msg::CollisionObject obj;
        obj.id = object_info->id;
        obj.header.frame_id = "world";
        obj.operation = obj.ADD;

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = {
            object_info->size.x,
            object_info->size.y,
            object_info->size.z
        };
        obj.primitives.push_back(primitive);

        geometry_msgs::msg::Pose pose;
        pose.position.x = object_info->center.x;
        pose.position.y = object_info->center.y;
        pose.position.z =  object_info->center.z;
        obj.primitive_poses.push_back(pose);

        moveit_msgs::msg::PlanningScene scene_msg;
        scene_msg.is_diff = true;
        scene_msg.world.collision_objects.push_back(obj);

        planning_scene_pub_->publish(scene_msg);
        RCLCPP_INFO(get_logger(), "Spawned object %s in the scene.", object_info->id.c_str());
    }

    void PlanningSceneModifier::allowCollision(
        const std::shared_ptr<interfaces::srv::AllowCollision::Request> request,
        std::shared_ptr<interfaces::srv::AllowCollision::Response> response)
    {
        auto& manager = cr::scene_management::SceneManager::instance(shared_from_this());
        auto psm = manager.getPlanningSceneMonitor();

        if (!psm || !psm->getPlanningScene()) {
            RCLCPP_ERROR(get_logger(), "PlanningSceneMonitor non disponibile!");
            return;
        }

        // Creiamo un lock in scrittura sulla planning scene
        planning_scene_monitor::LockedPlanningSceneRW locked_scene(psm);
        collision_detection::AllowedCollisionMatrix& acm =
            locked_scene->getAllowedCollisionMatrixNonConst();

        std::vector<std::string> link_names = {
            "robotiq_85_base_link",
            "robotiq_85_left_knuckle_link",
            "robotiq_85_right_knuckle_link",
            "robotiq_85_left_finger_link",
            "robotiq_85_right_finger_link",
            "robotiq_85_left_inner_knuckle_link"
            "robotiq_85_right_inner_knuckle_link",
            "robotiq_85_left_finger_tip_link",
            "robotiq_85_right_finger_tip_link"
        };

        for (int i = 0; i < link_names.size(); i++){
            acm.setEntry(link_names[i], request->object_id, request->is_allowed);
        }

        moveit_msgs::msg::PlanningScene scene_msg;
        scene_msg.is_diff = true;

        // 2) Copiamo la nuova ACM nel messaggio
        acm.getMessage(scene_msg.allowed_collision_matrix);

        // 3) Pubblicazione
        planning_scene_pub_->publish(scene_msg);

        RCLCPP_INFO(get_logger(), "Collisioni tra '%s' e il gripper %s",
                    request->object_id.c_str(),
                    request->is_allowed ? "PERMESSE" : "VIETATE");
    }

    void PlanningSceneModifier::attachObject(
        const std::shared_ptr<interfaces::srv::AttachObject::Request> request,
        std::shared_ptr<interfaces::srv::AttachObject::Response> response)
    {
        //TODO
    }

} // namespace scene_management
} // namespace cr

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(cr::scene_management::PlanningSceneModifier)
