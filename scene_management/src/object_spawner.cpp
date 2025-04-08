#include "scene_management/object_spawner.hpp"

namespace scene_management
{

    ObjectSpawner::ObjectSpawner(
        const rclcpp::NodeOptions &options,
        const std::string &robot_description,
        const std::string &psm_name
        ) : Node("object_spawner", options)
    {

        robot_description_ = robot_description;
        psm_name_ = psm_name;

        object_publisher_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);

        object_info_subscriber_ = this->create_subscription<interfaces::msg::ObjectInfo>(
            "object_info", 
            10,
            std::bind(&ObjectSpawner::spawnObject, this, std::placeholders::_1)
        );

    }

    void ObjectSpawner::initPlanningSceneMonitor()
    {   
        planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
            this->shared_from_this(), robot_description_, psm_name_
        );
      
        if (!planning_scene_monitor_->getPlanningScene())
        {
            RCLCPP_ERROR(this->get_logger(), "PlanningSceneMonitor failed.");
            return;
        }
    }

    void ObjectSpawner::spawnObject(const interfaces::msg::ObjectInfo object_info){

        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.id = "object";
        collision_object.header.frame_id = "world";

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = {
            object_info.size.x,
            object_info.size.y,
            object_info.size.z
        };
        collision_object.primitives.push_back(primitive);

        geometry_msgs::msg::Pose pose;
        pose.position.x = object_info.center.x;
        pose.position.y = object_info.center.y;
        pose.position.z =  object_info.center.z;
        collision_object.primitive_poses.push_back(pose);

        collision_object.operation = collision_object.ADD;

        auto current_scene = planning_scene_monitor_->getPlanningScene();
        if (!current_scene)
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to access PlanningScene");
            return;
        }

        collision_detection::AllowedCollisionMatrix allowed_collision_matrix = current_scene->getAllowedCollisionMatrix();
        allowed_collision_matrix.setEntry("object", "table", true);

        moveit_msgs::msg::PlanningScene planning_scene_msg;
        planning_scene_msg.is_diff = true;
        planning_scene_msg.world.collision_objects.push_back(collision_object);

        allowed_collision_matrix.getMessage(planning_scene_msg.allowed_collision_matrix);

        object_publisher_->publish(planning_scene_msg);

    }

}