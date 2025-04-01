#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>

using namespace std::chrono_literals;

class ScenePublisher : public rclcpp::Node
{
public:
    ScenePublisher() : Node("scene_publisher")
    {
        planning_scene_pub_ = this->create_publisher<moveit_msgs::msg::PlanningScene>(
            "planning_scene", 10);

        timer_ = this->create_wall_timer(2s, std::bind(&ScenePublisher::publishScene, this));
    }

private:
    void publishScene()
    {
        moveit_msgs::msg::PlanningScene planning_scene_msg;

        planning_scene_msg.is_diff = true;

        geometry_msgs::msg::PoseStamped table_pose;
        table_pose.header.frame_id = "world";
        table_pose.pose.position.x = 0.0;
        table_pose.pose.position.y = 0.0;
        table_pose.pose.position.z = 0.0;
        table_pose.pose.orientation.x = 0.0;
        table_pose.pose.orientation.y = 0.0;
        table_pose.pose.orientation.z = 0.0;
        table_pose.pose.orientation.w = 1.0;

        moveit_msgs::msg::CollisionObject table;
        table.id = "table";
        table.header.frame_id = "world";
        table.operation = moveit_msgs::msg::CollisionObject::ADD;

        std::string mesh_path = "package://coppelia_description/meshes/lab_table_mesh.stl";
        
        shapes::Mesh* shape_mesh = shapes::createMeshFromResource(mesh_path);

        if (!shape_mesh)
        {
            RCLCPP_ERROR(this->get_logger(), "Impossibile caricare la mesh da: %s", mesh_path.c_str());
            return;
        }

        shapes::ShapeMsg shape_msg;
        shapes::constructMsgFromShape(shape_mesh, shape_msg);
        shape_msgs::msg::Mesh mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);

        table.meshes.push_back(mesh_msg);
        table.mesh_poses.push_back(table_pose.pose);

        planning_scene_msg.world.collision_objects.push_back(table);

        planning_scene_pub_->publish(planning_scene_msg);
        timer_->cancel();
    }

    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ScenePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}