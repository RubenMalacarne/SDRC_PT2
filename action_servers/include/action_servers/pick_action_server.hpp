#ifndef PICK_ACTION_SERVER_HPP_
#define PICK_ACTION_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <interfaces/action/pick.hpp>
#include <interfaces/msg/object_info.hpp>

namespace action_servers
{
    class PickActionServer : public rclcpp::Node
    {
    public:

        explicit PickActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    
    private:

        using Pick = interfaces::action::Pick;
        using GoalHandlePick = rclcpp_action::ServerGoalHandle<Pick>;

        rclcpp_action::Server<Pick>::SharedPtr action_server_;

        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
        
        // Action Callbacks
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Pick::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandlePick> goal_handle);
        void handle_accepted(const std::shared_ptr<GoalHandlePick> goal_handle);

        // Main execution logic
        void execute(const std::shared_ptr<GoalHandlePick> goal_handle);

        // Helper methos
        bool reach_pre_grasp_height(const interfaces::msg::ObjectInfo &object_info);
        bool reach_pre_grasp(const interfaces::msg::ObjectInfo &object_info);
        bool move_above_object(const interfaces::msg::ObjectInfo &object_info);
        bool approach_object(const interfaces::msg::ObjectInfo &object_info);
        bool close_gripper();
        bool retreat();
        
        double pre_approach_distance_;
        double approach_distance_;
        double retreat_offset_;
    };
}

#endif