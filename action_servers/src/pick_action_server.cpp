#include "action_servers/pick_action_server.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/planning_interface/planning_interface.h>

using namespace std::placeholders;

namespace action_servers
{
    PickActionServer::PickActionServer(const rclcpp::NodeOptions &options)
        : Node("pick_action_server", options)
    {
        RCLCPP_INFO(this->get_logger(), "Starting PickActionServer...");

        // Config parameters
        this->declare_parameter("pre_approach_distance", 0.30);
        this->declare_parameter("approach_distance", 0.05);
        this->declare_parameter("retreat_offset", 0.20);

        this->get_parameter("pre_approach_distance", pre_approach_distance_);
        this->get_parameter("approach_distance", approach_distance_);
        this->get_parameter("retreat_offset", retreat_offset_);

        // Action Server
        action_server_ = rclcpp_action::create_server<Pick>(
            this,
            "pick_action",
            std::bind(&PickActionServer::handle_goal, this, _1, _2),
            std::bind(&PickActionServer::handle_cancel, this, _1),
            std::bind(&PickActionServer::handle_accepted, this, _1)
        );
    }

    rclcpp_action::GoalResponse PickActionServer::handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Pick::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(),
            "Received goal request:\n - Center: [x: %.3f, y: %.3f, z: %.3f]\n - Size: [x: %.3f, y: %.3f, z: %.3f]",
            goal->object_info.center.x, goal->object_info.center.y, goal->object_info.center.z,
            goal->object_info.size.x, goal->object_info.size.y, goal->object_info.size.z
        );
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse PickActionServer::handle_cancel(const std::shared_ptr<GoalHandlePick> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void PickActionServer::handle_accepted(const std::shared_ptr<GoalHandlePick> goal_handle)
    {
      std::thread{std::bind(&PickActionServer::execute, this, _1), goal_handle}.detach();
    }

    // Execution of the main logic
    void PickActionServer::execute(const std::shared_ptr<GoalHandlePick> goal_handle)
    {
        if (!arm_group_)
        {
            arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                shared_from_this(), "arm_manipulator");
        }
        if (!gripper_group_)
        {
            gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                shared_from_this(), "gripper");
        }

        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Pick::Feedback>();
        auto result = std::make_shared<Pick::Result>();
    
        const auto &object_info = goal->object_info;

        feedback->percentage = 10.0;
        feedback->feedback_msg = "Planning move above object...";
        goal_handle->publish_feedback(feedback);

        // if(!move_above_object(object_info))
        // {
        //     result->success = false;
        //     result->result_msg = "Failed to move above object.";
        //     goal_handle->abort(result);
        //     return;
        // } 

        if(!reach_pre_grasp_height(object_info))
        {
            result->success = false;
            result->result_msg = "Failed to reach pre-grasp height.";
            goal_handle->abort(result);
            return;
        } 

        feedback->percentage = 15.0;
        feedback->feedback_msg = "Reaching Pre-Grasping pose";
        goal_handle->publish_feedback(feedback);

        if(!reach_pre_grasp(object_info))
        {
            result->success = false;
            result->result_msg = "Failed to reach pre-grasp pose.";
            goal_handle->abort(result);
            return;
        } 

        feedback->percentage = 30.0;
        feedback->feedback_msg = "Approaching object...";
        goal_handle->publish_feedback(feedback);

        if(!approach_object(object_info))
        {
            result->success = false;
            result->result_msg = "Failed to approach object.";
            goal_handle->abort(result);
            return;
        }

        feedback->percentage = 60.0;
        feedback->feedback_msg = "Closing gripper...";
        goal_handle->publish_feedback(feedback);

        if(!close_gripper())
        {
            result->success = false;
            result->result_msg = "Failed to close gripper.";
            goal_handle->abort(result);
            return;
        }

        // feedback->percentage = 80.0;
        // feedback->feedback_msg = "Retreating...";
        // goal_handle->publish_feedback(feedback);
    
        // if(!retreat()){
        //     result->success = false;
        //     result->result_msg = "Failed to retreat.";
        //     goal_handle->abort(result);
        //     return;            
        // }

        feedback->percentage = 100.0;
        feedback->feedback_msg = "Pick completed.";
        goal_handle->publish_feedback(feedback);

        result->success = true;
        result->result_msg = "Pick task complete successfully.";
        goal_handle->succeed(result);

    }

    // Helper Methods
    bool PickActionServer::reach_pre_grasp_height(const interfaces::msg::ObjectInfo &object_info)
    {
        geometry_msgs::msg::Pose start_pose = arm_group_->getCurrentPose().pose;
        geometry_msgs::msg::Pose target_pose = start_pose;
        target_pose.position.z = object_info.center.z + object_info.size.z / 2.0 + pre_approach_distance_;
    
        RCLCPP_INFO(this->get_logger(),
            "object_info.center.z=%.3f  object_info.size.z=%.3f  approach_distance_=%.3f",
            object_info.center.z, object_info.size.z, approach_distance_);


        // 3. Debug: stampa le coordinate
        RCLCPP_INFO(
            this->get_logger(),
            "reach_pre_grasp_height:\n"
            "  Start pose: [x=%.3f, y=%.3f, z=%.3f]\n"
            "  Target pose: [x=%.3f, y=%.3f, z=%.3f]",
            start_pose.position.x, start_pose.position.y, start_pose.position.z,
            target_pose.position.x, target_pose.position.y, target_pose.position.z
        );
    
        // 4. Un solo waypoint (possiamo anche gestirne di più se serve)
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose);
    
        // 5. Calcolo cartesian path
        moveit_msgs::msg::RobotTrajectory trajectory;
        double eef_step = 0.01;
        double jump_threshold = 0.0;
    
        double fraction = arm_group_->computeCartesianPath(
            waypoints,
            eef_step,
            jump_threshold,
            trajectory
        );
    
        RCLCPP_INFO(
            this->get_logger(),
            "  Cartesian path fraction: %.2f",
            fraction
        );
    
        // 6. Se il planner realizza meno di un 80% del percorso, consideriamo fallito
        if (fraction < 0.8)
        {
            RCLCPP_WARN(this->get_logger(),
                        "  Fraction is below threshold (%.2f). Aborting.", fraction);
            return false;
        }
    
        // 7. Se ok, prepariamo il plan
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
    
        // 8. Esegui il piano
        auto error_code = arm_group_->execute(plan);
        if (error_code != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_WARN(this->get_logger(),
                        "  Execution of cartesian path failed (error code: %d)",
                        error_code.val);
            return false;
        }
    
        RCLCPP_INFO(this->get_logger(), "  Successfully reached pre-grasp height!");
        return true;
    }    

    bool PickActionServer::reach_pre_grasp(const interfaces::msg::ObjectInfo &object_info)
    {
        geometry_msgs::msg::Pose start_pose = arm_group_->getCurrentPose().pose;
        geometry_msgs::msg::Pose target_pose = start_pose;
        target_pose.position.x = object_info.center.x;
        target_pose.position.y = object_info.center.y;

        std::vector<geometry_msgs::msg::Pose> waypoints{target_pose};
        moveit_msgs::msg::RobotTrajectory trajectory;
        double eef_step = 0.01;
        double jump_threshold = 0.0;

        double fraction = arm_group_->computeCartesianPath(
            waypoints, 
            eef_step,
            jump_threshold,
            trajectory
        );

        if (fraction < 0.9)
            return false;
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        return arm_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    }   

    bool PickActionServer::move_above_object(const interfaces::msg::ObjectInfo &object_info)
    {
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "world";
        target_pose.pose.position.x = object_info.center.x;
        target_pose.pose.position.y = object_info.center.y;
        target_pose.pose.position.z = object_info.center.z + pre_approach_distance_;
        target_pose.pose.orientation.w = 1.0; 

        arm_group_->setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
    
        if (arm_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
            return false;
        return arm_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    }

    bool PickActionServer::approach_object(const interfaces::msg::ObjectInfo &object_info)
    {
        geometry_msgs::msg::Pose start_pose = arm_group_->getCurrentPose().pose;
        geometry_msgs::msg::Pose target_pose = start_pose;
        target_pose.position.z = object_info.center.z + object_info.size.z/2 + approach_distance_;

        std::vector<geometry_msgs::msg::Pose> waypoints{target_pose};
        moveit_msgs::msg::RobotTrajectory trajectory;
        double eef_step = 0.01;
        double jump_threshold = 0.0;

        double fraction = arm_group_->computeCartesianPath(
            waypoints, 
            eef_step,
            jump_threshold,
            trajectory
        );

        if (fraction < 0.9)
            return false;
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        return arm_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    }

    bool PickActionServer::close_gripper(){
        gripper_group_->setNamedTarget("gripper_close");
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if(gripper_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
            return false;
        return gripper_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    }

    bool PickActionServer::retreat()
    {
        geometry_msgs::msg::Pose start_pose = arm_group_->getCurrentPose().pose;
        geometry_msgs::msg::Pose target_pose = start_pose;
        target_pose.position.z += retreat_offset_;

        std::vector<geometry_msgs::msg::Pose> waypoints{target_pose};
        moveit_msgs::msg::RobotTrajectory trajectory;
        double eef_step = 0.01;
        double jump_threshold = 0.0;

        double fraction = arm_group_->computeCartesianPath(
            waypoints, 
            eef_step,
            jump_threshold,
            trajectory
        );

        if (fraction < 0.9)
            return false;
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        return arm_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    }

}