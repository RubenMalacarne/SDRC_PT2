#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

// Messaggio generato dall’action
#include "coppelia_msgs/action/coppelia_task.hpp"

#include <memory>
#include <thread>
#include <vector>

using namespace std::placeholders;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

namespace coppelia_pick_and_place
{
    class MoveActionServer : public rclcpp::Node
    {
    public:
        explicit MoveActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
            : Node("move_action_server", options)
        {
            RCLCPP_INFO(this->get_logger(), "Starting the Server");

            // Crea l'action server
            action_server_ = rclcpp_action::create_server<coppelia_msgs::action::CoppeliaTask>(
                this,
                "move_action_server",
                std::bind(&MoveActionServer::goalCallback, this, _1, _2),
                std::bind(&MoveActionServer::cancelCallback, this, _1),
                std::bind(&MoveActionServer::acceptedCallback, this, _1));
        }

    private:

        const float OFFSET_Z = 0.30; // [m]
        double arr[3] = { 0.9, 0.625, 0.939 }; // Center of the box

        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_move_group_;    
        rclcpp_action::Server<coppelia_msgs::action::CoppeliaTask>::SharedPtr action_server_;

        moveit::planning_interface::MoveGroupInterface::Plan our_plan_arm;
        moveit::planning_interface::MoveGroupInterface::Plan our_plan_gripper;

        // -----------------------------------------------------------------------------
        //                          CALLBACKS ACTION SERVER : GOAL
        // -----------------------------------------------------------------------------
        rclcpp_action::GoalResponse goalCallback(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const coppelia_msgs::action::CoppeliaTask::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "Received goal request with id %d", goal->task_number);
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        // -----------------------------------------------------------------------------
        //                          CALLBACKS ACTION SERVER : CANCELl
        // -----------------------------------------------------------------------------
        rclcpp_action::CancelResponse cancelCallback(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<coppelia_msgs::action::CoppeliaTask>> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            if (arm_move_group_)
                arm_move_group_->stop();
            if (gripper_move_group_)
                gripper_move_group_->stop();

            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        // -----------------------------------------------------------------------------
        //                          CALLBACKS ACTION SERVER : ACCEPT
        // -----------------------------------------------------------------------------
        void acceptedCallback(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<coppelia_msgs::action::CoppeliaTask>> goal_handle)
        {
            // L'esecuzione avviene in un thread dedicato
            std::thread{std::bind(&MoveActionServer::execute, this, goal_handle)}.detach();
        }

        // -----------------------------------------------------------------------------
        //                          CALLBACKS ACTION SERVER : EXECUTE
        // -----------------------------------------------------------------------------

        void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<coppelia_msgs::action::CoppeliaTask>> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Executing goal...");
            auto result = std::make_shared<coppelia_msgs::action::CoppeliaTask::Result>();

            // Inizializza i MoveGroup (se non già inizializzati)
            if (!arm_move_group_)
            {
                arm_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                    shared_from_this(), "arm_manipulator");
            }
            if (!gripper_move_group_)
            {
                gripper_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                    shared_from_this(), "gripper");
            }

            // resum info braccio e gripper
            RCLCPP_INFO(LOGGER, "Planning frame (arm): %s", arm_move_group_->getPlanningFrame().c_str());
            RCLCPP_INFO(LOGGER, "End-effector link (arm): %s", arm_move_group_->getEndEffectorLink().c_str());

            // FIRST FEEDBACK (0%)
            // -----------------------------------------------------------------------------
            auto feedback = std::make_shared<coppelia_msgs::action::CoppeliaTask::Feedback>();
            feedback->percentage = 0;
            feedback->current_pose = arm_move_group_->getCurrentPose().pose;
            goal_handle->publish_feedback(feedback);

            // CHOICE TASK
            // -----------------------------------------------------------------------------
            bool valid_task = true;
            switch (goal_handle->get_goal()->task_number)
            {
            case 0:
                // 0 => GO HOME
                RCLCPP_INFO(this->get_logger(), "choice task 0 => GO HOME !!!");
                our_plan_arm = go_home();
                break;
            case 1:
                // 1 => specific joint positions
                RCLCPP_INFO(this->get_logger(), "choice task 1 => specific joint positions");
                {
                    // Esempio di giunti
                    std::vector<double> target_joint_positions = {1.7704, -1.3538, 1.0, -1.1455, 1.215, 0.1041};
                    our_plan_arm = joint_movement(target_joint_positions);
                }
                break;
            case 2:
                // 2 => cartesian path example
                RCLCPP_INFO(this->get_logger(), "choice task 2 => cartesian path example");
                our_plan_arm = cartesian_movement();
                break;
            case 3:
                // 3 => close gripper example
                RCLCPP_INFO(this->get_logger(), "choice task 3 => close gripper example");
                our_plan_gripper = gripper_movement();
                break;
            case 4:
                // 4 => positive translation along Z axis
                RCLCPP_INFO(this->get_logger(), "choice task 4 => positive translation along Z axis");
                our_plan_arm = z_axis_translation(true);
                break;
            case 5:
                // 5 => negative translation along Z axis
                RCLCPP_INFO(this->get_logger(), "choice task 5 => negative translation along Z axis");
                our_plan_arm = z_axis_translation(false);
                break;
            case 6:
                // 6 => go above object
                RCLCPP_INFO(this->get_logger(), "choice task 6 => go above object");
                our_plan_arm = go_above_object();
                break;
            case 7:
                // 7 => open gripper
                RCLCPP_INFO(this->get_logger(), "choice task 7 => open gripper");
                our_plan_gripper = open_gripper();
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Invalid Task Number");
                result->success = false;
                goal_handle->abort(result);
                valid_task = false;
                break;
            }

            if (!valid_task)
                return;

            // SECOND FEEDBACK (50%)
            // -----------------------------------------------------------------------------
            feedback->percentage = 50;   // indica che il percorso è pianificato
            feedback->current_pose = arm_move_group_->getCurrentPose().pose;
            goal_handle->publish_feedback(feedback);

            // EXECUTE MOVEMENTS (GRIPPER + ARM)
            // -----------------------------------------------------------------------------
            if (!our_plan_arm.trajectory_.joint_trajectory.points.empty() || !our_plan_gripper.trajectory_.joint_trajectory.points.empty() )
            {
                arm_move_group_->execute(our_plan_arm);
                gripper_move_group_->execute(our_plan_gripper);
            }
            
            // FINAL FEEDBACK (100%) AND SUCCESS
            // -----------------------------------------------------------------------------
            feedback->percentage = 100;
            feedback->current_pose = arm_move_group_->getCurrentPose().pose;
            goal_handle->publish_feedback(feedback);

            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal Succeeded, NOW ROBOT IS MOVING");
        }

        // -----------------------------------------------------------------------------
        //                          FUNCTION MOVEMENTS
        // -----------------------------------------------------------------------------
        moveit::planning_interface::MoveGroupInterface::Plan go_home ()
        {
            arm_move_group_->setNamedTarget("home");
            moveit::planning_interface::MoveGroupInterface::Plan home_plan;
            bool success = (arm_move_group_->plan(home_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            if (success)
            {
                RCLCPP_INFO(LOGGER, "🏠 Moving to home position...");
            }
            else
            {
                RCLCPP_WARN(LOGGER, "⚠️ Failed to plan home.");
            }
            return home_plan;
        }

        //down about 3 cm
        moveit::planning_interface::MoveGroupInterface::Plan cartesian_movement()
        {
            moveit::planning_interface::MoveGroupInterface::Plan cart_plan;

            geometry_msgs::msg::Pose start_pose = arm_move_group_->getCurrentPose().pose;
            std::vector<geometry_msgs::msg::Pose> waypoints;

            geometry_msgs::msg::Pose target_pose = start_pose;
            target_pose.position.z -= 0.03; 
            waypoints.push_back(target_pose);

            target_pose.position.z -= 0.03;
            waypoints.push_back(target_pose);

            moveit_msgs::msg::RobotTrajectory trajectory;
            double jump_threshold = 0.0;
            double eef_step = 0.01;

            double fraction = arm_move_group_->computeCartesianPath(
                waypoints, eef_step, jump_threshold, trajectory);

            if (fraction > 0.0)
            {
                RCLCPP_INFO(this->get_logger(), "✅ Cartesian path planned successfully (fraction: %f)", fraction);
                cart_plan.trajectory_ = trajectory;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "⚠️Failed to compute Cartesian path.");
                cart_plan.trajectory_ = moveit_msgs::msg::RobotTrajectory();
            }
            return cart_plan;
        }

        // classic pianification
        moveit::planning_interface::MoveGroupInterface::Plan joint_movement(const std::vector<double> &target_joint_positions)
        {
            arm_move_group_->setJointValueTarget(target_joint_positions);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool success = (arm_move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            if (success)
            {
                RCLCPP_INFO(LOGGER, "✅ Successfully planned motion to target position.");
            }
            else
            {
                RCLCPP_WARN(LOGGER, "⚠️ Failed to plan motion to target position.");
            }
            return plan;
        }

        // gripper
        moveit::planning_interface::MoveGroupInterface::Plan gripper_movement()
        {
            gripper_move_group_->setNamedTarget("gripper_close");
            moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
            bool success = (gripper_move_group_->plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            if (success)
            {
                RCLCPP_INFO(LOGGER, "✅🤲 Gripper closed successfully.");
            }
            else
            {
                RCLCPP_WARN(LOGGER, "⚠️ Failed to plan motion for gripper.");
            }
            return gripper_plan;
        }
    
        // Translate along Z axis - up or down
        moveit::planning_interface::MoveGroupInterface::Plan z_axis_translation(bool up){

            moveit::planning_interface::MoveGroupInterface::Plan output_plan;

            geometry_msgs::msg::Pose current_pose = arm_move_group_->getCurrentPose().pose;
            std::vector<geometry_msgs::msg::Pose> z_traslation_waypoints;
            geometry_msgs::msg::Pose pose_z_traslated  = current_pose;

            if (up){
                pose_z_traslated.position.z += OFFSET_Z;
            } else {
                pose_z_traslated.position.z -= OFFSET_Z;
            }
            
            z_traslation_waypoints.push_back(pose_z_traslated);

            moveit_msgs::msg::RobotTrajectory trajectory_z_traslation;
            const double jump_threshold = 0.0;
            const double eef_step = 0.01;

            double fraction = arm_move_group_->computeCartesianPath(
                z_traslation_waypoints,
                eef_step,
                jump_threshold,
                trajectory_z_traslation
            );

            if (fraction > 0.0)
            {
                RCLCPP_INFO(this->get_logger(), "✅ Cartesian path planned successfully (fraction: %f)", fraction);
                output_plan.trajectory_ = trajectory_z_traslation;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "⚠️Failed to compute Cartesian path.");
                output_plan.trajectory_ = moveit_msgs::msg::RobotTrajectory();
            }

            return output_plan;           
        }
    
        // Move Above Object
        moveit::planning_interface::MoveGroupInterface::Plan go_above_object(){
            moveit::planning_interface::MoveGroupInterface::Plan output_plan;

            geometry_msgs::msg::Pose current_pose = arm_move_group_->getCurrentPose().pose;
            std::vector<geometry_msgs::msg::Pose> waypoints;
            geometry_msgs::msg::Pose target_pose  = current_pose;

            target_pose.position.x = arr[0];
            target_pose.position.y = arr[1];

            waypoints.push_back(target_pose);

            moveit_msgs::msg::RobotTrajectory trajectory;
            const double jump_threshold = 0.0;
            const double eef_step = 0.01;

            double fraction = arm_move_group_->computeCartesianPath(
                waypoints,
                eef_step,
                jump_threshold,
                trajectory
            );

            if (fraction > 0.0)
            {
                RCLCPP_INFO(this->get_logger(), "✅ Cartesian path planned successfully (fraction: %f)", fraction);
                output_plan.trajectory_ = trajectory;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "⚠️Failed to compute Cartesian path.");
                output_plan.trajectory_ = moveit_msgs::msg::RobotTrajectory();
            }

            return output_plan;   
        }
    
        // Open Gripper
        moveit::planning_interface::MoveGroupInterface::Plan open_gripper()
        {
            gripper_move_group_->setNamedTarget("gripper_open");
            moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
            bool success = (gripper_move_group_->plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            if (success)
            {
                RCLCPP_INFO(LOGGER, "✅🤲 Gripper opened successfully.");
            }
            else
            {
                RCLCPP_WARN(LOGGER, "⚠️ Failed to plan motion for gripper.");
            }
            return gripper_plan;
        }
    
    };
} // namespace coppelia_pick_and_place

RCLCPP_COMPONENTS_REGISTER_NODE(coppelia_pick_and_place::MoveActionServer)
