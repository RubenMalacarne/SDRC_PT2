#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "coppelia_msgs/action/coppelia_task.hpp"

#include <memory>
using namespace std::placeholders;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

namespace coppelia_pick_and_place
{
    class TestActionServer : public rclcpp::Node
    {
    public:
        explicit TestActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
            : Node("test_action_server", options)
        {
            RCLCPP_INFO(get_logger(), "Starting the Server");

            // Crea l'action server
            action_server_ = rclcpp_action::create_server<coppelia_msgs::action::CoppeliaTask>(
                this, "task_action_server",
                std::bind(&TestActionServer::goalCallback, this, _1, _2),
                std::bind(&TestActionServer::cancelCallback, this, _1),
                std::bind(&TestActionServer::acceptedCallback, this, _1));
        }

    private:
    
        moveit::planning_interface::MoveGroupInterface::Plan our_plan_arm;
        moveit::planning_interface::MoveGroupInterface::Plan our_plan_gripper;
        rclcpp_action::Server<coppelia_msgs::action::CoppeliaTask>::SharedPtr action_server_;
        
        // MoveGroupInterface per braccio e gripper
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group, gripper_move_group;

        // GOAL CALLBACK ---------------------------
        rclcpp_action::GoalResponse goalCallback(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const coppelia_msgs::action::CoppeliaTask::Goal> goal)
        {
            RCLCPP_INFO(get_logger(), "Received goal request with id %d", goal->task_number);
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        
        // CANCELL CALLBACK ---------------------------
        rclcpp_action::CancelResponse cancelCallback(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<coppelia_msgs::action::CoppeliaTask>> goal_handle)
        {
            (void)goal_handle;
            RCLCPP_INFO(get_logger(), "Received request to cancel goal");
            
            // Stop immediato di entrambi i MoveGroup
            if (arm_move_group)
                arm_move_group->stop();
            if (gripper_move_group)
                gripper_move_group->stop();

            return rclcpp_action::CancelResponse::ACCEPT;
        }

        // ACCEPT FUNCTION ---------------------------
        void acceptedCallback(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<coppelia_msgs::action::CoppeliaTask>> goal_handle)
        {
            // execution on a thread
            std::thread{std::bind(&TestActionServer::execute, this, goal_handle)}.detach();
        }

        // EXECUTE FUNCTION ---------------------------
        void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<coppelia_msgs::action::CoppeliaTask>> goal_handle)
        {
            
            // test configuration move group
            if (!arm_move_group)
            {
                arm_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                    shared_from_this(), "arm_manipulator");
            }
            if (!gripper_move_group)
            {
                gripper_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                    shared_from_this(), "gripper");
            }

            RCLCPP_INFO(get_logger(), "Executing goal...");
            auto result = std::make_shared<coppelia_msgs::action::CoppeliaTask::Result>();

            // resum info braccio e gripper
            RCLCPP_INFO(LOGGER, "Planning frame (arm): %s", arm_move_group->getPlanningFrame().c_str());
            RCLCPP_INFO(LOGGER, "End-effector link (arm): %s", arm_move_group->getEndEffectorLink().c_str());

            //TASK EVENT
            switch (goal_handle->get_goal()->task_number)
            {
            case 0:
            {
                // Task 0: Vai in posizione di "home" per il braccio
                RCLCPP_INFO(get_logger(), "choice task 0 => GO HOME !!!");
                our_plan_arm = go_home();
            }
            break;

            case 1:
            {
                // Task 1: Esempio di impostare un set di giunti per il braccio
                RCLCPP_INFO(get_logger(), "choice task 1 => specific joint positions");
                std::vector<double> target_joint_positions = {1.7704, -1.3538, 1.0, -1.1455, 1.215, 0.1041};
                our_plan_arm = normal_example(target_joint_positions);
            }
            break;

            case 2:
            {
                // Task 2: Esempio di moto cartesiano del braccio
                RCLCPP_INFO(get_logger(), "choice task 2 => cartesian path example");
                our_plan_arm = cartesian_movement_example();
            }
            break;

            case 3:
            {
                // Task 3: Esempio di movimento del gripper (apertura/chiusura)
                RCLCPP_INFO(get_logger(), "choice task 3 => close gripper example");
                our_plan_gripper = gripper_example();
                
            }
            break;

            case 4:
            {
                // Task 4: Esempio placeholder
                RCLCPP_INFO(get_logger(), "choice task 4 => do something else");
            }
            break;

            default:
            {
                // Task non riconosciuto
                RCLCPP_ERROR(get_logger(), "Invalid Task Number");
                result->success = false;
                goal_handle->abort(result);
                return;
            }
            }
            arm_move_group->execute(our_plan_arm);
            gripper_move_group->execute(our_plan_gripper);
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(), "Goal Succeeded");
        }

        // HOMMING MOVEMENTS ---------------------------
        moveit::planning_interface::MoveGroupInterface::Plan go_home ()
        {
            // Esegui il movimento verso la posizione di "home" per il braccio
            arm_move_group->setNamedTarget("home");
            moveit::planning_interface::MoveGroupInterface::Plan home_plan;
            bool home_success = (arm_move_group->plan(home_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            if (home_success)
            {
                RCLCPP_INFO(LOGGER, "🏠 Moving to home position...");
            }
            else
            {
                RCLCPP_WARN(LOGGER, "⚠️ Failed to plan close gripper.");
            }
            return home_plan;
        }
        
        // CARTESIAN MOVEMENTS examples---------------------------
        moveit::planning_interface::MoveGroupInterface::Plan cartesian_movement_example()
        {
                moveit::planning_interface::MoveGroupInterface::Plan carte_plan;
                // Partendo dall'End Effector Pose attuale
                geometry_msgs::msg::Pose start_pose = arm_move_group->getCurrentPose().pose;
                std::vector<geometry_msgs::msg::Pose> waypoints;

                // Esempio: abbassare di 3cm e poi di altri 3cm
                geometry_msgs::msg::Pose target_pose = start_pose;
                target_pose.position.z -= 0.03; 
                waypoints.push_back(target_pose);

                target_pose.position.z -= 0.03;
                waypoints.push_back(target_pose);

                moveit_msgs::msg::RobotTrajectory trajectory;
                const double jump_threshold = 0.0;
                const double eef_step = 0.01;

                double fraction = arm_move_group->computeCartesianPath(
                    waypoints, eef_step, jump_threshold, trajectory);

                if (fraction > 0.0)
                {
                    RCLCPP_INFO(get_logger(), "Cartesian path planned successfully (fraction: %f)", fraction);
                    carte_plan.trajectory_ = trajectory;
                }
                else
                {
                    RCLCPP_WARN(get_logger(), "Failed to compute Cartesian path.");
                    carte_plan.trajectory_ = moveit_msgs::msg::RobotTrajectory();
                }
                return carte_plan;
        }
                
        // NORMAL MOVEMENTS examples---------------------------
        moveit::planning_interface::MoveGroupInterface::Plan normal_example(std::vector<double> target_joint_positions)
        {
                arm_move_group->setJointValueTarget(target_joint_positions);
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                bool success = (arm_move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
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

        // GRIPPER MOVEMENTS examples---------------------------
        moveit::planning_interface::MoveGroupInterface::Plan gripper_example()
        {
            // Esegui il movimento del gripper
            gripper_move_group->setNamedTarget("gripper_close");
            moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
            bool gripper_success = (gripper_move_group->plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            if (gripper_success)
            {
                RCLCPP_INFO(LOGGER, "🤲 Gripper closed successfully.");
            }
            else
            {
                RCLCPP_WARN(LOGGER, "⚠️ Failed to plan motion for gripper.");
            }
            return gripper_plan;
        }

    };
} 

RCLCPP_COMPONENTS_REGISTER_NODE(coppelia_pick_and_place::TestActionServer)
