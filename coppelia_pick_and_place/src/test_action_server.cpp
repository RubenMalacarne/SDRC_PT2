#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "coppelia_msgs/action/coppelia_task.hpp"

#include <memory>
using namespace std::placeholders;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");
static const std::string PLANNING_GROUP = "arm_manipulator";

namespace coppelia_pick_and_place
{
    class TestActionServer : public rclcpp::Node
    {
    public:
        explicit TestActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("test_action_server", options)
        {
            RCLCPP_INFO(get_logger(), "Starting the Server");
            // create action_server
            action_server_ = rclcpp_action::create_server<coppelia_msgs::action::CoppeliaTask>(
                this, "task_action_server",
                std::bind(&TestActionServer::goalCallback, this, _1, _2),
                std::bind(&TestActionServer::cancelCallback, this, _1),
                std::bind(&TestActionServer::acceptedCallback, this, _1));
        }

    private:
        rclcpp_action::Server<coppelia_msgs::action::CoppeliaTask>::SharedPtr action_server_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group, gripper_move_group;
        std::vector<double> arm_joint_goal, gripper_joint_goal;

        rclcpp_action::GoalResponse goalCallback(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const coppelia_msgs::action::CoppeliaTask::Goal> goal)
        {
            RCLCPP_INFO(get_logger(), "Received goal request with id %d", goal->task_number);
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse cancelCallback(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<coppelia_msgs::action::CoppeliaTask>> goal_handle)
        {
            (void)goal_handle;
            RCLCPP_INFO(get_logger(), "Received reqeust to cancel goal");
            auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm_manipulator");
            auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "gripper");

            arm_move_group.stop();
            gripper_move_group.stop();

            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void acceptedCallback(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<coppelia_msgs::action::CoppeliaTask>> goal_handle)
        {

            std::thread{std::bind(&TestActionServer::execute, this, goal_handle)}.detach();
        }

        void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<coppelia_msgs::action::CoppeliaTask>> goal_handle)
        {
            // codice test_mini
            rclcpp::NodeOptions node_options;
            node_options.automatically_declare_parameters_from_overrides(true);
            auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

            rclcpp::executors::SingleThreadedExecutor executor;
            executor.add_node(move_group_node);
            std::thread([&executor]()
                        { executor.spin(); })
                .detach();

            moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
            const moveit::core::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

            RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
            RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

            moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
            std::vector<double> joint_group_positions;
            current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

            //-------------------------------------------------

            RCLCPP_INFO(get_logger(), "Executing goal");

            auto result = std::make_shared<coppelia_msgs::action::CoppeliaTask::Result>();

            // TMOVEIT INTERFACE
            if (!arm_move_group)
            {
                arm_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm_manipulator");
            }
            if (!gripper_move_group)
            {
                gripper_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "gripper");
            }

            if (goal_handle->get_goal()->task_number == 0)
            {
                // first task
                RCLCPP_INFO(get_logger(), "choice task 1");
                move_group.setNamedTarget("home");
                moveit::planning_interface::MoveGroupInterface::Plan home_plan;
                bool home_success = (move_group.plan(home_plan) == moveit::core::MoveItErrorCode::SUCCESS);
                if (home_success)
                {
                    RCLCPP_INFO(LOGGER, "🏠 Moving to home position...");
                    move_group.execute(home_plan);
                }
                else
                {
                    RCLCPP_WARN(LOGGER, "⚠️ Failed to plan motion to home position.");
                }
            }
            else if (goal_handle->get_goal()->task_number == 1)
            {
                // Second task
                RCLCPP_INFO(get_logger(), "choice task 2");

                // Set joint target positions
                std::vector<double> target_joint_positions = {1.7704, -1.3538, 1.0, -1.1455, 1.215, 0.1041};
                move_group.setJointValueTarget(target_joint_positions);

                // Plan and execute the motion
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                if (success)
                {
                    RCLCPP_INFO(LOGGER, "✅ Successfully planned motion to target position.");
                    move_group.execute(plan);
                }
                else
                {
                    RCLCPP_WARN(LOGGER, "⚠️ Failed to plan motion to target position.");
                }
            }
            else if (goal_handle->get_goal()->task_number == 2)
            {
                // Third task
                RCLCPP_INFO(get_logger(), "choice task 3");

                // Ensure the arm_move_group is initialized
                if (!arm_move_group)
                {
                    arm_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm_manipulator");
                }
                
                // Define the target pose and waypoints
                geometry_msgs::msg::Pose target_pose1 = arm_move_group->getCurrentPose().pose;
                
                RCLCPP_INFO(get_logger(), "arm_with_bounds: %d", target_pose1);
                std::vector<geometry_msgs::msg::Pose> approach_waypoints;

                target_pose1.position.z -= 0.03; // Move down slightly
                approach_waypoints.push_back(target_pose1);

                target_pose1.position.z -= 0.03; // Move down further
                approach_waypoints.push_back(target_pose1);

                // Plan the Cartesian path
                moveit_msgs::msg::RobotTrajectory trajectory_approach;
                const double jump_threshold = 0.0;
                const double eef_step = 0.01;

                double fraction = arm_move_group->computeCartesianPath(
                    approach_waypoints, eef_step, jump_threshold, trajectory_approach);

                if (fraction > 0.0)
                {
                    RCLCPP_INFO(get_logger(), "Cartesian path planned successfully with fraction: %f", fraction);
                    arm_move_group->execute(trajectory_approach);
                }
                else
                {
                    RCLCPP_WARN(get_logger(), "Failed to compute Cartesian path.");
                }
            }
            else if (goal_handle->get_goal()->task_number == 3)
            {
                // fourth task
                RCLCPP_INFO(get_logger(), "choice task 4");
            }
            else if (goal_handle->get_goal()->task_number == 4)
            {
                // fifth task
                RCLCPP_INFO(get_logger(), "choice task 5");
            }
            else
            {
                RCLCPP_ERROR(get_logger(), "Invalid Task Number");
                return;
            }

            bool arm_with_bounds = arm_move_group->setJointValueTarget(arm_joint_goal);
            bool gripper_with_bounds = gripper_move_group->setJointValueTarget(gripper_joint_goal);
            // vogli fare un print di arm_with_bounds e gripper_with_bounds
            RCLCPP_INFO(get_logger(), "arm_with_bounds: %d", arm_with_bounds);
            RCLCPP_INFO(get_logger(), "gripper_with_bounds: %d", gripper_with_bounds);

            if (!arm_with_bounds | !gripper_with_bounds)
            {
                RCLCPP_WARN(get_logger(),
                            "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
                return;
            }

            moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
            moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
            bool arm_plan_success = (arm_move_group->plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            bool gripper_plan_success = (gripper_move_group->plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);

            if (arm_plan_success && gripper_plan_success)
            {
                RCLCPP_INFO(get_logger(), "Planner SUCCEED, moving the arme and the gripper");
                arm_move_group->move();
                gripper_move_group->move();
            }
            else
            {
                RCLCPP_ERROR(get_logger(), "One or more planners failed!");
                return;
            }

            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(), "Goal Succeeded");
        };
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(coppelia_pick_and_place::TestActionServer)