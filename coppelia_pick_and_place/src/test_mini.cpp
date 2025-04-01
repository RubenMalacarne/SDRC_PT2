#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");
static const std::string PLANNING_GROUP = "arm_manipulator";

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    const moveit::core::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions[1] = -2.50;
    joint_group_positions[2] = 1.50;
    joint_group_positions[3] = -1.50;
    joint_group_positions[4] = -1.55;

    move_group.setJointValueTarget(joint_group_positions);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
        RCLCPP_INFO(LOGGER, "✅ Motion plan created successfully.");
        move_group.execute(my_plan);
        rclcpp::sleep_for(std::chrono::seconds(5));

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

        std::vector<std::vector<double>> points = {
            {1.7704, -1.3538, 1.4059, -1.1455, 1.215, 0.1041},
            {1.8704, -1.3538, 1.3059, -1.1455, 1.215, 0.1041},
            {1.7704, -1.2538, 1.4059, -1.2455, 1.215, 0.1041},
            {1.6704, -1.3538, 1.5059, -1.1455, 1.215, 0.1041}};

        while (rclcpp::ok())
        {
            for (const auto &point : points)
            {
                move_group.setJointValueTarget(point);
                moveit::planning_interface::MoveGroupInterface::Plan loop_plan;
                if (move_group.plan(loop_plan) == moveit::core::MoveItErrorCode::SUCCESS)
                {
                    move_group.execute(loop_plan);
                    rclcpp::sleep_for(std::chrono::seconds(1));
                }
                else
                {
                    RCLCPP_WARN(LOGGER, "⚠️ Failed to plan motion to one of the circular points.");
                }
            }
        }
    }

    rclcpp::shutdown();
    return 0;
}
