#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "coppelia_msgs/action/coppelia_task.hpp"
#include <memory>
#include <sstream>

using namespace std::placeholders;

namespace coppelia_pick_and_place
{
class MoveActionClient : public rclcpp::Node
{
public:
    explicit MoveActionClient(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("move_action_client", options)
    {
        RCLCPP_INFO(get_logger(), "Starting the Client");
        action_client_ = rclcpp_action::create_client<coppelia_msgs::action::CoppeliaTask>(this, "move_action_server");

            if (!action_client_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(get_logger(), "❌ Action server not available after waiting");
            rclcpp::shutdown();
            return;
        }

        sendGoal(1);  // <-- cambia qui il numero del task da eseguire
    }

private:
    rclcpp_action::Client<coppelia_msgs::action::CoppeliaTask>::SharedPtr action_client_;

    void sendGoal(int task_number)
    {
        auto goal_msg = coppelia_msgs::action::CoppeliaTask::Goal();
        goal_msg.task_number = task_number;

        RCLCPP_INFO(get_logger(), "📤 Sending goal with task number: %d", task_number);

        auto send_goal_options = rclcpp_action::Client<coppelia_msgs::action::CoppeliaTask>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&MoveActionClient::goalCallback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&MoveActionClient::feedbackCallback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&MoveActionClient::resultCallback, this, _1);
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }
    
    void goalCallback(const rclcpp_action::ClientGoalHandle<coppelia_msgs::action::CoppeliaTask>::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(get_logger(), "[❌] Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(get_logger(), "[✅] Goal accepted by server, waiting for result...");
        }
    }

    void feedbackCallback(
        const rclcpp_action::ClientGoalHandle<coppelia_msgs::action::CoppeliaTask>::SharedPtr,
        const std::shared_ptr<const coppelia_msgs::action::CoppeliaTask::Feedback> feedback)
    {
        auto pose = feedback->current_pose;
        RCLCPP_INFO(get_logger(),
                    "⏳ Feedback: %d%% complete | Pose: [x=%.3f, y=%.3f, z=%.3f]",
                    feedback->percentage,
                    pose.position.x,
                    pose.position.y,
                    pose.position.z);
    }

    void resultCallback(
        const rclcpp_action::ClientGoalHandle<coppelia_msgs::action::CoppeliaTask>::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "🎯 Goal succeeded!");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "🛑 Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(get_logger(), "❌ Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(get_logger(), "❓ Unknown result code");
            return;
        }

        RCLCPP_INFO(get_logger(), "[🏁✅] Result: success = %s", result.result->success ? "true" : "false");
        rclcpp::shutdown();
    }
};
} // namespace coppelia_pick_and_place

RCLCPP_COMPONENTS_REGISTER_NODE(coppelia_pick_and_place::MoveActionClient)
