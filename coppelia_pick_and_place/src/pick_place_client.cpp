#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vision_localizer/msg/object_info.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "coppelia_msgs/action/coppelia_task.hpp"

using namespace std::placeholders;

namespace coppelia_pick_and_place{

    class PickPlaceClient : public rclcpp::Node
    {
    public:
        explicit PickPlaceClient(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
            : Node("pick_place_client", options)
        {
            object_info_subscriber_ = this->create_subscription<vision_localizer::msg::ObjectInfo>(
                "/object_info", 10, std::bind(&PickPlaceClient::pickAndPlaceObject, this, std::placeholders::_1));
        
            client_ = rclcpp_action::create_client<coppelia_msgs::action::CoppeliaTask>(this, "move_action_server");

            if (!client_->wait_for_action_server(std::chrono::seconds(10)))
            {
                RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
                rclcpp::shutdown();
                return;
            }
        }

    private:
        void pickAndPlaceObject(const vision_localizer::msg::ObjectInfo object_info)
        {
            // Calcolo la posizione verticale a cui deve muoversi l'end-effector quando si alza 
            float z_axis_up_offset = object_info.center.z + object_info.size.z + PRE_GRASP_OFFESET;
            float z_axis_down_offset = object_info.center.z + object_info.size.z + 0.065;

            float params[2];

            // Chiedo di andare in home
            sendGoal(0, params);

            // Chiedo di alzarsi
            params[0] = z_axis_up_offset;
            sendGoal(4, params);
            
            // Chiedo di muoversi nella posizione di pregrasp
            params[0] = object_info.center.x;
            params[1] = object_info.center.y;
            sendGoal(5, params);

            // Chiedo di fare l'approach dell'oggetto
            params[0] = z_axis_down_offset;
            sendGoal(4, params);

            // Chiudo il gripper
            sendGoal(3, params);

            // Faccio il retreat
            params[0] = z_axis_up_offset;
            sendGoal(4, params);
        }

        void sendGoal(int task_number, float params[])
        {
            auto goal_msg = coppelia_msgs::action::CoppeliaTask::Goal();
            goal_msg.task_number = task_number;

            for (int i = 0; i < sizeof(params);  i++){
                goal_msg.params.push_back(params[i]);
            }
    
            RCLCPP_INFO(get_logger(), "📤 Sending goal with task number: %d", task_number);
    
            auto send_goal_options = rclcpp_action::Client<coppelia_msgs::action::CoppeliaTask>::SendGoalOptions();
            send_goal_options.goal_response_callback =
                std::bind(&PickPlaceClient::goalCallback, this, _1);
            send_goal_options.feedback_callback =
                std::bind(&PickPlaceClient::feedbackCallback, this, _1, _2);
            send_goal_options.result_callback =
                std::bind(&PickPlaceClient::resultCallback, this, _1);
            client_->async_send_goal(goal_msg, send_goal_options);

            std::this_thread::sleep_for(std::chrono::seconds(3));
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
            //rclcpp::shutdown();
        }

        const double PRE_GRASP_OFFESET = 0.300; // [m]
        rclcpp_action::Client<coppelia_msgs::action::CoppeliaTask>::SharedPtr client_;
        rclcpp::Subscription<vision_localizer::msg::ObjectInfo>::SharedPtr object_info_subscriber_;
    };

}

RCLCPP_COMPONENTS_REGISTER_NODE(coppelia_pick_and_place::PickPlaceClient)