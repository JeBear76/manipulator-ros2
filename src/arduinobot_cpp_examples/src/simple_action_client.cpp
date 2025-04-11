#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <arduinobot_msgs/action/fibonacci.hpp>

#include <memory>
#include <thread>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace arduinobot_cpp_examples
{
    class SimpleActionClient : public rclcpp::Node
    {
    public:
        explicit SimpleActionClient(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
            : Node("simple_action_client", options)
        {
            RCLCPP_INFO(get_logger(), "Starting fibonacci action cpp client");

            action_client_ = rclcpp_action::create_client<arduinobot_msgs::action::Fibonacci>(
                this,
                "fibonacci");
            timer_ = create_wall_timer(1s, std::bind(&SimpleActionClient::timerCallback, this));
        }

    private:
        rclcpp_action::Client<arduinobot_msgs::action::Fibonacci>::SharedPtr action_client_;
        rclcpp::TimerBase::SharedPtr timer_;

        void timerCallback()
        {
            timer_->cancel();
            action_client_->wait_for_action_server();

            auto goal = arduinobot_msgs::action::Fibonacci::Goal();
            goal.order = 12;

            RCLCPP_INFO(get_logger(), "Send Goal");

            auto send_goal_options = rclcpp_action::Client<arduinobot_msgs::action::Fibonacci>::SendGoalOptions();
            send_goal_options.goal_response_callback = std::bind(&SimpleActionClient::acceptedCallback, this, _1);
            send_goal_options.feedback_callback = std::bind(&SimpleActionClient::feedbackCallback, this, _1, _2);
            send_goal_options.result_callback = std::bind(&SimpleActionClient::completedCallback, this, _1);
            action_client_->async_send_goal(goal, send_goal_options);
        }

        void acceptedCallback(const rclcpp_action::ClientGoalHandle<arduinobot_msgs::action::Fibonacci>::SharedPtr &goalHandle)
        {
            if (!goalHandle)
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal Rejected :-(");
            else
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal Accepted :-D");
        }

        void feedbackCallback(rclcpp_action::ClientGoalHandle<arduinobot_msgs::action::Fibonacci>::SharedPtr,
                              const std::shared_ptr<const arduinobot_msgs::action::Fibonacci::Feedback> feedback)
        {
            std::stringstream ss;
            ss << "Received ";
            for (auto number : feedback->partial_sequence)
            {
                ss << number << " ";
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), ss.str().c_str());
        }

        void completedCallback(const rclcpp_action::ClientGoalHandle<arduinobot_msgs::action::Fibonacci>::WrappedResult &goalHandle)
        {
            switch (goalHandle.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
                /* code */
                break;
                case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal was canceled :-(");
                return;
                case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal was aborted :-(");
                return;
                default:
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unknown result code");
                return;
            }
            std::stringstream ss;
            ss << "Complete Sequence ";
            for (auto number : goalHandle.result->sequence)
            {
                ss << number << " ";
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), ss.str().c_str());
            rclcpp::shutdown();
        }
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(arduinobot_cpp_examples::SimpleActionClient)