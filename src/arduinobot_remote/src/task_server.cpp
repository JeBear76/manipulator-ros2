#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <arduinobot_msgs/action/arduinobot_task.hpp>
#include <memory>
#include <thread>

using namespace std::placeholders;

namespace arduinobot_cpp_examples
{
    class TaskServer : public rclcpp::Node
    {
    public:
        explicit TaskServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
            : Node("simple_action_server", options)
        {
            RCLCPP_INFO(get_logger(), "Starting Arduinotbot Task action cpp server");

            action_server_ = rclcpp_action::create_server<arduinobot_msgs::action::ArduinobotTask>(
                this,
                "task_server",
                std::bind(&TaskServer::goalCallback, this, _1, _2),
                std::bind(&TaskServer::cancelCallback, this, _1),
                std::bind(&TaskServer::acceptedCallback, this, _1));
        }

    private:
        rclcpp_action::Server<arduinobot_msgs::action::ArduinobotTask>::SharedPtr action_server_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_, gripper_move_group_;

        rclcpp_action::GoalResponse goalCallback(const rclcpp_action::GoalUUID &uuid,
                                                 std::shared_ptr<const arduinobot_msgs::action::ArduinobotTask::Goal> goal)
        {
            RCLCPP_INFO_STREAM(get_logger(), "Goal request - task_number: " << goal->task_number);
            (void)uuid;
            if(goal->task_number > 2)
                return rclcpp_action::GoalResponse::REJECT;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse cancelCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goalHandle)
        {
            RCLCPP_INFO(get_logger(), "Cancellation request recived");
            if (arm_move_group_)
            {
                arm_move_group_->stop();
            }
            if (gripper_move_group_)
            {
                gripper_move_group_->stop();
            }
            (void)goalHandle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void acceptedCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goalHandle)
        {
            std::thread(std::bind(&TaskServer::execute, this, _1), goalHandle).detach();
        }

        void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goal_handle)
        {
            RCLCPP_INFO(get_logger(), "Executing goal");
            if (!arm_move_group_)
            {
                arm_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
            }
            if (!gripper_move_group_)
            {
                gripper_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "gripper");
            }
            arm_move_group_->setStartState(*arm_move_group_->getCurrentState());
            gripper_move_group_->setStartState(*gripper_move_group_->getCurrentState());

            auto result = std::make_shared<arduinobot_msgs::action::ArduinobotTask::Result>();
            auto &success = result->success;

            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<arduinobot_msgs::action::ArduinobotTask::Feedback>();

            std::vector<double> arm_joint_goal{};
            std::vector<double> gripper_joint_goal{};

            switch (goal->task_number)
            {
            case 0:
                arm_joint_goal = {0.0, 0.0, 0.0};
                gripper_joint_goal = {-0.7, 0.7};
                break;
            case 1:
                arm_joint_goal = {-1.14, -0.6, -0.07};
                gripper_joint_goal = {0.0, 0.0};
                break;
            case 2:
                arm_joint_goal = {-1.57, 0.0, -0.9};
                gripper_joint_goal = {0.0, 0.0};
                break;
            default:
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "AUnknown Task Number");
                goal_handle->succeed(result);
                return;
            }            
            if (!arm_move_group_->setJointValueTarget(arm_joint_goal))
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Arm position unreachable");
            }
            else
            {
                success = true;
                moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
                if (arm_move_group_->plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS)
                {
                    arm_move_group_->move();
                }
                else
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Arm movement planning failed");
                }
            }

            if (!gripper_move_group_->setJointValueTarget(gripper_joint_goal))
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "gripper position unreachable");
            }
            else
            {
                success = true;
                moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
                if (gripper_move_group_->plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS)
                {
                    gripper_move_group_->move();
                }
                else
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Gripper movement planning failed");
                }
            }
            
            if (rclcpp::ok())
            {
                goal_handle->succeed(result);
                RCLCPP_INFO(get_logger(), "Goal succeeded");
            }
        }
    };

}

RCLCPP_COMPONENTS_REGISTER_NODE(arduinobot_cpp_examples::TaskServer)
