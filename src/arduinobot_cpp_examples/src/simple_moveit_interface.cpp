#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.hpp>

void move_robot(const std::shared_ptr<rclcpp::Node> node)
{
    auto arm_move_group = moveit::planning_interface::MoveGroupInterface(node, "arm");
    auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(node, "gripper");

    std::vector<double> arm_joint_goal{1.57, 0, 0};
    std::vector<double> gripper_joint_goal{-0.45, 0.45};

    if (!arm_move_group.setJointValueTarget(arm_joint_goal))
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Arm position unreachable");
    }
    else
    {
        moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
        if (arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS)
        {
            arm_move_group.move();
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Arm movement planning failed");
        }
    }

    if (!gripper_move_group.setJointValueTarget(gripper_joint_goal))
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "gripper position unreachable");
    }
    else
    {
        moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
        if (gripper_move_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS)
        {
            gripper_move_group.move();
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Gripper movement planning failed");
        }
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("simple_moveit_interface");
    move_robot(node);
    rclcpp::shutdown();
}