#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from arduinobot_msgs.action import ArduinobotTask
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
import numpy as np


class TaskServer(Node):
    def __init__(self):
        super().__init__("task_server")
        self.actionServer = ActionServer(
            self,
            ArduinobotTask,
            "task_server",
            self.goalCallback,
        )
        self.arduinobot = MoveItPy(node_name="moveit_py")
        self.arduinobot_arm = self.arduinobot.get_planning_component("arm")
        self.arduinobot_gripper = self.arduinobot.get_planning_component("gripper")

        self.get_logger().info("Starter task_server")

    def goalCallback(self, goalHandle):
        self.get_logger().info(
            f"New Goal received Task#: {goalHandle.request.task_number}"
        )

        result = False

        arm_state = RobotState(self.arduinobot.get_robot_model())
        gripper_state = RobotState(self.arduinobot.get_robot_model())

        arm_joint_goal = []
        gripper_joint_goal = []

        if goalHandle.request.task_number == 0:
            arm_joint_goal = np.array([0.0, 0.0, 0.0])
            gripper_joint_goal = np.array([-0.7, 0.7])
        elif goalHandle.request.task_number == 1:
            arm_joint_goal = np.array([-1.14, -0.6, -0.07])
            gripper_joint_goal = np.array([0.0, 0.0])
        elif goalHandle.request.task_number == 2:
            arm_joint_goal = np.array([-1.57, 0.0, -0.9])
            gripper_joint_goal = np.array([0.0, 0.0])
        else:
            self.get_logger().error("Unknown Task")
            result.success = result
            return result

        arm_state.set_joint_group_positions("arm", arm_joint_goal)
        gripper_state.set_joint_group_positions("gripper", gripper_joint_goal)

        self.arduinobot_arm.set_start_state_to_current_state()
        self.arduinobot_gripper.set_start_state_to_current_state()

        self.arduinobot_arm.set_goal_state(robot_state=arm_state)
        self.arduinobot_gripper.set_goal_state(robot_state=gripper_state)

        arm_plan = self.arduinobot_arm.plan()
        gripper_plan = self.arduinobot_gripper.plan()

        if arm_plan or gripper_plan:
            result = True

        if arm_plan:
            self.arduinobot.execute(arm_plan.trajectory, controllers=[])
        else:
            self.get_logger().error("Unable to plan arm movement")

        if gripper_plan:
            self.arduinobot.execute(gripper_plan.trajectory, controllers=[])
        else:
            self.get_logger().error("Unable to plan gripper movement")

        goalHandle.succeed()
        result = ArduinobotTask.Result()
        result.success = result
        return result

    def cancelCallback(self, goalHandle):
        pass


def main():
    rclpy.init()
    taskServer = TaskServer()
    rclpy.spin(taskServer)
    TaskServer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
