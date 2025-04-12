import rclpy
from rclpy.logging import get_logger
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
import numpy as np

def move_robot():
    arduinobot = MoveItPy(
        node_name="moveit_py"
    )

    arduinobot_arm = arduinobot.get_planning_component("arm")
    arduinobot_gripper = arduinobot.get_planning_component("gripper")

    arm_state = RobotState(arduinobot.get_robot_model())
    gripper_state = RobotState(arduinobot.get_robot_model())

    arm_state.set_joint_group_positions(
        "arm", np.array([1.57,0.0, 0.0])
    )

    gripper_state.set_joint_group_positions(
        "gripper", np.array([-0.83, 0.83])
    )

    arduinobot_arm.set_start_state_to_current_state()
    arduinobot_gripper.set_start_state_to_current_state()

    arduinobot_arm.set_goal_state(robot_state=arm_state)
    arduinobot_gripper.set_goal_state(robot_state=gripper_state)

    arm_plan = arduinobot_arm.plan()
    gripper_plan = arduinobot_gripper.plan()

    if arm_plan and gripper_plan:
        arduinobot.execute(arm_plan.trajectory, controllers=[])
    else:
        get_logger("rclpy").error("Unable to plan arm movement")
        
    if gripper_plan:
        arduinobot.execute(gripper_plan.trajectory, controllers=[])
    else:
        get_logger("rclpy").error("Unable to plan gripper movement")

def main():
    rclpy.init()
    move_robot()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


