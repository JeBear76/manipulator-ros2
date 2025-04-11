from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    arduinobot_description_dir = get_package_share_directory("arduinobot_description")

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    arduinobot_description_dir, "urdf", "arduinobot.urdf.xacro"
                ),
            ]
        ),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description},{'use_sim_time': False}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            os.path.join(
                get_package_share_directory("arduinobot_controller"),
                "config",
                "arduinobot_controllers.yaml",
            ),
        ],
        arguments=["--use_sim_time True"]
    )

    joint_state_broadcaster_spawnwer = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    arm_controller_spawnwer = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    gripper_controller_spawnwer = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )

    controller_start_event = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher, on_start=[controller_manager]
        )
    )

    spawner_event = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager,
            on_start=[
                joint_state_broadcaster_spawnwer,
                gripper_controller_spawnwer,
                arm_controller_spawnwer,
            ],
        )
    )

    return LaunchDescription(
        [robot_state_publisher, controller_start_event, spawner_event]
    )
