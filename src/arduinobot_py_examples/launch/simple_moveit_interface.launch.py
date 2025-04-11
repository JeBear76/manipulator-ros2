import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    arduinobot_description_dir = get_package_share_directory('arduinobot_description')
    arduinobot_moveit_dir = get_package_share_directory('arduinobot_moveit')

    moveit_config = MoveItConfigsBuilder(
        "arduinobot", 
        package_name="arduinobot_moveit"
        ).robot_description(
            file_path=os.path.join(arduinobot_description_dir,'urdf','arduinobot.urdf.xacro'),
        ).robot_description_semantic(
            file_path="config/arduinobot.srdf"
        ).trajectory_execution(
            file_path="config/moveit_controllers.yaml"
        ).moveit_cpp(file_path="config/planning_python_api.yaml").to_moveit_configs()
    
    simple_moveit_interface = Node(
        package="arduinobot_py_examples",
        executable="simple_moveit_interface",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True}
        ]
    )

    return LaunchDescription([
        simple_moveit_interface
    ])