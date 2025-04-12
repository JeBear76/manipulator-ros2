import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    is_sim_args = DeclareLaunchArgument(
        name="is_sim",
        default_value='True',
        description='Are we working in the simulation'
        )

    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="False"
    )
    
    is_sim = LaunchConfiguration("is_sim")
    use_python = LaunchConfiguration("use_python")

    arduinobot_description_dir = get_package_share_directory('arduinobot_description')

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
    
    task_server_node_py = Node(
        package="arduinobot_remote",
        executable="task_server.py",
        condition=IfCondition(use_python),
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": is_sim}
        ]
    )

    task_server_node = Node(
        package="arduinobot_remote",
        executable="task_server_node",
        condition=UnlessCondition(use_python),
        parameters=[
            {"use_sim_time": is_sim}
        ]
    )

    return LaunchDescription([
        is_sim_args,
        use_python_arg,
        task_server_node_py,
        task_server_node
    ])