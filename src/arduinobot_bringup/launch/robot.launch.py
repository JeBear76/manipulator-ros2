import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    port_args = DeclareLaunchArgument(
        name="port",
        description="COM Port for arduino",
        default_value="/dev/ttyACM0"
    )
    
    port = LaunchConfiguration("port")
    
    controller = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("arduinobot_controller"),
                "launch",
                "controller.launch.py"
            ),
            launch_arguments={"is_sim": "False"
                              ,"port": port}.items()
        )
    
    moveit = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("arduinobot_moveit"),
                "launch",
                "moveit.launch.py"
            ),
            launch_arguments={"is_sim": "False"}.items()
        )
    
    remote_interface = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("arduinobot_remote"),
                "launch",
                "remote_interface.launch.py"
            ),
            launch_arguments={"is_sim": "False"}.items()
        )
    
    return LaunchDescription([
        port_args,
        controller,
        moveit,
        remote_interface,
    ])