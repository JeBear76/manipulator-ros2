#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/jebear/arduinobot_ws/install/setup.bash
source /home/jebear/arduinobot_ws/cyclone.sh
ros2 launch arduinobot_bringup robot_server.launch.py
