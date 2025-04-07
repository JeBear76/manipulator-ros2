# Arduino Bot Manipulator Arm Course 

## useful URLs
[Install Gazebo on WSL](https://aleksandarhaber.com/how-to-install-gazebo-harmonic-in-windows-by-using-wsl-and-ubuntu-24-04-and-how-to-run-mobile-robot-simulation/)  
[Share USB device with WSL from Windows](https://learn.microsoft.com/en-us/windows/wsl/connect-usb)
## quick notes

### Creating the environment
```
mkdir -p <environemtn name>/src
cd <environment name>
colcon build
```
### Creating a package:  
```
ros2 pkg create --build_type <build type> <package name>
```
#### Build types:
- ament_python
- ament_cmake

### Activating the environment:
```
source ./install/setup.bash
```
Checking
```
ros2 pkg list
```

### Working with topics
```
ros2 topic list
ros2 topic echo <name of topic>
ros2 topic info <name of topic> [--verbose]
ros2 topic hz <name of topic>
ros2 topic pub <name of topic> <message type> <body>
```

## Packages Description

### arduinobot_XX_examples
Some basic tutorial and references packages in Python and C++
#### simple_publisher / simple_subscriber
This script provides a basic example of implementing a publisher in a publish-subscribe messaging system. It demonstrates how to send messages to a specific topic, enabling real-time communication with subscribers.

#### simple_parameter
This script demonstrates how to use ROS 2 parameters.  
It includes examples of declaring parameters, setting default values, and handling parameter changes dynamically through a callback function. 

### arduinobot_description
This package contains the Universal Robot Definition Format file of the robot and the STL files
- arduinobot.urdf.xacro
    The robot model
- arduinobot_colors.xacro
    The robot colors for gazebo
- arduinobot_gazebo.xacro 
    The gazebo configurations
- arduinobot_ros2_control.xacro
    The controller setup

```
ros2 launch urdf_tutorial display.launch.py model:=/home/jebear/arduinobot_ws/src/arduinobot_description/urdf/arduinobot.urdf.xacro
```
#### Launch file command
```
ros2 launch arduinobot_description display.launch.py
```

### arduinobot_controller
The package that contains the controller configurations for the robot