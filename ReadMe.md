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
```

## Packages Description

### arduinobot_XX_examples
Some basic tutorial and references packages in Python and C++
#### simple_publisher.py

This script provides a basic example of implementing a publisher in a publish-subscribe messaging system. It demonstrates how to send messages to a specific topic, enabling real-time communication with subscribers.
