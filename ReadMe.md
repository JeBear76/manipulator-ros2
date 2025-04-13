# Arduino Bot Manipulator Arm Course 

## useful URLs
[Install Gazebo on WSL](https://aleksandarhaber.com/how-to-install-gazebo-harmonic-in-windows-by-using-wsl-and-ubuntu-24-04-and-how-to-run-mobile-robot-simulation/)  
[Share USB device with WSL from Windows](https://learn.microsoft.com/en-us/windows/wsl/connect-usb)

## additional packages
### For MOVEIT2
```
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
sudo apt-get install ros-jazzy-moveit* ros-jazzy-ompl*
```
add `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` to ~/.bashrc  

### For Angle Conversion service in utils
```
sudo apt install ros-jazzy-tf-transformations
sudo apt install python3-transforms3d
```

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

```
ros2 launch urdf_tutorial display.launch.py model:=/home/jebear/arduinobot_ws/src/arduinobot_description/urdf/arduinobot.urdf.xacro
```
#### Launch file command
```
ros2 launch arduinobot_description display.launch.py
```

### WSL USB Commands
Powershell
```
usbipd list
usbipd bind --busid <busid>
usbipd attach --wsl --busid <busid>

usbipd detach --busid <busid>
```
Bash
```
lsusb
udevadm info --name=/dev/ttyACM0 --attribute-walk
```

### Life cycle nodes
```
ros2 lifecycle nodes
ros2 lifecycle list /simple_lifecycle_node
ros2 lifecycle set /simple_lifecycle_node <state>
```