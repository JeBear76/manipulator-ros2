#!/usr/bin/env python3
## Required os packages
# sudo apt install ros-jazzy-tf-transformations
# sudo apt install python3-transforms3d

import rclpy
from rclpy.node import Node
from arduinobot_msgs.srv import EulerToQuaternion, QuaternionToEuler
from tf_transformations import quaternion_from_euler, euler_from_quaternion

class AnglesConverter(Node):
    def __init__(self):
        super().__init__('angle_conversion_service_server')
        self.euler_to_quartenion_ = self.create_service(EulerToQuaternion, "euler_to_quaternion", self.eulerToQuarternionCallback)
        self.quartenion_to_euler_ = self.create_service(QuaternionToEuler, "quaternion_to_euler", self.quarternionToEulerCallback)
        self.get_logger().info("Angle Conversion Services Operational")

    def eulerToQuarternionCallback(self, req, res):
        self.get_logger().info(f"Converting Euler Angles - roll:{req.roll} pitch:{req.pitch} yaw:{req.yaw}")
        (res.x, res.y, res.z, res.w) = quaternion_from_euler(req.roll, req.pitch, req.yaw)
        self.get_logger().info(f"Corresponding Quaternion - x:{res.x} y:{res.y} z:{res.z} w:{res.w}")
        return res
        
    def quarternionToEulerCallback(self, req, res):
        self.get_logger().info(f"Converting Quaternion - x:{req.x} y:{req.y} z:{req.z} w:{req.w}")
        (res.roll, res.pitch, res.yaw) = euler_from_quaternion([req.x, req.y, req.z, req.w])
        self.get_logger().info(f"Corresponding Euler Angles - roll:{res.roll} pitch:{res.pitch} yaw:{res.yaw}")
        return res
    
def main():
    rclpy.init()
    angles_converter = AnglesConverter()
    rclpy.spin(angles_converter)
    angles_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    