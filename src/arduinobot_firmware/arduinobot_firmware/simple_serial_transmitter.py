#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SimpleSerialTransmitter(Node):
    def __init__(self):
        super().__init__('simple_serial_transmitter')
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)

        self.port = self.get_parameter("port").value
        self.baud_rate = self.get_parameter("baud_rate").value

        self.subcriber_ = self.create_subscription(String,'serial_transmitter', self.onMessageReceived, 10)
        self.arduino_ = serial.Serial(port=self.port,baudrate=self.baud_rate, timeout=0.1)

        self.get_logger().info('Subscribed to serial_transmitter')

    def onMessageReceived(self, msg):
        self.get_logger().info(f'Received message: {msg.data}')
        self.arduino_.write(msg.data.encode("utf-8"))
        self.get_logger().info(f'Message forwarded to robot')

def main(args=None):
    rclpy.init()
    simpleSerialTransmitter = SimpleSerialTransmitter()
    rclpy.spin(simpleSerialTransmitter)
    simpleSerialTransmitter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()