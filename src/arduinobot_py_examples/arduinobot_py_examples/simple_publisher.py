import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'example_topix', 10)
        self.frequency_ = 1.0  # seconds
        self.get_logger().info(f'Publishing at {self.frequency_} Hz')
        self.timer_ = self.create_timer(self.frequency_, self.publishMessage)
        self.counter_ = 0

    def publishMessage(self):
        msg = String()
        msg.data = f'Hello from ROS2 - {self.counter_}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter_ += 1

def main(args=None):
    rclpy.init()
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.detroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
