import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subcriber_ = self.create_subscription(String,'example_topix', self.onMessageReceived, 10)
        self.get_logger().info('Subscribed to ''example_topix''')

    def onMessageReceived(self, msg):
        self.get_logger().info(f'Received message: {msg.data}')

def main(args=None):
    rclpy.init()
    simpleSubscriber = SimpleSubscriber()
    rclpy.spin(simpleSubscriber)
    simpleSubscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()