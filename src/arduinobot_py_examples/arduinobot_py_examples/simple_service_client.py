import rclpy
from rclpy.node import Node
from arduinobot_msgs.srv import AddTwoInts
import sys

class SimpleServiceClient(Node):
    def __init__(self, a, b):
        super().__init__('simple_service_client')
        
        self.client_ = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger("Waiting for add_two_ints service to be ready...")

        self.req = AddTwoInts.Request()
        self.req.a = a
        self.req.b = b

        self.future_ = self.client_.call_async(self.req)

        self.future_.add_done_callback(self.responseCallback)

    def responseCallback(self, future):
        print(future.result())
        sum = future.result().sum
        self.get_logger().info(f'Response: {sum}')



def main():
    if len(sys.argv) != 3:
        print("Wrong number of arguments")
        return -1
        
    rclpy.init()
    simpleServiceClient = SimpleServiceClient(int(sys.argv[1]),int(sys.argv[2]))
    rclpy.spin(simpleServiceClient)
    simpleServiceClient.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()