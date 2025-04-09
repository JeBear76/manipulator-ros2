import rclpy
from rclpy.node import Node
from arduinobot_msgs.srv import AddTwoInts

class SimpleServiceServer(Node):
    def __init__(self):
        super().__init__('simple_service_server')
        self.service_ = self.create_service(AddTwoInts, "add_two_ints", self.serviceCallback)
        self.get_logger().info("Service Add_Two_Ints oprational")

    def serviceCallback(self, req, res):
        self.get_logger().info(f"New message received { req.a}, {req.b} ")
        res.sum = req.a + req.b
        self.get_logger().info(f"Response { res.sum }")
        return res
    
def main():
    rclpy.init()
    simpleServiceServer = SimpleServiceServer()
    rclpy.spin(simpleServiceServer)
    simpleServiceServer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    