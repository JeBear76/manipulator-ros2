import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from arduinobot_msgs.action import Fibonacci
import time

class SimpleActionClient(Node):
    def __init__(self):
        super().__init__("simple_action_client")
        self.actionClient = ActionClient(
            self,
            Fibonacci,
            "fibonacci")
        
        self.actionClient.wait_for_server()

        self.goal = Fibonacci.Goal()
        self.goal.order = 12
        self.future = self.actionClient.send_goal_async(self.goal, self.feedbackCallback)
        self.future.add_done_callback(self.acceptedCallback)

    def acceptedCallback(self, future):
        goalHandle = future.result()
        if not goalHandle.accepted:
            self.get_logger().info("Server rejected goal :-(")
            return
        self.get_logger().info("Goal accepted :-D")
        self.future = goalHandle.get_result_async()
        self.future.add_done_callback(self.completedCallback)

    def completedCallback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result - {result.sequence}")
        rclpy.shutdown()

    def feedbackCallback(self, feedback_msg):
        self.get_logger().info(f"Feedback - {feedback_msg.feedback.partial_sequence}")


def main():
    rclpy.init()
    simpleActionClient = SimpleActionClient()
    rclpy.spin(simpleActionClient)
    


if __name__ == "__main__":
    main()