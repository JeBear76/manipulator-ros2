import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from arduinobot_msgs.action import Fibonacci
import time


class SimpleActionServer(Node):
    def __init__(self):
        super().__init__("simple_action_server")
        self.actionServer_ = ActionServer(
            self,
            Fibonacci,
            "fibonacci",
            self.goalCallback,
        )
        self.get_logger().info("Starter simple_action_server")

    def goalCallback(self, goalHandle):
        self.get_logger().info(f"New Goal reveiced order: {goalHandle.request.order}")
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]
        for i in range(1, goalHandle.request.order):
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i - 1]
            )
            self.get_logger().info(f"Current status: {feedback_msg.partial_sequence}")
            goalHandle.publish_feedback(feedback_msg)
            time.sleep(1)
        goalHandle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result

    def cancelCallback(self, goalHandle):
        pass


def main():
    rclpy.init()
    simpleActionServer = SimpleActionServer()
    rclpy.spin(simpleActionServer)
    simpleActionServer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
