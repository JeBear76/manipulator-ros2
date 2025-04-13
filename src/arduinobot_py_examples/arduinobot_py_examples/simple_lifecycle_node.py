import time
import rclpy
import rclpy.executors
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from std_msgs.msg import String


class SimpleLifeCycleNode(Node):
    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.subcriber_ = self.create_subscription(
            String, "example_topix", self.msgCallback, 10
        )
        self.get_logger().info('simple_lifecycle_node on_configure() executed')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.destroy_subscription(self.subcriber_)
        self.get_logger().info('simple_lifecycle_node on_cleanup() executed')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('simple_lifecycle_node on_activate() executed')
        time.sleep(2)
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('simple_lifecycle_node on_deactivate() executed')
        return super().on_deactivate(state)

    def msgCallback(self, msg):
        if self._state_machine.current_state[1] == "active":
            self.get_logger().info(f'Received message: {msg.data}')


def main(args=None):
    rclpy.init()
    executor = rclpy.executors.SingleThreadedExecutor()
    simpleLifeCycleNode = SimpleLifeCycleNode("simple_lifecycle_node")
    executor.add_node(simpleLifeCycleNode)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        simpleLifeCycleNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()