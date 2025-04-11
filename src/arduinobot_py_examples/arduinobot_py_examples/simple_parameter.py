import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

class SimpleParameter(Node):
    def __init__(self):
        super().__init__('simple_parameter')
        self.declare_parameter("simple_int_param", 28)
        self.declare_parameter("simple_string_param", "I'm Jebear")
        self.add_on_set_parameters_callback(self.paramChangeCallback)

    def paramChangedCallback(self, params):
        result = SetParametersResult()
        for param in params:
            if param.name == 'simple_int_param' and param.type_ == Parameter.Type.INTEGER:
                self.get_logger().info(f'simple_int_param value now {param.value}' )
                result.successful =True
            if param.name == 'simple_string_param' and param.type_ == Parameter.Type.STRING:
                self.get_logger().info(f'simple_string_param value now {param.value}' )
                result.successful =True
        return result

def main():
    rclpy.init()
    simpleParameter = SimpleParameter()
    rclpy.spin(simpleParameter)
    simpleParameter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()