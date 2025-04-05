#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

class SimplePublisher : public rclcpp::Node
{
    public:
        SimplePublisher() : Node("simple_publisher")
        {
            publisher_ = create_publisher<std_msgs::msg::String>("example_topix", 10);
            timer_ = create_wall_timer(1s, std::bind(&SimplePublisher::publishMessage, this));
            
            RCLCPP_INFO(get_logger(), "Publishing at 1 Hz");
        }

        void publishMessage()
        {
            auto message = std_msgs::msg::String();
            message.data = "Hello from ROS2 - " + std::to_string(counter_++);
            publisher_->publish(message);
            RCLCPP_INFO(get_logger(), "Publishing: '%s'", message.data.c_str());
        }

    private:
        unsigned int counter_ = 0;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);
    auto simplePublisher = std::make_shared<SimplePublisher>();
    rclcpp::spin(simplePublisher);
    rclcpp::shutdown();
    return 0;
}