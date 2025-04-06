#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

using std::placeholders::_1;

class SimpleSubscriber : public rclcpp::Node
{
    public:
        SimpleSubscriber() : Node("simple_subscriber")
        {
            subscriber_ = create_subscription<std_msgs::msg::String>("example_topix", 10, 
                std::bind(&SimpleSubscriber::OnMessageReceived, this, _1));
        }

        void OnMessageReceived(const std_msgs::msg::String::SharedPtr msg) const
        {
            RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg->data.c_str());
        }

    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleSubscriber>());
    rclcpp::shutdown();
    return 0;
}