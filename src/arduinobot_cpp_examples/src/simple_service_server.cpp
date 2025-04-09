#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <arduinobot_msgs/srv/add_two_ints.hpp>

using namespace std::placeholders;

class SimpleServiceServer : public rclcpp::Node
{
    public:
    SimpleServiceServer() : Node("simple_service_server"){
        service_ = create_service<arduinobot_msgs::srv::AddTwoInts>("add_two_ints", std::bind(&SimpleServiceServer::serviceCallback, this, _1, _2));
        RCLCPP_INFO(get_logger(), "Service add_two_ints operational");
    }

    private:
    rclcpp::Service<arduinobot_msgs::srv::AddTwoInts>::SharedPtr service_;

    void serviceCallback(const std::shared_ptr<arduinobot_msgs::srv::AddTwoInts::Request> req,
                        const std::shared_ptr<arduinobot_msgs::srv::AddTwoInts::Response> res)
    {
        RCLCPP_INFO_STREAM(get_logger(), "Message received a: " << req->a << " b: " << req->b);
        res->sum =  req->a + req->b;
        RCLCPP_INFO_STREAM(get_logger(), "Response " << res->sum);
    }
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto simpleServiceServer = std::make_shared<SimpleServiceServer>();
    rclcpp::spin(simpleServiceServer);
    rclcpp::shutdown();
    return 0;

}