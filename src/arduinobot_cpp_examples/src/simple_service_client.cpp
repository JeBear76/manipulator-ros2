#include <rclcpp/rclcpp.hpp>
#include <arduinobot_msgs/srv/add_two_ints.hpp>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

class SimpleServiceClient: public rclcpp::Node{
    public:
    SimpleServiceClient(int a, int b) : Node("simple_service_client"){
        client_ = create_client<arduinobot_msgs::srv::AddTwoInts>("add_two_ints");
        
        auto request = std::make_shared<arduinobot_msgs::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        while (!client_->wait_for_service(1s))
        {
            RCLCPP_ERROR(get_logger(), "Waiting for service");
            if(!rclcpp::ok()){
                RCLCPP_ERROR(get_logger(), "Interrupted while waiting for service to start");
                return;
            }
        }

        auto result = client_->async_send_request(request, std::bind(&SimpleServiceClient::responseCallback, this, _1));
    }

    private:
    rclcpp::Client<arduinobot_msgs::srv::AddTwoInts>::SharedPtr client_;

    void responseCallback(rclcpp::Client<arduinobot_msgs::srv::AddTwoInts>::SharedFuture future){
        if (future.valid()){
            RCLCPP_INFO_STREAM(get_logger(), "Response: " << future.get()->sum);
            return;
        }
        RCLCPP_ERROR(get_logger(), "Invalid Response from the service");
    }

};

int main(int argc, char* argv[]){
    if(argc != 3)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Wrong number of arguments");
        return -1;
    }
    rclcpp::init(argc, argv);
    auto simpleServiceClient = std::make_shared<SimpleServiceClient>(atoi(argv[1]),atoi(argv[2]));
    rclcpp::spin(simpleServiceClient);
    rclcpp::shutdown();
    return 0;
}