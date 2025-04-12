#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <libserial/SerialPort.h>

using std::placeholders::_1;

class SimpleSerialTransmitter : public rclcpp::Node
{
    public:
        SimpleSerialTransmitter() : Node("simple_serial_transmitter")
        {
            declare_parameter<std::string>("port","/dev/ttyACM0");
            std::string port = get_parameter("port").as_string();

            subscriber_ = create_subscription<std_msgs::msg::String>("serial_transmitter", 10, 
                std::bind(&SimpleSerialTransmitter::OnMessageReceived, this, _1));

            arduino_.Open(port);
            arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

            RCLCPP_INFO(this->get_logger(), "CPP Arduino Transmitter ready");
        }

        ~SimpleSerialTransmitter(){
            arduino_.Close();
        }

        void OnMessageReceived(const std_msgs::msg::String::SharedPtr msg) 
        {
            RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg->data.c_str());
            arduino_.Write(msg->data);
            RCLCPP_INFO(this->get_logger(), "Message forwarded to robot");
        }

    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
        LibSerial::SerialPort arduino_;

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleSerialTransmitter>());
    rclcpp::shutdown();
    return 0;
}