#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <arduinobot_msgs/srv/euler_to_quaternion.hpp>
#include <arduinobot_msgs/srv/quaternion_to_euler.hpp>
#include <tf2/utils.h>

using namespace std::placeholders;

class AnglesConversionService : public rclcpp::Node
{
public:
    AnglesConversionService() : Node("angle_conversion_service_server")
    {
        euler_to_quaternion_ = create_service<arduinobot_msgs::srv::EulerToQuaternion>("euler_to_quaternion", std::bind(&AnglesConversionService::eulerToQuaternionCallback, this, _1, _2));
        quaternion_to_euler_ = create_service<arduinobot_msgs::srv::QuaternionToEuler>("quaternion_to_euler", std::bind(&AnglesConversionService::quaternionToEulerCallback, this, _1, _2));
        RCLCPP_INFO(get_logger(), "Angle Conversion Services Operational");
    }

private:
    rclcpp::Service<arduinobot_msgs::srv::EulerToQuaternion>::SharedPtr euler_to_quaternion_;
    rclcpp::Service<arduinobot_msgs::srv::QuaternionToEuler>::SharedPtr quaternion_to_euler_;

    void eulerToQuaternionCallback(const std::shared_ptr<arduinobot_msgs::srv::EulerToQuaternion::Request> req,
                                   const std::shared_ptr<arduinobot_msgs::srv::EulerToQuaternion::Response> res)
    {
        RCLCPP_INFO_STREAM(get_logger(), "Converting Euler Angles - roll: " << req->roll << " pitch: " << req->pitch << " yaw:" << req->yaw);
        tf2::Quaternion q;
        q.setRPY(req->roll, req->pitch, req->yaw);
        res->x = q.getX();
        res->y = q.getY();
        res->z = q.getZ();
        res->w = q.getW();
        RCLCPP_INFO_STREAM(get_logger(), "Corresponding Quaternion - x:" << res->x << " y: " << res->y << " z: " << res->z << " w: " << res->w);
    }

    void quaternionToEulerCallback(const std::shared_ptr<arduinobot_msgs::srv::QuaternionToEuler::Request> req,
                                   const std::shared_ptr<arduinobot_msgs::srv::QuaternionToEuler::Response> res)
    {
        RCLCPP_INFO_STREAM(get_logger(), "Converting Quaternion - x:" << req->x << " y: " << req->y << " z: " << req->z << " w: " << req->w);
        tf2::Quaternion q(req->x,req->y,req->z,req->w);
        tf2::Matrix3x3 m(q);
        m.getRPY(res->roll,res->pitch,res->yaw);
        RCLCPP_INFO_STREAM(get_logger(), "Corresponding Euler Angles - roll: " << res->roll << " pitch: " << res->pitch << " yaw:" << res->yaw);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto anglesConversionService = std::make_shared<AnglesConversionService>();
    rclcpp::spin(anglesConversionService);
    rclcpp::shutdown();
    return 0;
}