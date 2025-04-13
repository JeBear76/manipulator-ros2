#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <thread>

using std::placeholders::_1;
using namespace std::chrono_literals;

class SimpleLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit SimpleLifecycleNode(const std::string &node_name, bool intra_process_comms = false) : LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
    {
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
    {
        subscriber_ = create_subscription<std_msgs::msg::String>("example_topix", 10, std::bind(&SimpleLifecycleNode::OnMessageReceived, this, _1));
        RCLCPP_INFO(get_logger(), "simple_lifecycle_node on_configure() executed");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
    {
        subscriber_.reset();
        RCLCPP_INFO(get_logger(), "simple_lifecycle_node on_shutdown() executed");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
    {
        subscriber_.reset();
        RCLCPP_INFO(get_logger(), "simple_lifecycle_node on_cleanup() executed");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & state)
    {
        LifecycleNode::on_activate(state);
        std::this_thread::sleep_for(2s);
        RCLCPP_INFO(get_logger(), "simple_lifecycle_node on_activate() executed");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state)
    {
        LifecycleNode::on_deactivate(state);
        RCLCPP_INFO(get_logger(), "simple_lifecycle_node on_deactivate() executed");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    void OnMessageReceived(const std_msgs::msg::String::SharedPtr msg) const
    {
        auto state = get_current_state();
        if(state.label() == "active")
            RCLCPP_INFO_STREAM(get_logger(), "Received message: " << msg->data.c_str());
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor ste;
    auto simpleLifecycleNode = std::make_shared<SimpleLifecycleNode>("simple_lifecycle_node");
    ste.add_node(simpleLifecycleNode->get_node_base_interface());
    ste.spin();
    rclcpp::shutdown();
    return 0;
}