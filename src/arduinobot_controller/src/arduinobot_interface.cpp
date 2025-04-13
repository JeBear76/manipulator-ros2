#include <arduinobot_controller/arduinobot_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace arduinobot_controller
{
    std::string compensateZeros(int value)
    {
        std::string compensate_zeros = "";
        if (value < 10)
            compensate_zeros = "00";
        else if (value < 100)
            compensate_zeros = "0";

        return compensate_zeros;
    }
    ArduinobotInterface::ArduinobotInterface()
    {
    }

    ArduinobotInterface::~ArduinobotInterface()
    {
        if (arduino_.IsOpen())
        {
            try
            {
                arduino_.Close();
            }
            catch (...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("ArduinobotInterface"), "Arduinobot HW Interface Destruction - Fatal error - connection could not be closed on port: " << port_);
            }
        }
    }

    CallbackReturn ArduinobotInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
    {
        CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
        if (result != CallbackReturn::SUCCESS)
        {
            RCLCPP_FATAL(rclcpp::get_logger("ArduinobotInterface"), "Arduinobot HW Interface initialization Failed - Base initialization failed.");
            return result;
        }

        try
        {
            port_ = info_.hardware_parameters.at("port");
        }
        catch (const std::out_of_range &e)
        {
            RCLCPP_FATAL(rclcpp::get_logger("ArduinobotInterface"), "Arduinobot HW Interface initialization Failed - Missing serial port.");
            return CallbackReturn::FAILURE;
        }

        position_commands_.reserve(info_.joints.size());
        prev_position_commands_.reserve(info_.joints.size());
        position_states_.reserve(info_.joints.size());

        RCLCPP_INFO(rclcpp::get_logger("ArduinobotInterface"), "Arduinobot HW Interface initialized.");
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> ArduinobotInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> ArduinobotInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
        }
        return command_interfaces;
    }

    CallbackReturn ArduinobotInterface::on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("ArduinobotInterface"), "Activating Arduinobot HW Interface.");
        position_commands_ = {0.0, 0.0, 0.0, 0.0};
        prev_position_commands_ = {0.0, 0.0, 0.0, 0.0};
        position_states_ = {0.0, 0.0, 0.0, 0.0};

        try
        {
            arduino_.Open(port_);
            arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        }
        catch (...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("ArduinobotInterface"), "Arduinobot HW Interface activation Failed - Could not open port: " << port_);
            return CallbackReturn::FAILURE;
        }

        RCLCPP_INFO(rclcpp::get_logger("ArduinobotInterface"), "Arduinobot HW Interface activated!");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ArduinobotInterface::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("ArduinobotInterface"), "Deactivating Arduinobot HW Interface.");
        try
        {
            arduino_.Close();
        }
        catch (...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("ArduinobotInterface"), "Arduinobot HW Interface activation Failed - Could not close port: " << port_);
            return CallbackReturn::FAILURE;
        }
        RCLCPP_INFO(rclcpp::get_logger("ArduinobotInterface"), "Arduinobot HW Interface deactivated!");
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type ArduinobotInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
    {
        position_states_ = position_commands_;
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type ArduinobotInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
    {
        if (position_commands_ == prev_position_commands_)
        {
            return hardware_interface::return_type::OK;
        }

        std::string msg;
        
        int base = static_cast<int>(((position_commands_.at(0) + (M_PI / 2)) * 180) / M_PI);
        msg.append("b");
        msg.append(compensateZeros(base));
        msg.append(std::to_string(base));
        msg.append(",");

        int shoulder = static_cast<int>(((position_commands_.at(1) + (M_PI / 2)) * 180) / M_PI);
        msg.append("s");
        msg.append(compensateZeros(shoulder));
        msg.append(std::to_string(shoulder));
        msg.append(",");

        int elbow = static_cast<int>(((position_commands_.at(2) + (M_PI / 2)) * 180) / M_PI);
        msg.append("e");
        msg.append(compensateZeros(elbow));
        msg.append(std::to_string(elbow));
        msg.append(",");
        
        int gripper = static_cast<int>(((position_commands_.at(3) + (M_PI / 2)) * 180) / M_PI);
        msg.append("g");
        msg.append(compensateZeros(gripper));
        msg.append(std::to_string(gripper));
        msg.append(",");

        try{
            arduino_.Write(msg);            
        }catch(...){
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("ArduinobotInterface"), "Arduino communication error - msg: [" << msg << "] - port: " << port_);    
            return hardware_interface::return_type::ERROR;
        }

        prev_position_commands_ = position_commands_;

        return hardware_interface::return_type::OK;
    }
}

PLUGINLIB_EXPORT_CLASS(arduinobot_controller::ArduinobotInterface, hardware_interface::SystemInterface);