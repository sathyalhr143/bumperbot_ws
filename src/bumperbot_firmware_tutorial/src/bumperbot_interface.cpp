#include "bumperbot_firmware_tutorial/bumperbot_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>


namespace bumperbot_firmware_tutorial
{
BumperbotInterface::BumperbotInterface()
{

}

BumperbotInterface::~BumperbotInterface()
{
    if(arduino_.IsOpen())
    {
        try
        {
            arduino_.Close();
        }
        catch(...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("BumperbotInterface"), 
                "Something went wrong while closing the connection to the serial port " << port_);

        }
    }
}   

CallbackReturn BumperbotInterface::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
   CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
   if(result != CallbackReturn::SUCCESS)
   {
       return result;
   }

   try
   {
        port_ = info_.hardware_parameters.at("port");
   }
   catch(const std::out_of_range & e)
   {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("BumperbotInterface"), 
            "No serial port provided! Aborting ");
        return CallbackReturn::FAILURE;
   }

    velocity_command_.reserve(info_.joints.size());
    velocity_state_.reserve(info_.joints.size());
    position_state_.reserve(info_.joints.size());
    last_run_ = rclcpp::Clock().now();

    return CallbackReturn::SUCCESS;
}
std::vector<hardware_interface::StateInterface>  BumperbotInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for(size_t i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints.at(i).name, 
            hardware_interface::HW_IF_POSITION, &position_state_.at(i)));
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints.at(i).name, 
            hardware_interface::HW_IF_VELOCITY, &velocity_state_.at(i)));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> BumperbotInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for(size_t i = 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints.at(i).name, 
            hardware_interface::HW_IF_VELOCITY, &velocity_command_.at(i)));
    }

    return command_interfaces;
}


CallbackReturn BumperbotInterface::on_activate(const rclcpp_lifecycle::State & previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"), "Activating Robot Hardware...");
    velocity_command_ = {0.0, 0.0};
    velocity_state_ = {0.0, 0.0};
    position_state_ = {0.0, 0.0};

    try
    {
        arduino_.Open(port_);
        arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    }
    catch(...)
    {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("BumperbotInterface"), 
            "Something went wrong while interacting with the serial port " << port_);
        return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"), 
        "Hardware started successfully on port. Ready to take commands ");
    return CallbackReturn::SUCCESS;

}


CallbackReturn BumperbotInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"), 
        "Stopping Robot Hardware...");
    if(arduino_.IsOpen())
    {
        try
        {
            arduino_.Close();
        }
        catch(...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("BumperbotInterface"), 
            "Something went wrong while closing the port " << port_);
        return CallbackReturn::FAILURE;
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"), "Hardware stopped");
  return CallbackReturn::SUCCESS;
        
}

hardware_interface::return_type BumperbotInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    if(arduino_.IsDataAvailable())
    {
        auto dt = (rclcpp::Clock().now() - last_run_).seconds();
        std::string message;
        arduino_.ReadLine(message);
        // Parse the message and update the state variables
        // For example, if the message is in the format "pos1,pos2,pos3,pos4,vel1,vel2,vel3,vel4"

        std::stringstream ss(message);
        std::string res;
        int multiplier = 1;
        while(std::getline(ss, res, ','))
        {
            multiplier = res.at(1) == 'p' ? 1 : -1; // Assuming the message has 'p' for positive and 'n' for negative values
            if(res.at(0) == 'r')
            {
                velocity_state_.at(0) = multiplier * std::stod(res.substr(2, res.size()));
                position_state_.at(0) += velocity_state_.at(0) * dt;
            }
            else if(res.at(0) == 'l')
            {
                velocity_state_.at(1) = multiplier * std::stod(res.substr(2, res.size()));
                position_state_.at(1) += velocity_state_.at(1) * dt;
            }
        }
        last_run_ = rclcpp::Clock().now();
    }
    return hardware_interface::return_type::OK;
}
    
hardware_interface::return_type BumperbotInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period) 
{
    std::stringstream message_stream;
    char right_wheel_sign = velocity_command_.at(0) >= 0 ? 'p' : 'n';
    char left_wheel_sign = velocity_command_.at(1) >= 0 ? 'p' : 'n';
    std::string compensate_zeros_right = "";
    std::string compensate_zeros_left = "";

    if(std::abs(velocity_command_.at(0)) < 10.0)
    {
        compensate_zeros_right  = "0";
    }
    else
    {
        compensate_zeros_right  = "";
    }

    if(std::abs(velocity_command_.at(1)) < 10.0)
    {
        compensate_zeros_left  = "0";
    }
    else
    {
        compensate_zeros_left  = "";
    }

    message_stream << std::fixed << std::setprecision(2) << "r" << right_wheel_sign << compensate_zeros_right << std::abs(velocity_command_.at(0)) << 
        ",l" << left_wheel_sign << compensate_zeros_left << std::abs(velocity_command_.at(1)) << ",";

    try
    {
        arduino_.Write(message_stream.str());
    }
    catch(...)
    {   
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("BumperbotInterface"), 
            "Something went wrong while writing to the serial port " << message_stream.str() << port_);
        return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
}
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(bumperbot_firmware_tutorial::BumperbotInterface, hardware_interface::SystemInterface)
