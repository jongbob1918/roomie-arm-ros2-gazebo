
#include "roomiearm_hardware/roomiearm_hardware_interface.hpp"

#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace roomiearm_hardware
{

CallbackReturn RoomieArmHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  
  serial_port_name_ = info_.hardware_parameters["serial_port_name"];
  serial_baud_rate_ = std::stoi(info_.hardware_parameters["serial_baud_rate"]);

  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);

  return CallbackReturn::SUCCESS;
}

CallbackReturn RoomieArmHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RoomieArmHardwareInterface"), "Configuring hardware interface...");
  io_context_ = std::make_unique<drivers::common::IoContext>(1);
  serial_driver_ = std::make_unique<drivers::serial_driver::SerialDriver>(*io_context_);
  RCLCPP_INFO(rclcpp::get_logger("RoomieArmHardwareInterface"), "Hardware interface configured successfully.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RoomieArmHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RoomieArmHardwareInterface"), "Activating hardware interface...");
  try
  {
    drivers::serial_driver::SerialPortConfig port_config(
      serial_baud_rate_, drivers::serial_driver::FlowControl::NONE,
      drivers::serial_driver::Parity::NONE, drivers::serial_driver::StopBits::ONE);
    serial_driver_->init_port(serial_port_name_, port_config);
    if (!serial_driver_->port()->is_open())
    {
      serial_driver_->port()->open();
    }
    RCLCPP_INFO(rclcpp::get_logger("RoomieArmHardwareInterface"), "Serial port opened successfully: %s", serial_port_name_.c_str());
  }
  catch (const std::exception & e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("RoomieArmHardwareInterface"), "Error opening serial port: %s", e.what());
    return CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(rclcpp::get_logger("RoomieArmHardwareInterface"), "Hardware interface activated successfully.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RoomieArmHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RoomieArmHardwareInterface"), "Deactivating hardware interface...");
  if (serial_driver_ && serial_driver_->port()->is_open())
  {
    serial_driver_->port()->close();
  }
  RCLCPP_INFO(rclcpp::get_logger("RoomieArmHardwareInterface"), "Hardware interface deactivated.");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RoomieArmHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RoomieArmHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }
  return command_interfaces;
}

hardware_interface::return_type RoomieArmHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!serial_driver_ || !serial_driver_->port()->is_open()) return hardware_interface::return_type::ERROR;

  try
  {
    std::vector<uint8_t> buffer;
    serial_driver_->port()->receive(buffer);
    std::string response(buffer.begin(), buffer.end());

    size_t start_pos = response.find("<S:");
    size_t end_pos = response.find(">", start_pos);

    if (start_pos != std::string::npos && end_pos != std::string::npos)
    {
      std::string data = response.substr(start_pos + 3, end_pos - (start_pos + 3));
      std::stringstream ss(data);
      std::string value_str;

      for (uint i = 0; i < info_.joints.size(); ++i)
      {
        if (!std::getline(ss, value_str, ',')) continue;
        hw_positions_[i] = std::stod(value_str) * M_PI / 180.0;
        hw_velocities_[i] = 0.0;
      }
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("RoomieArmHardwareInterface"), "Error during serial read: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoomieArmHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!serial_driver_ || !serial_driver_->port()->is_open()) return hardware_interface::return_type::ERROR;

  std::stringstream command_ss;
  command_ss << "<M:";

  for (uint i = 0; i < info_.joints.size(); ++i)
  {
    // [수정] static_cat -> static_cast
    int angle_deg = static_cast<int>(round(hw_commands_[i] * 180.0 / M_PI));
    command_ss << angle_deg << (i < info_.joints.size() - 1 ? "," : "");
  }
  command_ss << ">\n";

  try
  {
    std::string command_str = command_ss.str();
    std::vector<uint8_t> buffer(command_str.begin(), command_str.end());
    serial_driver_->port()->send(buffer);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("RoomieArmHardwareInterface"), "Error during serial write: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace roomiearm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  roomiearm_hardware::RoomieArmHardwareInterface, hardware_interface::SystemInterface)