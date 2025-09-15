
#ifndef ROOMIEARM_HARDWARE_INTERFACE_HPP_
#define ROOMIEARM_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <serial_driver/serial_driver.hpp>

namespace roomiearm_hardware
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class RoomieArmHardwareInterface : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // SerialDriver 객체와 관련 컴포넌트
  std::unique_ptr<drivers::common::IoContext> io_context_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
  std::string serial_port_name_;
  int32_t serial_baud_rate_;

  // Storage for joint states and commands
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
};
}  // namespace roomiearm_hardware

#endif  // ROOMIEARM_HARDWARE_INTERFACE_HPP_