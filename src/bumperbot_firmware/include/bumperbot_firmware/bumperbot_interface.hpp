#ifndef BUMPERBOT_INTERFACE_HPP
#define BUMPERBOT_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <libserial/SerialPort.h>

#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

#include <std_msgs/msg/string.hpp>

#include <vector>
#include <string>

namespace bumperbot_firmware
{

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class BumperbotInterface : public hardware_interface::SystemInterface
{
public:
  BumperbotInterface();
  virtual ~BumperbotInterface();

  // Lifecycle
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  // Hardware Interface
  CallbackReturn on_init(
    const hardware_interface::HardwareInfo &hardware_info) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time &,
    const rclcpp::Duration &) override;

  hardware_interface::return_type write(
    const rclcpp::Time &,
    const rclcpp::Duration &) override;

private:
  // Serial
  LibSerial::SerialPort arduino_;
  std::string port_;

  // Joint data
  std::vector<double> velocity_commands_;
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;

  rclcpp::Time last_run_;

  // ROS node
  rclcpp::Node::SharedPtr node_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr jetson_write_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr jetson_read_pub_;
};

}  // namespace bumperbot_firmware

#endif  // BUMPERBOT_INTERFACE_HPP