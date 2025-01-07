#ifndef KUKA_IIWA_HARDWARE__KUKA_IIWA_HARDWARE_HPP_
#define KUKA_IIWA_HARDWARE__KUKA_IIWA_HARDWARE_HPP_

#include <gazebo_ros2_control/gazebo_system.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <vector>

namespace kuka_iiwa_hardware
{
class KukaIiwaHardware : public gazebo_ros2_control::GazeboSystem
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;


private:
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_efforts_;
  std::vector<double> joint_position_commands_;
};

}  // namespace kuka_iiwa_hardware

#endif  // KUKA_IIWA_HARDWARE__KUKA_IIWA_HARDWARE_HPP_
