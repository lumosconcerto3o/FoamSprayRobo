#include "kuka_iiwa_hardware/kuka_iiwa_hardware.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <gazebo_ros2_control/gazebo_system.hpp>

namespace kuka_iiwa_hardware
{

hardware_interface::CallbackReturn KukaIiwaHardware::on_init(
  const hardware_interface::HardwareInfo & info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }

    joint_positions_.resize(info_.joints.size(), 0.0);
    joint_velocities_.resize(info_.joints.size(), 0.0);
    joint_efforts_.resize(info_.joints.size(), 0.0);
    joint_position_commands_.resize(info_.joints.size(), 0.0);

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.joints[i].name,
          hardware_interface::HW_IF_POSITION,
          &joint_positions_[i]));
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.joints[i].name,
          hardware_interface::HW_IF_VELOCITY,
          &joint_velocities_[i]));
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.joints[i].name,
          hardware_interface::HW_IF_EFFORT,
          &joint_efforts_[i]));
    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.joints[i].name,
          hardware_interface::HW_IF_POSITION,
          &joint_position_commands_[i]));
    }
    return command_interfaces;
  }

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    for (size_t i = 0; i < joint_position_commands_.size(); i++) {
      joint_position_commands_[i] = joint_positions_[i];
    }
    RCLCPP_INFO(rclcpp::get_logger("KukaIiwaHardware"), "Successfully activated!");
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    // Read actual joint states from Gazebo
    for (size_t i = 0; i < info_.joints.size(); i++) {
      auto joint = parent_model_->GetJoint(info_.joints[i].name);
      if (joint) {
        joint_positions_[i] = joint->Position(0);
        joint_velocities_[i] = joint->GetVelocity(0);
        joint_efforts_[i] = joint->GetForce(0);
      }
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    // Write commanded joint positions to Gazebo
    for (size_t i = 0; i < info_.joints.size(); i++) {
      auto joint = parent_model_->GetJoint(info_.joints[i].name);
      if (joint) {
        joint->SetPosition(0, joint_position_commands_[i], true);
      }
    }
    return hardware_interface::return_type::OK;
  }

private:
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_efforts_;
  std::vector<double> joint_position_commands_;
};

}  // namespace kuka_iiwa_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  kuka_iiwa_hardware::KukaIiwaHardware,
  gazebo_ros2_control::GazeboSystem)
