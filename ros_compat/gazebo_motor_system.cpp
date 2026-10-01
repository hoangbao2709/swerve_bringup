// Retain native steering, command storage and measured read. Drive motor
// constraints enforce the original velocity targets throughout contact solving.
#include <cmath>
#include <optional>
#include <string>
#include <vector>
#include "gazebo_ros2_control/gazebo_system_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "motor_constraint.hpp"

namespace swerve_bringup
{
class GazeboMotorSystem : public gazebo_ros2_control::GazeboSystemInterface
{
  struct Motor
  {
    std::string name;
    gazebo::physics::JointPtr joint;
    double effort, velocity, passive_force;
    bool active = false;
    std::optional<hardware_interface::ReadOnlyHandle> command;
  };
  std::vector<Motor> motors_;
  std::vector<double> targets_;
  std::unique_ptr<pluginlib::ClassLoader<gazebo_ros2_control::GazeboSystemInterface>> loader_;
  std::shared_ptr<gazebo_ros2_control::GazeboSystemInterface> upstream_;

  hardware_interface::return_type fail_zero()
  {
    for (auto & motor : motors_) {
      if (motor.active) {
        motor.joint->SetParam("vel", 0, 0.0);
        motor.joint->SetParam("fmax", 0, motor.effort);
      }
    }
    return hardware_interface::return_type::ERROR;
  }

public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    if (!upstream_) {return hardware_interface::CallbackReturn::ERROR;}
    auto result = SystemInterface::on_init(info);
    if (result != hardware_interface::CallbackReturn::SUCCESS) {return result;}
    return upstream_->on_init(info);
  }
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {return upstream_->export_state_interfaces();}
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
  {return upstream_->on_activate(state);}
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    bool restored = true;
    for (auto & motor : motors_) {
      motor.active = false;
      const bool zero = motor.joint->SetParam("vel", 0, 0.0);
      const bool passive = motor.joint->SetParam("fmax", 0, motor.passive_force);
      restored = zero && passive && restored;
    }
    auto result = upstream_->on_deactivate(state);
    return restored ? result : hardware_interface::CallbackReturn::ERROR;
  }
  hardware_interface::return_type read(const rclcpp::Time & time,
    const rclcpp::Duration & period) override
  {return upstream_->read(time, period);}

  bool initSim(rclcpp::Node::SharedPtr & node, gazebo::physics::ModelPtr model,
    const hardware_interface::HardwareInfo & info, sdf::ElementPtr sdf) override
  {
    if (model->GetWorld()->Physics()->GetType() != "ode") {
      RCLCPP_ERROR(node->get_logger(), "GazeboMotorSystem requires ODE; refusing fallback");
      return false;
    }
    try {
      loader_ = std::make_unique<pluginlib::ClassLoader<gazebo_ros2_control::GazeboSystemInterface>>(
        "gazebo_ros2_control", "gazebo_ros2_control::GazeboSystemInterface");
      upstream_ = loader_->createSharedInstance("gazebo_ros2_control/GazeboSystem");
    } catch (const pluginlib::PluginlibException & error) {
      RCLCPP_ERROR(node->get_logger(), "Original GazeboSystem unavailable: %s", error.what());
      return false;
    }
    if (!upstream_->initSim(node, model, info, sdf)) {return false;}
    for (const auto & resource : info.joints) {
      for (const auto & interface : resource.command_interfaces) {
        if (interface.name != "velocity") {continue;}
        auto joint = model->GetJoint(resource.name);
        if (!joint) {return false;}
        const double effort = joint->GetEffortLimit(0), velocity = joint->GetVelocityLimit(0);
        if (!std::isfinite(effort) || effort <= 0 || !std::isfinite(velocity) || velocity <= 0) {
          RCLCPP_ERROR(node->get_logger(), "Drive motor requires finite positive URDF limits");
          return false;
        }
        const double passive_force = joint->GetParam("fmax", 0);
        if (!std::isfinite(passive_force) || passive_force < 0) {return false;}
        motors_.push_back({resource.name, joint, effort, velocity, passive_force});
        RCLCPP_INFO(node->get_logger(), "ODE_MOTOR_CONFIG joint=%s effort_limit=%.3f velocity_limit=%.3f",
          resource.name.c_str(), effort, velocity);
      }
    }
    targets_.resize(motors_.size(), 0.0);
    return true;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    auto interfaces = upstream_->export_command_interfaces();
    for (const auto & interface : interfaces) {
      for (auto & motor : motors_) {
        if (interface.get_name() == motor.name + "/velocity") {
          // Read-only view of ORIGINAL command storage; not a second writer.
          motor.command.emplace(static_cast<const hardware_interface::ReadOnlyHandle &>(interface));
        }
      }
    }
    return interfaces;
  }

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start, const std::vector<std::string> & stop) override
  {
    auto result = upstream_->perform_command_mode_switch(start, stop);
    if (result != hardware_interface::return_type::OK) {return result;}
    for (auto & motor : motors_) {
      const auto key = motor.name + "/velocity";
      for (const auto & name : stop) {
        if (name == key) {
          motor.active = false;
          const bool zero = motor.joint->SetParam("vel", 0, 0.0);
          const bool passive = motor.joint->SetParam("fmax", 0, motor.passive_force);
          if (!zero || !passive) {return fail_zero();}
        }
      }
      for (const auto & name : start) {
        if (name == key) {motor.active = true;}
      }
    }
    return result;
  }

  hardware_interface::return_type write(const rclcpp::Time & time,
    const rclcpp::Duration & period) override
  {
    // Validate and snapshot all drive targets before native write can use them.
    // Storage is preallocated; no per-physics-step mailbox or heap allocation.
    for (size_t i = 0; i < motors_.size(); ++i) {
      auto & motor = motors_[i];
      if (!motor.active) {continue;}
      if (!motor.command) {return fail_zero();}
      targets_[i] = motor.command->get_value();
      if (!valid_motor_target(targets_[i], motor.effort, motor.velocity)) {return fail_zero();}
    }
    auto result = upstream_->write(time, period);
    if (result != hardware_interface::return_type::OK) {return fail_zero();}
    for (size_t i = 0; i < motors_.size(); ++i) {
      auto & motor = motors_[i];
      if (motor.active && !apply_motor_target(*motor.joint, targets_[i], motor.effort, motor.velocity)) {
        return fail_zero();
      }
    }
    return hardware_interface::return_type::OK;
  }
};
}  // namespace swerve_bringup

PLUGINLIB_EXPORT_CLASS(swerve_bringup::GazeboMotorSystem, gazebo_ros2_control::GazeboSystemInterface)
