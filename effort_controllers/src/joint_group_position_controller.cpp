// Copyright 2020 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <inttypes.h>
#include <string>

#include "angles/angles.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "effort_controllers/joint_group_position_controller.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/parameter.hpp"
#include "joint_limits/joint_limits_rosparam.hpp"


namespace effort_controllers
{
using CallbackReturn = JointGroupPositionController::CallbackReturn;

JointGroupPositionController::JointGroupPositionController()
: forward_command_controller::ForwardCommandController()
{
  logger_name_ = "joint effort controller";
  interface_name_ = hardware_interface::HW_IF_EFFORT;
}

controller_interface::return_type
JointGroupPositionController::init(
  const std::string & controller_name)
{
  auto node_options_ = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  auto ret = ControllerInterface::init(controller_name, node_options_);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }
  return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration
JointGroupPositionController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint : joint_names_) {
    state_interfaces_config.names.push_back(joint + "/position");
  }

  return state_interfaces_config;
}

CallbackReturn JointGroupPositionController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret = ForwardCommandController::on_configure(previous_state);

  RCLCPP_INFO(get_node()->get_logger(), "got %zu joints\n", joint_names_.size());
  pids_.resize(joint_names_.size());
  limits_.resize(joint_names_.size());
//  limit_handles_.resize(joint_names_.size());  // TODO

  std::string gains_prefix = "gains";
  RCLCPP_INFO(get_node()->get_logger(), "JOINT_NAMES_SIZE: %d", joint_names_.size());
  for (auto k = 0u; k < joint_names_.size(); ++k) {
    auto p = get_node()->get_parameter(gains_prefix + "." + joint_names_[k] + ".p").as_double();
    auto i = get_node()->get_parameter(gains_prefix + "." + joint_names_[k] + ".i").as_double();
    auto d = get_node()->get_parameter(gains_prefix + "." + joint_names_[k] + ".d").as_double();
    pids_[k].initPid(p, i, d, 0.0, 0.0);
    RCLCPP_INFO(get_node()->get_logger(), "got gains for %s as (%f, %f, %f)\n", joint_names_[k].c_str(), p, i, d);

    // extract joint limits:
    // TODO: consider soft joint limits as well
    // prioritize rosparam definition
    if (joint_limits::get_joint_limits(joint_names_[k], get_node(), limits_[k])){
      RCLCPP_INFO(get_node()->get_logger(), "got joint limits from rosparam; previous values will be replaced!\n");
      if(limits_[k].has_position_limits)
        RCLCPP_INFO(get_node()->get_logger(), "  min_position: %f, max_position: %f\n", limits_[k].min_position, limits_[k].max_position);
      if(limits_[k].has_velocity_limits)
        RCLCPP_INFO(get_node()->get_logger(), "  max_velocity: %f\n", limits_[k].max_velocity);
      if(limits_[k].has_acceleration_limits)
        RCLCPP_INFO(get_node()->get_logger(), "  max_acceleration: %f\n", limits_[k].max_acceleration);
      if(limits_[k].has_jerk_limits)
        RCLCPP_INFO(get_node()->get_logger(), "  max_jerk: %f\n", limits_[k].max_jerk);
      if(limits_[k].has_effort_limits)
        RCLCPP_INFO(get_node()->get_logger(), "  max_effort: %f\n", limits_[k].max_effort);
      RCLCPP_INFO(get_node()->get_logger(), "  angle_wraparound: %s\n", limits_[k].angle_wraparound ? "true" : "false");
    }

    // IGNORING LIMITS FOR NOW; ONLY APPLYING JOINT WRAPAROUND; TODO: enforce limits
    RCLCPP_WARN(get_node()->get_logger(), "  URDF LIMIT PARSER NOT YET IMPLEMENTED. ONLY LIMITS SPECIFIED IN .YAML CONFIG FILES CONSIDERED.");
    RCLCPP_WARN(get_node()->get_logger(), "  JOINT_LIMITS_INTERFACE NOT YET FULLY IMPLEMENTED. IGNORING VELOCITY, ACCELERATION, JERK, AND SOFT LIMITS.");
  }

  return ret;
}

CallbackReturn JointGroupPositionController::on_activate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  if (command_interfaces_.size() != state_interfaces_.size()) {
    RCLCPP_ERROR(get_node()->get_logger(), "state interfaces don't match with command interfaces\n");
    return CallbackReturn::ERROR;
  }
  t0 = std::chrono::system_clock::now();

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointGroupPositionController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret = ForwardCommandController::on_deactivate(previous_state);

  // stop all joints
  for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }

  return ret;
}

controller_interface::return_type JointGroupPositionController::update()
{
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now() - t0).count();
  t0 = std::chrono::system_clock::now();

  auto joint_position_commands = *rt_command_ptr_.readFromRT();
  // no command received yet
  if (!joint_position_commands) {
    return controller_interface::return_type::OK;
  }

  if (joint_position_commands->data.size() != command_interfaces_.size()) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *node_->get_clock(), 1000,
      "command size (%zu) does not match number of interfaces (%zu)",
      joint_position_commands->data.size(), command_interfaces_.size());
    return controller_interface::return_type::ERROR;
  }

  for(auto i = 0u; i < joint_names_.size(); ++i)
  {
    double command_position = joint_position_commands->data[i];

    // check commanded position against joint limits; clip as required
    if (limits_[i].has_position_limits){
      if(command_position < limits_[i].min_position){
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(),
          *node_->get_clock(), 1000,
          "commanded position (%f) is below min joint position (%f); clipping",
          command_position, limits_[i].min_position);
        command_position = limits_[i].min_position;
      }
      else if (command_position > limits_[i].max_position){
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(),
          *node_->get_clock(), 1000,
          "commanded position (%f) is above max joint position (%f); clipping",
          command_position, limits_[i].max_position);
        command_position = limits_[i].max_position;
      }
    }

    double current_position = state_interfaces_[i].get_value();

    // calculate error terms depending on joint type
    double error; 
    
    if (limits_[i].angle_wraparound) {
      error = angles::shortest_angular_distance(current_position, command_position);
    }
    else {
      error = command_position - current_position;
    }

    auto commanded_effort = pids_[i].computeCommand(error, period);

    // check commanded effort against joint limits; clip as required
    if (limits_[i].has_effort_limits){
      if(std::abs(commanded_effort) > limits_[i].max_effort && commanded_effort < 0){
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(),
          *node_->get_clock(), 1000,
          "commanded effort (%f) is below min joint effort (%f); clipping",
          commanded_effort, -limits_[i].max_effort);
        commanded_effort = -limits_[i].max_effort;
      }
      else if (commanded_effort > limits_[i].max_effort){
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(),
          *node_->get_clock(), 1000,
          "commanded effort (%f) is above max joint effort (%f); clipping",
          commanded_effort, limits_[i].max_effort);
        commanded_effort = limits_[i].max_effort;
      }
    }
    command_interfaces_[i].set_value(commanded_effort);
  }

  t0 = std::chrono::system_clock::now();
  return controller_interface::return_type::OK;
}

}  // namespace effort_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  effort_controllers::JointGroupPositionController, controller_interface::ControllerInterface)