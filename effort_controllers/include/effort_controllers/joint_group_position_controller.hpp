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

#ifndef EFFORT_CONTROLLERS__JOINT_GROUP_POSITION_CONTROLLER_HPP_
#define EFFORT_CONTROLLERS__JOINT_GROUP_POSITION_CONTROLLER_HPP_

#include <string>

#include "control_toolbox/pid.hpp"
#include "forward_command_controller/forward_command_controller.hpp"
#include "effort_controllers/visibility_control.h"

//#include "joint_limits_interface/joint_limits_interface.hpp"
#include "joint_limits_interface/joint_limits.hpp"


namespace effort_controllers
{

/**
 * \brief Forward command controller for a set of effort controlled joints (linear or angular).
 *
 * This class applies a PID controller to track commanded positions using a set of effort-controlled joints.
 *
 * \param joints Names of the joints to control.
 *
 * Subscribes to:
 * - \b command (std_msgs::msg::Float64MultiArray) : The joint positions to target.
 */
class JointGroupPositionController : public forward_command_controller::ForwardCommandController
{
public:
  EFFORT_CONTROLLERS_PUBLIC
  JointGroupPositionController();

  EFFORT_CONTROLLERS_PUBLIC
  controller_interface::return_type
  init(const std::string & controller_name) override;

  EFFORT_CONTROLLERS_PUBLIC
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  EFFORT_CONTROLLERS_PUBLIC
  CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

  EFFORT_CONTROLLERS_PUBLIC
  CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override;

  EFFORT_CONTROLLERS_PUBLIC
  CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  EFFORT_CONTROLLERS_PUBLIC
  controller_interface::return_type
  update() override;

private:
  std::vector<control_toolbox::Pid> pids_;
  std::vector<joint_limits_interface::JointLimits> limits_;
//  std::vector<joint_limits_interface::EffortJointSaturationHandle> limit_handles_;
  std::chrono::time_point<std::chrono::system_clock> t0;
};


template <class Scalar>
Scalar wraparoundJointOffset(const Scalar& prev_position,
                           const Scalar& next_position,
                           const bool& angle_wraparound)
{
// Return value
Scalar pos_offset = 0.0;

if (angle_wraparound)
{
  Scalar dist = angles::shortest_angular_distance(prev_position, next_position);

  // Deal with singularity at M_PI shortest distance
  if (std::abs(std::abs(dist) - M_PI) < 1e-9)
  {
    dist = next_position > prev_position ? std::abs(dist) : -std::abs(dist);
  }
  pos_offset = (prev_position + dist) - next_position;
}

return pos_offset;
}

}  // namespace effort_controllers

#endif  // EFFORT_CONTROLLERS__JOINT_GROUP_POSITION_CONTROLLER_HPP_
