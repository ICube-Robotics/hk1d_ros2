// Copyright 2023 ICUBE Laboratory, University of Strasbourg
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
//
/// \authors: Thibault Poignonec

#ifndef HK1D_MOCK_ROBOT__HK1D_MOCK_ROBOT_HPP_
#define HK1D_MOCK_ROBOT__HK1D_MOCK_ROBOT_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "hk1d_mock_robot/visibility_control.h"

// include generated parameter library
#include "hk1d_mock_robot_parameters.hpp"

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
// #include "semantic_components/force_torque_sensor.hpp"


namespace hk1d_mock_robot
{

class Hk1dMockRobot : public controller_interface::ControllerInterface
{
public:
  HK1D_MOCK_ROBOT_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  /// Export configuration of required state interfaces.
  /**
   * Allowed types of state interfaces are \ref hardware_interface::POSITION,
   * \ref hardware_interface::VELOCITY, \ref hardware_interface::ACCELERATION.
   */
  HK1D_MOCK_ROBOT_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  /// Export configuration of required state interfaces.
  /**
   * Allowed types of state interfaces are \ref hardware_interface::POSITION,
   * \ref hardware_interface::VELOCITY, \ref hardware_interface::ACCELERATION.
   */
  HK1D_MOCK_ROBOT_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  HK1D_MOCK_ROBOT_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  HK1D_MOCK_ROBOT_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  HK1D_MOCK_ROBOT_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  HK1D_MOCK_ROBOT_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  HK1D_MOCK_ROBOT_PUBLIC
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  HK1D_MOCK_ROBOT_PUBLIC
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  // parameters
  std::shared_ptr<hk1d_mock_robot::ParamListener> parameter_handler_;
  hk1d_mock_robot::Params parameters_;

  // Read variables
  double current_joint_position_;
  double current_joint_velocity_;
  double current_joint_force_;
  double current_external_force_;

  double last_commanded_joint_force_;

  // Command interfaces
  double command_mock_joint_position_;
  double command_mock_joint_velocity_;
  double command_mock_external_force_;


  /// Retrieve parameters and update if applicable
  void apply_parameters_update();

  /**
   * @brief Read values from claimed hardware state interfaces
   */
  bool read_state_from_hardware();

  /**
   * @brief Write values to claimed hardware command interfaces
   */
  bool write_state_to_hardware();
};

}  // namespace hk1d_mock_robot

#endif  // HK1D_MOCK_ROBOT__HK1D_MOCK_ROBOT_HPP_
