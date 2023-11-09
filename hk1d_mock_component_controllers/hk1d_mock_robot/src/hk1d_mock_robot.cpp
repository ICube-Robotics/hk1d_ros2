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

#include "hk1d_mock_robot/hk1d_mock_robot.hpp"
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace hk1d_mock_robot
{
controller_interface::CallbackReturn Hk1dMockRobot::on_init()
{
  // initialize controller config
  try {
    parameter_handler_ =
      std::make_shared<hk1d_mock_robot::ParamListener>(get_node());
    parameters_ = parameter_handler_->get_params();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration Hk1dMockRobot::command_interface_configuration()
const
{
  std::vector<std::string> command_interfaces_config_names;

  // Joint motor
  command_interfaces_config_names.push_back(
    parameters_.joint_name + '/' + hardware_interface::HW_IF_POSITION);
  command_interfaces_config_names.push_back(
    parameters_.joint_name + '/' + hardware_interface::HW_IF_VELOCITY);
  // force sensor
  command_interfaces_config_names.push_back(parameters_.ft_sensor.interface_name);

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    command_interfaces_config_names};
}

controller_interface::InterfaceConfiguration
Hk1dMockRobot::state_interface_configuration()
const
{
  std::vector<std::string> state_interfaces_config_names;

  // Joint motor
  state_interfaces_config_names.push_back(
    parameters_.joint_name + '/' + hardware_interface::HW_IF_POSITION);
  state_interfaces_config_names.push_back(
    parameters_.joint_name + '/' + hardware_interface::HW_IF_VELOCITY);
  state_interfaces_config_names.push_back(
    parameters_.joint_name + '/' + hardware_interface::HW_IF_EFFORT);

  // force sensor
  state_interfaces_config_names.push_back(parameters_.ft_sensor.interface_name);

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL, state_interfaces_config_names};
}

controller_interface::return_type Hk1dMockRobot::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (!read_state_from_hardware()) {
    // TODO(tpoignonec): add verbose...
    return controller_interface::return_type::ERROR;
  }

  // Get time and period as double
  // double current_time = ...;
  double Ts = period.seconds();
  auto clock = get_node()->get_clock();
  RCLCPP_WARN_THROTTLE(
    get_node()->get_logger(), *clock, 1000,
    "Ts = %f", Ts
  );

  // ======================================================
  // Setup dynamics
  double ke = parameters_.simulation.env.stiffness;
  double be = parameters_.simulation.env.damping;

  double ks = parameters_.simulation.robot.stiffness;
  double bs = parameters_.simulation.robot.damping;
  double ms = parameters_.simulation.robot.mass;

  auto A = Eigen::Matrix<double, 2, 2>  {
    {1, Ts},
    {-(ks + ke) / ms * Ts, 1 - (bs + be) / ms * Ts}};
  auto B = Eigen::Matrix<double, 2, 1> {
    {0},
    {Ts *1 / ms}};
  auto C = Eigen::Matrix<double, 1, 2> {ke, be};

  auto X_last = Eigen::Matrix<double, 2, 1> {{current_joint_position_}, {current_joint_velocity_}};
  auto X = A * X_last + B * last_commanded_joint_force_;
  auto F_e = C * X;

  // Simulate mock system
  double dist_joint_to_sensor = 0.07;
  command_mock_external_force_ = dist_joint_to_sensor * F_e[0];
  command_mock_joint_position_ = X[0];
  command_mock_joint_velocity_ = X[1];
  // ======================================================

  // write calculated values to joint interfaces
  write_state_to_hardware();

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn Hk1dMockRobot::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // State
  current_joint_position_ = std::nan("0");
  current_joint_velocity_ = std::nan("0");
  current_joint_force_ = std::nan("0");
  current_external_force_ = std::nan("0");

  last_commanded_joint_force_ = std::nan("0");

  // Command (mock states)
  command_mock_joint_position_ = std::nan("0");
  command_mock_joint_velocity_ = std::nan("0");
  command_mock_external_force_ = std::nan("0");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Hk1dMockRobot::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // initialize states
  read_state_from_hardware();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Hk1dMockRobot::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Hk1dMockRobot::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Hk1dMockRobot::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

void Hk1dMockRobot::apply_parameters_update()
{
  if (parameter_handler_->is_old(parameters_)) {
    parameters_ = parameter_handler_->get_params();
  }
}

bool Hk1dMockRobot::read_state_from_hardware()
{
  current_joint_position_ = state_interfaces_[0].get_value();
  current_joint_velocity_ = state_interfaces_[1].get_value();
  current_joint_force_ = state_interfaces_[2].get_value();
  current_external_force_ = state_interfaces_[3].get_value();

  last_commanded_joint_force_ = current_joint_force_;

  auto nan_last_commanded_joint_force = std::isnan(last_commanded_joint_force_);

  auto clock = get_node()->get_clock();
  if (nan_last_commanded_joint_force) {
    // last_commanded_joint_force_ = 0.0;
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *clock, 1000, "State commanded joint force is NaN!");
    // return false;
  }
  return true;
}

bool Hk1dMockRobot::write_state_to_hardware()
{
  // TODO(tpoignonec): check NaN!!
  command_interfaces_[0].set_value(command_mock_joint_position_);
  command_interfaces_[1].set_value(command_mock_joint_velocity_);
  command_interfaces_[2].set_value(command_mock_external_force_);

  return true;
}

}  // namespace hk1d_mock_robot

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  hk1d_mock_robot::Hk1dMockRobot,
  controller_interface::ControllerInterface
)
