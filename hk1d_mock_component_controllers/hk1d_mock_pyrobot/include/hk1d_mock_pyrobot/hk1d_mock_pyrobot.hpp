// Copyright 2023 ICube-Robotics
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
// author: Maciej Bednarczyk

#ifndef HK1D_MOCK_PYROBOT__HK1D_MOCK_PYROBOT_HPP_
#define HK1D_MOCK_PYROBOT__HK1D_MOCK_PYROBOT_HPP_

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include "controller_interface/controller_interface.hpp"
#include "hk1d_mock_pyrobot/visibility_control.h"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "hk1d_mock_pyrobot_parameters.hpp"

namespace hk1d_mock_pyrobot
{
/**
 * \brief Python controller for a set of joints and interfaces.
 *
 * Subscribes to:
 * - \b commands : The commands to apply.
 */
class Hk1DMockPyrobot : public controller_interface::ControllerInterface
{
public:
  HK1D_MOCK_PYROBOT_PUBLIC
  Hk1DMockPyrobot();

  HK1D_MOCK_PYROBOT_PUBLIC
  ~Hk1DMockPyrobot() = default;

  HK1D_MOCK_PYROBOT_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  HK1D_MOCK_PYROBOT_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  HK1D_MOCK_PYROBOT_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  HK1D_MOCK_PYROBOT_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  HK1D_MOCK_PYROBOT_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  HK1D_MOCK_PYROBOT_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  HK1D_MOCK_PYROBOT_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  void declare_parameters();
  controller_interface::CallbackReturn read_parameters();

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  std::vector<std::string> command_interface_types_;
  std::unordered_map<std::string, double> states_;
  std::unordered_map<std::string, double> commands_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<rclcpp::SerializedMessage>> rt_command_ptr_;
  rclcpp::GenericSubscription::SharedPtr command_subscriber_;
};

}  // namespace hk1d_mock_pyrobot

#endif  // HK1D_MOCK_PYROBOT__FORWARD_CONTROLLERS_BASE_HPP_
