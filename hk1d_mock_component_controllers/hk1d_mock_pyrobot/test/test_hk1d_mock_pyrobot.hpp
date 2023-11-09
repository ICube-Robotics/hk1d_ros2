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

#ifndef TEST_HK1D_MOCK_PYROBOT_HPP_
#define TEST_HK1D_MOCK_PYROBOT_HPP_

#include <memory>
#include <string>
#include <vector>
#include <limits>

#include "gmock/gmock.h"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hk1d_mock_pyrobot/hk1d_mock_pyrobot.hpp"

using hardware_interface::CommandInterface;
using hardware_interface::StateInterface;

// subclassing and friending so we can access member variables
class FriendHk1DMockPyrobot : public hk1d_mock_pyrobot::Hk1DMockPyrobot
{
  FRIEND_TEST(TestHk1DMockPyrobot, CommandSuccessTest);
  FRIEND_TEST(TestHk1DMockPyrobot, WrongCommandCheckTest);
  FRIEND_TEST(TestHk1DMockPyrobot, CommandSuccessTestWithConstraints);
};

class TestHk1DMockPyrobot : public ::testing::Test
{
public:
  static void SetUpTestCase();
  static void TearDownTestCase();

  void SetUp();
  void TearDown();

  void SetUpController();
  void SetUpHandles();

protected:
  std::unique_ptr<FriendHk1DMockPyrobot> controller_;

  const std::vector<std::string> joint_names_ = {"joint1", "joint2"};
  // dummy joint state values used for tests
  std::vector<double> joint_commands_ = {
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()
  };
  std::vector<double> joint_states_position_ = {0.0, 0.0};
  std::vector<double> joint_states_velocity_ = {0.0, 0.0};

  CommandInterface joint1_ci_{joint_names_[0], hardware_interface::HW_IF_POSITION, &joint_commands_[0]};
  CommandInterface joint2_ci_{joint_names_[1], hardware_interface::HW_IF_POSITION, &joint_commands_[1]};
  StateInterface joint1_sip_{joint_names_[0], hardware_interface::HW_IF_POSITION, &joint_states_position_[0]};
  StateInterface joint2_sip_{joint_names_[1], hardware_interface::HW_IF_POSITION, &joint_states_position_[1]};
  StateInterface joint1_siv_{joint_names_[0], hardware_interface::HW_IF_VELOCITY, &joint_states_velocity_[0]};
  StateInterface joint2_siv_{joint_names_[1], hardware_interface::HW_IF_VELOCITY, &joint_states_velocity_[1]};
};

#endif  // TEST_MPI_CONTROLLER_HPP_
