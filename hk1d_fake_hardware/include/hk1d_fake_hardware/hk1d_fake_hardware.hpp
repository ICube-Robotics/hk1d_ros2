// Copyright 2023, ICube Laboratory, University of Strasbourg
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
// Author: Maciej Bednarczyk (macbednarczyk@gmail.com)

#ifndef HK1D_FAKE_HARDWARE__HK1D_HPP
#define HK1D_FAKE_HARDWARE__HK1D__HPP

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/QR>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/macros.hpp"
// #include "hk1d_fake_hardware/visibility_control.h"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace hk1d_fake_hardware
{
class HK1D : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(HK1D);

  // RRBOT_HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  // RRBOT_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // RRBOT_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // RRBOT_HARDWARE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  // RRBOT_HARDWARE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  

private:
  std::vector<double> hw_commands_effort_;
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;
  std::vector<double> hw_states_acceleration_;
  std::vector<double> hw_states_effort_;
  std::vector<double> hw_states_external_effort_;

  // Eigen::Vector2d q_pos_;
  // Eigen::Vector2d q_vel_;
  // Eigen::Vector2d q_acc_;
  Eigen::Vector2d tau_;

  // Eigen::Matrix2d bmat_;
  // Eigen::Vector2d cvec_;
  // Eigen::Vector2d gvec_;

  int counter_;

  Eigen::Matrix4d A; // 
  Eigen::Matrix<double,4,2> B;  
  Eigen::Matrix<double,4,1> Bd;
  Eigen::Matrix<double,1,4> C;  //fe = ke*xs+be*vs --> C = [0,0,ke,be]
  Eigen::Vector4d x; // xm,vm,xs,vs
  // Eigen::Vector2d u; // fcm, fcs  
  double fh;
  double fe;
  
  void set_Teleop_Matrices();
  
  // Set Membership debugging
  double Ts= 0.01;
  double ke= 10;
  double be= 2.0;
  double km= 1.995;
  double bm= 0.646;
  double mm= 0.175;
  double ks= 1.995;
  double bs= 0.646;
  double ms= 0.175;
    
  ////real hk1d in prismatic
  // double Ts = 0.001;
  // double  ke = 10.0;
  // double  be= 2;
  // double  km= 1.995;
  // double  bm= 0.646;
  // double  mm= 0.175*10;
  // double  ks= 1.995;
  // double  bs= 0.646;
  // double  ms= 0.175*10;

  //simple example 
    // double  ke = 10.0;
  // double  be= 2;
  // double mm = 2;
  // double bm = 0.1;
  // double km = 1;
  // double ms = 1;
  // double bs = 0.1;
  // double ks = 1;

  // //hashtrudi zaad example 
  // double mm = 0.7;
  // double bm = 25.2;
  // double km = 630;
  // double ms = 0.7;
  // double bs = 42;
  // double ks = 630;



  // std::vector<double> scale_x{100.0, 1.0, 100.0, 1.0};
  // double m1_ = 1.0;
  // double l1_ = 1.0;
  // double lc1_ = l1_/2; 
  // double i1_ = m1_/12*l1_*l1_;
  // double m2_ = 1.0;
  // double l2_ = 1.0;
  // double lc2_ = l2_/2; 
  // double i2_ = m2_/12*l2_*l2_;
  // double g_ = 9.8;  

};

}  // namespace rrbot_hardware

#endif  // RRBOT_HARDWARE__RRBOT_HPP
