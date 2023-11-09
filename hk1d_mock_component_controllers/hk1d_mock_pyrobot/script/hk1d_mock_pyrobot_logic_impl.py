# Copyright 2023 ICube-Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np
def pytroller_logic_impl(states, commands, msg, params):
  # controller sampling time
  Ts = 0.002
  # env
  ke = 100.0
  be = 8.0
  # robot
  ks = 52.
  bs = 8.0
  ms = 0.25
  dist_joint_to_sensor = 0.07

  A = np.array([
    [1.0, Ts],
    [-(ks + ke) / ms * Ts, 1 - (bs + be) / ms * Ts]
  ])
  B = np.array([
    [0.0],
    [Ts *1 / ms]
  ])
  C = np.array([[ke, be]])

  tau = np.array([states[b'joint_1/effort']]).reshape((-1,1))
  X_last = np.array([
     states[b'joint_1/position'],
     states[b'joint_1/velocity'],
  ]).reshape((-1,1))
  X = A @ X_last + B @ tau
  Fe = dist_joint_to_sensor * C @ X

  commands[b'joint_1/position'] = X[0]
  commands[b'joint_1/velocity'] = X[1]
  commands[b'force_sensor/force.0'] = Fe[0]

  return commands
