# hk1d_ros2

ROS2 stack to use a 1-Dof haptic kit used at the ICube laboratory.


[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![CI](../../actions/workflows/ci.yml/badge.svg?branch=main)](../../actions/workflows/ci.yml?query=branch:main)

***The current devs are based on the humble ROS 2 distribution (Ubuntu 22.04 LTS)***

## Installation

1) Install etherlab

See for instance the [procedure](https://github.com/ICube-Robotics/ethercat_driver_ros2/blob/main/.docker/Dockerfile) in the Dockerfile of `ICube-Robotics/ethercat_driver_ros2`.

2) Install ROS2 dependencies

```bash
cd <ros_ws>/src
git clone https://github.com/ICube-Robotics/hk1d_ros2.git
vcs import . < hk1d_ros2/hk1d_ros2.repos  # MANDATORY!!!
cd ..
rosdep install --ignore-src --from-paths . -y -r
```

2) Build the packages

```bash
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
source install/setup.bash
```
