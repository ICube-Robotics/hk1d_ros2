# hk1d_ros2

ROS2 stack to use a 1-Dof haptic kit used at the ICube laboratory.

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
