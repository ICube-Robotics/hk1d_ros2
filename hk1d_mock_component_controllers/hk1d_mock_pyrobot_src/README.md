# Requirements

Install first the [ICube-Robotics/pytroller](https://github.com/ICube-Robotics/pytroller) package:


```bash
cd <ws>/src/external_deps
git clone -b v0.1.0 https://github.com/ICube-Robotics/pytroller.git

cd ../..
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
```


# To build the pytroller

1) Create the empty pytroller

```bash
cd <ws>
source install/setup.bash
cd src/hk1d_mock_component_controllers

ros2 pytroller create hk1d_mock_pyrobot  --destination-directory .
```

2) Copy the logic files

```bash
cp hk1d_mock_pyrobot_src/hk1d_mock_pyrobot_logic_impl.py hk1d_mock_pyrobot/script/
cp hk1d_mock_pyrobot_src/hk1d_mock_pyrobot_parameters.yaml hk1d_mock_pyrobot/src/
```

3) Build :)

```bash
cd <ws>
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
```
