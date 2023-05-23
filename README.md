# ROS_EXAMPLE_PKG
[![main](https://github.com/juandpenan/ros_example_pkg/actions/workflows/main.yaml/badge.svg)](https://github.com/juandpenan/ros_example_pkg/actions/workflows/main.yaml)
[![codecov](https://codecov.io/gh/juandpenan/ros_example_pkg/branch/main/graph/badge.svg?token=7VKCEGZ84T)](https://codecov.io/gh/juandpenan/ros_example_pkg)

This repository contains an example ros2 package quality1 according to the Coresense project guidelines.

## Install and building

```
$ git clone https://github.com/juandpenan/ros_example_pkg.git
$ cd ..
$ colcon build --symlink-install
```
## Generate Doxygen documentation

Run `doxygen` in the root of the Nav2 repository.
It will generate a `/doc/*` directory containing the documentation.
The documentation entrypoint in a browser is index.html.

## Usage
```
$ ros2 run ros_example_pkg example_node
```
To execute the logic:
```
$ ros2 topic pub /input_vector std_msgs/Float64 "data: {1.0, 2.0, 3.0}"
```


## Nodes

* **ros_example_pkg**
  * Publishers:
    * `/divide_output` `[std_msgs/msg/Float64]` 
    * `/multiply_output` `[std_msgs/msg/Float64]`  
  * Subscribers:
    * `/input_vector` `[std_msgs/msg/Float64MultiArray]` 

## Acknowledgments

<img src="https://coresenseeu.github.io/_images/funding.png" width="200" height="50">
