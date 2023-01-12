// Copyright 2023 Intelligent Robotics Lab.
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

#ifndef ROS_EXAMPLE_PKG__EXAMPLENODE_HPP_
#define ROS_EXAMPLE_PKG__EXAMPLENODE_HPP_
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ros_example_pkg/Logic.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"


namespace ros_example_pkg
{

class ExampleNode : public rclcpp::Node
{
public:
  explicit ExampleNode(const rclcpp::NodeOptions & options);

protected:
  std::shared_ptr<ros_example_pkg::Logic> logic;

private:
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr multiply_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr divide_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;

  std_msgs::msg::Float64MultiArray::UniquePtr value_msg_;

  void timer_callback();
  void topic_callback(std_msgs::msg::Float64MultiArray::UniquePtr msg);
};
}  // namespace ros_example_pkg
#endif  // ROS_EXAMPLE_PKG__EXAMPLENODE_HPP_
