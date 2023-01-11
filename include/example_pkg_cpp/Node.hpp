// Copyright 2023 CORESENSE project
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

#ifndef EXAMPLE_PKG_CPP__NODE_HPP_
#define EXAMPLE_PKG_CPP__NODE_HPP_
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "example_pkg_cpp/Logic.hpp"

#include "std_msgs/msg/int32.hpp"


namespace example_pkg_cpp
{

class ExampleNode : public rclcpp::Node
{
public:
  ExampleNode();

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr value_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int counter_ = 0;
  std::shared_ptr<example_pkg_cpp::Logic> logic_;
  std_msgs::msg::Int32 value_msg_;
  void timer_callback();
};
}  // namespace example_pkg_cpp
#endif  // EXAMPLE_PKG_CPP__NODE_HPP_
