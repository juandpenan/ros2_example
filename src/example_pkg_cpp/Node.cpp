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

#include "example_pkg_cpp/Node.hpp"


namespace example_pkg_cpp
{

using namespace std::chrono_literals;
using std::placeholders::_1;

ExampleNode::ExampleNode()
: Node("example_node")
{
  value_pub_ = create_publisher<std_msgs::msg::Int32>("output_value", 10);
  timer_ = create_wall_timer(50ms, std::bind(&ExampleNode::timer_callback, this));
}

void
ExampleNode::timer_callback()
{
  counter_++;
  value_msg_.data = logic_->duplicate(counter_);
  RCLCPP_INFO(get_logger(), "Publishing: '%d'", value_msg_.data);
  value_pub_->publish(value_msg_);
}
}  // namespace example_pkg_cpp
