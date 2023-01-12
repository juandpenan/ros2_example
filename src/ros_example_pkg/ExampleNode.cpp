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

#include "ros_example_pkg/ExampleNode.hpp"


namespace ros_example_pkg
{

using namespace std::chrono_literals;
using std::placeholders::_1;
ExampleNode::ExampleNode(const rclcpp::NodeOptions & options)
: Node("example_node", options)
{
  multiply_pub_ = create_publisher<std_msgs::msg::Float64>("multiply_output", 10);
  divide_pub_ = create_publisher<std_msgs::msg::Float64>("divide_output", 10);
  timer_ = create_wall_timer(50ms, std::bind(&ExampleNode::timer_callback, this));
  logic = std::make_shared<ros_example_pkg::Logic>();
  subscriber_ = create_subscription<std_msgs::msg::Float64MultiArray>(
    "input_vector", 10, std::bind(&ExampleNode::topic_callback, this, _1));
}

void
ExampleNode::timer_callback()
{
  if (value_msg_ == nullptr) {
    return;
  }
  std_msgs::msg::Float64 multiply_msg;
  std_msgs::msg::Float64 divide_msg;
  multiply_msg.data = logic->multiply(value_msg_->data);
  divide_msg.data = logic->divide(value_msg_->data);
  multiply_pub_->publish(multiply_msg);
  divide_pub_->publish(divide_msg);
}
void
ExampleNode::topic_callback(std_msgs::msg::Float64MultiArray::UniquePtr msg)
{
  value_msg_ = std::move(msg);
  RCLCPP_INFO(this->get_logger(), "Got to callback");
}

}  // namespace ros_example_pkg


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(ros_example_pkg::ExampleNode)
