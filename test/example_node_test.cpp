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

#include <gtest/gtest.h>
#include <limits>
#include "ros_example_pkg/ExampleNode.hpp"


using namespace std::chrono_literals;


class ExampleNodeTest : public ros_example_pkg::ExampleNode
{
public:
  explicit ExampleNodeTest(rclcpp::NodeOptions options)
  : ros_example_pkg::ExampleNode(options)
  {
  }
  double
  get_multiply_test(const std_msgs::msg::Float64MultiArray & msg)
  {
    return logic->multiply(msg.data);
  }

  double
  get_divide_test(const std_msgs::msg::Float64MultiArray & msg)
  {
    return logic->divide(msg.data);
  }
};

std_msgs::msg::Float64MultiArray test_1()
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data = std::vector<double>(3, -1.0);
  return msg;
}

std_msgs::msg::Float64MultiArray test_2()
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data = std::vector<double>(3, 1.0);
  return msg;
}

std_msgs::msg::Float64MultiArray test_3()
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data = std::vector<double>(3, 0.0);
  return msg;
}


TEST(example_node_test, divide)
{
  rclcpp::NodeOptions options;
  auto example_node = std::make_shared<ExampleNodeTest>(options);

  double output = example_node->get_divide_test(test_1());
  double expected_output = -2.0;

  EXPECT_EQ(output, expected_output);
  output = example_node->get_divide_test(test_2());
  expected_output = 2.0;
  EXPECT_EQ(output, expected_output);
  EXPECT_EQ(example_node->get_divide_test(test_3()), std::numeric_limits<double>::infinity());
}

TEST(example_node_test, multiply)
{
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto example_node = std::make_shared<ExampleNodeTest>(options);
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto test_pub = test_node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "input_vector", 100);
  std_msgs::msg::Float64 last_msg;
  auto test_sub = test_node->create_subscription<std_msgs::msg::Float64>(
    "multiply_output", 1, [&last_msg](std_msgs::msg::Float64::SharedPtr msg) {
      last_msg = *msg;
    });
  exec.add_node(example_node);
  exec.add_node(test_node);

  rclcpp::Rate rate(30);
  auto start = example_node->now();
  while (rclcpp::ok() && (example_node->now() - start) < 1s) {
    test_pub->publish(test_1());
    exec.spin_some();
    rate.sleep();
  }


  EXPECT_EQ(last_msg.data, -2.0);

  start = example_node->now();
  while (rclcpp::ok() && (example_node->now() - start) < 1s) {
    test_pub->publish(test_2());
    exec.spin_some();
    rate.sleep();
  }

  EXPECT_EQ(last_msg.data, 2.0);

  start = example_node->now();
  while (rclcpp::ok() && (example_node->now() - start) < 1s) {
    test_pub->publish(test_3());
    exec.spin_some();
    rate.sleep();
  }
  EXPECT_EQ(last_msg.data, 0.0);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
