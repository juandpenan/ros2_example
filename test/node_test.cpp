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

#include <gtest/gtest.h>
#include "example_pkg_cpp/Node.hpp"

class ExampleNodeTest : public ::testing::Test
{
protected:
  std::shared_ptr<rclcpp::Node> test_node;
  std::shared_ptr<example_pkg_cpp::ExampleNode> example_node;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription;
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    test_node = std::make_shared<rclcpp::Node>("test_node");
    example_node = std::make_shared<example_pkg_cpp::ExampleNode>();
    const rclcpp::QoS qos(10);
    auto options = rclcpp::SubscriptionOptions();
    auto test_callback = [](std_msgs::msg::Int32::ConstSharedPtr) {};
    subscription =
      rclcpp::create_subscription<std_msgs::msg::Int32>(
      test_node,
      "output_value",
      qos,
      test_callback, options);
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(test_node);
    executor->add_node(example_node);
    executor->spin_once();
  }

  void TearDown() override
  {
    example_node.reset();
    test_node.reset();
    executor.reset();
    rclcpp::shutdown();
  }
};

TEST_F(ExampleNodeTest, test_sub_name)
{
  EXPECT_STREQ("/output_value", subscription->get_topic_name());
}

TEST_F(ExampleNodeTest, test_sub_count)
{
  EXPECT_EQ(example_node->count_subscribers("output_value"), 1U);
}
