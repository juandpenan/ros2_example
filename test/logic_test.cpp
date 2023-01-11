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
#include "example_pkg_cpp/Logic.hpp"

// Test case 1: Check that the duplicate function returns the correct value for positive input
TEST(LogicTest, PositiveInput)
{
  example_pkg_cpp::Logic logic;
  int input = 5;
  int expected_output = 10;
  int output = logic.duplicate(input);
  EXPECT_EQ(output, expected_output);
}

// Test case 2: Check that the duplicate function returns the correct value for negative input
TEST(LogicTest, NegativeInput)
{
  example_pkg_cpp::Logic logic;
  int input = -5;
  int expected_output = -10;
  int output = logic.duplicate(input);
  EXPECT_EQ(output, expected_output);
}

// Test case 3: Check that the duplicate function returns the correct value for zero
TEST(LogicTest, ZeroInput)
{
  example_pkg_cpp::Logic logic;
  int input = 0;
  int expected_output = 0;
  int output = logic.duplicate(input);
  EXPECT_EQ(output, expected_output);
}
