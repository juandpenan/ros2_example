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


#include "ros_example_pkg/Logic.hpp"


namespace ros_example_pkg
{

Logic::Logic()
{
}

double
Logic::multiply(const std::vector<double> & operands)
{
  double value = 2.0;
  for (const double & op : operands) {
    value = value * op;
  }
  return value;
}

double
Logic::divide(const std::vector<double> & operands)
{
  double value = 2.0;
  for (const double & op : operands) {
    value = value / op;
  }

  return value;
}


}  // namespace ros_example_pkg
