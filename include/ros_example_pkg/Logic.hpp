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

#ifndef ROS_EXAMPLE_PKG__LOGIC_HPP_
#define ROS_EXAMPLE_PKG__LOGIC_HPP_

#include <vector>

namespace ros_example_pkg
{
/**
 * @brief Logic class used for logic implementation within ROS nodes.
 */
class Logic
{
public:
  /**
   * Create a new Logic object.
   * @brief Constructor.
   */
  Logic();

  /// Multiplies a vector by a value.
  /**
   * \param value value to be duplicated
   * \return the value duplicated
   */
  double multiply(const std::vector<double> & operands);

  /// Divides a vector by a value.
  /**
   * \param value value to be duplicated
   * \return the value duplicated
   */
  double divide(const std::vector<double> & operands);
};
}  // namespace ros_example_pkg
#endif  // ROS_EXAMPLE_PKG__LOGIC_HPP_
