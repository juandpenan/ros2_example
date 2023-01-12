cmake_minimum_required(VERSION 3.8)
project(ros_example_pkg)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

set(dependencies
    rclcpp
    std_msgs
    rclcpp_components
)
include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/ros_example_pkg/ExampleNode.cpp
  src/ros_example_pkg/Logic.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "ros_example_pkg::ExampleNode"
    EXECUTABLE example_node
)

install(TARGETS
  ${PROJECT_NAME}
  example_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_package()
