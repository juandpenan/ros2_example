cmake_minimum_required(VERSION 3.8)
project(example_pkg_cpp)

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
    rclcpp
    std_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/example_pkg_cpp/Node.cpp
  src/example_pkg_cpp/Logic.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(example_node src/main.cpp)
ament_target_dependencies(example_node ${dependencies})
target_link_libraries(example_node ${PROJECT_NAME})

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
