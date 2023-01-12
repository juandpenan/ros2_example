ament_add_gtest(example_node_test example_node_test.cpp)
ament_target_dependencies(example_node_test ${dependencies})
target_link_libraries(example_node_test ${PROJECT_NAME})