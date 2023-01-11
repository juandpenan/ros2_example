
ament_add_gtest(logic_test logic_test.cpp)
ament_target_dependencies(logic_test ${dependencies})
target_link_libraries(logic_test ${PROJECT_NAME})

ament_add_gtest(node_test node_test.cpp)
ament_target_dependencies(node_test ${dependencies})
target_link_libraries(node_test ${PROJECT_NAME})