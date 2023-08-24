cmake_minimum_required(VERSION 3.15)
project(joint_state_merger VERSION 0.0.0 LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/joint_state_merger_node.cpp
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "joint_state_merger::JointStateMergerNode"
  EXECUTABLE joint_state_merger_node
)
generate_parameter_library(joint_state_merger_node_parameters
  src/joint_state_merger_node_parameters.yaml
)
target_link_libraries(${PROJECT_NAME}
  joint_state_merger_node_parameters
)
install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
