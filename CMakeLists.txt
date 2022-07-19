cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME behaviour_trees)
project(${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  ament_cmake_ros
  BehaviorTreeV3
  nav2_behavior_tree
  nav2_msgs
  rclcpp
  rclcpp_action
  as2_core
  as2_msgs
  std_srvs
  geometry_msgs
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  include/${PROJECT_NAME}
)

add_executable(${PROJECT_NAME}_main src/behaviour_trees_node.cpp)
target_include_directories(${PROJECT_NAME}_main
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_main ${PROJECT_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME}_main BT::behaviortree_cpp_v3)

install(DIRECTORY
  resource
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}_main
  DESTINATION lib/${PROJECT_NAME})

ament_package()