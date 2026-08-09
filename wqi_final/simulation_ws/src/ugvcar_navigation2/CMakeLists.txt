cmake_minimum_required(VERSION 3.8)
project(ugvcar_navigation2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(dwb_core REQUIRED)
find_package(dwb_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ugvcar_navigation2_interfaces REQUIRED)

add_library(ugvcar_navigation_algorithms SHARED
  src/dstar_lite.cpp
  src/path_heading_model.cpp
  src/predictive_collision_model.cpp
)
target_include_directories(ugvcar_navigation_algorithms PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(ugvcar_dstar_lite_planner SHARED
  src/dstar_lite_planner.cpp
)
target_include_directories(ugvcar_dstar_lite_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(
  ugvcar_dstar_lite_planner
  ugvcar_navigation_algorithms
)
ament_target_dependencies(
  ugvcar_dstar_lite_planner
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  tf2
)

add_library(ugvcar_predictive_collision_critic SHARED
  src/path_heading_critic.cpp
  src/predictive_collision_critic.cpp
)
target_include_directories(ugvcar_predictive_collision_critic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(
  ugvcar_predictive_collision_critic
  ugvcar_navigation_algorithms
)
ament_target_dependencies(
  ugvcar_predictive_collision_critic
  dwb_core
  dwb_msgs
  geometry_msgs
  nav_2d_msgs
  nav2_costmap_2d
  nav2_util
  pluginlib
  rclcpp
  std_msgs
  tf2
  tf2_geometry_msgs
  ugvcar_navigation2_interfaces
)

pluginlib_export_plugin_description_file(
  nav2_core dstar_lite_planner_plugin.xml
)
pluginlib_export_plugin_description_file(
  dwb_core predictive_collision_critic_plugin.xml
)

add_library(nav2_dynamic_obstacle_cleared_condition_bt_node SHARED
  src/dynamic_obstacle_cleared_condition.cpp
)
target_compile_definitions(
  nav2_dynamic_obstacle_cleared_condition_bt_node
  PRIVATE BT_PLUGIN_EXPORT
)
ament_target_dependencies(
  nav2_dynamic_obstacle_cleared_condition_bt_node
  behaviortree_cpp_v3
  nav2_behavior_tree
  rclcpp
  std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(test_navigation_config test/test_navigation_config.py)
  ament_add_pytest_test(
    test_dynamic_obstacle_prediction
    test/test_dynamic_obstacle_prediction.py
  )
  ament_add_gtest(test_dstar_lite test/test_dstar_lite.cpp)
  target_link_libraries(test_dstar_lite ugvcar_navigation_algorithms)
  target_include_directories(test_dstar_lite PRIVATE include)
  ament_add_gtest(
    test_predictive_collision_model
    test/test_predictive_collision_model.cpp
  )
  target_link_libraries(
    test_predictive_collision_model ugvcar_navigation_algorithms
  )
  target_include_directories(test_predictive_collision_model PRIVATE include)
  ament_add_gtest(
    test_path_heading_model
    test/test_path_heading_model.cpp
  )
  target_link_libraries(test_path_heading_model ugvcar_navigation_algorithms)
  target_include_directories(test_path_heading_model PRIVATE include)
endif()

install(DIRECTORY launch maps config behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/dynamic_obstacle_predictor.py
  scripts/gazebo_ground_truth_localizer.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  nav2_dynamic_obstacle_cleared_condition_bt_node
  ugvcar_navigation_algorithms
  ugvcar_dstar_lite_planner
  ugvcar_predictive_collision_critic
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES
  scripts/dynamic_obstacle_prediction.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(ugvcar_navigation_algorithms)

ament_package()
