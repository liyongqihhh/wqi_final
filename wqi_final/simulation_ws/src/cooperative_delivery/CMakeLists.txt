cmake_minimum_required(VERSION 3.8)
project(cooperative_delivery)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(uav_interfaces REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(peer_vehicle_layer SHARED src/peer_vehicle_layer.cpp)
target_compile_features(peer_vehicle_layer PUBLIC cxx_std_17)
target_include_directories(peer_vehicle_layer PUBLIC include)
ament_target_dependencies(peer_vehicle_layer nav2_costmap_2d nav_msgs pluginlib tf2_geometry_msgs)
pluginlib_export_plugin_description_file(nav2_costmap_2d peer_vehicle_plugins.xml)

add_library(ugv_uav_docking_plugin SHARED
  src/ugv_uav_docking_plugin.cpp
)
target_compile_features(ugv_uav_docking_plugin PUBLIC cxx_std_17)
ament_target_dependencies(ugv_uav_docking_plugin
  gazebo_ros
  rclcpp
  std_msgs
  std_srvs
  uav_interfaces
)
target_link_libraries(ugv_uav_docking_plugin ${GAZEBO_LIBRARIES})

ament_python_install_package(${PROJECT_NAME})

install(TARGETS ugv_uav_docking_plugin peer_vehicle_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(PROGRAMS
  scripts/cooperative_mission_manager
  scripts/ugv_energy_manager
  scripts/relay_mission_manager
  scripts/relay_localizer
  scripts/wait_for_carrier_control
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_peer_vehicle_layer test/test_peer_vehicle_layer.cpp)
  target_link_libraries(test_peer_vehicle_layer peer_vehicle_layer)
  ament_target_dependencies(test_peer_vehicle_layer nav2_costmap_2d nav_msgs tf2_geometry_msgs)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_logger_calls test/test_logger_calls.py)
  ament_add_pytest_test(test_energy_planner test/test_energy_planner.py)
  ament_add_pytest_test(test_energy_schedule test/test_energy_schedule.py)
  ament_add_pytest_test(test_px4_carrier_model test/test_px4_carrier_model.py)
  ament_add_pytest_test(test_carrier_home test/test_carrier_home.py)
  ament_add_pytest_test(test_ugv_energy_model test/test_ugv_energy_model.py)
  ament_add_pytest_test(test_mission_config test/test_mission_config.py)
  ament_add_pytest_test(test_mission_states test/test_mission_states.py)
  ament_add_pytest_test(test_relay_plan test/test_relay_plan.py)
  ament_add_pytest_test(test_relay_vehicle test/test_relay_vehicle.py)
  ament_add_pytest_test(test_relay_manager test/test_relay_manager.py)
  ament_add_pytest_test(test_relay_energy test/test_relay_energy.py)
endif()

ament_package()
