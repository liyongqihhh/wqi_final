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

add_library(ugv_uav_docking_plugin SHARED
  src/ugv_uav_docking_plugin.cpp
)
target_compile_features(ugv_uav_docking_plugin PUBLIC cxx_std_17)
ament_target_dependencies(ugv_uav_docking_plugin
  gazebo_ros
  rclcpp
  std_msgs
  std_srvs
)
target_link_libraries(ugv_uav_docking_plugin ${GAZEBO_LIBRARIES})

ament_python_install_package(${PROJECT_NAME})

install(TARGETS ugv_uav_docking_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(PROGRAMS scripts/cooperative_mission_manager
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_logger_calls test/test_logger_calls.py)
  ament_add_pytest_test(test_energy_planner test/test_energy_planner.py)
  ament_add_pytest_test(test_mission_config test/test_mission_config.py)
  ament_add_pytest_test(test_mission_states test/test_mission_states.py)
endif()

ament_package()
