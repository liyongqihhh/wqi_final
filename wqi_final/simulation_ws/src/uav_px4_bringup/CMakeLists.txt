cmake_minimum_required(VERSION 3.8)
project(uav_px4_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch patches scripts DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_moving_base test/test_moving_base.py)
  ament_add_pytest_test(test_source_materialization test/test_source_materialization.py)
endif()

ament_package()
