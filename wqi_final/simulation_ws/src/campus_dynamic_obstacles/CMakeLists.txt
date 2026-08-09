cmake_minimum_required(VERSION 3.8)
project(campus_dynamic_obstacles)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(gazebo REQUIRED)

add_library(campus_dynamic_obstacle_plugin SHARED
  src/dynamic_obstacle_plugin.cpp
)
target_compile_features(campus_dynamic_obstacle_plugin PUBLIC cxx_std_17)
target_include_directories(campus_dynamic_obstacle_plugin PRIVATE
  ${GAZEBO_INCLUDE_DIRS}
)
target_link_libraries(campus_dynamic_obstacle_plugin
  ${GAZEBO_LIBRARIES}
)

ament_python_install_package(${PROJECT_NAME})

install(TARGETS campus_dynamic_obstacle_plugin
  LIBRARY DESTINATION lib
)
install(PROGRAMS scripts/dynamic_obstacle_spawner
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_scenario test/test_scenario.py)
endif()

ament_package()
