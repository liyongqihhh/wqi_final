cmake_minimum_required(VERSION 3.8)
project(uav_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz urdf scripts
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_rviz_config test/test_rviz_config.py)
  ament_add_pytest_test(test_prepare_px4_model test/test_prepare_px4_model.py)
endif()

ament_package()
