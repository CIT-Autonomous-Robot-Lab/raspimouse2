cmake_minimum_required(VERSION 3.8)
project(imu_calibration_data)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(imu_calibration_data
  src/imu_calibration_data_node.cpp
  src/imu_calibration_data_core.cpp
)

ament_auto_package(
  INSTALL_TO_SHARE	
  config
  launch
)
