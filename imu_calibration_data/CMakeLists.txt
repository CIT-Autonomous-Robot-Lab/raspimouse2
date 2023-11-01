cmake_minimum_required(VERSION 3.8)
project(imu_calibration_data)

find_package(ament_cmake_auto REQUIRED)
find_package(yaml-cpp REQUIRED)  # Find the yaml-cpp package

ament_auto_find_build_dependencies()

ament_auto_add_executable(imu_calibration_data
  src/imu_calibration_data_node.cpp
  src/imu_calibration_data_core.cpp
)

# Link the yaml-cpp library to your executable
target_link_libraries(imu_calibration_data
  ${YAML_CPP_LIBRARIES}
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
