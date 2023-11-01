// SPDX-FileCopyrightText: 2023 Rion Miura <rionmiura39@gmail.com>
// SPDX-License-Identifier: Apache-2.0

#include "imu_calibration_data/node/imu_calibration_data.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<imu_c_data>());
  rclcpp::shutdown();
  return 0;
}
