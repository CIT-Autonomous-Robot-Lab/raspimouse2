// SPDX-FileCopyrightText: 2023 Rion Miura <rionmiura39@gmail.com>
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "imu_calibration_data/node/imu_calibration_data.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

imu_c_data::imu_c_data()
: Node("imu_c_data"),
  angular_velocity_z_ave(0),
  imu_i(0),
  sum(0),
  imuBiasCalibration(true)
  {
  // Subscriber for imu data
  imu_data_raw_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "imu/data_raw", 1,
    [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
      imuDataCallback(msg);
    });
  }
void imu_c_data::imuDataCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
   {
     if (!msg) {
       RCLCPP_INFO(get_logger(), "IMU data not received.");
       return;
    }

     if (imuBiasCalibration) {
    sum = sum + msg->angular_velocity.z;
    imu_i++;
    angular_velocity_z_ave = sum / imu_i;
    RCLCPP_INFO(get_logger(), "キャリブレーション中 %d : 1000", imu_i);
    if(imu_i == 1000) {
	    RCLCPP_INFO(get_logger(), "キャリブレーション終了");
	    imuBiasCalibration = false;
    }
  }
 if(imu_i == 1000) {
   RCLCPP_INFO(get_logger(), "ave : %.10f", angular_velocity_z_ave);
 }
}
