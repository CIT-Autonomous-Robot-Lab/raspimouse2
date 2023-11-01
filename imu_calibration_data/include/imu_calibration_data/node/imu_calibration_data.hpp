// SPDX-FileCopyrightText: 2023 Rion Miura <rionmiura39@gmail.com>
// SPDX-License-Identifier: Apache-2.0

#ifndef IMU_CALIBRATION_DATA__NODE__NAME_PUBSUB_CORE_HPP_
#define IMU_CALIBRATION_DATA__NODE__NAME_PUBSUB_CORE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class imu_c_data : public rclcpp::Node
{
  public:
    double angular_velocity_z_ave;
    int imu_i;
    double sum;
    bool imuBiasCalibration;
    imu_c_data();

  protected:
    void imuDataCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_data_raw_sub_;
 };

#endif  // IMU_CALIBRATION_DATA__NODE__NAME_PUBSUB_CORE_HPP_
