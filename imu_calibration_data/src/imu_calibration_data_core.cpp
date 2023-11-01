// SPDX-FileCopyrightText: 2023 Rion Miura <rionmiura39@gmail.com>
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "imu_calibration_data/node/imu_calibration_data.hpp"
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

imu_c_data::imu_c_data()
: Node("imu_c_data"),
  angular_velocity_z_ave(0),
  imu_i(0),
  sum(0),
  imuBiasCalibration(true),
  calibration_yaw(0)
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
   imu_i = 1001;
 }

 if(imu_i == 1001) {
const std::string file_path = "/home/ubuntu/raspicat2/src/raspicat_ros/raspicat/config/raspicat.param.yaml";  // YAMLファイルへのパス

    // ファイルを読み込む
    std::ifstream fin(file_path);
    YAML::Node yaml_node = YAML::Load(fin);

    // 新しい値にimu_calibration_yawの値を変更する
    calibration_yaw = angular_velocity_z_ave;  // 任意の新しい値
    yaml_node["raspimouse"]["ros__parameters"]["imu_calibration_yaw"] = calibration_yaw;

    // ファイルを上書き保存
    std::ofstream fout(file_path);
    fout << yaml_node;
    RCLCPP_INFO(get_logger(), "imu_calibration_yawの値が変更されました。");
//RCLCPP_INFO(node->get_logger(), "imu_calibration_yawの値が変更されました。");
    imu_i = 1002;
 }
}
