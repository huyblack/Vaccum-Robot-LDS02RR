// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Darby Lim

#include <array>

#include <memory>
#include <string>
#include <utility>

#include "turtlebot3_node/sensors/joint_state.hpp"

using robotis::turtlebot3::sensors::JointState;

// Static variables to track position differences for odometry
static std::array<double, robotis::turtlebot3::sensors::JOINT_NUM> last_position_rad = {0.0, 0.0};
static std::array<double, robotis::turtlebot3::sensors::JOINT_NUM> cumulative_position_rad = {0.0, 0.0};

JointState::JointState(
  std::shared_ptr<rclcpp::Node> & nh,
  std::shared_ptr<XiaoBLEI2CWrapper> & dxl_sdk_wrapper,
  const std::string & topic_name,
  const std::string & frame_id)
: Sensors(nh, frame_id)
{
  pub_ = nh->create_publisher<sensor_msgs::msg::JointState>(topic_name, this->qos_);
  
  // Initialize with current Arduino position  
  std::array<float, JOINT_NUM> initial_position_meters =
  {dxl_sdk_wrapper->get_data_from_device<float>(
      extern_control_table.present_position_left.addr,
      extern_control_table.present_position_left.length),
    dxl_sdk_wrapper->get_data_from_device<float>(
      extern_control_table.present_position_right.addr,
      extern_control_table.present_position_right.length)};
      
  last_position_rad[0] = initial_position_meters[0] / WHEEL_RADIUS;
  last_position_rad[1] = initial_position_meters[1] / WHEEL_RADIUS;
  cumulative_position_rad[0] = 0.0;  // Start from 0 for odometry
  cumulative_position_rad[1] = 0.0;

  nh_->get_parameter_or<std::string>(
    "namespace",
    name_space_,
    std::string(""));

  if (name_space_ != "") {
    frame_id_ = name_space_ + "/" + frame_id_;
    wheel_right_joint_ = name_space_ + "/" + wheel_right_joint_;
    wheel_left_joint_ = name_space_ + "/" + wheel_left_joint_;
  }
  RCLCPP_INFO(nh_->get_logger(), "Succeeded to create joint state publisher");
}

void JointState::publish(
  const rclcpp::Time & now,
  std::shared_ptr<XiaoBLEI2CWrapper> & dxl_sdk_wrapper) 
{
  auto msg = std::make_unique<sensor_msgs::msg::JointState>();

  // FIXED: Read Arduino data as float (since Arduino sends float, not int32_t)
  std::array<float, JOINT_NUM> position_meters =
  {dxl_sdk_wrapper->get_data_from_device<float>(
      extern_control_table.present_position_left.addr,
      extern_control_table.present_position_left.length),
    dxl_sdk_wrapper->get_data_from_device<float>(
      extern_control_table.present_position_right.addr,
      extern_control_table.present_position_right.length)};

  std::array<float, JOINT_NUM> velocity_mps =
  {dxl_sdk_wrapper->get_data_from_device<float>(
      extern_control_table.present_velocity_left.addr,
      extern_control_table.present_velocity_left.length),
    dxl_sdk_wrapper->get_data_from_device<float>(
      extern_control_table.present_velocity_right.addr,
      extern_control_table.present_velocity_right.length)};

  // Convert Arduino position (meters) to radians
  double current_position_rad[JOINT_NUM];
  current_position_rad[0] = position_meters[0] / WHEEL_RADIUS;  // Left wheel
  current_position_rad[1] = position_meters[1] / WHEEL_RADIUS;  // Right wheel
  
  // Calculate position differences (what odometry needs)
  double position_diff_rad[JOINT_NUM];
  position_diff_rad[0] = current_position_rad[0] - last_position_rad[0];
  position_diff_rad[1] = current_position_rad[1] - last_position_rad[1];
  
  // Update cumulative position for joint_state message
  cumulative_position_rad[0] += position_diff_rad[0];
  cumulative_position_rad[1] += position_diff_rad[1];
  
  // Store current position for next iteration
  last_position_rad[0] = current_position_rad[0];
  last_position_rad[1] = current_position_rad[1];
  
  // Convert velocity from m/s to rad/s (for wheel angular velocity)
  double velocity_rad_per_sec_left = velocity_mps[0] / WHEEL_RADIUS;
  double velocity_rad_per_sec_right = velocity_mps[1] / WHEEL_RADIUS;

  msg->header.frame_id = this->frame_id_;
  msg->header.stamp = now;

  msg->name.push_back(wheel_left_joint_);
  msg->name.push_back(wheel_right_joint_);

  // Use cumulative position (radians) for joint_state 
  msg->position.push_back(cumulative_position_rad[0]);
  msg->position.push_back(cumulative_position_rad[1]);

  // Use converted rad/s directly for velocity
  msg->velocity.push_back(velocity_rad_per_sec_left);
  msg->velocity.push_back(velocity_rad_per_sec_right);

  pub_->publish(std::move(msg));
}
