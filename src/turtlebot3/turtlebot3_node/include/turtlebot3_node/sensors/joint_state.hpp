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

#ifndef TURTLEBOT3_NODE__SENSORS__JOINT_STATE_HPP_
#define TURTLEBOT3_NODE__SENSORS__JOINT_STATE_HPP_

#include <sensor_msgs/msg/joint_state.hpp>

#include <memory>
#include <string>

#include "turtlebot3_node/sensors/sensors.hpp"

namespace robotis
{
namespace turtlebot3
{
namespace sensors
{
constexpr uint8_t JOINT_NUM = 2;

// ref) http://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#goal-velocity104
constexpr double RPM_TO_MS = 0.229 * 0.0034557519189487725;

// Arduino encoder system constants (must match Arduino code)
constexpr double WHEEL_RADIUS = 0.035;  // 3.5cm wheel radius
constexpr double GEAR_RATIO = 65.45;    // Combined gear ratio from Arduino  
constexpr double PULSES_PER_MOTOR_REV = 4.0;  // Hall sensor pulses per motor revolution
constexpr double ENCODER_PPR = GEAR_RATIO * PULSES_PER_MOTOR_REV;  // ~261.8 pulses per wheel revolution
constexpr double DISTANCE_PER_PULSE = (2.0 * M_PI * WHEEL_RADIUS) / ENCODER_PPR;  // ~0.000839 m/pulse

// FIXED: Tính toán cho hệ thống encoder thực tế
// Hệ thống của chúng ta: GEAR_RATIO=65.45, PULSES_PER_MOTOR_REV=4.0
// ENCODER_PPR = 65.45 * 4 = 261.8 pulses per wheel revolution
// 1 revolution = 2π radians, so: 1 pulse = 2π / 261.8 = 0.023998 rad/pulse
constexpr double TICK_TO_RAD = 0.023998; // Arduino encoder: 261.8 ticks/rev

// OLD Dynamixel value (WRONG for our system):
// 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f (4096 ticks/rev)

class JointState : public Sensors
{
public:
  explicit JointState(
    std::shared_ptr<rclcpp::Node> & nh,
    std::shared_ptr<XiaoBLEI2CWrapper> & dxl_sdk_wrapper,
    const std::string & topic_name = "joint_states",
    const std::string & frame_id = "base_link");

  void publish(
    const rclcpp::Time & now,
    std::shared_ptr<XiaoBLEI2CWrapper> & dxl_sdk_wrapper) override;

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;

  std::string name_space_;
  std::string wheel_left_joint_ = "wheel_left_joint";
  std::string wheel_right_joint_ = "wheel_right_joint";
};
}  // namespace sensors
}  // namespace turtlebot3
}  // namespace robotis
#endif  // TURTLEBOT3_NODE__SENSORS__JOINT_STATE_HPP_
