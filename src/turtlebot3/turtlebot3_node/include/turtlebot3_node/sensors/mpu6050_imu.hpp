// Copyright 2023
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

#ifndef TURTLEBOT3_NODE__SENSORS__MPU6050_IMU_HPP_
#define TURTLEBOT3_NODE__SENSORS__MPU6050_IMU_HPP_

#ifdef __linux__
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#endif

#include <memory>
#include <string>
#include <mutex>
#include <chrono>
#include <cmath>
#include <thread>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <rclcpp/rclcpp.hpp>

#include "turtlebot3_node/sensors/sensors.hpp"

// MPU6050 Constants
#define MPU6050_ADDRESS         0x68
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_CONFIG          0x1A
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_GYRO_XOUT_H     0x43

namespace robotis
{
namespace turtlebot3
{
namespace sensors
{
class MPU6050Imu : public Sensors
{
public:
  explicit MPU6050Imu(
    std::shared_ptr<rclcpp::Node> & nh,
    const std::string & i2c_device = "/dev/i2c-1",
    const std::string & imu_topic_name = "imu",
    const std::string & frame_id = "imu_link");

  virtual ~MPU6050Imu();

  void publish(
    const rclcpp::Time & now,
    std::shared_ptr<XiaoBLEI2CWrapper> & dxl_sdk_wrapper) override;

private:
  // I2C communication functions
  bool init_mpu6050();
  bool read_raw_data(int16_t* accel_data, int16_t* gyro_data);
  bool i2c_write_byte(uint8_t reg, uint8_t value);
  bool i2c_read_bytes(uint8_t reg, uint8_t* data, int length);
  
  // Data processing functions
  void convert_raw_data(const int16_t* accel_raw, const int16_t* gyro_raw,
                       float* accel, float* gyro);
  void update_madgwick_filter(float ax, float ay, float az, 
                             float gx, float gy, float gz);
  void calibrate_gyro();
  
  // Madgwick filter implementation
  void madgwick_update(float gx, float gy, float gz, 
                      float ax, float ay, float az, float dt);
  
  // Advanced filtering functions
  void apply_low_pass_filter(float* accel, float* gyro);
  float calculate_adaptive_beta(const float* gyro);
  
  // Member variables
  int i2c_fd_;
  std::string name_space_;
  
  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  
  // Calibration data
  float gyro_offset_x_, gyro_offset_y_, gyro_offset_z_;
  bool is_calibrated_;
  
  // Filter parameters
  float beta_;  // Madgwick filter gain
  float q0_, q1_, q2_, q3_;  // Quaternion state
  
  // Advanced filter parameters
  float alpha_accel_;  // Low-pass filter alpha for accelerometer
  float alpha_gyro_;   // Low-pass filter alpha for gyroscope
  float prev_accel_[3];  // Previous accelerometer values for filtering
  float prev_gyro_[3];   // Previous gyroscope values for filtering
  
  // Adaptive beta calculation
  float beta_min_, beta_max_;
  bool use_adaptive_beta_;
  
  // Timing
  std::chrono::high_resolution_clock::time_point last_update_time_;
  
  // Mutex for thread safety
  std::mutex i2c_mutex_;
  
  // Conversion factors
  static constexpr float ACCEL_SCALE = 9.80665f / 8192.0f;   // ±4g range
  static constexpr float GYRO_SCALE = M_PI / (180.0f * 65.5f);  // ±500°/s range
};
}  // namespace sensors
}  // namespace turtlebot3
}  // namespace robotis
#endif  // TURTLEBOT3_NODE__SENSORS__MPU6050_IMU_HPP_ 