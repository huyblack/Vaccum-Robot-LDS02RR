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

#include "turtlebot3_node/sensors/mpu6050_imu.hpp"

#include <memory>
#include <string>
#include <utility>
#include <cstring>

using robotis::turtlebot3::sensors::MPU6050Imu;

MPU6050Imu::MPU6050Imu(
  std::shared_ptr<rclcpp::Node> & nh,
  const std::string & i2c_device,
  const std::string & imu_topic_name,
  const std::string & frame_id)
: Sensors(nh, frame_id), i2c_fd_(-1), name_space_(nh->get_namespace()),
  gyro_offset_x_(0.0f), gyro_offset_y_(0.0f), gyro_offset_z_(0.0f),
  is_calibrated_(false), beta_(0.033f), q0_(1.0f), q1_(0.0f), q2_(0.0f), q3_(0.0f),
  alpha_accel_(0.02f), alpha_gyro_(0.98f), 
  beta_min_(0.01f), beta_max_(0.1f), use_adaptive_beta_(true)
{
  // Khởi tạo previous values
  for(int i = 0; i < 3; i++) {
    prev_accel_[i] = 0.0f;
    prev_gyro_[i] = 0.0f;
  }

  // Tạo publisher cho IMU data
  imu_pub_ = nh->create_publisher<sensor_msgs::msg::Imu>(imu_topic_name, this->qos_);

  nh_->get_parameter_or<std::string>(
    "namespace",
    name_space_,
    std::string(""));

  if (name_space_ != "") {
    frame_id_ = name_space_ + "/" + frame_id_;
  }

#ifdef __linux__
  // Mở thiết bị I2C
  i2c_fd_ = open(i2c_device.c_str(), O_RDWR);
  if (i2c_fd_ < 0) {
    RCLCPP_ERROR(nh_->get_logger(), "Không thể mở thiết bị I2C: %s", i2c_device.c_str());
    return;
  }

  // Đặt địa chỉ slave MPU6050
  if (ioctl(i2c_fd_, I2C_SLAVE, MPU6050_ADDRESS) < 0) {
    RCLCPP_ERROR(nh_->get_logger(), "Không thể đặt địa chỉ MPU6050");
    close(i2c_fd_);
    i2c_fd_ = -1;
    return;
  }

  // Khởi tạo MPU6050
  if (!init_mpu6050()) {
    RCLCPP_ERROR(nh_->get_logger(), "Không thể khởi tạo MPU6050");
    close(i2c_fd_);
    i2c_fd_ = -1;
    return;
  }

  // Hiệu chuẩn gyro
  calibrate_gyro();
#endif

  last_update_time_ = std::chrono::high_resolution_clock::now();
  
  RCLCPP_INFO(nh_->get_logger(), "Đã khởi tạo thành công MPU6050 IMU");
}

MPU6050Imu::~MPU6050Imu()
{
#ifdef __linux__
  if (i2c_fd_ >= 0) {
    close(i2c_fd_);
  }
#endif
}

bool MPU6050Imu::init_mpu6050()
{
#ifdef __linux__
  // Đánh thức MPU6050
  if (!i2c_write_byte(MPU6050_PWR_MGMT_1, 0x00)) {
    return false;
  }
  
  usleep(100000); // Đợi 100ms
  
  // Cấu hình Digital Low Pass Filter (DLPF) tốt hơn
  // 0x01 = 184Hz Accel, 188Hz Gyro, 1ms delay
  if (!i2c_write_byte(MPU6050_CONFIG, 0x01)) {
    return false;
  }
  
  // Cấu hình sample rate = 1kHz/(1+4) = 200Hz
  if (!i2c_write_byte(0x19, 0x04)) {  // SMPLRT_DIV register
    return false;
  }
  
  // Cấu hình thang đo gyro (±500°/s cho độ nhạy tốt hơn)
  if (!i2c_write_byte(MPU6050_GYRO_CONFIG, 0x08)) {
    return false;
  }
  
  // Cấu hình thang đo accelerometer (±4g cho động robot)
  if (!i2c_write_byte(MPU6050_ACCEL_CONFIG, 0x08)) {
    return false;
  }
  
  return true;
#else
  return true; // Giả lập thành công trên môi trường khác
#endif
}

bool MPU6050Imu::i2c_write_byte(uint8_t reg, uint8_t value)
{
#ifdef __linux__
  std::lock_guard<std::mutex> lock(i2c_mutex_);
  uint8_t data[2] = {reg, value};
  if (write(i2c_fd_, data, 2) != 2) {
    return false;
  }
  return true;
#else
  return true;
#endif
}

bool MPU6050Imu::i2c_read_bytes(uint8_t reg, uint8_t* data, int length)
{
#ifdef __linux__
  std::lock_guard<std::mutex> lock(i2c_mutex_);
  
  // Ghi địa chỉ register
  if (write(i2c_fd_, &reg, 1) != 1) {
    return false;
  }
  
  // Đọc dữ liệu
  if (read(i2c_fd_, data, length) != length) {
    return false;
  }
  
  return true;
#else
  // Giả lập dữ liệu
  memset(data, 0, length);
  return true;
#endif
}

bool MPU6050Imu::read_raw_data(int16_t* accel_data, int16_t* gyro_data)
{
  uint8_t buffer[14];
  
  // Đọc tất cả dữ liệu sensor trong một lần
  if (!i2c_read_bytes(MPU6050_ACCEL_XOUT_H, buffer, 14)) {
    return false;
  }
  
  // Chuyển đổi dữ liệu accelerometer (big-endian)
  accel_data[0] = (int16_t)((buffer[0] << 8) | buffer[1]);   // X
  accel_data[1] = (int16_t)((buffer[2] << 8) | buffer[3]);   // Y
  accel_data[2] = (int16_t)((buffer[4] << 8) | buffer[5]);   // Z
  
  // Bỏ qua dữ liệu nhiệt độ (buffer[6], buffer[7])
  
  // Chuyển đổi dữ liệu gyroscope (big-endian)
  gyro_data[0] = (int16_t)((buffer[8] << 8) | buffer[9]);    // X
  gyro_data[1] = (int16_t)((buffer[10] << 8) | buffer[11]);  // Y
  gyro_data[2] = (int16_t)((buffer[12] << 8) | buffer[13]);  // Z
  
  return true;
}

void MPU6050Imu::convert_raw_data(const int16_t* accel_raw, const int16_t* gyro_raw,
                                 float* accel, float* gyro)
{
  // Chuyển đổi accelerometer sang m/s²
  accel[0] = accel_raw[0] * ACCEL_SCALE;
  accel[1] = accel_raw[1] * ACCEL_SCALE;
  accel[2] = accel_raw[2] * ACCEL_SCALE;
  
  // Chuyển đổi gyroscope sang rad/s và áp dụng offset hiệu chuẩn
  gyro[0] = (gyro_raw[0] * GYRO_SCALE) - gyro_offset_x_;
  gyro[1] = (gyro_raw[1] * GYRO_SCALE) - gyro_offset_y_;
  gyro[2] = (gyro_raw[2] * GYRO_SCALE) - gyro_offset_z_;
  
  // Áp dụng low-pass filter
  apply_low_pass_filter(accel, gyro);
}

void MPU6050Imu::apply_low_pass_filter(float* accel, float* gyro)
{
  for(int i = 0; i < 3; i++) {
    // Low-pass filter cho accelerometer (loại bỏ noise tần số cao)
    accel[i] = alpha_accel_ * accel[i] + (1.0f - alpha_accel_) * prev_accel_[i];
    prev_accel_[i] = accel[i];
    
    // High-pass filter cho gyroscope (giữ lại thông tin động)
    gyro[i] = alpha_gyro_ * gyro[i] + (1.0f - alpha_gyro_) * prev_gyro_[i];
    prev_gyro_[i] = gyro[i];
  }
}

float MPU6050Imu::calculate_adaptive_beta(const float* gyro)
{
  if (!use_adaptive_beta_) {
    return beta_;
  }
  
  // Tính độ lớn của angular velocity
  float gyro_magnitude = sqrt(gyro[0]*gyro[0] + gyro[1]*gyro[1] + gyro[2]*gyro[2]);
  
  // Beta cao khi robot đang quay nhanh, beta thấp khi đứng yên
  float adaptive_beta = beta_min_ + (beta_max_ - beta_min_) * 
                       (1.0f - exp(-gyro_magnitude * 10.0f));
  
  return std::max(beta_min_, std::min(beta_max_, adaptive_beta));
}

void MPU6050Imu::calibrate_gyro()
{
  RCLCPP_INFO(nh_->get_logger(), "Bắt đầu hiệu chuẩn gyroscope... Giữ robot đứng yên");
  
  const int num_samples = 1000;
  float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
  
  for (int i = 0; i < num_samples; i++) {
    int16_t accel_raw[3], gyro_raw[3];
    
    if (read_raw_data(accel_raw, gyro_raw)) {
      sum_x += gyro_raw[0] * GYRO_SCALE;
      sum_y += gyro_raw[1] * GYRO_SCALE;
      sum_z += gyro_raw[2] * GYRO_SCALE;
    }
    
    std::this_thread::sleep_for(std::chrono::microseconds(5000)); // Đợi 5ms
  }
  
  gyro_offset_x_ = sum_x / num_samples;
  gyro_offset_y_ = sum_y / num_samples;
  gyro_offset_z_ = sum_z / num_samples;
  
  is_calibrated_ = true;
  
  RCLCPP_INFO(nh_->get_logger(), 
              "Hiệu chuẩn hoàn tất. Offset: X=%.4f, Y=%.4f, Z=%.4f", 
              gyro_offset_x_, gyro_offset_y_, gyro_offset_z_);
}

void MPU6050Imu::madgwick_update(float gx, float gy, float gz, 
                                float ax, float ay, float az, float dt)
{
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  // Tính adaptive beta dựa trên gyroscope magnitude
  float gyro_data[3] = {gx, gy, gz};
  float current_beta = calculate_adaptive_beta(gyro_data);

  // Normalize accelerometer measurement
  recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
  ax *= recipNorm;
  ay *= recipNorm;
  az *= recipNorm;

  // Auxiliary variables to avoid repeated arithmetic
  _2q0 = 2.0f * q0_;
  _2q1 = 2.0f * q1_;
  _2q2 = 2.0f * q2_;
  _2q3 = 2.0f * q3_;
  _4q0 = 4.0f * q0_;
  _4q1 = 4.0f * q1_;
  _4q2 = 4.0f * q2_;
  _8q1 = 8.0f * q1_;
  _8q2 = 8.0f * q2_;
  q0q0 = q0_ * q0_;
  q1q1 = q1_ * q1_;
  q2q2 = q2_ * q2_;
  q3q3 = q3_ * q3_;

  // Gradient decent algorithm corrective step
  s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
  s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1_ - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
  s2 = 4.0f * q0q0 * q2_ + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
  s3 = 4.0f * q1q1 * q3_ - _2q1 * ax + 4.0f * q2q2 * q3_ - _2q2 * ay;
  recipNorm = 1.0f / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
  s0 *= recipNorm;
  s1 *= recipNorm;
  s2 *= recipNorm;
  s3 *= recipNorm;

  // Apply feedback step
  qDot1 = 0.5f * (-q1_ * gx - q2_ * gy - q3_ * gz) - current_beta * s0;
  qDot2 = 0.5f * (q0_ * gx + q2_ * gz - q3_ * gy) - current_beta * s1;
  qDot3 = 0.5f * (q0_ * gy - q1_ * gz + q3_ * gx) - current_beta * s2;
  qDot4 = 0.5f * (q0_ * gz + q1_ * gy - q2_ * gx) - current_beta * s3;

  // Integrate rate of change of quaternion to yield quaternion
  q0_ += qDot1 * dt;
  q1_ += qDot2 * dt;
  q2_ += qDot3 * dt;
  q3_ += qDot4 * dt;

  // Normalise quaternion
  recipNorm = 1.0f / sqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
  q0_ *= recipNorm;
  q1_ *= recipNorm;
  q2_ *= recipNorm;
  q3_ *= recipNorm;
}

void MPU6050Imu::publish(
  const rclcpp::Time & now,
  std::shared_ptr<XiaoBLEI2CWrapper> & dxl_sdk_wrapper)
{
  (void)dxl_sdk_wrapper; // Không sử dụng tham số này nữa
  
  int16_t accel_raw[3], gyro_raw[3];
  float accel[3], gyro[3];
  
  // Đọc dữ liệu thô từ MPU6050
  if (!read_raw_data(accel_raw, gyro_raw)) {
    RCLCPP_WARN(nh_->get_logger(), "Không thể đọc dữ liệu từ MPU6050");
    return;
  }
  
  // Chuyển đổi dữ liệu thô
  convert_raw_data(accel_raw, gyro_raw, accel, gyro);
  
  // Tính toán dt cho Madgwick filter
  auto current_time = std::chrono::high_resolution_clock::now();
  float dt = std::chrono::duration<float>(current_time - last_update_time_).count();
  last_update_time_ = current_time;
  
  // Cập nhật Madgwick filter
  if (is_calibrated_ && dt > 0.0f && dt < 0.1f) { // Giới hạn dt hợp lý
    madgwick_update(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], dt);
  }
  
  // Tạo và publish IMU message
  auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
  
  imu_msg->header.frame_id = this->frame_id_;
  imu_msg->header.stamp = now;
  
  // Quaternion orientation
  imu_msg->orientation.w = q0_;
  imu_msg->orientation.x = q1_;
  imu_msg->orientation.y = q2_;
  imu_msg->orientation.z = q3_;
  
  // Angular velocity (rad/s)
  imu_msg->angular_velocity.x = gyro[0];
  imu_msg->angular_velocity.y = gyro[1];
  imu_msg->angular_velocity.z = gyro[2];
  
  // Linear acceleration (m/s²)
  imu_msg->linear_acceleration.x = accel[0];
  imu_msg->linear_acceleration.y = accel[1];
  imu_msg->linear_acceleration.z = accel[2];
  
  // Đặt covariance dựa trên chất lượng sensor thực tế
  for (int i = 0; i < 9; i++) {
    imu_msg->orientation_covariance[i] = 0.0;
    imu_msg->angular_velocity_covariance[i] = 0.0;
    imu_msg->linear_acceleration_covariance[i] = 0.0;
  }
  
  // Orientation covariance (tốt hơn nhờ Madgwick filter)
  imu_msg->orientation_covariance[0] = 0.001;  // x
  imu_msg->orientation_covariance[4] = 0.001;  // y  
  imu_msg->orientation_covariance[8] = 0.01;   // z (yaw ít chính xác hơn)
  
  // Angular velocity covariance (MPU6050 ±500°/s)
  imu_msg->angular_velocity_covariance[0] = 0.0001;
  imu_msg->angular_velocity_covariance[4] = 0.0001;
  imu_msg->angular_velocity_covariance[8] = 0.0001;
  
  // Linear acceleration covariance (MPU6050 ±4g)
  imu_msg->linear_acceleration_covariance[0] = 0.01;
  imu_msg->linear_acceleration_covariance[4] = 0.01;
  imu_msg->linear_acceleration_covariance[8] = 0.01;
  
  imu_pub_->publish(std::move(imu_msg));
} 