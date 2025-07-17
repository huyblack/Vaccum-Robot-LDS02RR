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

#include "turtlebot3_node/xiao_ble_i2c_wrapper.hpp"

using robotis::turtlebot3::XiaoBLEI2CWrapper;

XiaoBLEI2CWrapper::XiaoBLEI2CWrapper(const Device & device)
: i2c_fd_(-1), i2c_slave_addr_(device.i2c_addr)
{
  // Khởi tạo control table với kích thước đủ lớn (512 bytes)
  control_table_data_.resize(512, 0);

#ifdef __linux__
  // Mở thiết bị I2C
  i2c_fd_ = open(device.i2c_device.c_str(), O_RDWR);
  if (i2c_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("XiaoBLEI2CWrapper"), 
                 "Không thể mở thiết bị I2C: %s", device.i2c_device.c_str());
    return;
  }

  // Đặt địa chỉ slave I2C
  if (ioctl(i2c_fd_, I2C_SLAVE, i2c_slave_addr_) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("XiaoBLEI2CWrapper"), 
                 "Không thể đặt địa chỉ I2C slave: 0x%02X", i2c_slave_addr_);
    close(i2c_fd_);
    i2c_fd_ = -1;
    return;
  }
#endif

  RCLCPP_INFO(rclcpp::get_logger("XiaoBLEI2CWrapper"), 
              "Đã khởi tạo XiaoBLEI2CWrapper cho thiết bị %s, địa chỉ 0x%02X", 
              device.i2c_device.c_str(), i2c_slave_addr_);
}

XiaoBLEI2CWrapper::~XiaoBLEI2CWrapper()
{
#ifdef __linux__
  if (i2c_fd_ >= 0) {
    close(i2c_fd_);
  }
#endif

  if (read_memory_.data != nullptr) {
    delete[] read_memory_.data;
  }
}

void XiaoBLEI2CWrapper::init_read_memory(const uint16_t & start_addr, const uint16_t & length)
{
  if (read_memory_.data != nullptr) {
    delete[] read_memory_.data;
  }

  read_memory_.start_addr = start_addr;
  read_memory_.length = length;
  read_memory_.data = new uint8_t[length];
  memset(read_memory_.data, 0, length);
}

bool XiaoBLEI2CWrapper::i2c_write(uint8_t* data, size_t length)
{
#ifdef __linux__
  std::lock_guard<std::mutex> lock(i2c_mutex_);
  if (i2c_fd_ < 0) return false;
  
  ssize_t result = write(i2c_fd_, data, length);
  return (result == static_cast<ssize_t>(length));
#else
  return true;  // Giả lập thành công trên môi trường không phải Linux
#endif
}

bool XiaoBLEI2CWrapper::i2c_read(uint8_t* data, size_t length)
{
#ifdef __linux__
  std::lock_guard<std::mutex> lock(i2c_mutex_);
  if (i2c_fd_ < 0) return false;
  
  ssize_t result = read(i2c_fd_, data, length);
  return (result == static_cast<ssize_t>(length));
#else
  memset(data, 0, length);  // Giả lập dữ liệu rỗng
  return true;
#endif
}

bool XiaoBLEI2CWrapper::is_connected_to_device()
{
#ifdef __linux__
  if (i2c_fd_ < 0) return false;
  
  // Gửi lệnh PING để kiểm tra kết nối
  uint8_t cmd = CMD_PING;
  if (!i2c_write(&cmd, 1)) {
    return false;
  }
  
  delay_ms(10);  // Đợi 10ms
  
  uint8_t response;
  if (!i2c_read(&response, 1)) {
    return false;
  }
  
  return (response == 0xCC);  // Xiao BLE trả về 0xCC khi PING thành công
#else
  return true;  // Giả lập kết nối thành công
#endif
}

bool XiaoBLEI2CWrapper::control_motors(float linear_x, float angular_z, std::string * msg)
{
#ifdef __linux__
  uint8_t buffer[9];
  buffer[0] = CMD_SET_MOTOR;
  memcpy(buffer + 1, &linear_x, 4);
  memcpy(buffer + 5, &angular_z, 4);
  
  if (!i2c_write(buffer, 9)) {
    if (msg) *msg = "Lỗi gửi lệnh điều khiển động cơ";
    return false;
  }
  
  delay_ms(5);  // Đợi 5ms
  
  uint8_t response;
  if (!i2c_read(&response, 1)) {
    if (msg) *msg = "Lỗi đọc phản hồi điều khiển động cơ";
    return false;
  }
  
  if (response != 1) {
    if (msg) *msg = "Xiao BLE từ chối lệnh điều khiển động cơ";
    return false;
  }
  
  if (msg) *msg = "Điều khiển động cơ thành công";
  return true;
#else
  if (msg) *msg = "Giả lập: Điều khiển động cơ thành công";
  return true;
#endif
}

bool XiaoBLEI2CWrapper::set_motor_torque_enable(uint8_t enable, std::string * msg)
{
#ifdef __linux__
  uint8_t buffer[2];
  buffer[0] = CMD_MOTOR_TORQUE;
  buffer[1] = enable;
  
  if (!i2c_write(buffer, 2)) {
    if (msg) *msg = "Lỗi gửi lệnh bật/tắt motor torque";
    return false;
  }
  
  delay_ms(5);  // Đợi 5ms
  
  uint8_t response;
  if (!i2c_read(&response, 1)) {
    if (msg) *msg = "Lỗi đọc phản hồi motor torque";
    return false;
  }
  
  if (response != 1) {
    if (msg) *msg = "Xiao BLE từ chối lệnh motor torque";
    return false;
  }
  
  if (msg) *msg = enable ? "Đã bật motor torque" : "Đã tắt motor torque";
  return true;
#else
  if (msg) *msg = enable ? "Giả lập: Đã bật motor torque" : "Giả lập: Đã tắt motor torque";
  return true;
#endif
}

void XiaoBLEI2CWrapper::read_data_set()
{
#ifdef __linux__
  // Chỉ đọc trạng thái động cơ (không còn IMU)
  uint8_t cmd = CMD_GET_STATUS;
  if (!i2c_write(&cmd, 1)) {
    RCLCPP_ERROR(rclcpp::get_logger("XiaoBLEI2CWrapper"), "Lỗi gửi lệnh đọc trạng thái");
    return;
  }
  
  delay_ms(10);  // 10ms
  
  // Đọc đầy đủ trạng thái động cơ (6 float + 1 byte = 25 bytes)
  // present_current_left/right, present_velocity_left/right, 
  // present_position_left/right, motor_torque_enable
  uint8_t motor_buffer[25];
  if (!i2c_read(motor_buffer, sizeof(motor_buffer))) {
    RCLCPP_ERROR(rclcpp::get_logger("XiaoBLEI2CWrapper"), "Lỗi đọc trạng thái động cơ");
    return;
  }
  
  // Cập nhật vào control table
  {
    std::lock_guard<std::mutex> data_lock(read_data_mutex_);
    
    // Cập nhật dòng điện động cơ
    memcpy(&control_table_data_[120], motor_buffer, 4);     // present_current_left
    memcpy(&control_table_data_[124], motor_buffer + 4, 4); // present_current_right
    
    // Cập nhật vận tốc động cơ
    memcpy(&control_table_data_[128], motor_buffer + 8, 4);  // present_velocity_left
    memcpy(&control_table_data_[132], motor_buffer + 12, 4); // present_velocity_right
    
    // Cập nhật vị trí động cơ (encoder)
    memcpy(&control_table_data_[136], motor_buffer + 16, 4); // present_position_left
    memcpy(&control_table_data_[140], motor_buffer + 20, 4); // present_position_right
    
    // Cập nhật trạng thái motor torque enable
    control_table_data_[149] = motor_buffer[24];            // motor_torque_enable
    
    // Cập nhật thời gian
    uint32_t current_millis = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
    memcpy(&control_table_data_[10], &current_millis, 4);   // millis
    
    // Sao chép dữ liệu vào vùng đọc
    if (read_memory_.start_addr + read_memory_.length <= control_table_data_.size()) {
      memcpy(read_memory_.data, &control_table_data_[read_memory_.start_addr], read_memory_.length);
    }
  }
#else
  // Mô phỏng dữ liệu nếu không phải Linux
  std::lock_guard<std::mutex> lock(read_data_mutex_);
  
  // Cập nhật thời gian
  uint32_t current_millis = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
  memcpy(&control_table_data_[10], &current_millis, 4);   // millis
  
  // Mô phỏng dữ liệu encoder và vận tốc
  int32_t velocity[2] = {0, 0};
  int32_t position[2] = {0, 0};
  
  memcpy(&control_table_data_[128], velocity, 8);   // present_velocity_left, right
  memcpy(&control_table_data_[136], position, 8);   // present_position_left, right
  
  // Sao chép dữ liệu vào vùng đọc
  if (read_memory_.start_addr + read_memory_.length <= control_table_data_.size()) {
    memcpy(read_memory_.data, &control_table_data_[read_memory_.start_addr], read_memory_.length);
  }
#endif
}

bool XiaoBLEI2CWrapper::set_data_to_device(
  const uint16_t & addr,
  const uint16_t & length,
  uint8_t * data,
  std::string * msg)
{
  // Cập nhật control table local
  {
    std::lock_guard<std::mutex> lock(write_data_mutex_);
    if (addr + length <= control_table_data_.size()) {
      memcpy(&control_table_data_[addr], data, length);
    }
  }
  
#ifdef __linux__
  // Xử lý các lệnh đặc biệt
  if (addr == 149 && length == 1) {
    // Motor torque enable/disable
    return set_motor_torque_enable(data[0], msg);
  }
  else if (addr == 150 && length >= 4) {
    // Điều khiển động cơ với linear_x
    float linear_x = 0.0f;
    memcpy(&linear_x, data, 4);
    
    // Đọc angular_z từ control table
    float angular_z = 0.0f;
    {
      std::lock_guard<std::mutex> data_lock(read_data_mutex_);
      memcpy(&angular_z, &control_table_data_[170], 4);
    }
    
    return control_motors(linear_x, angular_z, msg);
  }
  else if (addr == 170 && length >= 4) {
    // Điều khiển động cơ với angular_z
    float angular_z = 0.0f;
    memcpy(&angular_z, data, 4);
    
    // Đọc linear_x từ control table
    float linear_x = 0.0f;
    {
      std::lock_guard<std::mutex> data_lock(read_data_mutex_);
      memcpy(&linear_x, &control_table_data_[150], 4);
    }
    
    return control_motors(linear_x, angular_z, msg);
  }
  
  if (msg) *msg = "Lệnh được xử lý thành công";
  return true;
#else
  if (msg) *msg = "Giả lập: Lệnh được xử lý thành công";
  return true;
#endif
}