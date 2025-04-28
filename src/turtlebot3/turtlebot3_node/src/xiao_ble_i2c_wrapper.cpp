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

#include <thread>
#include <chrono>
#include <iostream>

using robotis::turtlebot3::XiaoBLEI2CWrapper;

XiaoBLEI2CWrapper::XiaoBLEI2CWrapper(const Device & device)
: i2c_fd_(-1), i2c_slave_addr_(device.i2c_addr)
{
#ifdef __linux__
  // Mở thiết bị I2C
  i2c_fd_ = open(device.i2c_device.c_str(), O_RDWR);
  if (i2c_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("XiaoBLEI2CWrapper"), 
                "Không thể mở thiết bị I2C: %s", device.i2c_device.c_str());
    return;
  }

  // Đặt địa chỉ slave
  if (ioctl(i2c_fd_, I2C_SLAVE, i2c_slave_addr_) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("XiaoBLEI2CWrapper"), 
                "Không thể đặt địa chỉ slave: 0x%02X", i2c_slave_addr_);
    close(i2c_fd_);
    i2c_fd_ = -1;
    return;
  }


#else
  RCLCPP_ERROR(rclcpp::get_logger("XiaoBLEI2CWrapper"), 
              "I2C chỉ được hỗ trợ trên Linux");
#endif

  RCLCPP_INFO(rclcpp::get_logger("XiaoBLEI2CWrapper"), 
              "Đã khởi tạo kết nối I2C với Xiao BLE tại địa chỉ 0x%02X", i2c_slave_addr_);
              
  // Khởi tạo buffer control table mặc định có dung lượng đủ lớn
  control_table_data_.resize(512, 0);
  
  // Đặt một số giá trị mặc định quan trọng
  // Model number (đầu tiên được kiểm tra bởi TurtleBot3)
  control_table_data_[0] = 0x34;  // Giá trị ID cho Xiao BLE
  control_table_data_[1] = 0x12;

  // Kiểm tra kết nối với thiết bị
  if (!is_connected_to_device()) {
    RCLCPP_WARN(rclcpp::get_logger("XiaoBLEI2CWrapper"), 
                "Không thể kết nối với Xiao BLE - hãy kiểm tra kết nối I2C và địa chỉ thiết bị");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("XiaoBLEI2CWrapper"), 
                "Đã kết nối thành công với Xiao BLE");
  }
}

XiaoBLEI2CWrapper::~XiaoBLEI2CWrapper()
{
#ifdef __linux__
  if (i2c_fd_ >= 0) {
    close(i2c_fd_);
    RCLCPP_INFO(rclcpp::get_logger("XiaoBLEI2CWrapper"), "Đã đóng kết nối I2C");
  }
#endif
  
  if (read_memory_.data != nullptr) {
    delete[] read_memory_.data;
    read_memory_.data = nullptr;
  }
}

bool XiaoBLEI2CWrapper::is_connected_to_device()
{
#ifdef __linux__
  std::lock_guard<std::mutex> lock(i2c_mutex_);
  
  // Gửi lệnh ping
  uint8_t cmd = CMD_PING;
  if (!i2c_write(&cmd, 1)) {
    RCLCPP_ERROR(rclcpp::get_logger("XiaoBLEI2CWrapper"), "Lỗi gửi lệnh kiểm tra kết nối");
    return false;
  }
  
  // Đợi một chút cho Xiao xử lý
  delay_ms(10);  // 10ms
  
  // Đọc phản hồi
  uint8_t response;
  if (!i2c_read(&response, 1)) {
    RCLCPP_ERROR(rclcpp::get_logger("XiaoBLEI2CWrapper"), "Lỗi đọc phản hồi kiểm tra kết nối");
    return false;
  }
  
  return (response == 0xAA);
#else
  // Trong quá trình phát triển hoặc trên Windows, luôn trả về true
  return true;
#endif
}

void XiaoBLEI2CWrapper::init_read_memory(const uint16_t & start_addr, const uint16_t & length)
{
  // Lưu thông tin vùng nhớ để đọc
  read_memory_.start_addr = start_addr;
  read_memory_.length = length;
  
  // Cấp phát bộ nhớ cho dữ liệu
  if (read_memory_.data != nullptr) {
    delete[] read_memory_.data;
  }
  read_memory_.data = new uint8_t[length];
  memset(read_memory_.data, 0, length);
  
  RCLCPP_INFO(rclcpp::get_logger("XiaoBLEI2CWrapper"), 
              "Đã khởi tạo vùng đọc: Địa chỉ %d, Độ dài %d", start_addr, length);
}

void XiaoBLEI2CWrapper::read_data_set()
{
#ifdef __linux__
  std::lock_guard<std::mutex> lock(i2c_mutex_);
  
  // Yêu cầu dữ liệu IMU
  uint8_t cmd = CMD_GET_IMU;
  if (!i2c_write(&cmd, 1)) {
    RCLCPP_ERROR(rclcpp::get_logger("XiaoBLEI2CWrapper"), "Lỗi gửi lệnh đọc IMU");
    return;
  }
  
  delay_ms(10);  // 10ms
  
  // Đọc dữ liệu IMU (10 giá trị float = 40 bytes)
  // 6 giá trị IMU (gyro, accel) + 4 quaternion
  uint8_t imu_buffer[40];
  if (!i2c_read(imu_buffer, sizeof(imu_buffer))) {
    RCLCPP_ERROR(rclcpp::get_logger("XiaoBLEI2CWrapper"), "Lỗi đọc dữ liệu IMU");
    return;
  }
  
  // Cập nhật vào control table
  {
    std::lock_guard<std::mutex> data_lock(read_data_mutex_);
    
    // Lưu dữ liệu angular_velocity (gyro) - 12 bytes (3 giá trị float)
    memcpy(&control_table_data_[60], imu_buffer, 12);
    
    // Lưu dữ liệu linear_acceleration (accel) - 12 bytes (3 giá trị float)
    memcpy(&control_table_data_[72], imu_buffer + 12, 12);
    
    // Lưu dữ liệu quaternion (orientation) - 16 bytes (4 giá trị float)
    memcpy(&control_table_data_[96], imu_buffer + 24, 16);
    
    // Đọc trạng thái động cơ
    cmd = CMD_GET_STATUS;
    if (!i2c_write(&cmd, 1)) {
      RCLCPP_ERROR(rclcpp::get_logger("XiaoBLEI2CWrapper"), "Lỗi gửi lệnh đọc trạng thái");
      return;
    }
    
    delay_ms(10);  // 10ms
    
    // Đọc cả vận tốc và vị trí của cả hai động cơ (4 giá trị int32 = 16 bytes)
    uint8_t motor_buffer[16];
    if (!i2c_read(motor_buffer, sizeof(motor_buffer))) {
      RCLCPP_ERROR(rclcpp::get_logger("XiaoBLEI2CWrapper"), "Lỗi đọc trạng thái động cơ");
      return;
    }
    
    // Cập nhật vận tốc động cơ
    memcpy(&control_table_data_[128], motor_buffer, 4);     // present_velocity_left
    memcpy(&control_table_data_[132], motor_buffer + 4, 4); // present_velocity_right
    
    // Cập nhật vị trí động cơ (encoder)
    memcpy(&control_table_data_[136], motor_buffer + 8, 4);  // present_position_left
    memcpy(&control_table_data_[140], motor_buffer + 12, 4); // present_position_right
    
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
  
  // Mô phỏng dữ liệu IMU
  // Gyro data
  float gyro[3] = {0.01f, 0.02f, 0.03f};
  memcpy(&control_table_data_[60], gyro, 12);  // imu_angular_velocity_x,y,z
  
  // Accel data
  float accel[3] = {0.0f, 0.0f, 9.8f};
  memcpy(&control_table_data_[72], accel, 12); // imu_linear_acceleration_x,y,z
  
  // Đặt magnetic data thành zero (không sử dụng) nhưng vẫn chiếm vị trí trong control table
  float magnetic[3] = {0.0f, 0.0f, 0.0f};
  memcpy(&control_table_data_[84], magnetic, 12); // imu_magnetic_x,y,z
  
  // Quaternion (không xoay)
  float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // w, x, y, z
  memcpy(&control_table_data_[96], quat, 16);  // imu_orientation_w,x,y,z
  
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
  RCLCPP_DEBUG(rclcpp::get_logger("XiaoBLEI2CWrapper"), 
              "Yêu cầu ghi - Địa chỉ: %d, Độ dài: %d", addr, length);

  // In ra dữ liệu nhận được
  RCLCPP_INFO(rclcpp::get_logger("XiaoBLEI2CWrapper"), "Dữ liệu nhận được:");
  for(int i = 0; i < length; i++) {
    RCLCPP_INFO(rclcpp::get_logger("XiaoBLEI2CWrapper"), 
                "data[%d] = 0x%02X", i, data[i]);
  }
  
  float test_float;
  memcpy(&test_float, data, 4);
  RCLCPP_INFO(rclcpp::get_logger("XiaoBLEI2CWrapper"), 
              "Giá trị float kiểm tra: %f", test_float);
  
  // Cập nhật control table local
  {
    std::lock_guard<std::mutex> lock(write_data_mutex_);
    if (addr + length <= control_table_data_.size()) {
      memcpy(&control_table_data_[addr], data, length);
    }
  }
  
#ifdef __linux__
  std::lock_guard<std::mutex> lock(i2c_mutex_);
  
  // Xử lý các lệnh đặc biệt
  if (addr == 59 && length == 1 && data[0] == 1) {
    // Hiệu chuẩn IMU
    return calibrate_imu(msg);
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
    
    RCLCPP_INFO(rclcpp::get_logger("XiaoBLEI2CWrapper"), 
                "Giá trị đọc được - linear_x: %f, angular_z: %f", 
                linear_x, angular_z);
    
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

bool XiaoBLEI2CWrapper::calibrate_imu(std::string * msg)
{
#ifdef __linux__
  uint8_t cmd = CMD_CALIBRATE_IMU;
  if (!i2c_write(&cmd, 1)) {
    if (msg) *msg = "Lỗi gửi lệnh hiệu chuẩn IMU";
    return false;
  }
  
  // Đợi quá trình hiệu chuẩn hoàn tất
  delay_ms(3000);  // Đợi 3 giây để hoàn tất hiệu chuẩn
  
  uint8_t response;
  if (!i2c_read(&response, 1)) {
    if (msg) *msg = "Lỗi đọc phản hồi hiệu chuẩn IMU";
    return false;
  }
  
  if (response != 1) {
    if (msg) *msg = "Hiệu chuẩn IMU thất bại";
    return false;
  }
  
  if (msg) *msg = "Hiệu chuẩn IMU thành công";
  return true;
#else
  if (msg) *msg = "Giả lập: Hiệu chuẩn IMU thành công";
  return true;
#endif
}

bool XiaoBLEI2CWrapper::control_motors(float linear_x, float angular_z, std::string * msg)
{
#ifdef __linux__
  RCLCPP_INFO(rclcpp::get_logger("XiaoBLEI2CWrapper"), 
              "Gửi lệnh điều khiển động cơ - linear_x: %f, angular_z: %f", 
              linear_x, angular_z);
              
  uint8_t buffer[9];
  buffer[0] = CMD_SET_MOTOR;
  
  // Chuyển đổi float thành bytes
  union {
    float f;
    uint8_t b[4];
  } linear_union, angular_union;
  
  linear_union.f = linear_x;
  angular_union.f = angular_z;
  
  // Copy bytes vào buffer
  for(int i = 0; i < 4; i++) {
    buffer[1 + i] = linear_union.b[i];
    buffer[5 + i] = angular_union.b[i];
  }
  
  // In ra buffer để debug
  RCLCPP_INFO(rclcpp::get_logger("XiaoBLEI2CWrapper"), "Buffer I2C:");
  for(int i = 0; i < sizeof(buffer); i++) {
    RCLCPP_INFO(rclcpp::get_logger("XiaoBLEI2CWrapper"), 
                "buffer[%d] = 0x%02X", i, buffer[i]);
  }
  
  if (!i2c_write(buffer, sizeof(buffer))) {
    if (msg) *msg = "Lỗi gửi lệnh điều khiển động cơ";
    return false;
  }
  
  // Đợi phản hồi
  delay_ms(10);
  
  uint8_t response;
  if (!i2c_read(&response, 1)) {
    if (msg) *msg = "Lỗi đọc phản hồi điều khiển động cơ";
    return false;
  }
  
  if (response != 1) {
    if (msg) *msg = "Điều khiển động cơ thất bại";
    return false;
  }
  
  if (msg) *msg = "Điều khiển động cơ thành công";
  return true;
#else
  // Trong trường hợp mô phỏng, luôn trả về thành công
  if (msg) *msg = "Giả lập: Điều khiển động cơ thành công";
  return true;
#endif
}

bool XiaoBLEI2CWrapper::read_imu(float &quat_w, float &quat_x, float &quat_y, float &quat_z,
  float &gyro_x, float &gyro_y, float &gyro_z, 
  float &accel_x, float &accel_y, float &accel_z, 
  std::string * msg)
{
#ifdef __linux__
std::lock_guard<std::mutex> lock(i2c_mutex_);

uint8_t cmd = CMD_GET_IMU;
if (!i2c_write(&cmd, 1)) {
if (msg) *msg = "Lỗi gửi lệnh đọc IMU";
return false;
}

delay_ms(10);

uint8_t buffer[40];  // 10 giá trị float = 40 bytes
if (!i2c_read(buffer, sizeof(buffer))) {
if (msg) *msg = "Lỗi đọc dữ liệu IMU";
return false;
}

  // Chuyển đổi dữ liệu theo thứ tự Gyro → Accel → Quaternion
  // Đọc gyro từ 3 float đầu tiên
  memcpy(&gyro_x, buffer, 4);
  memcpy(&gyro_y, buffer + 4, 4);
  memcpy(&gyro_z, buffer + 8, 4);

  // Đọc accel từ 3 float tiếp theo
  memcpy(&accel_x, buffer + 12, 4);
  memcpy(&accel_y, buffer + 16, 4);
  memcpy(&accel_z, buffer + 20, 4);

  // Đọc quaternion từ 4 float cuối
  memcpy(&quat_w, buffer + 24, 4);
  memcpy(&quat_x, buffer + 28, 4);
  memcpy(&quat_y, buffer + 32, 4);
  memcpy(&quat_z, buffer + 36, 4);

  // Cập nhật control table với các địa chỉ đúng
  {
    std::lock_guard<std::mutex> data_lock(read_data_mutex_);
    
    // Lưu dữ liệu gyro (địa chỉ 60, 64, 68)
    memcpy(&control_table_data_[60], buffer, 12);
    
    // Lưu dữ liệu accel (địa chỉ 72, 76, 80)
    memcpy(&control_table_data_[72], buffer + 12, 12);
    
    // Lưu dữ liệu quaternion (địa chỉ 96, 100, 104, 108)
    memcpy(&control_table_data_[96], buffer + 24, 16);
  }

if (msg) *msg = "Đọc IMU thành công";
return true;
#else
// Giả lập dữ liệu quaternion
quat_w = 1.0f;
quat_x = 0.0f;
quat_y = 0.0f;
quat_z = 0.0f;

// Giả lập dữ liệu IMU
gyro_x = 0.01f;
gyro_y = 0.02f;
gyro_z = 0.03f;
accel_x = 0.0f;
accel_y = 0.0f;
accel_z = 9.8f;

if (msg) *msg = "Giả lập: Đọc IMU thành công";
return true;
#endif
}

bool XiaoBLEI2CWrapper::i2c_write(uint8_t* data, size_t length)
{
#ifdef __linux__
  if (i2c_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("XiaoBLEI2CWrapper"), "Thiết bị I2C chưa được mở");
    return false;
  }
  
  if (write(i2c_fd_, data, length) != static_cast<ssize_t>(length)) {
    RCLCPP_ERROR(rclcpp::get_logger("XiaoBLEI2CWrapper"), "Lỗi ghi dữ liệu I2C");
    return false;
  }
  
  return true;
#else
  return true;  // Luôn thành công trong môi trường mô phỏng
#endif
}

bool XiaoBLEI2CWrapper::i2c_read(uint8_t* data, size_t length)
{
#ifdef __linux__
  if (i2c_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("XiaoBLEI2CWrapper"), "Thiết bị I2C chưa được mở");
    return false;
  }
  
  if (read(i2c_fd_, data, length) != static_cast<ssize_t>(length)) {
    RCLCPP_ERROR(rclcpp::get_logger("XiaoBLEI2CWrapper"), "Lỗi đọc dữ liệu I2C");
    return false;
  }
  
  return true;
#else
  // Giả lập dữ liệu trong môi trường mô phỏng
  if (length == 1) {
    data[0] = 0xAA;  // Giá trị phản hồi PING
  } else {
    // Điền giá trị giả lập khác nếu cần
    memset(data, 0, length);
  }
  return true;
#endif
}