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

#ifndef TURTLEBOT3_NODE__XIAO_BLE_I2C_WRAPPER_HPP_
#define TURTLEBOT3_NODE__XIAO_BLE_I2C_WRAPPER_HPP_

#ifdef __linux__
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#endif

#include <string>
#include <vector>
#include <mutex>
#include <memory>
#include <cstring>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>

// Định nghĩa các lệnh I2C (đã loại bỏ IMU)
#define CMD_PING           0x01  // Kiểm tra kết nối
#define CMD_SET_MOTOR      0x03  // Điều khiển động cơ
#define CMD_GET_STATUS     0x05  // Đọc trạng thái
#define CMD_MOTOR_TORQUE   0x06  // Bật/tắt motor torque

// Địa chỉ I2C mặc định của Xiao BLE
#define DEFAULT_XIAO_BLE_I2C_ADDRESS 0x08

namespace robotis
{
namespace turtlebot3
{
class XiaoBLEI2CWrapper
{
public:
  typedef struct
  {
    std::string i2c_device;  // Đường dẫn thiết bị I2C (ví dụ: "/dev/i2c-1")
    uint8_t i2c_addr;        // Địa chỉ I2C của Xiao BLE
  } Device;

  explicit XiaoBLEI2CWrapper(const Device & device);
  virtual ~XiaoBLEI2CWrapper();

  template<typename DataByteT>
  DataByteT get_data_from_device(const uint16_t & addr, const uint16_t & length)
  {
    DataByteT data = 0;
    uint8_t * p_data = reinterpret_cast<uint8_t *>(&data);
    uint16_t index = addr - read_memory_.start_addr;

    std::lock_guard<std::mutex> lock(read_data_mutex_);
    switch (length) {
      case 1:
        p_data[0] = read_memory_.data[index + 0];
        break;

      case 2:
        p_data[0] = read_memory_.data[index + 0];
        p_data[1] = read_memory_.data[index + 1];
        break;

      case 4:
        p_data[0] = read_memory_.data[index + 0];
        p_data[1] = read_memory_.data[index + 1];
        p_data[2] = read_memory_.data[index + 2];
        p_data[3] = read_memory_.data[index + 3];
        break;

      default:
        p_data[0] = read_memory_.data[index + 0];
        break;
    }

    return data;
  }

  // Đặt dữ liệu cho thiết bị thông qua I2C
  bool set_data_to_device(
    const uint16_t & addr,
    const uint16_t & length,
    uint8_t * data,
    std::string * msg);

  // Khởi tạo vùng nhớ đọc
  void init_read_memory(const uint16_t & start_addr, const uint16_t & length);
  
  // Cập nhật dữ liệu từ Xiao BLE (chỉ motor status)
  void read_data_set();

  // Kiểm tra kết nối với thiết bị
  bool is_connected_to_device();
  
  // Điều khiển động cơ với vận tốc tuyến tính và góc
  bool control_motors(float linear_x, float angular_z, std::string * msg = nullptr);
  
  // Bật/tắt motor torque
  bool set_motor_torque_enable(uint8_t enable, std::string * msg = nullptr);

private:
  // Ghi dữ liệu qua I2C
  bool i2c_write(uint8_t* data, size_t length);
  
  // Đọc dữ liệu từ I2C
  bool i2c_read(uint8_t* data, size_t length);
  
  // Đợi một khoảng thời gian cụ thể (millisecond)
  void delay_ms(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
  
  // File descriptor cho thiết bị I2C
  int i2c_fd_;
  
  // Địa chỉ slave I2C
  uint8_t i2c_slave_addr_;
  
  // Giả lập control table (tương thích với DynamixelSDKWrapper)
  std::vector<uint8_t> control_table_data_;
  
  // Cấu trúc vùng nhớ
  typedef struct
  {
    uint16_t start_addr;
    uint16_t length;
    uint8_t * data;
  } Memory;

  Memory read_memory_ = {0, 0, nullptr};

  // Mutex bảo vệ dữ liệu và giao tiếp I2C
  std::mutex read_data_mutex_;
  std::mutex write_data_mutex_;
  std::mutex i2c_mutex_;
};
}  // namespace turtlebot3
}  // namespace robotis
#endif  // TURTLEBOT3_NODE__XIAO_BLE_I2C_WRAPPER_HPP_