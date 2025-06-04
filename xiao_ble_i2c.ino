// Arduino sketch cho Xiao BLE với giao tiếp I2C
#include <Arduino.h>
#include <LSM6DS3.h>    // Thư viện IMU
#include <Wire.h>       // Thư viện I2C
#include <Adafruit_AHRS.h>    // Thư viện Adafruit AHRS
#include <PID_v1.h>


Adafruit_Madgwick filter; // Thuật toán Madgwick

#define I2C_SLAVE_ADDR 0x08   // Địa chỉ I2C của Xiao BLE (có thể thay đổi)

// Định nghĩa các lệnh I2C
#define CMD_PING          0x01  // Kiểm tra kết nối
#define CMD_CALIBRATE_IMU 0x02  // Hiệu chuẩn IMU
#define CMD_SET_MOTOR     0x03  // Điều khiển động cơ
#define CMD_GET_IMU       0x04  // Đọc dữ liệu IMU
#define CMD_GET_STATUS    0x05  // Đọc trạng thái

#define G_STANDARD        9.80665 // Giá trị tiêu chuẩn chính xác

#define PWM_LEFT  0
#define DIR_LEFT  1
#define PWM_RIGH  2
#define DIR_RIGH  3

LSM6DS3 imu(I2C_MODE, 0x6A);    // I2C mode với địa chỉ 0x6A

// Biến toàn cục cho động cơ
float g_left_speed = 0.0;
float g_right_speed = 0.0;

// Offset cho hiệu chuẩn IMU
float gyro_offset_x = 0.0, gyro_offset_y = 0.0, gyro_offset_z = 0.0;

// Dữ liệu IMU hiện tại
float gyro_x = 0.0, gyro_y = 0.0, gyro_z = 0.0;
float accel_x = 0.0, accel_y = 0.0, accel_z = 0.0;

// Dữ liệu quaternion
float q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;   // w, x, y, z

// Thời gian cho thuật toán Madgwick
unsigned long previousTime = 0;
float deltaTime = 0.0;

// --- Thời gian cho bộ lọc AHRS ---
#define FILTER_UPDATE_RATE_HZ 100 // Tần số cập nhật bộ lọc (Hz)
#define FILTER_UPDATE_INTERVAL_MS (1000 / FILTER_UPDATE_RATE_HZ) // Khoảng thời gian cập nhật (ms)

// Buffer nhận dữ liệu I2C
volatile uint8_t i2c_recv_buffer[32];
volatile uint8_t i2c_recv_len = 0;
volatile uint8_t i2c_cmd = 0;

// Buffer gửi dữ liệu I2C
uint8_t i2c_send_buffer[40];
uint8_t i2c_send_len = 0;

// --- Các biến mới cho LED và trạng thái giao tiếp ---
#define LED_PIN         LED_GREEN // Sử dụng LED_BUILTIN hoặc thay bằng chân GPIO bạn muốn dùng cho LED đỏ
#define I2C_TIMEOUT_MS   500        // Thời gian timeout để coi là không có giao tiếp (ms)

unsigned long lastI2CActivityTime = 0; // Thời điểm cuối cùng nhận được dữ liệu I2C
unsigned long lastLedToggleTime = 0;   // Thời điểm cuối cùng LED đổi trạng thái
bool ledState = LOW;                 // Trạng thái hiện tại của LED

// --- Khai báo hàm mới ---
void updateLedStatus();

void setup() {
  // Debug qua USB (tùy chọn)
  Serial.begin(115200);
  
  // Khởi tạo I2C slave
  Wire.begin(I2C_SLAVE_ADDR);
  Wire.setClock(1000000);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  
  // Khởi tạo IMU
  if (!imu.begin()) {
    Serial.println("Lỗi kết nối IMU");
  }

  // Đặt khoảng thời gian đo IMU
  imu.settings.gyroSampleRate = 208;    // Hz
  imu.settings.accelSampleRate = 208; // Hz
  
  // Khởi tạo Madgwick filter với tần số cập nhật 100Hz
  filter.begin(FILTER_UPDATE_RATE_HZ);
  Serial.println("Khoi tao bo loc AHRS thanh cong.");

  // Khởi tạo chân điều khiển động cơ
  pinMode(PWM_LEFT, OUTPUT);    // PWM cho động cơ trái
  pinMode(DIR_LEFT, OUTPUT);    // DIR cho động cơ trái
  pinMode(PWM_RIGH, OUTPUT);    // PWM cho động cơ phải
  pinMode(DIR_RIGH, OUTPUT);    // DIR cho động cơ phải

  // --- Khởi tạo chân LED ---
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Đảm bảo LED tắt ban đầu

  previousTime = millis();
  
  Serial.println("Xiao BLE đã sẵn sàng (I2C Slave mode)");
}

void loop() {
  // Xử lý lệnh nhận được qua I2C
  processCommand();
  
  // Đọc IMU và cập nhật dữ liệu
  if ((millis() - previousTime) >= FILTER_UPDATE_INTERVAL_MS) {
    updateIMUData();       // Đọc IMU, cập nhật bộ lọc, lấy quaternion
    previousTime = millis(); // Cập nhật thời điểm
  }
  
  // Điều khiển động cơ theo tốc độ hiện tại
  updateMotors();
  
  // --- Cập nhật trạng thái LED ---
  updateLedStatus();

  delay(1);   // Giảm tải CPU
}

// Hàm callback khi có dữ liệu nhận từ master
void receiveEvent(int numBytes) {
  // --- Cập nhật thời gian nhận I2C để theo dõi trạng thái giao tiếp ---
  lastI2CActivityTime = millis();

  if (numBytes < 1) return;
  
  // Đọc lệnh
  i2c_cmd = Wire.read();
  numBytes--;
  
  // Đọc dữ liệu còn lại vào buffer
  i2c_recv_len = 0;
  while (Wire.available() && i2c_recv_len < sizeof(i2c_recv_buffer)) {
    i2c_recv_buffer[i2c_recv_len++] = Wire.read();
  }
}

// Hàm callback khi master yêu cầu dữ liệu
void requestEvent() {
  // Gửi dữ liệu trong buffer trả về
  Wire.write(i2c_send_buffer, i2c_send_len);
}

// Xử lý lệnh nhận được qua I2C
void processCommand() {
  if (i2c_cmd == 0) return;   // Không có lệnh mới
  
  switch (i2c_cmd) {
    case CMD_PING:
      // Chuẩn bị phản hồi ping
      i2c_send_buffer[0] = 0xAA;   // Magic value
      i2c_send_len = 1;
      break;
      
    case CMD_CALIBRATE_IMU:
      // Hiệu chuẩn IMU
      calibrateIMU();
      
      // Trả về trạng thái hoàn thành
      i2c_send_buffer[0] = 1;   // 1 = thành công
      i2c_send_len = 1;
      break;
      
    case CMD_SET_MOTOR:
      // Đọc vận tốc từ buffer
      if (i2c_recv_len >= 8) {
        float linear_x, angular_z;
        
        // Sao chép dữ liệu từ buffer vào biến
        memcpy(&linear_x, (void*)i2c_recv_buffer, 4);
        memcpy(&angular_z, (void*)(i2c_recv_buffer + 4), 4);
        
        // Điều khiển động cơ
        controlMotors(linear_x, angular_z);
        
        // Chuẩn bị phản hồi thành công
        i2c_send_buffer[0] = 1;
        i2c_send_len = 1;
      } else {
        // Dữ liệu không đủ
        i2c_send_buffer[0] = 0;
        i2c_send_len = 1;
      }
      break;
      
    case CMD_GET_IMU:
      // Chuẩn bị dữ liệu IMU để gửi về
      prepareIMUData();
      break;
      
    case CMD_GET_STATUS:
      // Chuẩn bị dữ liệu trạng thái
      prepareStatusData();
      break;
      
    default:
      // Lệnh không được hỗ trợ
      i2c_send_buffer[0] = 0xFF;   // Mã lỗi
      i2c_send_len = 1;
      break;
  }
  
  // Đã xử lý xong lệnh
  i2c_cmd = 0;
}

// Chuẩn bị dữ liệu IMU để trả về
void prepareIMUData() {
  // Chuyển dữ liệu IMU sang mảng bytes
  // Thêm quaternion vào dữ liệu trả về
  memcpy(i2c_send_buffer + 0,    &gyro_x, 4);   // [0-3]: Gyro X (địa chỉ 60)
  memcpy(i2c_send_buffer + 4,    &gyro_y, 4);   // [4-7]: Gyro Y (địa chỉ 64)
  memcpy(i2c_send_buffer + 8,    &gyro_z, 4);   // [8-11]: Gyro Z (địa chỉ 68)
  memcpy(i2c_send_buffer + 12, &accel_x, 4);   // [12-15]: Accel X (địa chỉ 72)
  memcpy(i2c_send_buffer + 16, &accel_y, 4);   // [16-19]: Accel Y (địa chỉ 76)
  memcpy(i2c_send_buffer + 20, &accel_z, 4);   // [20-23]: Accel Z (địa chỉ 80)
  memcpy(i2c_send_buffer + 24, &q0, 4);       // [24-27]: Quaternion W (địa chỉ 96)
  memcpy(i2c_send_buffer + 28, &q1, 4);       // [28-31]: Quaternion X (địa chỉ 100)
  memcpy(i2c_send_buffer + 32, &q2, 4);       // [32-35]: Quaternion Y (địa chỉ 104)
  memcpy(i2c_send_buffer + 36, &q3, 4);       // [36-39]: Quaternion Z (địa chỉ 108)
  i2c_send_len = 40;   // 10 giá trị float, mỗi giá trị 4 bytes
}

// Chuẩn bị dữ liệu trạng thái
void prepareStatusData() {
  // Trả về trạng thái động cơ
  memcpy(i2c_send_buffer, &g_left_speed, 4);
  memcpy(i2c_send_buffer + 4, &g_right_speed, 4);
  
  // Thêm các thông tin khác nếu cần
  
  i2c_send_len = 8;   // 2 giá trị float, mỗi giá trị 4 bytes
}

void calibrateIMU() {
  Serial.println("Bắt đầu hiệu chuẩn IMU");
  
  // Khởi tạo các biến tổng
  float sum_gx = 0, sum_gy = 0, sum_gz = 0;
  int count = 200;
  
  // Thu thập dữ liệu trong khi đứng yên
  for (int i = 0; i < count; i++) {
    sum_gx += imu.readFloatGyroX();
    sum_gy += imu.readFloatGyroY();
    sum_gz += imu.readFloatGyroZ();
    delay(5);
  }
  
  // Tính toán offset
  gyro_offset_x = sum_gx / count;
  gyro_offset_y = sum_gy / count;
  gyro_offset_z = sum_gz / count;

  // Reset quaternion về giá trị mặc định (không xoay)
  q0 = 1.0;
  q1 = 0.0;
  q2 = 0.0;
  q3 = 0.0;
  
  Serial.println("Hoàn thành hiệu chuẩn IMU");
}

void updateIMUData() {
  
  // Đọc giá trị từ IMU và áp dụng offset
  gyro_x = imu.readFloatGyroX() - gyro_offset_x;
  gyro_y = imu.readFloatGyroY() - gyro_offset_y;
  gyro_z = imu.readFloatGyroZ() - gyro_offset_z;

  float raw_ax_g = imu.readFloatAccelX();
  float raw_ay_g = imu.readFloatAccelY();
  float raw_az_g = imu.readFloatAccelZ();

  accel_x = raw_ax_g * G_STANDARD;
  accel_y = raw_ay_g * G_STANDARD;
  accel_z = raw_az_g * G_STANDARD;
  // Chuyển đổi đơn vị từ deg/s sang rad/s cho thuật toán Madgwick
  float gx_rad = gyro_x * DEG_TO_RAD;
  float gy_rad = gyro_y * DEG_TO_RAD;
  float gz_rad = gyro_z * DEG_TO_RAD;
  
  // Cập nhật thuật toán Madgwick
  filter.updateIMU(gx_rad, gy_rad, gz_rad, raw_ax_g, raw_ay_g, raw_az_g);
  filter.getQuaternion(&q0, &q1, &q2, &q3);
}

void controlMotors(float linear_x, float angular_z) {
  float wheel_separation = 0.160;   // Khoảng cách giữa hai bánh xe (m)
  
  // Tính toán vận tốc cho từng bánh xe
  g_left_speed = linear_x - (angular_z * wheel_separation / 2.0);
  g_right_speed = linear_x + (angular_z * wheel_separation / 2.0);
  
  Serial.print("Motor control: L=");
  Serial.print(g_left_speed);
  Serial.print(" R=");
  Serial.println(g_right_speed);
}

void updateMotors() {
  // Chuyển đổi từ vận tốc (m/s) sang giá trị PWM (0-255)
  // Giả sử vận tốc tối đa 0.3 m/s tương ứng với PWM 255.
  int left_pwm = map(abs(g_left_speed * 100), 0, 30, 0, 255);
  int right_pwm = map(abs(g_right_speed * 100), 0, 30, 0, 255);
  
  // Đặt hướng quay
  // Đối với một số module điều khiển động cơ, HIGH/LOW có thể ngược lại tùy theo cách đấu dây.
  digitalWrite(DIR_RIGH, g_left_speed >= 0 ? HIGH : LOW);
  digitalWrite(DIR_LEFT, g_right_speed >= 0 ? HIGH : LOW);
  
  // Đặt tốc độ PWM
  analogWrite(PWM_LEFT, left_pwm);
  analogWrite(PWM_RIGH, right_pwm);
}

// --- Hàm mới: Cập nhật trạng thái nháy LED ---
void updateLedStatus() {
  unsigned long currentTime = millis();
  unsigned long blinkInterval;

  // Kiểm tra nếu có giao tiếp I2C gần đây
  if ((currentTime - lastI2CActivityTime) < I2C_TIMEOUT_MS) {
    // Có giao tiếp I2C, nháy 5Hz (chu kỳ 200ms, mỗi trạng thái 100ms)
    blinkInterval = 1000 / 5 / 2;
  } else {
    // Không có giao tiếp I2C, nháy 1Hz (chu kỳ 1000ms, mỗi trạng thái 500ms)
    blinkInterval = 1000 / 1 / 2;
  }

  // Đổi trạng thái LED nếu đã đến lúc
  if ((currentTime - lastLedToggleTime) >= blinkInterval) {
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
    lastLedToggleTime = currentTime;
  }
}