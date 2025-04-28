#include <Arduino.h>
#include <LSM6DS3.h>
#include <Wire.h>
#include <Adafruit_AHRS.h>
#include <PID_v1.h>

// --- Định nghĩa các chân cho động cơ và encoder ---
#define PWM_LEFT    0  // PWM cho động cơ trái
#define DIR_LEFT    1  // DIR cho động cơ trái
#define PWM_RIGHT   2  // PWM cho động cơ phải
#define DIR_RIGHT   3  // DIR cho động cơ phải

// Chân encoder (thay đổi theo thiết kế phần cứng)
#define ENC_LEFT_A  7  // Encoder A của động cơ trái
#define ENC_LEFT_B  8  // Encoder B của động cơ trái
#define ENC_RIGHT_A 10  // Encoder A của động cơ phải
#define ENC_RIGHT_B 9  // Encoder B của động cơ phải

// --- Cấu hình I2C và các lệnh ---
#define I2C_SLAVE_ADDR 0x08
#define CMD_PING           0x01
#define CMD_CALIBRATE_IMU  0x02
#define CMD_SET_MOTOR      0x03
#define CMD_GET_IMU        0x04
#define CMD_GET_STATUS     0x05

// --- Các thông số của robot ---
#define WHEEL_DIAMETER     0.066     // Đường kính bánh xe (m)
#define WHEEL_SEPARATION   0.160     // Khoảng cách giữa 2 bánh xe (m)
#define ENCODER_RESOLUTION 12        // Số xung encoder trên 1 vòng quay
#define GEAR_RATIO         1        // Tỉ số truyền (nếu có hộp giảm tốc)
#define MAX_SPEED          0.3       // Tốc độ tối đa (m/s)
#define CONTROL_INTERVAL   20        // Chu kỳ điều khiển (ms)

// --- Biến toàn cục cho IMU ---
LSM6DS3 imu(I2C_MODE, 0x6A);
Adafruit_Madgwick filter;

float gyro_x = 0.0, gyro_y = 0.0, gyro_z = 0.0;
float accel_x = 0.0, accel_y = 0.0, accel_z = 0.0;
float q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;  // w, x, y, z
float gyro_offset_x = 0.0, gyro_offset_y = 0.0, gyro_offset_z = 0.0;

// --- Biến toàn cục cho động cơ ---
// Tốc độ đặt
float g_left_speed_setpoint = 0.0;   // Tốc độ đặt cho bánh trái (m/s)
float g_right_speed_setpoint = 0.0;  // Tốc độ đặt cho bánh phải (m/s)

// Đọc từ encoder
volatile long enc_left_count = 0;    // Số xung encoder của bánh trái
volatile long enc_right_count = 0;   // Số xung encoder của bánh phải
long enc_left_prev = 0;              // Giá trị trước đó để tính vận tốc
long enc_right_prev = 0;             // Giá trị trước đó để tính vận tốc

// Vận tốc thực tế (đo được)
float g_left_speed_actual = 0.0;     // Vận tốc thực tế của bánh trái (m/s)
float g_right_speed_actual = 0.0;    // Vận tốc thực tế của bánh phải (m/s)

// --- Biến cho PID ---
// Tham số PID (điều chỉnh dựa trên thực nghiệm)
double kp = 2.0;    // Hệ số tỉ lệ
double ki = 5.0;    // Hệ số tích phân
double kd = 1.0;    // Hệ số vi phân

// Biến cho PID trái
double left_input = 0, left_output = 0, left_setpoint = 0;
PID leftPID(&left_input, &left_output, &left_setpoint, kp, ki, kd, DIRECT);

// Biến cho PID phải
double right_input = 0, right_output = 0, right_setpoint = 0;
PID rightPID(&right_input, &right_output, &right_setpoint, kp, ki, kd, DIRECT);

// --- Biến cho I2C ---
volatile uint8_t i2c_recv_buffer[32];
volatile uint8_t i2c_recv_len = 0;
volatile uint8_t i2c_cmd = 0;
uint8_t i2c_send_buffer[40];
uint8_t i2c_send_len = 0;

// --- Biến cho điều khiển và thời gian ---
unsigned long previous_imu_time = 0;
unsigned long previous_control_time = 0;
unsigned long previous_pid_time = 0;

void setup() {
  // Khởi tạo Serial
  Serial.begin(115200);
  Serial.println("Xiao BLE Motor Control với PID");
  
  // Khởi tạo I2C
  Wire.begin(I2C_SLAVE_ADDR);
  Wire.setClock(400000);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  
  // Khởi tạo IMU
  if (!imu.begin()) {
    Serial.println("Lỗi kết nối IMU");
  }
  imu.settings.gyroSampleRate = 208;
  imu.settings.accelSampleRate = 208;
  
  // Khởi tạo bộ lọc Madgwick
  filter.begin(100);
  
  // Khởi tạo chân động cơ
  pinMode(PWM_LEFT, OUTPUT);
  pinMode(DIR_LEFT, OUTPUT);
  pinMode(PWM_RIGHT, OUTPUT);
  pinMode(DIR_RIGHT, OUTPUT);
  
  // Khởi tạo encoder với interrupt
  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);
  
  // Đính kèm interrupt cho encoder
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), encLeftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), encRightISR, RISING);
  
  // Khởi tạo PID
  leftPID.SetMode(AUTOMATIC);
  leftPID.SetOutputLimits(0, 255);
  leftPID.SetSampleTime(CONTROL_INTERVAL);
  
  rightPID.SetMode(AUTOMATIC);
  rightPID.SetOutputLimits(0, 255);
  rightPID.SetSampleTime(CONTROL_INTERVAL);
  
  // Hiệu chuẩn IMU
  calibrateIMU();
  
  // Khởi tạo thời gian
  previous_imu_time = millis();
  previous_control_time = millis();
  previous_pid_time = millis();
  
  Serial.println("Khởi tạo hoàn tất!");
}

void loop() {
  // Xử lý lệnh từ I2C
  processCommand();
  
  // Cập nhật IMU (100Hz)
  unsigned long current_time = millis();
  if (current_time - previous_imu_time >= 10) {
    updateIMUData();
    previous_imu_time = current_time;
  }
  
  // Cập nhật vận tốc thực tế từ encoder (50Hz)
  if (current_time - previous_control_time >= 20) {
    updateActualSpeeds();
    previous_control_time = current_time;
  }
  
  // Cập nhật điều khiển PID (50Hz)
  if (current_time - previous_pid_time >= 20) {
    updatePIDControl();
    previous_pid_time = current_time;
  }
}

// --- Các hàm xử lý IMU ---
void calibrateIMU() {
  Serial.println("Đang hiệu chuẩn IMU...");
  
  float sum_gx = 0, sum_gy = 0, sum_gz = 0;
  int count = 200;
  
  // Bỏ qua các mẫu đầu
  for (int i = 0; i < 50; i++) {
    imu.readFloatGyroX();
    delay(5);
  }
  
  // Thu thập dữ liệu
  for (int i = 0; i < count; i++) {
    sum_gx += imu.readFloatGyroX();
    sum_gy += imu.readFloatGyroY();
    sum_gz += imu.readFloatGyroZ();
    delay(5);
  }
  
  gyro_offset_x = sum_gx / count;
  gyro_offset_y = sum_gy / count;
  gyro_offset_z = sum_gz / count;
  
  Serial.println("Hoàn thành hiệu chuẩn IMU!");
}

void updateIMUData() {
  // Đọc dữ liệu từ IMU
  gyro_x = imu.readFloatGyroX() - gyro_offset_x;
  gyro_y = imu.readFloatGyroY() - gyro_offset_y;
  gyro_z = imu.readFloatGyroZ() - gyro_offset_z;
  
  accel_x = imu.readFloatAccelX();
  accel_y = imu.readFloatAccelY();
  accel_z = imu.readFloatAccelZ();
  
  // Cập nhật bộ lọc Madgwick
  float gx_rad = gyro_x * DEG_TO_RAD;
  float gy_rad = gyro_y * DEG_TO_RAD;
  float gz_rad = gyro_z * DEG_TO_RAD;
  
  filter.updateIMU(gx_rad, gy_rad, gz_rad, accel_x, accel_y, accel_z);
  filter.getQuaternion(&q0, &q1, &q2, &q3);
}

// --- Các hàm xử lý encoder và vận tốc ---
// Hàm ngắt cho encoder trái
void encLeftISR() {
  if (digitalRead(ENC_LEFT_B) == LOW) {
    enc_left_count++;
  } else {
    enc_left_count--;
  }
}

// Hàm ngắt cho encoder phải
void encRightISR() {
  if (digitalRead(ENC_RIGHT_B) == LOW) {
    enc_right_count++;
  } else {
    enc_right_count--;
  }
}

// Cập nhật vận tốc thực tế từ encoder
void updateActualSpeeds() {
  // Đọc giá trị encoder hiện tại
  long enc_left_current = enc_left_count;
  long enc_right_current = enc_right_count;
  
  // Tính số xung thay đổi
  long enc_left_delta = enc_left_current - enc_left_prev;
  long enc_right_delta = enc_right_current - enc_right_prev;
  
  // Cập nhật giá trị trước đó
  enc_left_prev = enc_left_current;
  enc_right_prev = enc_right_current;
  
  // Chu vi bánh xe (m)
  float wheel_circumference = PI * WHEEL_DIAMETER;
  
  // Tính vận tốc (m/s)
  // vận tốc = (số xung / (số xung trên vòng * tỉ số truyền)) * chu vi / thời gian(s)
  float delta_time = CONTROL_INTERVAL / 1000.0;  // Chuyển ms sang s
  float pulses_per_revolution = ENCODER_RESOLUTION * GEAR_RATIO;
  
  g_left_speed_actual = (enc_left_delta / pulses_per_revolution) * wheel_circumference / delta_time;
  g_right_speed_actual = (enc_right_delta / pulses_per_revolution) * wheel_circumference / delta_time;
  
  // Cập nhật input cho PID
  left_input = g_left_speed_actual;
  right_input = g_right_speed_actual;
  
  // In thông tin debug (nếu cần)
  // Serial.print("L: "); Serial.print(g_left_speed_actual);
  // Serial.print(" R: "); Serial.println(g_right_speed_actual);
}

// --- Các hàm điều khiển động cơ ---
// Điều khiển động cơ từ vận tốc tuyến tính và góc
void controlMotors(float linear_x, float angular_z) {
  // Tính vận tốc bánh xe từ vận tốc tuyến tính và góc
  g_left_speed_setpoint = linear_x - (angular_z * WHEEL_SEPARATION / 2.0);
  g_right_speed_setpoint = linear_x + (angular_z * WHEEL_SEPARATION / 2.0);
  Serial.print("linear_x : "); Serial.print(linear_x);
  
  // Giới hạn vận tốc
  g_left_speed_setpoint = constrain(g_left_speed_setpoint, -MAX_SPEED, MAX_SPEED);
  g_right_speed_setpoint = constrain(g_right_speed_setpoint, -MAX_SPEED, MAX_SPEED);
  
  // Cập nhật setpoint cho PID
  left_setpoint = abs(g_left_speed_setpoint);
  right_setpoint = abs(g_right_speed_setpoint);
  
  Serial.print("Setpoint - L: "); Serial.print(g_left_speed_setpoint);
  Serial.print(" R: "); Serial.println(g_right_speed_setpoint);
}

// Cập nhật điều khiển PID
void updatePIDControl() {
  // Tính toán PID
  leftPID.Compute();
  rightPID.Compute();
  
  // Đặt hướng quay
  digitalWrite(DIR_LEFT, g_left_speed_setpoint >= 0 ? HIGH : LOW);
  digitalWrite(DIR_RIGHT, g_right_speed_setpoint >= 0 ? HIGH : LOW);
  
  // Đặt giá trị PWM dựa trên đầu ra PID
  analogWrite(PWM_LEFT, (int)left_output);
  analogWrite(PWM_RIGHT, (int)right_output);
}

// --- Các hàm I2C ---
void receiveEvent(int numBytes) {
  if (numBytes < 1) return;
  
  i2c_cmd = Wire.read();
  numBytes--;
  
  i2c_recv_len = 0;
  while (Wire.available() && i2c_recv_len < sizeof(i2c_recv_buffer)) {
    i2c_recv_buffer[i2c_recv_len++] = Wire.read();
  }
}

void requestEvent() {
  Wire.write(i2c_send_buffer, i2c_send_len);
}

void processCommand() {
  if (i2c_cmd == 0) return;
  
  switch (i2c_cmd) {
    case CMD_PING:
      i2c_send_buffer[0] = 0xAA;
      i2c_send_len = 1;
      break;
      
    case CMD_CALIBRATE_IMU:
      calibrateIMU();
      i2c_send_buffer[0] = 1;
      i2c_send_len = 1;
      break;
      
    case CMD_SET_MOTOR:
      if (i2c_recv_len >= 8) {
        // In ra từng byte để debug
        Serial.println("Received I2C data:");
        for(int i = 0; i < i2c_recv_len; i++) {
          Serial.print("Byte ");
          Serial.print(i);
          Serial.print(": 0x");
          Serial.println(i2c_recv_buffer[i], HEX);
        }
        
        // Kiểm tra endianness
        union {
          float f;
          uint8_t b[4];
        } linear_union, angular_union;
        
        // Copy dữ liệu vào union
        for(int i = 0; i < 4; i++) {
          linear_union.b[i] = i2c_recv_buffer[i];
          angular_union.b[i] = i2c_recv_buffer[i+4];
        }
        
        float linear_x = linear_union.f;
        float angular_z = angular_union.f;
        
        Serial.print("linear_x (decoded): ");
        Serial.println(linear_x, 4);
        Serial.print("angular_z (decoded): ");
        Serial.println(angular_z, 4);
        
        controlMotors(linear_x, angular_z);
        i2c_send_buffer[0] = 1;
        i2c_send_len = 1;
      } else {
        Serial.print("Invalid data length: ");
        Serial.println(i2c_recv_len);
        i2c_send_buffer[0] = 0;
        i2c_send_len = 1;
      }
      break;
      
    case CMD_GET_IMU:
      prepareIMUData();
      break;
      
    case CMD_GET_STATUS:
      prepareStatusData();
      break;
      
    default:
      i2c_send_buffer[0] = 0xFF;
      i2c_send_len = 1;
      break;
  }
  
  i2c_cmd = 0;
}

void prepareIMUData() {
  // Đóng gói dữ liệu IMU theo thứ tự phù hợp với control_table
  memcpy(i2c_send_buffer + 0, &gyro_x, 4);      // Gyro X
  memcpy(i2c_send_buffer + 4, &gyro_y, 4);      // Gyro Y
  memcpy(i2c_send_buffer + 8, &gyro_z, 4);      // Gyro Z
  memcpy(i2c_send_buffer + 12, &accel_x, 4);    // Accel X
  memcpy(i2c_send_buffer + 16, &accel_y, 4);    // Accel Y
  memcpy(i2c_send_buffer + 20, &accel_z, 4);    // Accel Z
  memcpy(i2c_send_buffer + 24, &q0, 4);         // Quaternion W
  memcpy(i2c_send_buffer + 28, &q1, 4);         // Quaternion X
  memcpy(i2c_send_buffer + 32, &q2, 4);         // Quaternion Y
  memcpy(i2c_send_buffer + 36, &q3, 4);         // Quaternion Z
  
  i2c_send_len = 40;
}

void prepareStatusData() {
  // Đóng gói dữ liệu trạng thái động cơ
  memcpy(i2c_send_buffer, &g_left_speed_actual, 4);     // Vận tốc thực tế bánh trái
  memcpy(i2c_send_buffer + 4, &g_right_speed_actual, 4);  // Vận tốc thực tế bánh phải
  
  // Có thể thêm thông tin PID nếu cần
  float left_error = left_setpoint - left_input;
  float right_error = right_setpoint - right_input;
  
  memcpy(i2c_send_buffer + 8, &left_error, 4);
  memcpy(i2c_send_buffer + 12, &right_error, 4);
  
  i2c_send_len = 16;  // 4 giá trị float
}