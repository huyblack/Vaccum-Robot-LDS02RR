#include <Arduino.h>
#include <LSM6DS3.h>
#include <Wire.h>
#include <Adafruit_AHRS.h>
#include <PID_v1.h>         // Thư viện PID chính

// --- Định nghĩa chân ---
// *** KIỂM TRA KỸ VÀ THAY ĐỔI CHO PHÙ HỢP VỚI XIAO BLE CỦA BẠN ***
// Motor Trái
#define PWM_L 3   // Chân PWM Motor Trái
#define IN1_L 6   // Chân DIR 1 Motor Trái (Ví dụ: HIGH=Forward, LOW=Backward với IN2_L=LOW)
#define IN2_L 7   // Chân DIR 2 Motor Trái (Giữ LOW hoặc dùng cho Brake)
#define ENCA_L 0  // Chân Encoder A Trái (Cần hỗ trợ ngắt)
#define ENCB_L 10 // Chân Encoder B Trái (Cần hỗ trợ ngắt) - Đã đổi từ code trước

// Motor Phải
#define PWM_R 5   // Chân PWM Motor Phải (Ví dụ D5)
#define IN1_R 8   // Chân DIR 1 Motor Phải (Ví dụ D8)
#define IN2_R 9   // Chân DIR 2 Motor Phải (Ví dụ D9)
#define ENCA_R 2  // Chân Encoder A Phải (Ví dụ D2 - Cần hỗ trợ ngắt)
#define ENCB_R 4  // Chân Encoder B Phải (Ví dụ D4 - Cần hỗ trợ ngắt)

// --- Cấu hình I2C và các lệnh ---
#define I2C_SLAVE_ADDR 0x08
#define CMD_PING           0x01
#define CMD_CALIBRATE_IMU  0x02
#define CMD_SET_MOTOR      0x03 // Nhận linear_x, angular_z
#define CMD_GET_IMU        0x04
#define CMD_GET_ODOM       0x06 // Lệnh lấy dữ liệu Encoder ticks L/R

// --- Các thông số của robot và động cơ ---
// *** ĐIỀN CÁC GIÁ TRỊ THỰC TẾ CỦA BẠN VÀO ĐÂY ***
#define WHEEL_DIAMETER         0.066  // Đường kính bánh xe (mét)
#define WHEEL_RADIUS           (WHEEL_DIAMETER / 2.0)
#define WHEEL_CIRCUMFERENCE    (PI * WHEEL_DIAMETER)
#define WHEEL_SEPARATION       0.160  // Khoảng cách giữa hai bánh xe (mét)
#define ENCODER_PPR            11     // Số xung/vòng encoder (trên trục motor) - Ví dụ: 11
#define GEAR_RATIO             46.8   // Tỉ số truyền động cơ - Ví dụ: 46.8
#define COUNTS_PER_MOTOR_REV_4X (ENCODER_PPR * 4) // 44 nếu PPR=11
#define COUNTS_PER_OUTPUT_REV_4X (COUNTS_PER_MOTOR_REV_4X * GEAR_RATIO) // 2059.2 nếu PPR=11, GR=46.8
#define CONTROL_INTERVAL       20     // Chu kỳ điều khiển PID và tính tốc độ (ms) -> 50Hz

// --- Biến toàn cục cho IMU ---
LSM6DS3 imu(I2C_MODE, 0x6A);
Adafruit_Madgwick filter;
float gyro_x = 0.0, gyro_y = 0.0, gyro_z = 0.0;
float accel_x = 0.0, accel_y = 0.0, accel_z = 0.0;
float q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;
float gyro_offset_x = 0.0, gyro_offset_y = 0.0, gyro_offset_z = 0.0;
#define G_STANDARD 9.80665

// --- Biến cho Encoder và RPM (Trái và Phải) ---
volatile long pos_i_L = 0;
volatile long pos_i_R = 0;
long posPrev_L = 0;
long posPrev_R = 0;
float v1Filt_L = 0;
float v1Filt_R = 0;
float v1Prev_L = 0;
float v1Prev_R = 0;
volatile byte last_AB_state_L = 0;
volatile byte last_AB_state_R = 0;
unsigned long prevControlTime = 0; // Thay thế previous_control_time và previous_pid_time

// --- Biến cho PID (Trái và Phải) ---
// PID Inputs/Outputs/Setpoints (RPM)
double target_rpm_L = 0.0;
double target_rpm_R = 0.0;
double current_rpm_L = 0.0; // Input PID L (RPM đã lọc)
double current_rpm_R = 0.0; // Input PID R (RPM đã lọc)
double motor_output_L = 0.0; // Output PID L (PWM 0-255)
double motor_output_R = 0.0; // Output PID R (PWM 0-255)

// Hướng mục tiêu (tính từ vận tốc)
int target_dir_L = 0;
int target_dir_R = 0;

// Hệ số PID cuối cùng đã chọn
// *** ĐIỀN GIÁ TRỊ ĐÃ TUNING CỦA BẠN ***
double Kp_param = 1.0; // Có thể tách Kp_L, Kp_R nếu cần
double Ki_param = 10.0; // Có thể tách Ki_L, Ki_R nếu cần
double Kd_param = 0.03; // Có thể tách Kd_L, Kd_R nếu cần

// Khởi tạo đối tượng PID
PID motorPID_L(&current_rpm_L, &motor_output_L, &target_rpm_L, Kp_param, Ki_param, Kd_param, DIRECT);
PID motorPID_R(&current_rpm_R, &motor_output_R, &target_rpm_R, Kp_param, Ki_param, Kd_param, DIRECT);

// --- Biến I2C ---
volatile uint8_t i2c_recv_buffer[32];
volatile uint8_t i2c_recv_len = 0;
volatile uint8_t i2c_cmd = 0;
uint8_t i2c_send_buffer[40]; // Kích thước đủ cho IMU hoặc Odom
uint8_t i2c_send_len = 0;

// --- Thời gian ---
unsigned long previousImuTime = 0;

// --- Bảng tra 4X Encoder ---
const int8_t QEM [16] = {0, +1, -1, 0, -1, 0, 0, +1, +1, 0, 0, -1, 0, -1, +1, 0};

// --- Hàm ISR 4X cho Encoder Trái ---
void quadEncoderISR_L() {
  byte current_A = digitalRead(ENCA_L);
  byte current_B = digitalRead(ENCB_L);
  byte current_AB_state = (current_A << 1) | current_B;
  if (current_AB_state == last_AB_state_L) return;
  int8_t change = QEM[(last_AB_state_L << 2) | current_AB_state];
  pos_i_L += change;
  last_AB_state_L = current_AB_state;
}

// --- Hàm ISR 4X cho Encoder Phải ---
void quadEncoderISR_R() {
  byte current_A = digitalRead(ENCA_R);
  byte current_B = digitalRead(ENCB_R);
  byte current_AB_state = (current_A << 1) | current_B;
  if (current_AB_state == last_AB_state_R) return;
  int8_t change = QEM[(last_AB_state_R << 2) | current_AB_state];
  pos_i_R += change;
  last_AB_state_R = current_AB_state;
}
// --------------------------------

void setup() {
  Serial.begin(115200);
  Serial.println("Xiao BLE I2C Slave with 4X Encoder PID RPM Control");

  Wire.begin(I2C_SLAVE_ADDR);
  Wire.setClock(400000);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  if (!imu.begin()) { Serial.println("IMU Connection Error!"); /* Xử lý lỗi */ }
  imu.settings.gyroSampleRate = 208; // Tần số phù hợp với vòng điều khiển
  imu.settings.accelSampleRate = 208;

  filter.begin(100); // Tần số bộ lọc AHRS

  // Cấu hình chân Motor L/R
  pinMode(PWM_L, OUTPUT); pinMode(IN1_L, OUTPUT); pinMode(IN2_L, OUTPUT);
  pinMode(PWM_R, OUTPUT); pinMode(IN1_R, OUTPUT); pinMode(IN2_R, OUTPUT);

  // Cấu hình chân Encoder L/R
  pinMode(ENCA_L, INPUT_PULLUP); pinMode(ENCB_L, INPUT_PULLUP);
  pinMode(ENCA_R, INPUT_PULLUP); pinMode(ENCB_R, INPUT_PULLUP);

  // Khởi tạo trạng thái encoder
  noInterrupts(); // Tắt ngắt tạm thời khi đọc và ghi biến volatile
  last_AB_state_L = (digitalRead(ENCA_L) << 1) | digitalRead(ENCB_L);
  last_AB_state_R = (digitalRead(ENCA_R) << 1) | digitalRead(ENCB_R);
  // Reset bộ đếm encoder (tùy chọn)
  pos_i_L = 0;
  pos_i_R = 0;
  posPrev_L = 0;
  posPrev_R = 0;
  interrupts();

  // Gắn ngắt Encoder 4X
  // *** KIỂM TRA LẠI CHÂN NGẮT TRÊN XIAO BLE ***
  bool interrupt_ok = true;
  // Kiểm tra cả 4 chân
  if (digitalPinToInterrupt(ENCA_L) == NOT_AN_INTERRUPT || digitalPinToInterrupt(ENCB_L) == NOT_AN_INTERRUPT ||
      digitalPinToInterrupt(ENCA_R) == NOT_AN_INTERRUPT || digitalPinToInterrupt(ENCB_R) == NOT_AN_INTERRUPT) {
     Serial.println("ERROR: One or more encoder pins cannot be used for interrupts!");
     interrupt_ok = false;
  }

  if (interrupt_ok) {
    attachInterrupt(digitalPinToInterrupt(ENCA_L), quadEncoderISR_L, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCB_L), quadEncoderISR_L, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCA_R), quadEncoderISR_R, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCB_R), quadEncoderISR_R, CHANGE);
    Serial.println("4X Encoder interrupts attached (L & R).");
  } else {
    Serial.println("Halting due to interrupt pin error."); while(1);
  }

  // Cấu hình PID L/R
  motorPID_L.SetMode(AUTOMATIC);
  motorPID_L.SetOutputLimits(0, 255);
  motorPID_L.SetSampleTime(CONTROL_INTERVAL);

  motorPID_R.SetMode(AUTOMATIC);
  motorPID_R.SetOutputLimits(0, 255);
  motorPID_R.SetSampleTime(CONTROL_INTERVAL);

  // Hiệu chuẩn IMU
  calibrateIMU();

  // Khởi tạo thời gian
  previousImuTime = millis();
  prevControlTime = millis(); // Dùng chung cho tính tốc độ và chạy PID

  Serial.println("Initialization complete!");
  Serial.print("Using Gains: Kp="); Serial.print(motorPID_L.GetKp()); // Giả sử Kp L/R giống nhau ban đầu
  Serial.print(", Ki="); Serial.print(motorPID_L.GetKi());
  Serial.print(", Kd="); Serial.println(motorPID_L.GetKd());
  Serial.println("I2C Slave Ready. Waiting for commands...");
}

void loop() {
  // Xử lý lệnh từ I2C nếu có
  processCommand();

  unsigned long currentTime = millis();

  // Cập nhật IMU (Ví dụ: 100Hz)
  if (currentTime - previousImuTime >= (1000 / 100)) {
    updateIMUData();
    previousImuTime = currentTime;
  }

  // Chạy vòng điều khiển PID và tính tốc độ (Ví dụ: 50Hz)
  if (currentTime - prevControlTime >= CONTROL_INTERVAL) {
    // 1. Tính toán RPM hiện tại cho cả hai bánh
    calculateCurrentRPMs();

    // 2. Chạy bộ điều khiển PID cho cả hai bánh
    //    (Input và Setpoint đã được cập nhật ở calculateCurrentRPMs và processCommand)
    motorPID_L.Compute(); // Tính motor_output_L
    motorPID_R.Compute(); // Tính motor_output_R

    // 3. Điều khiển động cơ dựa trên Output của PID và hướng mục tiêu
    //    (target_dir_L/R được cập nhật trong calculateAndSetTargets khi có lệnh CMD_SET_MOTOR)
    setMotor(target_dir_L, (int)motor_output_L, PWM_L, IN1_L, IN2_L);
    setMotor(target_dir_R, (int)motor_output_R, PWM_R, IN1_R, IN2_R);

    prevControlTime = currentTime; // Cập nhật thời gian cho vòng điều khiển tiếp theo

     // --- Gửi log qua Serial USB (Tùy chọn - Bỏ comment nếu cần debug) ---
     // Serial.print(target_rpm_L); Serial.print(","); Serial.print(current_rpm_L);
     // Serial.print(" | ");
     // Serial.print(target_rpm_R); Serial.print(","); Serial.println(current_rpm_R);
     // Serial.print(" | Out L:"); Serial.print(motor_output_L);
     // Serial.print(" Out R:"); Serial.println(motor_output_R);
  }

  // Không cần delay(1) nếu vòng lặp chính không quá nặng
  // Có thể thêm delay nhỏ nếu cần nhường CPU
}

// --- Hàm tính toán RPM hiện tại cho cả hai bánh ---
void calculateCurrentRPMs() {
  long current_pos_L = 0;
  long current_pos_R = 0;
  unsigned long now_us = micros(); // Dùng micros() để tính deltaT chính xác hơn

  // Biến cục bộ để lưu thời gian trước đó của hàm này
  static unsigned long prevRPMTime_us = now_us;

  noInterrupts();
  current_pos_L = pos_i_L;
  current_pos_R = pos_i_R;
  interrupts();

  float deltaT_s = ((float)(now_us - prevRPMTime_us)) / 1.0e6; // deltaT tính bằng giây

  // Chỉ tính nếu deltaT hợp lệ (tránh chia cho 0 ở lần đầu hoặc lỗi thời gian)
  if (deltaT_s <= 0.0) {
      prevRPMTime_us = now_us;
      return;
  }

  // Tính cho bánh trái
  long deltaPos_L = current_pos_L - posPrev_L;
  float velocity_L = (float)deltaPos_L / deltaT_s; // Xung/giây
  float v1_L = velocity_L / COUNTS_PER_OUTPUT_REV_4X * 60.0; // RPM thô trái
  v1Filt_L = 0.910 * v1Filt_L + 0.045 * v1_L + 0.045 * v1Prev_L; // Lọc (Ví dụ fc=1.5Hz)
  v1Prev_L = v1_L;
  current_rpm_L = v1Filt_L; // Input cho PID L

  // Tính cho bánh phải
  long deltaPos_R = current_pos_R - posPrev_R;
  float velocity_R = (float)deltaPos_R / deltaT_s; // Xung/giây
  float v1_R = velocity_R / COUNTS_PER_OUTPUT_REV_4X * 60.0; // RPM thô phải
  v1Filt_R = 0.910 * v1Filt_R + 0.045 * v1_R + 0.045 * v1Prev_R; // Lọc (Ví dụ fc=1.5Hz)
  v1Prev_R = v1_R;
  current_rpm_R = v1Filt_R; // Input cho PID R

  // Cập nhật giá trị cho lần tính sau
  posPrev_L = current_pos_L;
  posPrev_R = current_pos_R;
  prevRPMTime_us = now_us;
}

// --- Hàm callback I2C (Giữ nguyên) ---
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
  i2c_send_len = 0; // Reset độ dài sau khi gửi
}

// --- Xử lý lệnh I2C ---
void processCommand() {
  if (i2c_cmd == 0) return; // Không có lệnh mới

  switch (i2c_cmd) {
    case CMD_PING:
      i2c_send_buffer[0] = 0xAA; i2c_send_len = 1; break;
    case CMD_CALIBRATE_IMU:
      calibrateIMU(); i2c_send_buffer[0] = 1; i2c_send_len = 1; break;
    case CMD_SET_MOTOR:
      if (i2c_recv_len >= 8) {
        union { float f; uint8_t b[4]; } linear_union, angular_union;
        for(int i = 0; i < 4; i++) {
          linear_union.b[i] = i2c_recv_buffer[i];
          angular_union.b[i] = i2c_recv_buffer[i+4];
        }
        calculateAndSetTargets(linear_union.f, angular_union.f); // Gọi hàm tính RPM và hướng
        i2c_send_buffer[0] = 1; i2c_send_len = 1;
      } else {
        Serial.print("CMD_SET_MOTOR Error: Invalid data length "); Serial.println(i2c_recv_len);
        i2c_send_buffer[0] = 0; i2c_send_len = 1;
      }
      break;
    case CMD_GET_IMU:
      prepareIMUData(); break; // Chuẩn bị dữ liệu để gửi ở requestEvent
    case CMD_GET_ODOM:
      prepareOdomData(); break; // Chuẩn bị dữ liệu để gửi ở requestEvent
    // Bỏ CMD_GET_STATUS vì đã có CMD_GET_ODOM hoặc có thể thêm lại nếu cần
    default:
      i2c_send_buffer[0] = 0xFF; i2c_send_len = 1; break; // Lệnh không hợp lệ
  }
  i2c_cmd = 0; // Đã xử lý lệnh
}

// --- Hàm chuẩn bị dữ liệu trả về ---
void prepareIMUData() {
  memcpy(i2c_send_buffer + 0,  &gyro_x, 4); memcpy(i2c_send_buffer + 4,  &gyro_y, 4); memcpy(i2c_send_buffer + 8,  &gyro_z, 4);
  memcpy(i2c_send_buffer + 12, &accel_x, 4); memcpy(i2c_send_buffer + 16, &accel_y, 4); memcpy(i2c_send_buffer + 20, &accel_z, 4);
  memcpy(i2c_send_buffer + 24, &q0, 4); memcpy(i2c_send_buffer + 28, &q1, 4); memcpy(i2c_send_buffer + 32, &q2, 4); memcpy(i2c_send_buffer + 36, &q3, 4);
  i2c_send_len = 40;
}

void prepareOdomData() {
  // Gửi về số xung encoder tích lũy của mỗi bánh
  long odom_pos_l, odom_pos_r;
  noInterrupts();
  odom_pos_l = pos_i_L;
  odom_pos_r = pos_i_R;
  interrupts();

  // Đảm bảo buffer đủ lớn (8 bytes)
  if (sizeof(i2c_send_buffer) >= 8) {
      memcpy(i2c_send_buffer + 0, &odom_pos_l, sizeof(long));
      memcpy(i2c_send_buffer + sizeof(long), &odom_pos_r, sizeof(long));
      i2c_send_len = 2 * sizeof(long);
  } else {
      i2c_send_len = 0; // Không gửi nếu buffer quá nhỏ
      Serial.println("Error: i2c_send_buffer too small for Odom data!");
  }
}

// --- Hiệu chuẩn IMU (Giữ nguyên) ---
void calibrateIMU() {
    // ... (code giữ nguyên) ...
}

// --- Cập nhật IMU (Giữ nguyên) ---
void updateIMUData() {
    // ... (code giữ nguyên) ...
}

// --- Hàm tính toán và đặt mục tiêu RPM/Hướng từ V/W ---
void calculateAndSetTargets(float linear_x, float angular_z) {
  // Tính vận tốc dài tuyến tính cho mỗi bánh xe (m/s)
  float v_left = linear_x - (angular_z * WHEEL_SEPARATION / 2.0);
  float v_right = linear_x + (angular_z * WHEEL_SEPARATION / 2.0);

  // Xác định hướng mục tiêu cho mỗi bánh (+1: tiến, -1: lùi, 0: dừng)
  float stop_threshold = 0.001; // Ngưỡng để coi là dừng
  target_dir_L = (abs(v_left) < stop_threshold) ? 0 : ((v_left > 0) ? 1 : -1);
  target_dir_R = (abs(v_right) < stop_threshold) ? 0 : ((v_right > 0) ? 1 : -1);

  // Chuyển đổi vận tốc dài (m/s) sang vận tốc góc bánh xe (rad/s)
  float omega_left = v_left / WHEEL_RADIUS;
  float omega_right = v_right / WHEEL_RADIUS;

  // Chuyển đổi vận tốc góc bánh xe (rad/s) sang RPM (luôn dương)
  target_rpm_L = abs(omega_left * (60.0 / (2.0 * PI)));
  target_rpm_R = abs(omega_right * (60.0 / (2.0 * PI)));

  // In debug (tùy chọn)
  // Serial.print("Set Targets -> RPM_L: "); Serial.print(target_rpm_L); ...
}

// --- Hàm điều khiển một động cơ (Trái hoặc Phải) ---
// *** Hàm này nhận các chân PWM, IN1, IN2 làm tham số ***
void setMotor(int dir, int pwmVal, int pwmPin, int in1Pin, int in2Pin){
  // Giới hạn PWM trong khoảng 0-255
  analogWrite(pwmPin, constrain(pwmVal, 0, 255));

  // Điều khiển hướng (Giả sử driver cần IN1/IN2 đối nghịch)
  // Đảm bảo logic này đúng với mạch điều khiển động cơ của bạn (L298N, TB6612, etc.)
  if(dir == 1){ // Tiến
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  } else if(dir == -1){ // Lùi
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  } else { // Dừng (dir = 0)
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW); // Phanh động cơ
  }
}
// ---------------------------