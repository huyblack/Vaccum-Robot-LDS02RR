// Arduino sketch đơn giản hóa cho Xiao BLE - chỉ điều khiển động cơ
// - Sửa lỗi PID cho tốc độ âm bằng cách suy luận chiều quay.
// - Tích hợp bộ điều khiển PID cho động cơ (điều khiển theo RPM).
// - Đóng vai trò I2C Slave để nhận lệnh từ Raspberry Pi.
// - Điều khiển động cơ và đọc encoder.
// - Có LED báo trạng thái giao tiếp I2C.

#include <Arduino.h>
#include <Wire.h>
#include <PID_v1.h>

// --- Chế độ Test Động Cơ ---
// Đặt giá trị là 1 để bật chế độ test, 0 để tắt.
#define TEST_DC 0

// --- Cấu hình I2C ---
#define I2C_SLAVE_ADDR 0x08   // Địa chỉ I2C của Xiao BLE khi làm Slave

// --- Định nghĩa các lệnh I2C (đã loại bỏ IMU) ---
#define CMD_PING          0x01  // Kiểm tra kết nối
#define CMD_SET_MOTOR     0x03  // Điều khiển động cơ
#define CMD_GET_STATUS    0x05  // Đọc trạng thái động cơ (encoder)
#define CMD_MOTOR_TORQUE  0x06  // Bật/tắt torque động cơ
#define CMD_RESET_ODOM    0x07  // Reset odometry position

// --- Định nghĩa chân cắm cho động cơ và encoder ---
#define STBY_PIN      11  // Chân Standby cho driver TB6612FNG
#define ENCODER_PIN_R 7
#define PWM_PIN_R     10
#define IN1_PIN_R     1
#define IN2_PIN_R     0
#define ENCODER_PIN_L 8
#define PWM_PIN_L     9
#define IN1_PIN_L     2
#define IN2_PIN_L     3

// --- Cấu hình LED trạng thái ---
#define LED_PIN           LEDB // Sử dụng LED có sẵn trên bo mạch
#define I2C_TIMEOUT_MS    1000        // Thời gian timeout để coi là mất kết nối (1 giây)

// --- Biến toàn cục cho động cơ ---
double g_left_speed_target_mps = 0.0;  // Tốc độ mục tiêu (m/s)
double g_right_speed_target_mps = 0.0; // Tốc độ mục tiêu (m/s)
bool motor_torque_enabled = false;

// --- Biến toàn cục cho Encoder ---
volatile long encoder_left_count = 0;
volatile long encoder_right_count = 0;
double present_velocity_left_mps = 0.0;  // Vận tốc thực tế (m/s)
double present_velocity_right_mps = 0.0; // Vận tốc thực tế (m/s)
float present_position_left = 0.0;
float present_position_right = 0.0;

// --- Thông số vật lý của Robot ---
const float WHEEL_RADIUS = 0.035;      // Bán kính bánh xe (m)
const float GEAR_RATIO = (38.0 / 11.0) * (53.0 / 12.0) * (33.0 / 13.0) * (22.0 / 13.0); // Tỷ số truyền ~65.45
const float PULSES_PER_MOTOR_REV = 4.0; // Số xung trên 1 vòng quay của motor
const float ENCODER_PPR = GEAR_RATIO * PULSES_PER_MOTOR_REV; // Số xung trên 1 vòng quay của bánh xe
const float DISTANCE_PER_PULSE = (2.0 * PI * WHEEL_RADIUS) / ENCODER_PPR;

// --- Biến và hằng số cho bộ điều khiển PID ---
// --- HƯỚNG DẪN TINH CHỈNH (TUNING) PID ---
// Kp (Tỷ lệ): Tăng Kp để động cơ phản ứng nhanh hơn. Nếu quá cao sẽ gây vọt lố/dao động.
// Ki (Tích phân): Tăng Ki để triệt tiêu sai số khi đã ổn định. Nếu quá cao sẽ gây vọt lố lớn.
// Kd (Vi phân): Tăng Kd để giảm vọt lố và làm hệ thống ổn định nhanh hơn.
double Kp = 8.0, Ki = 10.0, Kd = 0.1; // Các thông số PID mới, "quyết đoán" hơn

// Biến cho PID động cơ trái (điều khiển theo RPM)
double g_left_rpm_target = 0.0;
double present_rpm_left = 0.0;
double pid_output_left;
PID motorPID_L(&present_rpm_left, &pid_output_left, &g_left_rpm_target, Kp, Ki, Kd, DIRECT);

// Biến cho PID động cơ phải (điều khiển theo RPM)
double g_right_rpm_target = 0.0;
double present_rpm_right = 0.0;
double pid_output_right;
PID motorPID_R(&present_rpm_right, &pid_output_right, &g_right_rpm_target, Kp, Ki, Kd, DIRECT);

// --- Biến quản lý thời gian ---
#define PRINT_INTERVAL_MS 250 // In dữ liệu ra Serial mỗi 250ms
unsigned long last_velocity_update_time = 0;
unsigned long last_print_time = 0; // Biến thời gian cho việc in dữ liệu
long last_encoder_left = 0;
long last_encoder_right = 0;
unsigned long lastI2CActivityTime = 0;
unsigned long lastLedToggleTime = 0;
bool ledState = LOW;

// --- Buffer và biến trạng thái I2C ---
volatile uint8_t i2c_recv_buffer[32];
volatile uint8_t i2c_recv_len = 0;
volatile uint8_t i2c_cmd = 0;
uint8_t i2c_send_buffer[40];
uint8_t i2c_send_len = 0;

// --- Khai báo hàm ---
void resetOdometry();
void receiveEvent(int numBytes);
void requestEvent();
void processCommand();
void controlMotors(float linear_x, float angular_z);
void updateMotors();
void updateMotorStatus();
void prepareStatusData();
void encoderLeftISR();
void encoderRightISR();
void updateLedStatus();
void printMotorData();

void setup() {
  Serial.begin(115200);

  // Khởi tạo I2C ở chế độ Slave.
  Wire.begin(I2C_SLAVE_ADDR);
  Wire.setClock(400000); // Sử dụng tốc độ 400kHz để ổn định
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  // Cấu hình chân động cơ và encoder
  pinMode(STBY_PIN, OUTPUT);
  pinMode(PWM_PIN_L, OUTPUT);
  pinMode(IN1_PIN_L, OUTPUT);
  pinMode(IN2_PIN_L, OUTPUT);
  pinMode(PWM_PIN_R, OUTPUT);
  pinMode(IN1_PIN_R, OUTPUT);
  pinMode(IN2_PIN_R, OUTPUT);
  pinMode(ENCODER_PIN_L, INPUT_PULLUP);
  pinMode(ENCODER_PIN_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_L), encoderLeftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_R), encoderRightISR, RISING);
  
  // Mặc định tắt driver khi mới khởi động để an toàn
  digitalWrite(STBY_PIN, LOW);

  // Cấu hình chân LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Khởi tạo PID
  motorPID_L.SetMode(AUTOMATIC);
  motorPID_L.SetSampleTime(10); // Cập nhật PID mỗi 10ms
  motorPID_L.SetOutputLimits(-255, 255); // Giới hạn output từ -255 đến 255

  motorPID_R.SetMode(AUTOMATIC);
  motorPID_R.SetSampleTime(10);
  motorPID_R.SetOutputLimits(-255, 255);

  // Lấy thời gian ban đầu
  last_velocity_update_time = millis();
  lastI2CActivityTime = millis();
  last_print_time = millis();

  #if TEST_DC
    Serial.println("---!!! DC MOTOR TEST MODE ENABLED !!!---");
    motor_torque_enabled = true;
    digitalWrite(STBY_PIN, HIGH); // Bật driver
    controlMotors(0.1, 0.0); // Đặt lệnh xoay tại chỗ để test
  #endif

  Serial.println("Xiao BLE Motor Controller Ready.");
}

void loop() {
  #if !TEST_DC
    processCommand(); // Xử lý lệnh từ Pi (nếu có)
  #endif

  // Vô hiệu hóa việc in Serial khi không debug để đảm bảo hiệu năng I2C
  #if TEST_DC
    if ((millis() - last_print_time) >= PRINT_INTERVAL_MS) {
      printMotorData();
      Serial.println("---------------------------------");
      last_print_time = millis();
    }
  #endif

  updateMotorStatus();
  updateMotors(); // Hàm này giờ sẽ bao gồm cả tính toán PID
  updateLedStatus();
  
  // Debug: Check for serial commands (non-blocking)
  if (Serial.available() > 0) {
    String command = Serial.readString();
    command.trim();
    if (command == "reset") {
      resetOdometry();
    } else if (command == "status") {
      Serial.print("Position L/R: ");
      Serial.print(present_position_left, 4);
      Serial.print(" / ");
      Serial.println(present_position_right, 4);
      Serial.print("Velocity L/R: ");
      Serial.print(present_velocity_left_mps, 4);
      Serial.print(" / ");
      Serial.println(present_velocity_right_mps, 4);
      Serial.print("Encoder L/R: ");
      Serial.print(encoder_left_count);
      Serial.print(" / ");
      Serial.println(encoder_right_count);
    }
  }

}

// --- Hàm callback I2C ---
void receiveEvent(int numBytes) {
  lastI2CActivityTime = millis();
  
  if (numBytes < 1) return;
  i2c_cmd = Wire.read();
  i2c_recv_len = 0;
  while (Wire.available() && i2c_recv_len < sizeof(i2c_recv_buffer)) {
    i2c_recv_buffer[i2c_recv_len++] = Wire.read();
  }
}

void requestEvent() {
  Wire.write(i2c_send_buffer, i2c_send_len);
}

// --- Xử lý lệnh (đã loại bỏ IMU) ---
void processCommand() {
  if (i2c_cmd == 0) return;

  switch (i2c_cmd) {
    case CMD_PING:
      i2c_send_buffer[0] = 0xCC;
      i2c_send_len = 1;
      break;
    case CMD_SET_MOTOR:
      if (i2c_recv_len >= 8) {
        float linear_x, angular_z;
        memcpy(&linear_x, (void*)i2c_recv_buffer, 4);
        memcpy(&angular_z, (void*)(i2c_recv_buffer + 4), 4);
        controlMotors(linear_x, angular_z);
        i2c_send_buffer[0] = 1;
        i2c_send_len = 1;
      } else { i2c_send_buffer[0] = 0; i2c_send_len = 1; }
      break;
    case CMD_MOTOR_TORQUE:
      if (i2c_recv_len >= 1) {
        motor_torque_enabled = (i2c_recv_buffer[0] == 1);
        digitalWrite(STBY_PIN, motor_torque_enabled);

        if (!motor_torque_enabled) {
          controlMotors(0.0, 0.0); // Đặt tốc độ về 0
          motorPID_L.SetMode(MANUAL); // Tắt PID để tránh tích lũy sai số
          pid_output_left = 0;
          motorPID_R.SetMode(MANUAL);
          pid_output_right = 0;
        } else {
          motorPID_L.SetMode(AUTOMATIC); // Bật lại PID
          motorPID_R.SetMode(AUTOMATIC);
        }
        i2c_send_buffer[0] = 1;
        i2c_send_len = 1;
      } else { i2c_send_buffer[0] = 0; i2c_send_len = 1; }
      break;
    case CMD_GET_STATUS:
      prepareStatusData();
      break;
    case CMD_RESET_ODOM:
      resetOdometry();
      i2c_send_buffer[0] = 1; // Success
      i2c_send_len = 1;
      break;
    default:
      i2c_send_buffer[0] = 0xFF;
      i2c_send_len = 1;
      break;
  }
  i2c_cmd = 0;
}

// --- Các hàm chức năng ---

void controlMotors(float linear_x, float angular_z) {
  // --- UPDATED LOGIC ---
  // Nếu lệnh là dừng, luôn tắt driver và reset PID để đảm bảo an toàn.
  if (linear_x == 0.0f && angular_z == 0.0f) {
    digitalWrite(STBY_PIN, LOW);
    motorPID_L.SetMode(MANUAL); // Tắt PID để reset và tránh tích lũy sai số
    pid_output_left = 0;
    motorPID_R.SetMode(MANUAL);
    pid_output_right = 0;
  } else {
    // Đối với bất kỳ lệnh di chuyển nào khác, hãy đảm bảo driver được bật
    // và PID ở chế độ tự động, nhưng CHỈ khi torque đã được cho phép từ trước.
    if (motor_torque_enabled) {
      digitalWrite(STBY_PIN, HIGH);
      motorPID_L.SetMode(AUTOMATIC);
      motorPID_R.SetMode(AUTOMATIC);
    }
  }
  // --- END UPDATED LOGIC ---

  if (!motor_torque_enabled) {
    g_left_speed_target_mps = 0.0;
    g_right_speed_target_mps = 0.0;
    g_left_rpm_target = 0.0;
    g_right_rpm_target = 0.0;
    return;
  }
  
  const float wheel_separation = 0.24;
  g_left_speed_target_mps = linear_x - (angular_z * wheel_separation / 2.0);
  g_right_speed_target_mps = linear_x + (angular_z * wheel_separation / 2.0);

  // Chuyển đổi từ m/s sang RPM
  const float mps_to_rpm_factor = 60.0 / (2.0 * PI * WHEEL_RADIUS);
  g_left_rpm_target = g_left_speed_target_mps * mps_to_rpm_factor;
  g_right_rpm_target = g_right_speed_target_mps * mps_to_rpm_factor;
}

void updateMotors() {
  // Tính toán PID chỉ khi torque được bật
  if (motor_torque_enabled) {
    motorPID_L.Compute();
    motorPID_R.Compute();
  } else {
    // Nếu torque tắt, reset output
    pid_output_left = 0;
    pid_output_right = 0;
  }

  // Lấy giá trị PWM và hướng từ output của PID
  int left_pwm = abs(pid_output_left);
  int right_pwm = abs(pid_output_right);

  // Điều khiển động cơ trái
  if (pid_output_left >= 0) {
    digitalWrite(IN1_PIN_L, HIGH);
    digitalWrite(IN2_PIN_L, LOW);
  } else {
    digitalWrite(IN1_PIN_L, LOW);
    digitalWrite(IN2_PIN_L, HIGH);
  }
  analogWrite(PWM_PIN_L, constrain(left_pwm, 0, 255));

  // Điều khiển động cơ phải (đảo ngược logic)
  if (pid_output_right >= 0) {
    digitalWrite(IN1_PIN_R, LOW);
    digitalWrite(IN2_PIN_R, HIGH);
  } else {
    digitalWrite(IN1_PIN_R, HIGH);
    digitalWrite(IN2_PIN_R, LOW);
  }
  analogWrite(PWM_PIN_R, constrain(right_pwm, 0, 255));
}

void encoderLeftISR() { encoder_left_count++; }
void encoderRightISR() { encoder_right_count++; }

void updateMotorStatus() {
  unsigned long current_time = millis();
  if (current_time - last_velocity_update_time >= 50) {
    double dt = (double)(current_time - last_velocity_update_time) / 1000.0;
    if (dt <= 0) return; // Tránh chia cho 0

    long left_pulses = encoder_left_count - last_encoder_left;
    long right_pulses = encoder_right_count - last_encoder_right;
    
    // --- SAFETY: Bounds checking cho encoder deltas ---
    // TurtleBot3 Real Specs: Max 27 RPM, Max 0.1 m/s
    // Calculation: 27 RPM = 0.45 rev/s = 117.9 pulses/s = 5.9 pulses/50ms
    const long MAX_PULSES_PER_UPDATE = 20; // 3x safety margin for 27 RPM max
    
    // Nếu encoder delta quá lớn → có thể là glitch → reset
    if (abs(left_pulses) > MAX_PULSES_PER_UPDATE) {
      Serial.print("WARNING: Left encoder glitch detected: ");
      Serial.println(left_pulses);
      left_pulses = 0; // Reset to prevent position explosion
    }
    
    if (abs(right_pulses) > MAX_PULSES_PER_UPDATE) {
      Serial.print("WARNING: Right encoder glitch detected: ");
      Serial.println(right_pulses);
      right_pulses = 0; // Reset to prevent position explosion
    }
    
    // --- Tính RPM với bounds checking ---
    double abs_rpm_left = ((double)left_pulses / dt / ENCODER_PPR) * 60.0;
    double abs_rpm_right = ((double)right_pulses / dt / ENCODER_PPR) * 60.0;
    
    // Clamp RPM to TurtleBot3 real limits (Max 27 RPM)
    abs_rpm_left = constrain(abs_rpm_left, 0, 30);   // 30 RPM safety margin
    abs_rpm_right = constrain(abs_rpm_right, 0, 30); // 30 RPM safety margin

    // Gán dấu (+/-) cho RPM thực tế dựa vào dấu của output từ PID
    present_rpm_left = (pid_output_left >= 0) ? abs_rpm_left : -abs_rpm_left;
    present_rpm_right = (pid_output_right >= 0) ? abs_rpm_right : -abs_rpm_right;

    // --- Tính vận tốc (m/s) với safety bounds ---
    double abs_vel_left = (left_pulses * DISTANCE_PER_PULSE) / dt;
    double abs_vel_right = (right_pulses * DISTANCE_PER_PULSE) / dt;
    
    // Clamp velocity to TurtleBot3 real limits (Max 0.1 m/s)
    abs_vel_left = constrain(abs_vel_left, 0, 0.12);   // 0.12 m/s safety margin
    abs_vel_right = constrain(abs_vel_right, 0, 0.12); // 0.12 m/s safety margin
    
    present_velocity_left_mps = (pid_output_left >= 0) ? abs_vel_left : -abs_vel_left;
    present_velocity_right_mps = (pid_output_right >= 0) ? abs_vel_right : -abs_vel_right;

    // --- Cập nhật vị trí với bounds checking ---
    double pos_delta_left = present_velocity_left_mps * dt;
    double pos_delta_right = present_velocity_right_mps * dt;
    
    // Safety: Clamp position delta per update (based on 0.1 m/s max)
    pos_delta_left = constrain(pos_delta_left, -0.006, 0.006);   // Max 6mm per 50ms update
    pos_delta_right = constrain(pos_delta_right, -0.006, 0.006); // Max 6mm per 50ms update
    
    present_position_left += pos_delta_left;
    present_position_right += pos_delta_right;
    
    // Reset position if it gets too large (prevent overflow)
    if (abs(present_position_left) > 100.0) {  // Reset after 100m (reasonable for indoor robot)
      present_position_left = 0.0;
      Serial.println("WARNING: Left position reset due to overflow");
    }
    
    if (abs(present_position_right) > 100.0) { // Reset after 100m (reasonable for indoor robot)
      present_position_right = 0.0;
      Serial.println("WARNING: Right position reset due to overflow");
    }
    
    last_encoder_left = encoder_left_count;
    last_encoder_right = encoder_right_count;
    last_velocity_update_time = current_time;
  }
}

void prepareStatusData() {
  float dummy_current = 0.0;
  float vel_left_float = present_velocity_left_mps;
  float vel_right_float = present_velocity_right_mps;

  memcpy(i2c_send_buffer + 0,  &dummy_current, 4);
  memcpy(i2c_send_buffer + 4,  &dummy_current, 4);
  memcpy(i2c_send_buffer + 8,  &vel_left_float, 4);
  memcpy(i2c_send_buffer + 12, &vel_right_float, 4);
  memcpy(i2c_send_buffer + 16, &present_position_left, 4);
  memcpy(i2c_send_buffer + 20, &present_position_right, 4);
  i2c_send_buffer[24] = motor_torque_enabled ? 1 : 0;
  i2c_send_len = 25;
}

void updateLedStatus() {
  unsigned long currentTime = millis();
  unsigned long blinkInterval;

  if ((currentTime - lastI2CActivityTime) < I2C_TIMEOUT_MS) {
    blinkInterval = 100;
  } else {
    blinkInterval = 500;
  }

  if ((currentTime - lastLedToggleTime) >= blinkInterval) {
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
    lastLedToggleTime = currentTime;
  }
}

void printMotorData() {
  Serial.print("Target RPM (L/R): ");
  Serial.print(g_left_rpm_target, 2);
  Serial.print(" / ");
  Serial.print(g_right_rpm_target, 2);

  Serial.print(" | Actual RPM (L/R): ");
  Serial.print(present_rpm_left, 2);
  Serial.print(" / ");
  Serial.println(present_rpm_right, 2);

  // --- Dòng mới được thêm vào ---
  Serial.print("Position (m) (L/R): ");
  Serial.print(present_position_left, 3);
  Serial.print(" / ");
  Serial.println(present_position_right, 3);
}

void resetOdometry() {
  // Reset position counters
  present_position_left = 0.0;
  present_position_right = 0.0;
  
  // Reset encoder counts to prevent sudden jumps
  noInterrupts();
  encoder_left_count = 0;
  encoder_right_count = 0;
  last_encoder_left = 0;
  last_encoder_right = 0;
  interrupts();
  
  Serial.println("Odometry reset - position and encoders cleared");
}
