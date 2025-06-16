#include <PID_v1.h>

// --- Định nghĩa chân (ĐÃ CẬP NHẬT) ---
#define ENCODER_PIN 11   // Chân đọc xung Encoder (PHẢI LÀ CHÂN HỖ TRỢ NGẮT, ví dụ D2 hoặc D3 trên Uno/Nano)
#define PWM 2           // Chân PWM điều khiển tốc độ
#define IN1 0           // Chân điều khiển chiều 1
#define IN2 1           // Chân điều khiển chiều 2

// --- Biến cho Encoder và tính toán RPM ---
long prevT = 0;
volatile long pos_i = 0;    // Vị trí encoder (đếm xung)
long posPrev = 0;           // Vị trí trước đó
float v1Filt = 0;           // RPM đã lọc
float v1Prev = 0;           // RPM thô trước đó
// --- ĐÃ LOẠI BỎ: last_AB_state ---

// --- Biến cho Thư viện PID (kiểu double) ---
double target_rpm = 0.0;
double current_rpm = 0.0;
double motor_output = 0.0;

// --- Hệ số PID ---
// *** BẠN VẪN SẼ CẦN TINH CHỈNH LẠI CÁC HỆ SỐ NÀY CHO HỆ THỐNG MỚI ***
double Kp_param = 0.8;
double Ki_param = 3.0;
double Kd_param = 0.02;

// --- Khởi tạo đối tượng PID ---
PID motorPID(&current_rpm, &motor_output, &target_rpm, Kp_param, Ki_param, Kd_param, DIRECT);

// --- ĐÃ LOẠI BỎ: Bảng tra QEM không còn cần thiết ---

//-------------------------------------------

// --- HÀM NGẮT MỚI: Đơn giản hơn cho Encoder 1 kênh ---
void pulseCounterISR() {
  pos_i++; // Mỗi khi có xung, tăng biến đếm lên 1
}
// ------------------------------------------------

void setup() {
  Serial.begin(115200);

  // --- Cấu hình chân (ĐÃ CẬP NHẬT) ---
  pinMode(ENCODER_PIN, INPUT_PULLUP); // Dùng điện trở kéo lên nội
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // --- ĐÃ LOẠI BỎ: Khởi tạo trạng thái encoder 2 kênh ---

  // --- Gắn ngắt Encoder (ĐÃ CẬP NHẬT) ---
  // Gắn ngắt vào chân encoder, kích hoạt mỗi khi có sườn lên (RISING) của xung
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), pulseCounterISR, RISING);
  Serial.println("Single-pulse encoder interrupt attached.");


  // --- Cấu hình PID ---
  motorPID.SetMode(AUTOMATIC);
  motorPID.SetOutputLimits(0, 255);
  motorPID.SetSampleTime(10);

  Serial.println("Arduino PID Motor Control Ready (Single Pulse Encoder).");
  Serial.println("Commands: P<val>, I<val>, D<val>, S<val>");
}

void loop() {
  handleSerialInput();

  // --- Đọc Encoder và tính RPM ---
  long current_pos = 0;
  noInterrupts(); // Tắt ngắt để đọc biến volatile
  current_pos = pos_i;
  interrupts();   // Bật lại ngắt ngay lập tức

  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
  float velocity1 = (deltaT > 0) ? ((float)(current_pos - posPrev)) / deltaT : 0;
  posPrev = current_pos;
  prevT = currT;

  // --- TÍNH TOÁN RPM (ĐÃ CẬP NHẬT) ---
  // Sử dụng công thức tính toán trực tiếp để có độ chính xác cao nhất
  float gear_ratio = (38.0/11.0) * (53.0/12.0) * (33.0/13.0) * (22.0/13.0);
  float pulses_per_motor_rev = 4.0;
  float counts_per_output_rev = pulses_per_motor_rev * gear_ratio; // ~262.18
  
  float v1 = (velocity1 / counts_per_output_rev) * 60.0; // RPM thô

  // Lọc RPM (giữ nguyên)
  v1Filt = 0.910 * v1Filt + 0.045 * v1 + 0.045 * v1Prev;
  v1Prev = v1;
  current_rpm = v1Filt;

  // --- Logic điều khiển PID ---
  motorPID.Compute();

  // --- Điều khiển Motor ---
  // LƯU Ý QUAN TRỌNG: Encoder 1 kênh không thể xác định chiều quay.
  // Code này giả định động cơ luôn quay theo chiều dương (dir=1).
  // Việc đảo chiều sẽ cần được xử lý bằng logic riêng nếu bạn đặt target_rpm là số âm.
  int pwr = (int)motor_output;
  int dir = 1; // Giả định chiều quay tới
  if (target_rpm == 0 && abs(current_rpm) < 1.0) {
      pwr = 0;
      dir = 0; // Dừng hẳn
  }
  setMotor(dir, pwr, PWM, IN1, IN2);
  
  // --- In dữ liệu Serial ---
  Serial.print(target_rpm);
  Serial.print(",");
  Serial.println(current_rpm);

  delay(10); // Giảm tải nhẹ cho vòng lặp
}

// --- Hàm xử lý lệnh Serial (Giữ nguyên) ---
void handleSerialInput() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.length() > 0) {
      char type = command.charAt(0);
      String valueStr = (command.length() > 1) ? command.substring(1) : "";
      double value = (valueStr.length() > 0) ? valueStr.toDouble() : 0.0;
      bool updated_pid_manually = false;

      if (type == 'P' || type == 'p') {
        Kp_param = value; updated_pid_manually = true;
      } else if (type == 'I' || type == 'i') {
        Ki_param = value; updated_pid_manually = true;
      } else if (type == 'D' || type == 'd') {
        Kd_param = value; updated_pid_manually = true;
      } else if (type == 'S' || type == 's') {
        target_rpm = value;
        Serial.print("Setpoint set to: "); Serial.println(target_rpm);
      } else {
        Serial.print("Invalid command prefix. Use P, I, D, or S. Received: ");
        Serial.println(command);
      }

      if (updated_pid_manually) {
          motorPID.SetTunings(Kp_param, Ki_param, Kd_param);
          Serial.print("Manual PID tunings updated: Kp="); Serial.print(motorPID.GetKp());
          Serial.print(", Ki="); Serial.print(motorPID.GetKi());
          Serial.print(", Kd="); Serial.println(motorPID.GetKd());
      }
    }
    while(Serial.available() > 0) { Serial.read(); }
  }
}

// --- Hàm điều khiển Motor (Giữ nguyên) ---
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm, constrain(pwmVal, 0, 255));
  if(dir == 1){
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  } else if(dir == -1){
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  }
}