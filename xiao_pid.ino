#include <PID_v1.h>        // Chỉ còn thư viện PID chính

// --- Định nghĩa chân ---
// *** Nhắc lại: Đảm bảo các chân này hợp lệ trên bo mạch của bạn ***
#define ENCA 0  // Chân A của Encoder (Cần hỗ trợ ngắt)
#define ENCB 10 // Chân B của Encoder (Cần hỗ trợ ngắt)
#define PWM 3   // Chân PWM điều khiển tốc độ
#define IN1 6   // Chân điều khiển chiều 1
#define IN2 7   // Chân điều khiển chiều 2

// --- Biến cho Encoder và tính toán RPM ---
long prevT = 0;
volatile long pos_i = 0;     // Vị trí encoder (4X)
long posPrev = 0;            // Vị trí trước đó (long)
float v1Filt = 0;            // RPM đã lọc
float v1Prev = 0;            // RPM thô trước đó
volatile byte last_AB_state = 0; // Trạng thái encoder trước đó

// --- Biến cho Thư viện PID (kiểu double) ---
double target_rpm = 0.0;   // Setpoint - Điểm đặt (RPM)
double current_rpm = 0.0;  // Input - RPM đo được
double motor_output = 0.0; // Output - PWM (0-255)

// --- Hệ số PID cuối cùng bạn đã chọn ---
// *** THAY CÁC GIÁ TRỊ NÀY BẰNG Kp, Ki, Kd TỐT NHẤT BẠN TÌM ĐƯỢC ***
double Kp_param = 1.0;  // Ví dụ: Kp cuối cùng
double Ki_param = 10.0; // Ví dụ: Ki cuối cùng
double Kd_param = 0.03; // Ví dụ: Kd cuối cùng

// --- Khởi tạo đối tượng PID với tham số cố định ---
PID motorPID(&current_rpm, &motor_output, &target_rpm, Kp_param, Ki_param, Kd_param, DIRECT);

// --- Bảng tra cho 4X Quadrature Decoding (Giữ lại) ---
const int8_t QEM [16] = {
    0, +1, -1, 0, -1, 0, 0, +1, +1, 0, 0, -1, 0, -1, +1, 0
};
//-------------------------------------------

// --- Hàm xử lý ngắt Quadrature Encoder (Giữ lại) ---
void quadEncoderISR() {
  byte current_A = digitalRead(ENCA);
  byte current_B = digitalRead(ENCB);
  byte current_AB_state = (current_A << 1) | current_B;
  if (current_AB_state == last_AB_state) return;
  int8_t change = QEM[(last_AB_state << 2) | current_AB_state];
  pos_i += change;
  last_AB_state = current_AB_state;
}
// ------------------------------------------------

void setup() {
  Serial.begin(115200);
  // while (!Serial);

  // --- Cấu hình chân ---
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // --- Khởi tạo trạng thái encoder ---
  byte initial_A = digitalRead(ENCA);
  byte initial_B = digitalRead(ENCB);
  last_AB_state = (initial_A << 1) | initial_B;

  // --- Gắn ngắt Encoder ---
  // *** KIỂM TRA LẠI CHÂN NGẮT ***
  bool interrupt_ok = true;
  if (digitalPinToInterrupt(ENCA) == NOT_AN_INTERRUPT) {
      Serial.println("ERROR: Pin ENCA (D0) cannot be used for interrupts!"); interrupt_ok = false;
  }
  if (digitalPinToInterrupt(ENCB) == NOT_AN_INTERRUPT) {
      Serial.println("ERROR: Pin ENCB (D10) cannot be used for interrupts!"); interrupt_ok = false;
  }
  if (interrupt_ok) {
      attachInterrupt(digitalPinToInterrupt(ENCA), quadEncoderISR, CHANGE);
      attachInterrupt(digitalPinToInterrupt(ENCB), quadEncoderISR, CHANGE);
      Serial.println("Encoder interrupts attached.");
  } else {
      Serial.println("Halting due to interrupt pin error."); while(1);
  }

  // --- Cấu hình PID ---
  motorPID.SetMode(AUTOMATIC);        // Bật PID
  motorPID.SetOutputLimits(0, 255);   // Đặt giới hạn PWM
  motorPID.SetSampleTime(10);         // Đặt thời gian lấy mẫu PID

  // --- In thông tin khởi động (đã loại bỏ phần AutoTune) ---
  Serial.println("Arduino PID Motor Control Ready (Final Tunings).");
  Serial.println("Commands: P<val>, I<val>, D<val>, S<val>");
  Serial.print("Using fixed gains: Kp="); Serial.print(motorPID.GetKp());
  Serial.print(", Ki="); Serial.print(motorPID.GetKi());
  Serial.print(", Kd="); Serial.println(motorPID.GetKd());
  Serial.print("Initial Setpoint: "); Serial.println(target_rpm);
  Serial.println("Format: Target RPM | Actual RPM");
}

void loop() {
  // Xử lý lệnh từ Serial (chỉnh P, I, D, S thủ công)
  handleSerialInput();

  // --- Đọc Encoder và tính RPM ---
  long current_pos = 0;
  noInterrupts();
  current_pos = pos_i;
  interrupts();

  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
  float velocity1 = (deltaT > 0) ? ((float)(current_pos - posPrev)) / deltaT : 0;
  posPrev = current_pos;
  prevT = currT;

  // Tính RPM (sử dụng giá trị counts/rev đã cập nhật)
  float counts_per_output_rev = 44.0 * 46.8; // = 2059.2
  float v1 = velocity1 / counts_per_output_rev * 60.0; // RPM thô

  // Lọc RPM (Sử dụng bộ lọc bạn đã chọn là phù hợp)
  // Ví dụ: dùng bộ lọc fc=1.5Hz (fs=100Hz)
  v1Filt = 0.910 * v1Filt + 0.045 * v1 + 0.045 * v1Prev;
  // Hoặc dùng bộ lọc fc=2.5Hz (fs=100Hz)
  // v1Filt = 0.854 * v1Filt + 0.0728 * v1 + 0.0728 * v1Prev;
  v1Prev = v1;
  current_rpm = v1Filt; // Cập nhật Input cho PID
  // ------------------------------------------

  // --- Logic điều khiển PID (luôn chạy) ---
  motorPID.Compute(); // PID tính toán giá trị motor_output
  // -------------------------------------------

  // --- Điều khiển Motor ---
  int pwr = (int)motor_output;
  int dir = 1;
  if (target_rpm == 0 && abs(current_rpm) < 1.0) {
       pwr = 0;
       dir = 0;
  }
  setMotor(dir, pwr, PWM, IN1, IN2);
  // -----------------------

  // --- In dữ liệu Serial (luôn in) ---
  Serial.print(target_rpm);
  Serial.print(",");
  Serial.println(current_rpm);
  // -----------------------

  delay(1);
}

// --- Hàm xử lý lệnh Serial (Đã loại bỏ lệnh T và C) ---
void handleSerialInput() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.length() > 0) {
      char type = command.charAt(0);
      String valueStr = (command.length() > 1) ? command.substring(1) : "";
      double value = (valueStr.length() > 0) ? valueStr.toDouble() : 0.0;
      bool updated_pid_manually = false;

      // Chỉ xử lý lệnh P, I, D, S
      if (type == 'P' || type == 'p') {
        Kp_param = value; updated_pid_manually = true;
      } else if (type == 'I' || type == 'i') {
        Ki_param = value; updated_pid_manually = true;
      } else if (type == 'D' || type == 'd') {
        Kd_param = value; updated_pid_manually = true;
      } else if (type == 'S' || type == 's') {
        target_rpm = value; // Cập nhật Setpoint
        Serial.print("Setpoint set to: "); Serial.println(target_rpm);
      } else {
        // Bỏ qua các lệnh không hợp lệ hoặc in thông báo lỗi
        Serial.print("Invalid command prefix. Use P, I, D, or S. Received: ");
        Serial.println(command);
      }

      // Cập nhật PID nếu người dùng thay đổi P, I, D
      if (updated_pid_manually) {
          motorPID.SetTunings(Kp_param, Ki_param, Kd_param);
          Serial.print("Manual PID tunings updated: Kp="); Serial.print(motorPID.GetKp());
          Serial.print(", Ki="); Serial.print(motorPID.GetKi());
          Serial.print(", Kd="); Serial.println(motorPID.GetKd());
      }
    }
    while(Serial.available() > 0) { Serial.read(); } // Clear buffer
  }
}
// -----------------------------

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
// ---------------------------