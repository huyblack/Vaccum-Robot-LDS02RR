#include <PID_v1.h>

// --- Định nghĩa chân ---
#define STBY_PIN 11 // Chân bật/tắt cho driver TB6612FNG

// Động cơ Phải (RIGHT)
#define ENCODER_PIN_R 7
#define PWM_PIN_R     10
#define IN1_PIN_R     2
#define IN2_PIN_R     3

// Động cơ Trái (LEFT)
#define ENCODER_PIN_L 8
#define PWM_PIN_L     9
#define IN1_PIN_L     0
#define IN2_PIN_L     1

// --- Các biến (giữ nguyên) ---
volatile long pos_i_r = 0;
long          prevT_r = 0;
long          posPrev_r = 0;
float         rpm_filt_r = 0;
float         rpm_prev_r = 0;

volatile long pos_i_l = 0;
long          prevT_l = 0;
long          posPrev_l = 0;
float         rpm_filt_l = 0;
float         rpm_prev_l = 0;

double target_rpm = 0.0;
double current_rpm_r = 0.0;
double motor_output_r = 0.0;
double current_rpm_l = 0.0;
double motor_output_l = 0.0;

double Kp_param = 0.8;
double Ki_param = 3.0;
double Kd_param = 0.02;

PID motorPID_R(&current_rpm_r, &motor_output_r, &target_rpm, Kp_param, Ki_param, Kd_param, DIRECT);
PID motorPID_L(&current_rpm_l, &motor_output_l, &target_rpm, Kp_param, Ki_param, Kd_param, DIRECT);

void pulseCounterISR_R() { pos_i_r++; }
void pulseCounterISR_L() { pos_i_l++; }

// --- Cài đặt ---
void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(STBY_PIN, OUTPUT);
  pinMode(ENCODER_PIN_R, INPUT_PULLUP);
  pinMode(PWM_PIN_R, OUTPUT);
  pinMode(IN1_PIN_R, OUTPUT);
  pinMode(IN2_PIN_R, OUTPUT);
  pinMode(ENCODER_PIN_L, INPUT_PULLUP);
  pinMode(PWM_PIN_L, OUTPUT);
  pinMode(IN1_PIN_L, OUTPUT);
  pinMode(IN2_PIN_L, OUTPUT);
  


  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_R), pulseCounterISR_R, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_L), pulseCounterISR_L, RISING);
  
  motorPID_R.SetMode(AUTOMATIC);
  motorPID_R.SetOutputLimits(0, 255);
  motorPID_R.SetSampleTime(10);

  motorPID_L.SetMode(AUTOMATIC);
  motorPID_L.SetOutputLimits(0, 255);
  motorPID_L.SetSampleTime(10);

  Serial.println("Dual Motor PID Control Ready.");
  Serial.println("Driver is OFF by default. Set speed to turn on.");
}

// --- Vòng lặp chính ---
void loop() {
  handleSerialInput();
  
  // Chỉ tính toán và điều khiển PID nếu driver được bật
  if (digitalRead(STBY_PIN) == HIGH) {
    // Hằng số tính toán
    float gear_ratio = (38.0 / 11.0) * (53.0 / 12.0) * (33.0 / 13.0) * (22.0 / 13.0);
    float pulses_per_motor_rev = 4.0;
    float counts_per_output_rev = pulses_per_motor_rev * gear_ratio;

    // Tính toán RPM cho động cơ Phải
    long current_pos_r;
    noInterrupts(); current_pos_r = pos_i_r; interrupts();
    long currT_r = micros();
    float deltaT_r = ((float)(currT_r - prevT_r)) / 1.0e6;
    float velocity_r = (deltaT_r > 0) ? ((float)(current_pos_r - posPrev_r)) / deltaT_r : 0;
    posPrev_r = current_pos_r; prevT_r = currT_r;
    float v1_r = (velocity_r / counts_per_output_rev) * 60.0;
    rpm_filt_r = 0.910 * rpm_filt_r + 0.045 * v1_r + 0.045 * rpm_prev_r;
    rpm_prev_r = v1_r; current_rpm_r = rpm_filt_r;
    
    // Tính toán RPM cho động cơ Trái
    long current_pos_l;
    noInterrupts(); current_pos_l = pos_i_l; interrupts();
    long currT_l = micros();
    float deltaT_l = ((float)(currT_l - prevT_l)) / 1.0e6;
    float velocity_l = (deltaT_l > 0) ? ((float)(current_pos_l - posPrev_l)) / deltaT_l : 0;
    posPrev_l = current_pos_l; prevT_l = currT_l;
    float v1_l = (velocity_l / counts_per_output_rev) * 60.0;
    rpm_filt_l = 0.910 * rpm_filt_l + 0.045 * v1_l + 0.045 * rpm_prev_l;
    rpm_prev_l = v1_l; current_rpm_l = rpm_filt_l;

    // Tính toán PID
    motorPID_R.Compute();
    motorPID_L.Compute();

    // Điều khiển Motor
    int pwr_r = (int)motor_output_r;
    int pwr_l = (int)motor_output_l;
    int dir = 1;

    // Logic dừng an toàn (dù driver đã bật)
    if (target_rpm == 0 && abs(current_rpm_r) < 1.0 && abs(current_rpm_l) < 1.0) {
      pwr_r = 0; pwr_l = 0; dir = 0;
    }
    setMotor(dir, pwr_r, PWM_PIN_R, IN1_PIN_R, IN2_PIN_R);
    setMotor(dir, pwr_l, PWM_PIN_L, IN1_PIN_L, IN2_PIN_L);
    
    // In dữ liệu Serial
    Serial.print("Target:"); Serial.print(target_rpm);
    Serial.print(", R_RPM:"); Serial.print(current_rpm_r);
    Serial.print(", L_RPM:"); Serial.println(current_rpm_l);
  }

  delay(10);
}

// --- Hàm xử lý lệnh Serial (ĐÃ CẬP NHẬT) ---
void handleSerialInput() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.length() > 0) {
      char type = command.charAt(0);
      String valueStr = (command.length() > 1) ? command.substring(1) : "";
      double value = (valueStr.length() > 0) ? valueStr.toFloat() : 0.0;
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
        
        // <<< THAY ĐỔI QUAN TRỌNG Ở ĐÂY >>>
        if (target_rpm == 0.0) {
          digitalWrite(STBY_PIN, LOW); // Tắt driver
          setMotor(0, 0, PWM_PIN_R, IN1_PIN_R, IN2_PIN_R); // Dừng hẳn motor Phải
          setMotor(0, 0, PWM_PIN_L, IN1_PIN_L, IN2_PIN_L); // Dừng hẳn motor Trái
          Serial.println("Speed is 0. Motor driver OFF.");
        } else {
          digitalWrite(STBY_PIN, HIGH); // Bật driver
          Serial.println("Motor driver ON.");
        }
        
      } else {
        Serial.print("Invalid command. Received: "); Serial.println(command);
      }

      if (updated_pid_manually) {
        motorPID_R.SetTunings(Kp_param, Ki_param, Kd_param);
        motorPID_L.SetTunings(Kp_param, Ki_param, Kd_param);
        Serial.print("PID tunings updated: Kp="); Serial.print(Kp_param);
        Serial.print(", Ki="); Serial.print(Ki_param);
        Serial.print(", Kd="); Serial.println(Kd_param);
      }
    }
    while (Serial.available() > 0) { Serial.read(); }
  }
}

// --- Hàm điều khiển Motor (không đổi) ---
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, constrain(pwmVal, 0, 255));
  if (dir == 1) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  } else if (dir == -1) {
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  }
}