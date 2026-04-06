#include "driver/pcnt.h"

// ======================
// Pin & Hardware Config
// ======================
const int ENCODER_A = 25;
const int ENCODER_B = 26;
const int PWM_PIN = 9;
const int DIR_PIN = 10;
const pcnt_unit_t pcnt_unit = PCNT_UNIT_0;

// ======================
// Constants & Parameters
// ======================
const uint32_t Ts_ms = 10;
const float Ts = 0.01; // 10ms in seconds
const float D_mm = 49.7;
const float wheel_circ_mm = PI * D_mm;
const float distance_target = 1010.0; 
const float dist_stop_threshold = distance_target * 0.9975;

// ======================
// State Variables
// ======================
unsigned long lastTime = 0;
int16_t lastCount = 0;
float distance_est = 0.0;
float speed_integral = 0.0;
float current_rpm = 0.0;

// ======================
// Controllers
// ======================

// Outer Loop: Distance -> RPM Target
float distance_P(float target, float measured) {
  const float Kp = 1.2;
  float rpm_target = Kp * (target - measured);

  // Constraints
  if (rpm_target < 0) rpm_target = 0;
  if (rpm_target > 186) rpm_target = 186; 
  return rpm_target;
}

// Inner Loop: RPM -> PWM
float speed_PI(float rpm_target, float rpm_measured) {
  const float Kp = 6.0;
  const float Ki = 0.5; 

  float error = rpm_target - rpm_measured;
  
  speed_integral += error * Ts;

  float u = (Kp * error) + (Ki * speed_integral);

  if (u < 0) u = 0;
  if (u > 202) u = 202;
  return u;
}

// ======================
// Setup
// ======================
void setup() {
  Serial.begin(115200);


  // 1. Hardware Encoder Setup (PCNT)
  pcnt_config_t pcnt_config = {
    .pulse_gpio_num = ENCODER_A,
    .ctrl_gpio_num = ENCODER_B,
    .lctrl_mode = PCNT_MODE_KEEP,
    .hctrl_mode = PCNT_MODE_REVERSE,
    .pos_mode = PCNT_COUNT_INC,
    .neg_mode = PCNT_COUNT_DIS,
    .counter_h_lim = 30000,
    .counter_l_lim = -30000,
    .unit = pcnt_unit,
    .channel = PCNT_CHANNEL_0,
  };
  pcnt_unit_config(&pcnt_config);
  pcnt_set_filter_value(pcnt_unit, 1000);
  pcnt_filter_enable(pcnt_unit);
  pcnt_counter_clear(pcnt_unit);
  pcnt_counter_resume(pcnt_unit);

  // 2. Motor Pins
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, HIGH); // Set Direction

  distance_est = 0.0;
  speed_integral = 0.0;
}

// ======================
// Main Loop (Runs every 10ms)
// ======================
void loop() {
  unsigned long now = millis();
  if (now - lastTime < Ts_ms) return;
  lastTime = now;

  // 1. SENSE: Read Hardware Encoder
  int16_t currentCount;
  pcnt_get_counter_value(pcnt_unit, &currentCount);
  
  // 2. CALCULATE DISTANCE
  distance_est = (currentCount / 600.0) * wheel_circ_mm;

  // 3. CALCULATE RPM
  int16_t delta = currentCount - lastCount;
  current_rpm = (delta / 600.0) * (1000.0 / Ts_ms) * 60.0;
  lastCount = currentCount;

  // 4. CONTROL: PID Logic
  float rpm_target = distance_P(distance_target, distance_est);
  float pwm_output = speed_PI(rpm_target, current_rpm);

  // 5. SAFETY: Hard stop at target & Signal Generation
  uint8_t signal_995 = 0;
  if (distance_est >= dist_stop_threshold) {
    pwm_output = 0;
    speed_integral = 0; 
    signal_995 = 100; // Match the HIL plotting value
  }

  // 6. ACTUATE: Apply to Motor
  analogWrite(PWM_PIN, (uint8_t)pwm_output);

  // 7. DATA PACKET SENDING (Replaces Telemetry)
  // [Header (0xBB)][PWM (uint8)][Signal (uint8)][Distance (float)]
  uint8_t packet[7];
  packet[0] = 0xBB;                                   // Header
  packet[1] = (uint8_t)pwm_output;                    // PWM
  packet[2] = signal_995;                             // Signal
  memcpy(&packet[3], &distance_est, sizeof(float));   // Distance (4 bytes)

  // Send to both ports so your CSV script and Serial Monitor tools work
  Serial.write(packet, 7);
}