#include "driver/pcnt.h"
#include "mpc_data.h"

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
const float Ts = 0.01;
const float epsilon = 0.0001;
const float D_mm = 49.7;
const float wheel_circ_mm = PI * D_mm;

// Target Configuration
const float distance_target = 300.0; // Updated to match your HIL test
const float dist_stop_threshold = distance_target * 0.9975;

// MPC Data Mapping (Updated to match HIL Logic)
static const int n_constraints[N_REGIONS] = {
    7, 6, 6, 5, 8, 6, 6, 5, 5, 5, 
    5, 5, 6, 6, 6, 7, 6, 5, 6, 6, 
    6, 6, 5, 6, 6, 6, 5, 6, 6, 5, 
    5, 6, 7, 5, 6, 6, 5, 6, 6, 5, 
    7, 6, 5, 6, 6, 6, 6, 6, 7, 6
};
static const float* H_all[N_REGIONS] = {
    H_1, H_2, H_3, H_4, H_5, H_6, H_7, H_8, H_9, H_10,
    H_11, H_12, H_13, H_14, H_15, H_16, H_17, H_18, H_19, H_20,
    H_21, H_22, H_23, H_24, H_25, H_26, H_27, H_28, H_29, H_30,
    H_31, H_32, H_33, H_34, H_35, H_36, H_37, H_38, H_39, H_40,
    H_41, H_42, H_43, H_44, H_45, H_46, H_47, H_48, H_49, H_50
};
static const float* K_all[N_REGIONS] = {
    K_1, K_2, K_3, K_4, K_5, K_6, K_7, K_8, K_9, K_10,
    K_11, K_12, K_13, K_14, K_15, K_16, K_17, K_18, K_19, K_20,
    K_21, K_22, K_23, K_24, K_25, K_26, K_27, K_28, K_29, K_30,
    K_31, K_32, K_33, K_34, K_35, K_36, K_37, K_38, K_39, K_40,
    K_41, K_42, K_43, K_44, K_45, K_46, K_47, K_48, K_49, K_50
};

// ======================
// State Variables
// ======================
unsigned long lastTime = 0;
int16_t lastCount = 0;
float distance_est = 0.0;
float speed_integral = 0.0;
float current_rpm = 0.0;
float last_rpm_target = 0.0;

// ======================
// MPC Controller
// ======================
float get_mpc_rpm(float current_error, float prev_u) {
    if (isnan(current_error) || isinf(current_error)) return 0;

    // State vector [Error, Reference, Previous_U]
    float z[3] = {current_error, 0.0, prev_u}; 

    for (int i = 0; i < N_REGIONS; i++) {
        bool in_region = true;
        for (int c = 0; c < n_constraints[i]; c++) {
            float h_dot_z = (H_all[i][c*3 + 0] * z[0]) + 
                            (H_all[i][c*3 + 1] * z[1]) + 
                            (H_all[i][c*3 + 2] * z[2]);

            if (h_dot_z > K_all[i][c] + epsilon) {
                in_region = false;
                break;
            }
        }

        if (in_region) {
            float target = (F[i][0] * z[0]) + (F[i][1] * z[1]) + (F[i][2] * z[2]) + G[i];
            return target;
        }
    }
    
    // Updated Fallback from HIL script
    if (abs(current_error) > 1.0) {
        return current_error * 0.3; // Nudge factor
    }
    return prev_u; 
}

// ======================
// Speed PI Controller
// ======================
float speed_PI(float rpm_target, float rpm_measured) {
  const float Kp = 6.0;
  const float Ki = 0.5; // Tuned for hardware

  float error = rpm_target - rpm_measured;
  speed_integral += error * Ts;

  float u = (Kp * error) + (Ki * speed_integral);

  if (u < 0) u = 0;
  if (u > 202) u = 202; // Clamp to motor limits
  return u;
}

// ======================
// Setup
// ======================
void setup() {
  Serial.begin(115200);


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

  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, HIGH);

  distance_est = 0.0;
  speed_integral = 0.0;
  last_rpm_target = 0.0;
}

// ======================
// Main Loop
// ======================
void loop() {
  unsigned long now = millis();
  if (now - lastTime < Ts_ms) return;
  lastTime = now;

  // 1. SENSE
  int16_t currentCount;
  pcnt_get_counter_value(pcnt_unit, &currentCount);
  
  // 2. STATE ESTIMATION
  distance_est = (currentCount / 600.0) * wheel_circ_mm;
  int16_t delta = currentCount - lastCount;
  current_rpm = (delta / 600.0) * (1000.0 / Ts_ms) * 60.0;
  lastCount = currentCount;

  // 3. MPC OUTER LOOP
  float error_mm = distance_target - distance_est;
  float raw_mpc_output = get_mpc_rpm(error_mm, last_rpm_target);
  
  // Apply hard constraints to MPC output
  if (raw_mpc_output < 0) raw_mpc_output = 0; 
  if (raw_mpc_output > 186) raw_mpc_output = 186;
  
  float rpm_target = raw_mpc_output;
  last_rpm_target = rpm_target;

  // 4. PI INNER LOOP
  float pwm_output = speed_PI(rpm_target, current_rpm);

  // 5. SAFETY & SIGNAL
  uint8_t signal_995 = 0;
  if (distance_est >= dist_stop_threshold) {
    pwm_output = 0;
    speed_integral = 0;
    signal_995 = 100;
  }

  // 6. ACTUATE
  analogWrite(PWM_PIN, (uint8_t)pwm_output);

  // 7. TELEMETRY (Binary Packet for MATLAB)
  uint8_t packet[7];
  packet[0] = 0xBB;                                   
  packet[1] = (uint8_t)pwm_output;                    
  packet[2] = signal_995;                             
  memcpy(&packet[3], &distance_est, sizeof(float));   

  Serial.write(packet, 7);
}