// ======================
// Configuration
// ======================
static const int RX2_PIN = 16;
static const int TX2_PIN = 17;
const uint32_t BAUDRATE = 921600;
const uint32_t Ts_ms = 10;
const float Ts = 0.01;
// Wheel parameters
const float D_mm = 49.7;
const float wheel_circ_mm = PI * D_mm;

// Distance target
const float distance_target = 1010.0; // mm
const float dist_995 = distance_target * 0.9975;
// ======================
// State
// ======================
unsigned long lastTime = 0;
float distance_est = 0.0;
float speed_integral = 0.0;
float v_mm_s = 0.0;

// MPC Data Mapping 
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
    
    // fallback
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
  Serial2.begin(BAUDRATE, SERIAL_8N1, RX2_PIN, TX2_PIN);
  Serial.begin(115200);

  distance_est = 0.0;
  speed_integral = 0.0;
}

// ======================
// Loop
// ======================
void loop() {
  unsigned long now = millis();
  if (now - lastTime < Ts_ms) return;
  lastTime = now;

  // Expect exactly one float (RPM) from Simulink
  if (Serial2.available() >= sizeof(float)) {

   // READ from Simulink
   float rpm_sim = 0; 
   if (Serial2.available() >= sizeof(float)) {
    Serial2.readBytes((char*)&rpm_sim, sizeof(float));
   }

    // Integrate distance from RPM
    float v_mm_s = (rpm_sim / 60.0) * wheel_circ_mm;
    distance_est += v_mm_s * Ts;

    // Outer Loop (MPC)
    float error_mm = distance_target - distance_est;
    float raw_mpc_output = get_mpc_rpm(error_mm, last_rpm_target);
  
    
    if (raw_mpc_output < 0) raw_mpc_output = 0; 
    if (raw_mpc_output > 186) raw_mpc_output = 186;
  
    float rpm_target = raw_mpc_output;
    last_rpm_target = rpm_target;

    // Inner Loop (PI)
    float pwm_output = speed_PI(rpm_target, current_rpm);

    
    uint8_t signal_995 = 0;
      if (distance_est >= dist_stop_threshold) {
      pwm_output = 0;
      speed_integral = 0;
      signal_995 = 100;
    }
    uint8_t pwm_u8 = (uint8_t)pwm;

    uint8_t packet[7];
    packet[0] = 0xBB;                                   // Header
    packet[1] = (uint8_t)pwm;                           // PWM
    packet[2] = signal_995;                             // Signal
    memcpy(&packet[3], &distance_est, sizeof(float));   // Distance (4 bytes)
    // Send the entire 7-byte packet in one go
    Serial2.write(packet, 7);
    Serial.write(packet, 7);
  }
}