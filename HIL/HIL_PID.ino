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

// Distance target (owned by MCU)
const float distance_target = 1010.0; // mm
const float dist_995 = distance_target * 0.9975;
// ======================
// State
// ======================
unsigned long lastTime = 0;
float distance_est = 0.0;
float speed_integral = 0.0;
float v_mm_s = 0.0;


// ======================
// Controllers
// ======================
float distance_P(float target, float measured) {
  const float Kp = 1.2;

  float rpm_target = Kp * (target - measured);

  if (rpm_target < 0) rpm_target = 0;
  if (rpm_target > 186) rpm_target = 186;

  return rpm_target;
}

float speed_PI(float rpm_target, float rpm_measured) {
  const float Kp = 6.0;
  const float Ki = 0.002;

  float error = rpm_target - rpm_measured;
  speed_integral += error * Ts;

  float u = Kp * error + Ki * speed_integral;

  if (u < 0) u = 0;
  if (u > 255) u = 255;

  return u;
}

// ======================
// Setup
// ======================
void setup() {
  Serial2.begin(BAUDRATE, SERIAL_8N1, RX2_PIN, TX2_PIN);
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

    // Outer loop: distance → rpm target
    float rpm_target = distance_P(distance_target, distance_est);

    // Inner loop: rpm → pwm
    float pwm = speed_PI(rpm_target, rpm_sim);

    // check if at 99.5% of target?
    uint8_t signal_995 = 0;
    if (distance_est >= dist_995) {
      pwm = 0; 
      signal_995 = 100; // Value for plotting
    }
    // THE DATA PACKET (6 Bytes total)
    // [PWM (uint8)][Signal (uint8)][Distance (float)]
    uint8_t pwm_u8 = (uint8_t)pwm;

    uint8_t packet[7];
    packet[0] = 0xBB;                                   // Header
    packet[1] = (uint8_t)pwm;                           // PWM
    packet[2] = signal_995;                             // Signal
    memcpy(&packet[3], &distance_est, sizeof(float));   // Distance (4 bytes)
    // Send the entire 7-byte packet in one go
    Serial2.write(packet, 7);
  }
}
