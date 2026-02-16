#include "DengFOC.h"

#define _2PI 6.28318530718f


//  Knob 1 
int Sensor_DIR_M0 = -1;
int Motor_PP_M0   = 7;

// Knob 2 
int Sensor_DIR_M1 = 1;
int Motor_PP_M1   = 7;

// ============ Auto Zero Offset ============
float zero_offset_M0 = 0.0f;   
float zero_offset_M1 = 0.0f;   

// ============ Torque Commands (from Python) ============
// Python sends normalized torque [-1,1], here scaled to actual FOC amplitude
float torque_cmd_1 = 0.0f;   
float torque_cmd_2 = 0.0f;   


const float torque_scale = 4.0f;   // actual torque scale (current/voltage unit depends on library implementation)

// Serial receive buffer
String recv_line = "";

// ============ Send Angles to Python ============
// Note: The second knob angle is negated here
void send_knob_angles() {
  static unsigned long last_send_us = 0;
  unsigned long now = micros();

  // Sending frequency, approx. 100Hz
  if (now - last_send_us < 10000) return;
  last_send_us = now;

  // Read current absolute angle (radians), convert to relative angle (deg)
  float angle1_deg = DFOC_M0_Angle() * 180.0f / PI - zero_offset_M0;
  float angle2_deg = DFOC_M1_Angle() * 180.0f / PI - zero_offset_M1;

  // Knob2 output is negated
  float out1 = angle1_deg;
  float out2 = -angle2_deg;   // Q2: Knob 2 final output angle is negated to ensure consistent direction

  Serial.print("KNOB1:");
  Serial.print(out1, 2);
  Serial.print(":KNOB2:");
  Serial.println(out2, 2);
}

// ============ Parse T:t1,t2 command sent from Python ============
void process_command_line(const String &line) {
  if (!line.startsWith("T:")) return;

  int comma_pos = line.indexOf(',');
  if (comma_pos < 2) return;

  String s1 = line.substring(2, comma_pos);
  String s2 = line.substring(comma_pos + 1);

  float t1 = s1.toFloat();
  float t2 = s2.toFloat();

  if (!isnan(t1) && !isnan(t2)) {
    // Clamp to [-1,1]
    if (t1 > 1.0f) t1 = 1.0f;
    if (t1 < -1.0f) t1 = -1.0f;
    if (t2 > 1.0f) t2 = 1.0f;
    if (t2 < -1.0f) t2 = -1.0f;

    torque_cmd_1 = t1;
    torque_cmd_2 = t2;
  }
}

// Non-blocking serial reading
void poll_serial_for_torque() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      process_command_line(recv_line);
      recv_line = "";
    } else if (c != '\r') {
      recv_line += c;
      if (recv_line.length() > 64) {
        // Prevent abnormal data from overflowing the buffer
        recv_line = "";
      }
    }
  }
}

// ============ Map torque_cmd to FOC torque output ============
// Note: M1 has an extra negative sign to correct hardware direction difference
void apply_torque() {
  static float tau1_smooth = 0.0f;
  static float tau2_smooth = 0.0f;

  float target_tau1 = torque_cmd_1 * torque_scale;
  float target_tau2 = torque_cmd_2 * torque_scale;

  // Simple first-order low-pass filter to suppress vibration
  const float alpha = 1.0f; 
  tau1_smooth = (1.0f - alpha) * tau1_smooth + alpha * target_tau1;
  tau2_smooth = (1.0f - alpha) * tau2_smooth + alpha * target_tau2;

 
  DFOC_M0_setTorque(tau1_smooth);
  DFOC_M1_setTorque(-tau2_smooth);
}


// ===============================================================
// setup: FOC initialization + auto zeroing
// ===============================================================
void setup() {
  Serial.begin(115200);

  DFOC_enable();
  DFOC_Vbus(12.6f);

  // Sensor / motor alignment 
  DFOC_M0_alignSensor(Motor_PP_M0, Sensor_DIR_M0);
  DFOC_M1_alignSensor(Motor_PP_M1, Sensor_DIR_M1);

  // === Auto zero: use current angle as 0° ===
  zero_offset_M0 = DFOC_M0_Angle() * 180.0f / PI;
  zero_offset_M1 = DFOC_M1_Angle() * 180.0f / PI;
}

// ===============================================================
// loop: only do three things
//   1. runFOC()
//   2. Receive T command via serial and update torque_cmd_1/2
//   3. Output torque based on torque_cmd + send current angle to Python
// ===============================================================
void loop() {
  runFOC();

  poll_serial_for_torque();
  apply_torque();
  send_knob_angles();
}
