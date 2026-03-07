// Example: Detent + Vibration Mode
//  DengFOC V0.2
//  DengFOC open source, follow GNU protocol, please indicate copyright when reprinting!
//  GNU GPL (GNU General Public License) is a free software license that ensures users can freely use, study, share and modify software.
//  The main feature of this protocol is that any modifications or derivative works must be publicly released in the same way, i.e., must be open source. Additionally, the protocol requires that copyright information and license agreements be retained when using or distributing software. GNU GPL is a protocol established and maintained by the Free Software Foundation (FSF), commonly used in GNU project software and other free software.
//  Only tested on official DengFOC hardware, welcome to purchase hardware/support the author, search Taobao shop: DengFOC open source
//  Your support will be the funding for making videos and continuing open source, DengGe thanks everyone here

#include "DengFOC.h"
#include <stdint.h>
#include <math.h>

// ==================== MOTOR CONFIGURATION ====================
int Sensor_DIR = 1; // Sensor direction, reverse this value if motor operation is abnormal
int Motor_PP = 7;   // Motor pole pairs

// ==================== ANGLE LIMITS ====================
#define MAX_ANGLE_TURNS 30                          // Maximum angle in full rotations (decidegrees = turns * 36000)
#define MAX_ANGLE_DECIDEG (MAX_ANGLE_TURNS * 36000) // ±720,000 decidegrees = ±20 rotations

// ==================== TORQUE CONTROL CONFIGURATION ====================
/// Enable/disable individual torque contributions
struct DialConfig
{
  bool enable_detent = false;            // Enable detent mode (spring-like torque towards nearest detent positions)
  bool enable_vibration = false;         // Enable periodic vibration pulses (for testing/debugging)
  bool enable_bounds_restoration = true; // Enable strong corrective torque when exceeding angle bounds
  bool enable_oob_kick = true;           // Enable out-of-bounds (OOB) kicking mode (pulsed corrective force when outside bounds)
  bool enable_tracking = true;           // Enable position tracking towards a target

  // Detent mode parameters
  float detent_distance = 10 * 3.1415926 / 180.0; // One detent position every ~10 degrees
  float detent_kp = 5.0;                          // Proportional gain for detent spring effect
  float detent_max_torque = 1.0;                  // Maximum absolute torque magnitude (A)

  // Bounds restoration parameters
  float bounds_min_angle = -3.1415926; // Minimum allowed angle (radians, ~-180°)
  float bounds_max_angle = 3.1415926;  // Maximum allowed angle (radians, ~+180°)
  float bounds_kp = 20.0;              // Proportional gain for bounds correction (larger than detent)
  float bounds_max_torque = 3.0;       // Maximum absolute torque magnitude (A)

  // Tracking position parameters
  float tracking_position = 0.0;   // Target position to track towards (radians)
  float tracking_kp = 5.0;         // Proportional gain for position tracking
  float tracking_kd = 0.1;         // Derivative gain for position tracking (damping)
  float tracking_max_torque = 2.0; // Maximum absolute torque magnitude (A)

  // Vibration/debug parameters
  float vibration_amplitude = 1.0;                  // Amplitude of vibration test pulse (A)
  unsigned long vibration_pulse_interval_ms = 1000; // Interval between vibration pulses (ms)

  // Out-of-bounds (OOB) kicking mode
  float oob_kick_amplitude = 1.0;                // Amplitude of OOB kick (A)
  unsigned long oob_kick_pulse_interval_ms = 40; // Interval between OOB kicks (ms)
};

// Dial class encapsulates per-motor state and behavior
class Dial
{
private:
  unsigned long last_vibration_time_local; // Timestamp of last vibration pulse
  unsigned long last_kick_time_local;      // Timestamp of last out-of-bounds kick
  bool kick_state;                         // State flag for kick pulse (unused in current implementation)

public:
  int motor_index;   // Which motor this dial controls (0 or 1)
  DialConfig *cfg;   // Pointer to configuration settings
  float last_torque; // Last calculated torque value (A)
  float last_angle;  // Last measured motor angle (radians)
  float last_speed;  // Last measured motor speed (rad/s)

  Dial(int idx = 0, DialConfig *c = nullptr)
  {
    motor_index = idx;             // Store motor index referencing
    cfg = c;                       // Bind to configuration structure
    last_vibration_time_local = 0; // Initialize vibration timer
    last_angle = 0.0;              // Initialize angle tracking
  }

  void begin()
  {
    // Initialize timing for vibration and kick effects
    last_vibration_time_local = millis();
    last_kick_time_local = 0;
    kick_state = false;
  }

  /// @brief Get motor angle (supports both M0 and M1)
  /// @param motor_index 0 for M0, 1 for M1
  /// @return Current motor angle (radians)
  float get_motor_angle(int motor_index)
  {
    // Retrieve angle from appropriate motor based on index
    return (motor_index == 0) ? DFOC_M0_Angle() : DFOC_M1_Angle();
    last_kick_time_local = millis();
  }

  /// @brief Get motor speed (supports both M0 and M1)
  /// @param motor_index 0 for M0, 1 for M1
  /// @return Current motor speed (rad/s)
  float get_motor_speed(int motor_index)
  {
    // Retrieve speed from appropriate motor based on index
    return (motor_index == 0) ? DFOC_M0_Velocity() : DFOC_M1_Velocity();
  }

  float calculate_detent_torque(float current_angle)
  {
    // Find nearest detent position and apply spring-like restoration force
    float nearest_detent_angle = round(current_angle / cfg->detent_distance) * cfg->detent_distance;
    float error = nearest_detent_angle - current_angle;
    // Compute torque using spring formula: torque = kp * error, where kp is the detent stiffness
    float torque = cfg->detent_kp * error;

    // Clamp torque to maximum allowed magnitude
    if (torque > cfg->detent_max_torque)
      torque = cfg->detent_max_torque;
    else if (torque < -cfg->detent_max_torque)
      torque = -cfg->detent_max_torque;
    return torque;
  }

  float calculate_tracking_torque(float current_angle, float current_speed)
  {
    float error = cfg->tracking_position - current_angle;

    // Spring
    float torque = cfg->tracking_kp * error;

    // Damping
    torque -= cfg->tracking_kd * current_speed;

    // Clamp
    if (torque > cfg->tracking_max_torque)
      torque = cfg->tracking_max_torque;
    else if (torque < -cfg->tracking_max_torque)
      torque = -cfg->tracking_max_torque;

    return torque;
  }

  float calculate_oob_kick_torque(unsigned long now)
  {
    // Out-of-bounds (OOB) kicking: click at configured frequency, direction towards corrective side
    // Only generate kick when outside bounds
    if (last_angle <= cfg->bounds_max_angle && last_angle >= cfg->bounds_min_angle)
      return 0.0; // Within bounds, no corrective kicks needed

    // Determine direction: negative if above max, positive if below min
    float sign = (last_angle > cfg->bounds_max_angle) ? -1.0f : 1.0f;

    // Apply pulsed kick at configured interval
    if (now - last_kick_time_local >= cfg->oob_kick_pulse_interval_ms)
    {
      last_kick_time_local = now;
      kick_state = !kick_state; // Toggle for alternating kicks (currently unused)
    }

    if (kick_state)
      return sign * cfg->oob_kick_amplitude;
    return 0.0;
  }

  float calculate_vibration_torque(unsigned long now)
  {
    // Generate periodic vibration pulses at configured interval
    if (now - last_vibration_time_local >= cfg->vibration_pulse_interval_ms)
    {
      last_vibration_time_local = now;
      return cfg->vibration_amplitude; // Send pulse
    }
    return 0.0; // No pulse in this cycle
  }

  float calculate_bounds_torque(float current_angle)
  {
    // Apply corrective torque when angle exceeds bounds (stronger than detent)
    if (current_angle > cfg->bounds_max_angle)
    {
      // Above maximum: apply negative (restoring) torque
      float error = current_angle - cfg->bounds_max_angle;
      float torque = -cfg->bounds_kp * error;
      if (torque < -cfg->bounds_max_torque)
        torque = -cfg->bounds_max_torque;
      return torque;
    }
    else if (current_angle < cfg->bounds_min_angle)
    {
      // Below minimum: apply positive (restoring) torque
      float error = cfg->bounds_min_angle - current_angle;
      float torque = cfg->bounds_kp * error;
      if (torque > cfg->bounds_max_torque)
        torque = cfg->bounds_max_torque;
      return torque;
    }
    return 0.0; // Within bounds
  }

  float calculate_composite_torque(unsigned long now)
  {
    // Combine all enabled torque effects into single control value
    last_angle = get_motor_angle(motor_index);
    last_speed = get_motor_speed(motor_index);
    float total = 0.0;

    // Add individual torque contributions based on enabled features
    if (cfg->enable_detent)
      total += calculate_detent_torque(last_angle);
    if (cfg->enable_tracking)
      total += calculate_tracking_torque(last_angle, last_speed);
    if (cfg->enable_vibration)
      total += calculate_vibration_torque(now);
    if (cfg->enable_bounds_restoration)
      total += calculate_bounds_torque(last_angle);

    // Out-of-bounds kicking: additional pulsed corrective force when outside bounds
    if (cfg->enable_oob_kick)
      total += calculate_oob_kick_torque(now);

    last_torque = total;
    return total;
  }

  void apply_torque(float torque)
  {
    // Send calculated torque command to appropriate motor
    if (motor_index == 0)
      DFOC_M0_setTorque_current(torque);
    else
      DFOC_M1_setTorque_current(torque);
    last_torque = torque;
  }

  void calculate_and_apply_composite_torque()
  {
    // Main control loop: calculate all torques and apply to motor
    unsigned long now = millis();
    float t = calculate_composite_torque(now);
    apply_torque(t);
  }
};

// ==================== GLOBAL INSTANCES AND CONSTANTS ====================

// Current configuration (easy to modify)
DialConfig dial_config0;
DialConfig dial_config1;

// Create per-dial instances (after class definition)
Dial dial0(0, &dial_config0); // Motor 0 controller
Dial dial1(1, &dial_config1); // Motor 1 controller

// Serial input buffer for receiving commands from Python
const int SERIAL_BUFFER_SIZE = 128;
char serial_buffer[SERIAL_BUFFER_SIZE];
int serial_buffer_index = 0;

// Protocol state
uint32_t last_processed_seq = 0; // seq from last processed C command

// FPS and timing statistics for performance monitoring
#define DEBUG_PRINT_INTERVAL 20    // Print interval (ms) - can be changed without affecting FPS accuracy
#define FPS_MEASURE_INTERVAL 1000  // FPS measurement window (ms) - independent of print interval
unsigned long last_print_time = 0; // Last time debug info was printed
unsigned long last_fps_time = 0;   // Last time FPS was calculated
unsigned long frame_count = 0;     // Number of loop cycles in current FPS window
float last_calculated_fps = 0.0;   // Last calculated FPS value

// ==================== INITIALIZATION ====================

void setup()
{
  // Initialize serial communication for debugging
  Serial.begin(230400);

  // Initialize motor enable pin (GPIO 12)
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH); // Motor Enable, must be placed before motor calibration

  // Configure motor driver voltage and align Hall sensors
  DFOC_Vbus(12);                             // Set driver power supply voltage to 12V
  DFOC_M0_alignSensor(Motor_PP, Sensor_DIR); // Calibrate motor 0 sensor
  DFOC_M1_alignSensor(Motor_PP, Sensor_DIR); // Calibrate motor 1 sensor

  // Initialize per-dial state and timers
  dial0.begin();
  dial1.begin();

  // Initialize serial input buffer
  serial_buffer_index = 0;
}

// ==================== SERIAL INPUT HANDLING ====================

void parse_host_command(const char *line)
{
  // Protocol tokens: comma-separated
  // First token: command letter (C/S/E)
  // Second token: seq (uint32)

  char copy[SERIAL_BUFFER_SIZE];
  strncpy(copy, line, SERIAL_BUFFER_SIZE - 1);
  copy[SERIAL_BUFFER_SIZE - 1] = '\0';

  // Tokenize
  const char *delim = ",";
  char *token = strtok(copy, delim);
  if (!token)
    return;

  // Command letter
  char cmd = token[0];

  // Seq
  token = strtok(NULL, delim);
  if (!token)
    return;
  uint32_t seq = (uint32_t)strtoul(token, NULL, 10);

  if (cmd == 'C')
  {
    // Expect: C,seq,pos0,pos1,min0,max0,min1,max1
    char *f[8];
    int i = 0;
    while (i < 8 && (f[i] = strtok(NULL, delim)) != NULL)
      i++;

    if (i >= 2)
    {
      // pos0,pos1 required
      long pos0 = atol(f[0]);
      long pos1 = atol(f[1]);

      // Convert decidegrees -> radians: rad = decideg * pi / 1800.0
      float pos0_rad = (float)pos0 * 3.1415926f / 1800.0f;
      float pos1_rad = (float)pos1 * 3.1415926f / 1800.0f;

      dial_config0.tracking_position = pos0_rad;
      dial_config1.tracking_position = pos1_rad;

      if (i >= 4)
      {
        long min0 = atol(f[2]);
        long max0 = atol(f[3]);
        dial_config0.bounds_min_angle = (float)min0 * 3.1415926f / 1800.0f;
        dial_config0.bounds_max_angle = (float)max0 * 3.1415926f / 1800.0f;
      }
      if (i >= 6)
      {
        long min1 = atol(f[4]);
        long max1 = atol(f[5]);
        dial_config1.bounds_min_angle = (float)min1 * 3.1415926f / 1800.0f;
        dial_config1.bounds_max_angle = (float)max1 * 3.1415926f / 1800.0f;
      }

      last_processed_seq = seq;
    }
  }
  else if (cmd == 'S')
  {
    // Expect: S,seq,param,value
    char *param = strtok(NULL, delim);
    char *valstr = strtok(NULL, delim);
    if (param && valstr)
    {
      long v = atol(valstr);
      // Interpret parameter numeric value as fixed-point with 1000 scale
      float fval = (float)v / 1000.0f;

      // Handle common parameter names
      if (strcmp(param, "tracking_kp_0") == 0)
        dial_config0.tracking_kp = fval;
      else if (strcmp(param, "tracking_kp_1") == 0)
        dial_config1.tracking_kp = fval;
      else if (strcmp(param, "tracking_kd_0") == 0)
        dial_config0.tracking_kd = fval;
      else if (strcmp(param, "tracking_kd_1") == 0)
        dial_config1.tracking_kd = fval;
      else if (strcmp(param, "detent_kp_0") == 0)
        dial_config0.detent_kp = fval;
      else if (strcmp(param, "detent_kp_1") == 0)
        dial_config1.detent_kp = fval;
      else if (strcmp(param, "bounds_kp_0") == 0)
        dial_config0.bounds_kp = fval;
      else if (strcmp(param, "bounds_kp_1") == 0)
        dial_config1.bounds_kp = fval;
      else if (strcmp(param, "detent_distance_0") == 0)
        dial_config0.detent_distance = fval * 3.1415926f / 1800.0f; // assume value in decideg -> convert to rad
      else if (strcmp(param, "detent_distance_1") == 0)
        dial_config1.detent_distance = fval * 3.1415926f / 1800.0f;
      else if (strcmp(param, "vibration_amplitude_0") == 0)
        dial_config0.vibration_amplitude = fval;
      else if (strcmp(param, "vibration_amplitude_1") == 0)
        dial_config1.vibration_amplitude = fval;
      else if (strcmp(param, "oob_kick_amplitude_0") == 0)
        dial_config0.oob_kick_amplitude = fval;
      else if (strcmp(param, "oob_kick_amplitude_1") == 0)
        dial_config1.oob_kick_amplitude = fval;
      // Unknown parameters are ignored
    }
  }
  else if (cmd == 'E')
  {
    // Echo test: send R,seq\n
    Serial.print("R,");
    Serial.println(seq);
  }
}

void process_serial_input()
{
  // Non-blocking serial input processing
  // Reads available bytes and buffers them until newline is received
  // Format: "M0C:value,M1C:value\n"

  while (Serial.available() > 0)
  {
    char inByte = Serial.read();

    // Check for end-of-message (newline)
    if (inByte == '\n')
    {
      // Process the complete message
      serial_buffer[serial_buffer_index] = '\0'; // Null terminate
      parse_host_command(serial_buffer);
      serial_buffer_index = 0; // Reset buffer for next message
    }
    else if (inByte == '\r')
    {
      // Ignore carriage return
      continue;
    }
    else if (serial_buffer_index < SERIAL_BUFFER_SIZE - 1)
    {
      // Add character to buffer
      serial_buffer[serial_buffer_index++] = inByte;
    }
    else
    {
      // Buffer overflow protection: reset buffer
      serial_buffer_index = 0;
      Serial.println("ERROR: Serial buffer overflow");
    }
  }
}

// ==================== MAIN CONTROL LOOP ====================

void loop()
{
  unsigned long current_time = millis();
  frame_count++;

  // ==================== SERIAL INPUT ====================
  // Non-blocking read of joint position commands from Python
  process_serial_input();

  // ==================== FOC & MOTOR CONTROL ====================
  // Run field-oriented control updates and read sensor feedback
  runFOC_M0();
  runFOC_M1();

  // Calculate all enabled torque effects and apply to motors
  dial0.calculate_and_apply_composite_torque();
  dial1.calculate_and_apply_composite_torque();

  // ==================== PERFORMANCE METRICS ====================
  // Measure FPS over independent measurement window (decoupled from print interval)
  if (current_time - last_fps_time >= FPS_MEASURE_INTERVAL)
  {
    last_calculated_fps = (float)frame_count * 1000.0 / (current_time - last_fps_time);
    frame_count = 0;
    last_fps_time = current_time;
  }

  // ==================== DATA TRANSMISSION (DengFOC V4 protocol) ====================
  // Transmit at configured debug interval: T,seq,ang0,ang1,tor0,tor1\n
  if (current_time - last_print_time >= DEBUG_PRINT_INTERVAL)
  {
    // Angle: radians -> decidegrees (0.1 deg per unit)
    float ang0_deg = dial0.last_angle * 180.0f / 3.1415926f;
    float ang1_deg = dial1.last_angle * 180.0f / 3.1415926f;
    float ang0_decideg_f = ang0_deg * 10.0f;
    float ang1_decideg_f = ang1_deg * 10.0f;

    long ang0_decideg = (long)(ang0_decideg_f > 0 ? ang0_decideg_f + 0.5f : ang0_decideg_f - 0.5f);
    long ang1_decideg = (long)(ang1_decideg_f > 0 ? ang1_decideg_f + 0.5f : ang1_decideg_f - 0.5f);

    // Clamp to ±MAX_ANGLE_DECIDEG
    if (ang0_decideg > MAX_ANGLE_DECIDEG)
      ang0_decideg = MAX_ANGLE_DECIDEG;
    else if (ang0_decideg < -MAX_ANGLE_DECIDEG)
      ang0_decideg = -MAX_ANGLE_DECIDEG;
    if (ang1_decideg > MAX_ANGLE_DECIDEG)
      ang1_decideg = MAX_ANGLE_DECIDEG;
    else if (ang1_decideg < -MAX_ANGLE_DECIDEG)
      ang1_decideg = -MAX_ANGLE_DECIDEG;

    // Torque: amps -> milliamps
    float tor0_ma_f = dial0.last_torque * 1000.0f;
    float tor1_ma_f = dial1.last_torque * 1000.0f;
    long tor0_ma = (long)(tor0_ma_f > 0 ? tor0_ma_f + 0.5f : tor0_ma_f - 0.5f);
    long tor1_ma = (long)(tor1_ma_f > 0 ? tor1_ma_f + 0.5f : tor1_ma_f - 0.5f);

    // Clamp torque to ±10000 mA
    if (tor0_ma > 10000)
      tor0_ma = 10000;
    else if (tor0_ma < -10000)
      tor0_ma = -10000;
    if (tor1_ma > 10000)
      tor1_ma = 10000;
    else if (tor1_ma < -10000)
      tor1_ma = -10000;

    // Compute FPS averaged value (int) and clamp to allowed range
    long fps_val = (long)(last_calculated_fps + 0.5f);
    if (fps_val < 0)
      fps_val = 0;
    if (fps_val > 2000)
      fps_val = 2000;

    // Emit: T,seq,ang0,ang1,tor0,tor1,fps\n
    Serial.print("T,");
    Serial.print(last_processed_seq);
    Serial.print(",");
    Serial.print(ang0_decideg);
    Serial.print(",");
    Serial.print(ang1_decideg);
    Serial.print(",");
    Serial.print(tor0_ma);
    Serial.print(",");
    Serial.print(tor1_ma);
    Serial.print(",");
    Serial.print(fps_val);
    Serial.println();

    last_print_time = current_time;
  }
}
