// Example: Detent + Vibration Mode
//  DengFOC V0.2
//  DengFOC open source, follow GNU protocol, please indicate copyright when reprinting!
//  GNU GPL (GNU General Public License) is a free software license that ensures users can freely use, study, share and modify software.
//  The main feature of this protocol is that any modifications or derivative works must be publicly released in the same way, i.e., must be open source. Additionally, the protocol requires that copyright information and license agreements be retained when using or distributing software. GNU GPL is a protocol established and maintained by the Free Software Foundation (FSF), commonly used in GNU project software and other free software.
//  Only tested on official DengFOC hardware, welcome to purchase hardware/support the author, search Taobao shop: DengFOC open source
//  Your support will be the funding for making videos and continuing open source, DengGe thanks everyone here

#include "DengFOC.h"

// ==================== MOTOR CONFIGURATION ====================
int Sensor_DIR = 1; // Sensor direction, reverse this value if motor operation is abnormal
int Motor_PP = 7;   // Motor pole pairs

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

  float calculate_tracking_torque(float current_angle)
  {
    // Apply corrective force to bring dial towards tracking_position target
    float error = cfg->tracking_position - current_angle;
    float torque = cfg->tracking_kp * error;

    // Clamp torque to maximum allowed magnitude
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
    float total = 0.0;

    // Add individual torque contributions based on enabled features
    if (cfg->enable_detent)
      total += calculate_detent_torque(last_angle);
    if (cfg->enable_tracking)
      total += calculate_tracking_torque(last_angle);
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
DialConfig dial_config;

// Create per-dial instances (after class definition)
Dial dial0(0, &dial_config); // Motor 0 controller
Dial dial1(1, &dial_config); // Motor 1 controller

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
  Serial.begin(115200);

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
}

// ==================== MAIN CONTROL LOOP ====================

void loop()
{
  unsigned long current_time = millis();
  frame_count++;

  // ==================== FOC & MOTOR CONTROL ====================
  // Run field-oriented control updates and read sensor feedback
  runFOC();

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

  // ==================== DATA TRANSMISSION ====================
  // Send structured telemetry data at regular intervals for Python script processing
  // Format: Field labels with colon separators, comma-delimited values
  // Example: T:12345,F:60.5,M0T:0.5,M0A:45.2,M1T:0.3,M1A:-90.1
  if (current_time - last_print_time >= DEBUG_PRINT_INTERVAL)
  {
    // Transmit data in structured key:value format separated by commas
    Serial.print("T:");
    Serial.print(current_time);
    Serial.print(",F:");
    Serial.print(last_calculated_fps, 1); // FPS with 1 decimal place
    Serial.print(",M0T:");
    Serial.print(dial0.last_torque, 2); // Motor 0 torque with 2 decimal places
    Serial.print(",M0A:");
    Serial.print(dial0.last_angle * 180 / 3.1415926, 1); // Motor 0 angle in degrees with 1 decimal
    Serial.print(",M1T:");
    Serial.print(dial1.last_torque, 2); // Motor 1 torque with 2 decimal places
    Serial.print(",M1A:");
    Serial.print(dial1.last_angle * 180 / 3.1415926, 1); // Motor 1 angle in degrees with 1 decimal
    Serial.println();                                    // End of message with newline for frame synchronization

    last_print_time = current_time;
  }
}
