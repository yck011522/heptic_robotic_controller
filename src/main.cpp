// Example: Detent + Vibration Mode
//  DengFOC V0.2
//  灯哥开源，遵循GNU协议，转载请著名版权！
//  GNU开源协议（GNU General Public License, GPL）是一种自由软件许可协议，保障用户能够自由地使用、研究、分享和修改软件。
//  该协议的主要特点是，要求任何修改或衍生的作品必须以相同的方式公开发布，即必须开源。此外，该协议也要求在使用或分发软件时，必须保留版权信息和许可协议。GNU开源协议是自由软件基金会（FSF）制定和维护的一种协议，常用于GNU计划的软件和其他自由软件中。
//  仅在DengFOC官方硬件上测试过，欢迎硬件购买/支持作者，淘宝搜索店铺：灯哥开源
//  你的支持将是接下来做视频和持续开源的经费，灯哥在这里先谢谢大家了

#include "DengFOC.h"

// ==================== MOTOR CONFIGURATION ====================
int Sensor_DIR = 1; // 传感器方向，若电机运动不正常，将此值取反
int Motor_PP = 7;   // 电机极对数

// ==================== TORQUE CONTROL CONFIGURATION ====================
/// Enable/disable individual torque contributions
struct TorqueConfig
{
  bool enable_detent = false;
  bool enable_vibration = true;
  bool enable_bounds_restoration = true;

  // Detent mode parameters
  float detent_kp = 10.0;          // Proportional gain for detent spring effect
  float detent_deadband_pct = 0.0; // Deadband as percentage of detent spacing
  float detent_max_torque = 1.0;   // Maximum absolute torque magnitude (A)

  // Bounds restoration parameters
  float bounds_min_angle = -3.1415926; // Minimum allowed angle (radians, ~-180°)
  float bounds_max_angle = 3.1415926;  // Maximum allowed angle (radians, ~+180°)
  float bounds_kp = 20.0;              // Proportional gain for bounds correction (larger than detent)
  float bounds_max_torque = 3.0;       // Maximum absolute torque magnitude (A)

  // Vibration/debug parameters
  float vibration_amplitude = 1.0;                  // Amplitude of vibration test pulse (A)
  unsigned long vibration_pulse_interval_ms = 1000; // Interval between vibration pulses (ms)
};

// Current configuration (easy to modify)
TorqueConfig torque_config;

// Detent mode state
static float detent_distance = 10 * 3.1415926 / 180.0; // 八分度，一分度45°
static float detent_deadband_angle_rad = 0;

// FPS stats
#define DEBUG_PRINT_INTERVAL 20   // Print interval (ms) - can be changed without affecting FPS accuracy
#define FPS_MEASURE_INTERVAL 1000 // FPS measurement window (ms) - independent of print interval
unsigned long last_print_time = 0;
unsigned long last_fps_time = 0;
unsigned long last_vibration_time[2] = {0, 0};
unsigned long frame_count = 0;
float last_calculated_fps = 0.0;

// ==================== TORQUE CONTRIBUTION FUNCTIONS ====================

/// @brief Calculate detent/spring torque contribution
/// Creates a spring-like force pulling toward nearest detent position
/// @param current_angle Current motor angle (radians)
/// @param config Torque configuration
/// @return Torque command (A) to maintain detent position, clamped to [-max_torque, +max_torque]
float calculate_detent_torque(float current_angle, const TorqueConfig &config)
{
  // Calculate nearest detent angle
  float nearest_detent_angle = round(current_angle / detent_distance) * detent_distance;
  float error = nearest_detent_angle - current_angle;

  // Update deadband each time config changes (optional optimization)
  detent_deadband_angle_rad = detent_distance * config.detent_deadband_pct;

  // Apply spring force proportional to distance (with deadband)
  if (abs(error) > detent_deadband_angle_rad)
  {
    float torque = config.detent_kp * error;
    // Clamp torque to maximum absolute value
    if (torque > config.detent_max_torque)
      torque = config.detent_max_torque;
    else if (torque < -config.detent_max_torque)
      torque = -config.detent_max_torque;
    return torque;
  }
  return 0.0;
}

/// @brief Calculate vibration/test pulse torque contribution
/// Generates periodic torque pulses for haptic feedback or vibration testing
/// @param current_time Current timestamp (ms)
/// @param config Torque configuration
/// @return Torque command (A) for vibration pulse
float calculate_vibration_torque(int motor_index, unsigned long current_time, const TorqueConfig &config)
{
  // Generate pulse per-motor using separate timers
  if (current_time - last_vibration_time[motor_index] >= config.vibration_pulse_interval_ms)
  {
    last_vibration_time[motor_index] = current_time;
    return config.vibration_amplitude;
  }
  return 0.0;
}

/// @brief Calculate bounds restoration torque contribution
/// Applies restoring force when motor angle exceeds allowed bounds
/// @param current_angle Current motor angle (radians)
/// @param config Torque configuration
/// @return Torque command (A) to restore position within bounds, zero if within bounds
float calculate_bounds_torque(float current_angle, const TorqueConfig &config)
{
  // Check if angle is outside bounds
  if (current_angle > config.bounds_max_angle)
  {
    // Out of bounds on high side - apply negative (restoring) torque
    float error = current_angle - config.bounds_max_angle;
    float torque = -config.bounds_kp * error;
    // Clamp to maximum magnitude
    if (torque < -config.bounds_max_torque)
      torque = -config.bounds_max_torque;
    return torque;
  }
  else if (current_angle < config.bounds_min_angle)
  {
    // Out of bounds on low side - apply positive (restoring) torque
    float error = config.bounds_min_angle - current_angle;
    float torque = config.bounds_kp * error;
    // Clamp to maximum magnitude
    if (torque > config.bounds_max_torque)
      torque = config.bounds_max_torque;
    return torque;
  }

  // Within bounds - no restoring force needed
  return 0.0;
}

/// @brief Composite torque calculator combining all enabled contributions
/// @param current_angle Current motor angle (radians)
/// @param current_time Current timestamp (ms)
/// @param config Torque configuration
/// @return Total composite torque command (A)
float calculate_composite_torque(int motor_index, float current_angle, unsigned long current_time,
                                 const TorqueConfig &config)
{
  float total_torque = 0.0;

  // Sum all enabled torque contributions
  if (config.enable_detent)
  {
    total_torque += calculate_detent_torque(current_angle, config);
  }

  if (config.enable_vibration)
  {
    total_torque += calculate_vibration_torque(motor_index, current_time, config);
  }

  if (config.enable_bounds_restoration)
  {
    total_torque += calculate_bounds_torque(current_angle, config);
  }

  return total_torque;
}

/// @brief Get motor angle (supports both M0 and M1)
/// @param motor_index 0 for M0, 1 for M1
/// @return Current motor angle (radians)
float get_motor_angle(int motor_index)
{
  return (motor_index == 0) ? DFOC_M0_Angle() : DFOC_M1_Angle();
}

void setup()
{
  Serial.begin(115200);
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH); // 使能，一定要放在校准电机前

  DFOC_Vbus(12); // 设定驱动器供电电压
  DFOC_M0_alignSensor(Motor_PP, Sensor_DIR);
  DFOC_M1_alignSensor(Motor_PP, Sensor_DIR);

  // Initialize vibration timing
  last_vibration_time[0] = last_vibration_time[1] = millis();
}

void loop()
{
  unsigned long current_time = millis();
  frame_count++;

  // ==================== MOTOR 0 CONTROL ====================

  // Update sensors
  runFOC();

  // Calculate composite torque from all sources (pass motor index for per-motor vibration)
  float motor0_torque = calculate_composite_torque(0, get_motor_angle(0), current_time, torque_config);
  float motor1_torque = calculate_composite_torque(1, get_motor_angle(1), current_time, torque_config);

  // Apply torque to motor
  DFOC_M0_setTorque_current(motor0_torque);
  DFOC_M1_setTorque_current(motor1_torque);

  // ==================== FPS CALCULATION ====================
  // Measure FPS over independent measurement window (decoupled from print interval)
  if (current_time - last_fps_time >= FPS_MEASURE_INTERVAL)
  {
    last_calculated_fps = (float)frame_count * 1000.0 / (current_time - last_fps_time);
    frame_count = 0;
    last_fps_time = current_time;
  }

  // ==================== DEBUG OUTPUT ====================

  // Print debug info every DEBUG_PRINT_INTERVAL milliseconds (independent of FPS measurement)
  if (current_time - last_print_time >= DEBUG_PRINT_INTERVAL)
  {
    Serial.print("t=");
    Serial.print(current_time);
    Serial.print(" | FPS: ");
    Serial.print(last_calculated_fps, 1); // Display last calculated FPS (1 decimal place)
    Serial.print(" | M0 Torque: ");
    Serial.print(motor0_torque, 2);
    Serial.print(" A | M0 Angle: ");
    Serial.print(get_motor_angle(0) * 180 / 3.1415926, 1);
    Serial.print(" deg | M1 Torque: ");
    Serial.print(motor1_torque, 2);
    Serial.print(" A | M1 Angle: ");
    Serial.print(get_motor_angle(1) * 180 / 3.1415926, 1);
    Serial.println(" deg");

    last_print_time = current_time;
  }

  // Optional: Uncomment for detailed debugging
  // Serial.print("Mechanical Angle: ");
  // Serial.print(get_motor_angle(0) * 180 / 3.1415926);
  // Serial.print(" deg, Torque: ");
  // Serial.println(motor0_torque);
}
