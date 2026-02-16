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
struct DialConfig
{
  bool enable_detent = true;
  bool enable_vibration = false;
  bool enable_bounds_restoration = true;
  bool enable_oob_kick = true;

  // Detent mode parameters
  float detent_distance = 10 * 3.1415926 / 180.0; // 八分度，一分度45°
  float detent_kp = 10.0;                         // Proportional gain for detent spring effect
  float detent_max_torque = 1.0;                  // Maximum absolute torque magnitude (A)

  // Bounds restoration parameters
  float bounds_min_angle = -3.1415926; // Minimum allowed angle (radians, ~-180°)
  float bounds_max_angle = 3.1415926;  // Maximum allowed angle (radians, ~+180°)
  float bounds_kp = 20.0;              // Proportional gain for bounds correction (larger than detent)
  float bounds_max_torque = 3.0;       // Maximum absolute torque magnitude (A)

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
public:
  int motor_index;
  DialConfig *cfg;
  unsigned long last_vibration_time_local;
  float last_torque;
  float last_angle;
  unsigned long last_kick_time_local;
  bool kick_state;

  Dial(int idx = 0, DialConfig *c = nullptr)
  {
    motor_index = idx;
    cfg = c;
    last_vibration_time_local = 0;
    last_angle = 0.0;
  }

  void begin()
  {
    last_vibration_time_local = millis();
    last_kick_time_local = 0;
    kick_state = false;
  }

  /// @brief Get motor angle (supports both M0 and M1)
  /// @param motor_index 0 for M0, 1 for M1
  /// @return Current motor angle (radians)
  float get_motor_angle(int motor_index)
  {
    return (motor_index == 0) ? DFOC_M0_Angle() : DFOC_M1_Angle();
    last_kick_time_local = millis();
  }

  float calculate_detent_torque(float current_angle)
  {
    float nearest_detent_angle = round(current_angle / cfg->detent_distance) * cfg->detent_distance;
    float error = nearest_detent_angle - current_angle;
    float torque = cfg->detent_kp * error;
    if (torque > cfg->detent_max_torque)
      torque = cfg->detent_max_torque;
    else if (torque < -cfg->detent_max_torque)
      torque = -cfg->detent_max_torque;
    return torque;
  }
  // Out-of-bounds (OOB) kicking: click at configured frequency, direction towards corrective side
  float calculate_oob_kick_torque(unsigned long now)
  {
    // Only generate kick when outside bounds
    if (last_angle <= cfg->bounds_max_angle && last_angle >= cfg->bounds_min_angle)
      return 0.0;
    float sign = (last_angle > cfg->bounds_max_angle) ? -1.0f : 1.0f;

    if (now - last_kick_time_local >= cfg->oob_kick_pulse_interval_ms)
    {
      last_kick_time_local = now;
      // kick_state = !kick_state;
      return sign * cfg->oob_kick_amplitude;
    }

    // if (!kick_state)
    return 0.0;

    // Direction: if above max angle => restore (negative), if below min => positive
    // return sign * cfg->oob_kick_amplitude;
  }

  float calculate_vibration_torque(unsigned long now)
  {
    if (now - last_vibration_time_local >= cfg->vibration_pulse_interval_ms)
    {
      last_vibration_time_local = now;
      return cfg->vibration_amplitude;
    }
    return 0.0;
  }

  float calculate_bounds_torque(float current_angle)
  {
    if (current_angle > cfg->bounds_max_angle)
    {
      float error = current_angle - cfg->bounds_max_angle;
      float torque = -cfg->bounds_kp * error;
      if (torque < -cfg->bounds_max_torque)
        torque = -cfg->bounds_max_torque;
      return torque;
    }
    else if (current_angle < cfg->bounds_min_angle)
    {
      float error = cfg->bounds_min_angle - current_angle;
      float torque = cfg->bounds_kp * error;
      if (torque > cfg->bounds_max_torque)
        torque = cfg->bounds_max_torque;
      return torque;
    }
    return 0.0;
  }

  float calculate_composite_torque(unsigned long now)
  {
    last_angle = get_motor_angle(motor_index);
    float total = 0.0;
    if (cfg->enable_detent)
      total += calculate_detent_torque(last_angle);
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
    if (motor_index == 0)
      DFOC_M0_setTorque_current(torque);
    else
      DFOC_M1_setTorque_current(torque);
    last_torque = torque;
  }

  void calculate_and_apply_composite_torque()
  {
    unsigned long now = millis();
    float t = calculate_composite_torque(now);
    apply_torque(t);
  }
};

// Current configuration (easy to modify)
DialConfig dial_config;

// Create per-dial instances (after class definition)
Dial dial0(0, &dial_config);
Dial dial1(1, &dial_config);

// FPS stats
#define DEBUG_PRINT_INTERVAL 20   // Print interval (ms) - can be changed without affecting FPS accuracy
#define FPS_MEASURE_INTERVAL 1000 // FPS measurement window (ms) - independent of print interval
unsigned long last_print_time = 0;
unsigned long last_fps_time = 0;
unsigned long frame_count = 0;
float last_calculated_fps = 0.0;

// ==================== TORQUE CONTRIBUTION FUNCTIONS ====================

void setup()
{
  Serial.begin(115200);
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH); // 使能，一定要放在校准电机前

  DFOC_Vbus(12); // 设定驱动器供电电压
  DFOC_M0_alignSensor(Motor_PP, Sensor_DIR);
  DFOC_M1_alignSensor(Motor_PP, Sensor_DIR);

  // Initialize per-dial state
  dial0.begin();
  dial1.begin();
}

void loop()
{
  unsigned long current_time = millis();
  frame_count++;

  // ==================== MOTOR 0 CONTROL ====================

  // Update sensors
  runFOC();

  // Calculate and apply composite torque per-dial
  dial0.calculate_and_apply_composite_torque();
  dial1.calculate_and_apply_composite_torque();

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
    Serial.print(dial0.last_torque, 2);
    Serial.print(" A | M0 Angle: ");
    Serial.print(dial0.last_angle * 180 / 3.1415926, 1);
    Serial.print(" deg | M1 Torque: ");
    Serial.print(dial1.last_torque, 2);
    Serial.print(" A | M1 Angle: ");
    Serial.print(dial1.last_angle * 180 / 3.1415926, 1);
    Serial.println(" deg");

    last_print_time = current_time;
  }

  // Optional: Uncomment for detailed debugging
  // Serial.print("Mechanical Angle: ");
  // Serial.print(get_motor_angle(0) * 180 / 3.1415926);
  // Serial.print(" deg, Torque: ");
  // Serial.println(dial0.last_torque);
}
