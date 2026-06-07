// Example: Detent + Vibration Mode
//  DengFOC V0.2
//  DengFOC open source, follow GNU protocol, please indicate copyright when reprinting!
//  GNU GPL (GNU General Public License) is a free software license that ensures users can freely use, study, share and modify software.
//  The main feature of this protocol is that any modifications or derivative works must be publicly released in the same way, i.e., must be open source. Additionally, the protocol requires that copyright information and license agreements be retained when using or distributing software. GNU GPL is a protocol established and maintained by the Free Software Foundation (FSF), commonly used in GNU project software and other free software.
//  Only tested on official DengFOC hardware, welcome to purchase hardware/support the author, search Taobao shop: DengFOC open source
//  Your support will be the funding for making videos and continuing open source, DengGe thanks everyone here

#include "DengFOC.h"
#include "Dial.h"
#include <Preferences.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// ==================== FIRMWARE VERSION ====================
#define FW_VERSION "0.3.0"

// ==================== MOTOR CONFIGURATION ====================
int sensor_dir = 1;       // Sensor direction, reverse this value if motor operation is abnormal
int motor_pole_pairs = 14; // Motor pole pairs
float startup_alignment_torque = 6.0f; // Startup sensor-alignment torque

// ==================== ANGLE LIMITS ====================
#define MAX_ANGLE_TURNS 30                          // Maximum angle in full rotations (decidegrees = turns * 36000)
#define MAX_ANGLE_DECIDEG (MAX_ANGLE_TURNS * 36000) // ±1,080,000 decidegrees = ±30 rotations

#define USE_CURRENT_CONTROL false // Set to true to enable current control mode (requires current sensing hardware and configuration)

// ==================== TORQUE CONTROL CONFIGURATION ====================
// Configuration now in Dial.h

// ==================== MOTOR IDENTITY (NVS) ====================
Preferences nvs_prefs;
uint8_t dial_id = 0; // Persistent identity for this board, 0 = unconfigured

// ==================== GLOBAL INSTANCES AND CONSTANTS ====================

// Current configuration (easy to modify)
DialConfig dial_config0;

// Create per-dial instances (after class definition)
Dial dial0(&dial_config0);

// Serial input buffer for receiving commands from host
const int SERIAL_BUFFER_SIZE = 128;
char serial_buffer[SERIAL_BUFFER_SIZE];
int serial_buffer_index = 0;

// Protocol state
uint32_t last_processed_seq = 0; // seq from last processed C command
bool fault_active = false;
unsigned long fault_latched_until_ms = 0;

// Telemetry and FOC rate measurement
unsigned long telemetry_interval_ms = 20; // Telemetry reporting interval (ms) - modifiable via S command
unsigned long last_telemetry_time = 0;    // Last time telemetry was sent

// FOC Rate measurement
#define FOC_RATE_MEASURE_INTERVAL 200 // FOC rate measurement window (ms)
unsigned long last_foc_rate_time = 0; // Last time FOC rate was calculated
unsigned long foc_cycle_count = 0;    // Number of FOC cycles in current measurement window
float last_foc_rate_hz = 0.0;         // Last calculated FOC rate (Hz)

static double decideg_to_rad(long angle_decideg)
{
  return (double)angle_decideg * 3.1415926 / 1800.0;
}

static long rad_to_decideg(double angle_rad)
{
  double angle_decideg = angle_rad * 1800.0 / 3.1415926;
  return (long)(angle_decideg > 0 ? angle_decideg + 0.5f : angle_decideg - 0.5f);
}

static bool parse_long_field(const char *field, long *value)
{
  if (!field || field[0] == '\0')
    return false;

  char *end = nullptr;
  long parsed = strtol(field, &end, 10);
  if (*end != '\0')
    return false;

  *value = parsed;
  return true;
}

static bool parse_uint32_field(const char *field, uint32_t *value)
{
  if (!field || field[0] == '\0')
    return false;

  char *end = nullptr;
  unsigned long parsed = strtoul(field, &end, 10);
  if (*end != '\0')
    return false;

  *value = (uint32_t)parsed;
  return true;
}

static bool is_valid_angle_decideg(long angle_decideg)
{
  return angle_decideg >= -MAX_ANGLE_DECIDEG && angle_decideg <= MAX_ANGLE_DECIDEG;
}

static uint16_t build_status_bits()
{
  uint16_t status_bits = 0;

  if (dial_config0.enable_tracking)
    status_bits |= 1u << 0;
  if (dial_config0.enable_bounds_restoration)
    status_bits |= 1u << 1;
  if (dial_config0.enable_oob_kick)
    status_bits |= 1u << 2;
  if (dial_config0.enable_detent)
    status_bits |= 1u << 3;
  if (dial_config0.enable_vibration)
    status_bits |= 1u << 4;
  if (dial0.is_out_of_bounds())
    status_bits |= 1u << 5;
  if (fault_active)
    status_bits |= 1u << 6;

  return status_bits;
}

static void latch_fault_for_ms(unsigned long duration_ms)
{
  unsigned long now = millis();
  unsigned long candidate_until = now + duration_ms;

  fault_active = true;
  if (fault_latched_until_ms == 0 || (long)(candidate_until - fault_latched_until_ms) > 0)
    fault_latched_until_ms = candidate_until;
}

static void clear_fault_if_not_latched(unsigned long now)
{
  if (fault_latched_until_ms != 0 && (long)(now - fault_latched_until_ms) < 0)
    return;

  fault_active = false;
  fault_latched_until_ms = 0;
}

static uint8_t load_persistent_dial_id()
{
  nvs_prefs.begin("robot", false);
  uint8_t stored_dial_id = nvs_prefs.getUChar("dial_id", 0);
  nvs_prefs.end();
  return stored_dial_id;
}

static void store_persistent_dial_id(uint8_t new_dial_id)
{
  nvs_prefs.begin("robot", false);
  nvs_prefs.putUChar("dial_id", new_dial_id);
  nvs_prefs.end();
}

static bool apply_runtime_parameter(const char *param, long value)
{
  float fixed_point_value = (float)value / 1000.0f;

  if (strcmp(param, "tracking_kp") == 0)
    dial_config0.tracking_kp = fixed_point_value;
  else if (strcmp(param, "tracking_kd") == 0)
    dial_config0.tracking_kd = fixed_point_value;
  else if (strcmp(param, "detent_kp") == 0)
    dial_config0.detent_kp = fixed_point_value;
  else if (strcmp(param, "bounds_kp") == 0)
    dial_config0.bounds_kp = fixed_point_value;
  else if (strcmp(param, "detent_distance") == 0)
    dial_config0.detent_distance = fixed_point_value * 3.1415926f / 1800.0f;
  else if (strcmp(param, "vibration_amplitude") == 0)
    dial_config0.vibration_amplitude = fixed_point_value;
  else if (strcmp(param, "oob_kick_amplitude") == 0)
    dial_config0.oob_kick_amplitude = fixed_point_value;
  else if (strcmp(param, "tracking_max_torque") == 0)
    dial_config0.tracking_max_torque = fixed_point_value;
  else if (strcmp(param, "bounds_max_torque") == 0)
    dial_config0.bounds_max_torque = fixed_point_value;
  else if (strcmp(param, "detent_max_torque") == 0)
    dial_config0.detent_max_torque = fixed_point_value;
  else if (strcmp(param, "vibration_pulse_interval_ms") == 0)
    dial_config0.vibration_pulse_interval_ms = value > 0 ? (unsigned long)value : dial_config0.vibration_pulse_interval_ms;
  else if (strcmp(param, "oob_kick_pulse_interval_ms") == 0)
    dial_config0.oob_kick_pulse_interval_ms = value > 0 ? (unsigned long)value : dial_config0.oob_kick_pulse_interval_ms;
  else if (strcmp(param, "enable_tracking") == 0)
    dial_config0.enable_tracking = (value != 0);
  else if (strcmp(param, "enable_detent") == 0)
    dial_config0.enable_detent = (value != 0);
  else if (strcmp(param, "enable_bounds_restoration") == 0)
    dial_config0.enable_bounds_restoration = (value != 0);
  else if (strcmp(param, "enable_oob_kick") == 0)
    dial_config0.enable_oob_kick = (value != 0);
  else if (strcmp(param, "enable_vibration") == 0)
    dial_config0.enable_vibration = (value != 0);
  else if (strcmp(param, "telemetry_interval") == 0)
  {
    if (value <= 0)
      return false;
    telemetry_interval_ms = (unsigned long)value;
  }
  else
    return false;

  return true;
}

// ==================== INITIALIZATION ====================

void setup()
{
  // Initialize serial communication for debugging
  Serial.begin(230400);

  dial_id = load_persistent_dial_id();

  // Initialize motor enable pin (GPIO 12)
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH); // Motor Enable, must be placed before motor calibration

  // Configure motor driver voltage and align Hall sensors
  DFOC_Vbus(12);                                                             // Set driver power supply voltage to 12V
  DFOC_M0_alignSensor(motor_pole_pairs, sensor_dir, startup_alignment_torque); // Calibrate motor 0 sensor

  // Initialize per-dial state and timers
  dial0.begin();

  // Initialize serial input buffer
  serial_buffer_index = 0;
}

// ==================== SERIAL INPUT HANDLING ====================

void parse_host_command(const char *line)
{
  char copy[SERIAL_BUFFER_SIZE];
  strncpy(copy, line, SERIAL_BUFFER_SIZE - 1);
  copy[SERIAL_BUFFER_SIZE - 1] = '\0';

  const char *delim = ",";
  char *token = strtok(copy, delim);
  if (!token)
    return;

  char cmd = token[0];

  token = strtok(NULL, delim);
  uint32_t seq = 0;
  if (!parse_uint32_field(token, &seq))
  {
    fault_active = true;
    return;
  }

  if (cmd == 'C')
  {
    char *target_str = strtok(NULL, delim);
    char *min_str = strtok(NULL, delim);
    char *max_str = strtok(NULL, delim);
    char *extra = strtok(NULL, delim);

    long target_decideg = 0;
    long min_decideg = 0;
    long max_decideg = 0;

    if (!target_str || !min_str || !max_str || extra ||
        !parse_long_field(target_str, &target_decideg) ||
        !parse_long_field(min_str, &min_decideg) ||
        !parse_long_field(max_str, &max_decideg) ||
        !is_valid_angle_decideg(target_decideg) ||
        !is_valid_angle_decideg(min_decideg) ||
        !is_valid_angle_decideg(max_decideg) ||
        min_decideg > max_decideg)
    {
      fault_active = true;
      return;
    }

    dial_config0.tracking_position = decideg_to_rad(target_decideg);
    dial_config0.bounds_min_angle = decideg_to_rad(min_decideg);
    dial_config0.bounds_max_angle = decideg_to_rad(max_decideg);
    last_processed_seq = seq;
    clear_fault_if_not_latched(millis());
  }
  else if (cmd == 'R')
  {
    char *current_pos_str = strtok(NULL, delim);
    char *extra = strtok(NULL, delim);
    long current_pos_decideg = 0;

    if (!current_pos_str || extra ||
        !parse_long_field(current_pos_str, &current_pos_decideg) ||
        !is_valid_angle_decideg(current_pos_decideg))
    {
      fault_active = true;
      return;
    }

    dial0.set_current_position(decideg_to_rad(current_pos_decideg), true);
    clear_fault_if_not_latched(millis());

    Serial.print("R,");
    Serial.println(seq);
  }
  else if (cmd == 'S')
  {
    char *param = strtok(NULL, delim);
    char *valstr = strtok(NULL, delim);
    char *extra = strtok(NULL, delim);
    long value = 0;

    if (!param || !valstr || extra || !parse_long_field(valstr, &value))
    {
      fault_active = true;
      return;
    }

    apply_runtime_parameter(param, value);
    clear_fault_if_not_latched(millis());
    Serial.print("S,");
    Serial.println(seq);
  }
  else if (cmd == 'I')
  {
    char *dial_id_str = strtok(NULL, delim);
    char *extra = strtok(NULL, delim);

    if (extra)
    {
      fault_active = true;
      return;
    }

    if (dial_id_str)
    {
      long requested_dial_id = 0;
      if (!parse_long_field(dial_id_str, &requested_dial_id) || requested_dial_id < 0 || requested_dial_id > 255)
      {
        fault_active = true;
        return;
      }

      dial_id = (uint8_t)requested_dial_id;
      store_persistent_dial_id(dial_id);
    }

    clear_fault_if_not_latched(millis());

    Serial.print("I,");
    Serial.print(seq);
    Serial.print(",");
    Serial.println(dial_id);
  }
  else if (cmd == 'V')
  {
    Serial.print("V,");
    Serial.print(seq);
    Serial.print(",");
    Serial.println(FW_VERSION);
  }
  else if (cmd == 'E')
  {
    Serial.print("E,");
    Serial.println(seq);
  }
}

void process_serial_input()
{
  // Non-blocking serial input processing
  // Reads available bytes and buffers them until newline is received

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
      // Buffer overflow protection: discard the partial line and surface a
      // latched fault through telemetry instead of printing ad hoc text.
      serial_buffer_index = 0;
      latch_fault_for_ms(1000);
    }
  }
}

// ==================== MAIN CONTROL LOOP ====================

void loop()
{
  unsigned long current_time = millis();
  foc_cycle_count++;

  if (fault_latched_until_ms != 0 && (long)(current_time - fault_latched_until_ms) >= 0)
    clear_fault_if_not_latched(current_time);

  // ==================== SERIAL INPUT ====================
  // Non-blocking read of joint position commands from host
  process_serial_input();

  // ==================== FOC & MOTOR CONTROL ====================
  runFOC_M0();

  // Calculate all enabled torque effects and apply to motor
  dial0.calculate_and_apply_composite_torque(USE_CURRENT_CONTROL);

  // ==================== FOC RATE MEASUREMENT ====================
  // Measure FOC rate over independent measurement window (decoupled from telemetry interval)
  if (current_time - last_foc_rate_time >= FOC_RATE_MEASURE_INTERVAL)
  {
    last_foc_rate_hz = (float)foc_cycle_count * 1000.0 / (current_time - last_foc_rate_time);
    foc_cycle_count = 0;
    last_foc_rate_time = current_time;
  }

  // ==================== TELEMETRY TRANSMISSION ====================
  // Transmit at configured telemetry interval: T,dial_id,seq,ang,spd,tor,foc_rate,status_bits\n
  if (current_time - last_telemetry_time >= telemetry_interval_ms)
  {
    long angle_decideg = rad_to_decideg(dial0.last_angle);

    // Clamp to ±MAX_ANGLE_DECIDEG
    if (angle_decideg > MAX_ANGLE_DECIDEG)
      angle_decideg = MAX_ANGLE_DECIDEG;
    else if (angle_decideg < -MAX_ANGLE_DECIDEG)
      angle_decideg = -MAX_ANGLE_DECIDEG;

    // Torque: amps -> milliamps
    float torque_ma_float = dial0.last_torque * 1000.0f;
    long torque_ma = (long)(torque_ma_float > 0 ? torque_ma_float + 0.5f : torque_ma_float - 0.5f);

    // Clamp torque to ±10000 mA
    if (torque_ma > 10000)
      torque_ma = 10000;
    else if (torque_ma < -10000)
      torque_ma = -10000;

    // Speed: rad/s -> decidegrees/s (0.1 deg/s per unit)
    float speed_decideg_float = dial0.last_speed * 1800.0f / 3.1415926f;
    long speed_decideg = (long)(speed_decideg_float > 0 ? speed_decideg_float + 0.5f : speed_decideg_float - 0.5f);

    // Compute FOC rate (int Hz) and clamp to allowed range
    long foc_rate_hz = (long)(last_foc_rate_hz + 0.5f);
    if (foc_rate_hz < 0)
      foc_rate_hz = 0;
    if (foc_rate_hz > 2000)
      foc_rate_hz = 2000;

    uint16_t status_bits = build_status_bits();

    Serial.print("T,");
    Serial.print(dial_id);
    Serial.print(",");
    Serial.print(last_processed_seq);
    Serial.print(",");
    Serial.print(angle_decideg);
    Serial.print(",");
    Serial.print(speed_decideg);
    Serial.print(",");
    Serial.print(torque_ma);
    Serial.print(",");
    Serial.print(foc_rate_hz);
    Serial.print(",");
    Serial.print(status_bits);
    Serial.println();

    last_telemetry_time = current_time;
  }
}
