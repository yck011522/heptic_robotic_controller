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
#include <stdint.h>
#include <math.h>

// ==================== FIRMWARE VERSION ====================
#define FW_VERSION "0.2.0"

// ==================== MOTOR CONFIGURATION ====================
int sensor_dir = 1;       // Sensor direction, reverse this value if motor operation is abnormal
int motor_pole_pairs = 14; // Motor pole pairs

// ==================== ANGLE LIMITS ====================
#define MAX_ANGLE_TURNS 30                          // Maximum angle in full rotations (decidegrees = turns * 36000)
#define MAX_ANGLE_DECIDEG (MAX_ANGLE_TURNS * 36000) // ±720,000 decidegrees = ±20 rotations

#define USE_CURRENT_CONTROL false // Set to true to enable current control mode (requires current sensing hardware and configuration)

// ==================== TORQUE CONTROL CONFIGURATION ====================
// Configuration now in Dial.h

// ==================== MOTOR IDENTITY (NVS) ====================
Preferences nvs_prefs;
uint8_t motor_id_0 = 0; // Persistent identity for motor 0, 0 = unconfigured

// ==================== GLOBAL INSTANCES AND CONSTANTS ====================

// Current configuration (easy to modify)
DialConfig dial_config0;

// Create per-dial instances (after class definition)
Dial dial0(0, &dial_config0); // Motor 0 controller

// Serial input buffer for receiving commands from host
const int SERIAL_BUFFER_SIZE = 128;
char serial_buffer[SERIAL_BUFFER_SIZE];
int serial_buffer_index = 0;

// Protocol state
uint32_t last_processed_seq = 0; // seq from last processed C command

// Telemetry and FOC rate measurement
unsigned long telemetry_interval_ms = 20; // Telemetry reporting interval (ms) - modifiable via S command
unsigned long last_telemetry_time = 0;    // Last time telemetry was sent

// FOC Rate measurement
#define FOC_RATE_MEASURE_INTERVAL 200 // FOC rate measurement window (ms)
unsigned long last_foc_rate_time = 0; // Last time FOC rate was calculated
unsigned long foc_cycle_count = 0;    // Number of FOC cycles in current measurement window
float last_foc_rate_hz = 0.0;         // Last calculated FOC rate (Hz)

// ==================== INITIALIZATION ====================

void setup()
{
  // Initialize serial communication for debugging
  Serial.begin(230400);

  // Load persistent motor identity from NVS (read-write to create namespace on first boot)
  nvs_prefs.begin("robot", false);
  motor_id_0 = nvs_prefs.getUChar("motor_id_0", 0);
  nvs_prefs.end();

  // Initialize motor enable pin (GPIO 12)
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH); // Motor Enable, must be placed before motor calibration

  // Configure motor driver voltage and align Hall sensors
  DFOC_Vbus(12);                                     // Set driver power supply voltage to 12V
  DFOC_M0_alignSensor(motor_pole_pairs, sensor_dir); // Calibrate motor 0 sensor

  // Initialize per-dial state and timers
  dial0.begin();

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
    // Expect: C,seq,pos0[,min0,max0]
    // pos0 required, min/max optional (if not provided, bounds are not updated)
    char *f[4];
    int i = 0;
    while (i < 4 && (f[i] = strtok(NULL, delim)) != NULL)
      i++;

    if (i >= 1)
    {
      long pos0 = atol(f[0]);

      // Convert decidegrees -> radians: rad = decideg * pi / 1800.0
      float pos0_rad = (float)pos0 * 3.1415926f / 1800.0f;

      dial_config0.tracking_position = pos0_rad;

      if (i >= 3)
      {
        long min0 = atol(f[1]);
        long max0 = atol(f[2]);
        dial_config0.bounds_min_angle = (float)min0 * 3.1415926f / 1800.0f;
        dial_config0.bounds_max_angle = (float)max0 * 3.1415926f / 1800.0f;
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
      else if (strcmp(param, "tracking_kd_0") == 0)
        dial_config0.tracking_kd = fval;
      else if (strcmp(param, "detent_kp_0") == 0)
        dial_config0.detent_kp = fval;
      else if (strcmp(param, "bounds_kp_0") == 0)
        dial_config0.bounds_kp = fval;
      else if (strcmp(param, "detent_distance_0") == 0)
        dial_config0.detent_distance = fval * 3.1415926f / 1800.0f; // assume value in decideg -> convert to rad
      else if (strcmp(param, "vibration_amplitude_0") == 0)
        dial_config0.vibration_amplitude = fval;
      else if (strcmp(param, "oob_kick_amplitude_0") == 0)
        dial_config0.oob_kick_amplitude = fval;
      // Max torque limits
      else if (strcmp(param, "tracking_max_torque_0") == 0)
        dial_config0.tracking_max_torque = fval;
      else if (strcmp(param, "bounds_max_torque_0") == 0)
        dial_config0.bounds_max_torque = fval;
      else if (strcmp(param, "detent_max_torque_0") == 0)
        dial_config0.detent_max_torque = fval;
      // Timing intervals (raw ms, no 1000x scaling)
      else if (strcmp(param, "vibration_pulse_interval_ms_0") == 0)
        dial_config0.vibration_pulse_interval_ms = (unsigned long)v;
      else if (strcmp(param, "oob_kick_pulse_interval_ms_0") == 0)
        dial_config0.oob_kick_pulse_interval_ms = (unsigned long)v;
      // Mode enable/disable flags (value: 0 = disable, non-zero = enable)
      else if (strcmp(param, "enable_tracking_0") == 0)
        dial_config0.enable_tracking = (v != 0);
      else if (strcmp(param, "enable_detent_0") == 0)
        dial_config0.enable_detent = (v != 0);
      else if (strcmp(param, "enable_bounds_restoration_0") == 0)
        dial_config0.enable_bounds_restoration = (v != 0);
      else if (strcmp(param, "enable_oob_kick_0") == 0)
        dial_config0.enable_oob_kick = (v != 0);
      else if (strcmp(param, "enable_vibration_0") == 0)
        dial_config0.enable_vibration = (v != 0);
      else if (strcmp(param, "telemetry_interval") == 0)
        telemetry_interval_ms = (unsigned long)v; // value in ms (no 1000x scaling)
      // Unknown parameters are ignored

      // Acknowledge: S,seq\n
      Serial.print("S,");
      Serial.println(seq);
    }
  }
  else if (cmd == 'I')
  {
    // Identity command, host send: I,seq,<id0>\n to set, I,seq\n to query
    // Respond with "I,seq,<motor_id_0>\n"
    char *id0str = strtok(NULL, delim);
    if (id0str)
    {
      motor_id_0 = (uint8_t)atoi(id0str);
      nvs_prefs.begin("robot", false); // read-write
      nvs_prefs.putUChar("motor_id_0", motor_id_0);
      nvs_prefs.end();
    }
    // Respond with current motor identity
    Serial.print("I,");
    Serial.print(seq);
    Serial.print(",");
    Serial.println(motor_id_0);
  }
  else if (cmd == 'V')
  {
    // Version query: respond with "V,seq,FW_VERSION\n"
    Serial.print("V,");
    Serial.print(seq);
    Serial.print(",");
    Serial.println(FW_VERSION);
  }
  else if (cmd == 'E')
  {
    // Echo test: host sends "E,seq\n", respond with "E,seq\n"
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
  foc_cycle_count++;

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

  // ==================== TELEMETRY TRANSMISSION (DengFOC V4 protocol) ====================
  // Transmit at configured telemetry interval: T,id,seq,ang0,spd0,tor0,foc_rate\n
  if (current_time - last_telemetry_time >= telemetry_interval_ms)
  {
    // Angle: radians -> decidegrees (0.1 deg per unit)
    float ang0_deg = dial0.last_angle * 180.0f / 3.1415926f;
    float ang0_decideg_f = ang0_deg * 10.0f;

    long ang0_decideg = (long)(ang0_decideg_f > 0 ? ang0_decideg_f + 0.5f : ang0_decideg_f - 0.5f);

    // Clamp to ±MAX_ANGLE_DECIDEG
    if (ang0_decideg > MAX_ANGLE_DECIDEG)
      ang0_decideg = MAX_ANGLE_DECIDEG;
    else if (ang0_decideg < -MAX_ANGLE_DECIDEG)
      ang0_decideg = -MAX_ANGLE_DECIDEG;

    // Torque: amps -> milliamps
    float tor0_ma_f = dial0.last_torque * 1000.0f;
    long tor0_ma = (long)(tor0_ma_f > 0 ? tor0_ma_f + 0.5f : tor0_ma_f - 0.5f);

    // Clamp torque to ±10000 mA
    if (tor0_ma > 10000)
      tor0_ma = 10000;
    else if (tor0_ma < -10000)
      tor0_ma = -10000;

    // Speed: rad/s -> decidegrees/s (0.1 deg/s per unit)
    float spd0_decideg_f = dial0.last_speed * 1800.0f / 3.1415926f;
    long spd0_decideg = (long)(spd0_decideg_f > 0 ? spd0_decideg_f + 0.5f : spd0_decideg_f - 0.5f);

    // Compute FOC rate (int Hz) and clamp to allowed range
    long foc_rate_hz = (long)(last_foc_rate_hz + 0.5f);
    if (foc_rate_hz < 0)
      foc_rate_hz = 0;
    if (foc_rate_hz > 2000)
      foc_rate_hz = 2000;

    // Emit: T,motor_id_0,seq,ang0,spd0,tor0,foc_rate\n
    Serial.print("T,");
    Serial.print(motor_id_0);
    Serial.print(",");
    Serial.print(last_processed_seq);
    Serial.print(",");
    Serial.print(ang0_decideg);
    Serial.print(",");
    Serial.print(spd0_decideg);
    Serial.print(",");
    Serial.print(tor0_ma);
    Serial.print(",");
    Serial.print(foc_rate_hz);
    Serial.println();

    last_telemetry_time = current_time;
  }
}
