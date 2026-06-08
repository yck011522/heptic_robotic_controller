#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// Firmware ID for host-side version checks.
#define FW_VERSION "debug-encoder-0.3.0"

// Hardware mapping (matches current production firmware wiring).
static const int PIN_PWM_A = 32;
static const int PIN_PWM_B = 33;
static const int PIN_PWM_C = 25;
static const int PIN_ENABLE = 12;
static const int I2C_SDA_PIN = 19;
static const int I2C_SCL_PIN = 18;

static const uint8_t AS5600_ADDR = 0x36;
static const uint8_t AS5600_STATUS_REG = 0x0B;
static const uint8_t AS5600_ANGLE_REG = 0x0C;
static const uint8_t AS5600_AGC_REG = 0x1A;
static const uint8_t AS5600_MAGNITUDE_REG = 0x1B;

static const float VBUS_VOLTS = 12.0f;
static const float OPEN_LOOP_UQ_VOLTS = 2.0f;
static const int MOTOR_POLE_PAIRS = 14;
static const int ELECTRICAL_DIRECTION = -1;

static const uint32_t LOOP_HZ = 800;
static const uint32_t LOOP_PERIOD_US = 1000000UL / LOOP_HZ; // 1250 us
static const uint32_t LOG_DURATION_MS = 2000;
static const uint32_t SAMPLE_COUNT = (LOOP_HZ * LOG_DURATION_MS) / 1000UL;
static const uint8_t DEFAULT_MECHANICAL_TURNS = 1;

static const size_t SERIAL_BUFFER_SIZE = 128;

struct LogSample
{
  // Time from capture start in microseconds.
  uint32_t t_us;
  // Commanded electrical angle used for open-loop commutation.
  float openloop_el_angle_rad;
  // Raw 12-bit AS5600 position register value.
  uint16_t sensor_raw;
  // 1 when I2C read failed for this sample, else 0.
  uint8_t sensor_error;
  // AS5600 status register (MD/ML/MH bits).
  uint8_t status;
  // AS5600 AGC register.
  uint8_t agc;
  // AS5600 magnitude register (0x1B/0x1C).
  uint16_t magnitude;
};

static LogSample samples[SAMPLE_COUNT];

static char serial_buffer[SERIAL_BUFFER_SIZE];
static size_t serial_buffer_index = 0;

static bool run_requested = false;
static bool running = false;
static bool log_ready = false;
static uint32_t run_seq = 0;
static uint8_t run_turns_requested = DEFAULT_MECHANICAL_TURNS;
static uint8_t run_turns_last = DEFAULT_MECHANICAL_TURNS;

static float normalize_angle(float angle)
{
  const float two_pi = 6.28318530718f;
  float a = fmodf(angle, two_pi);
  return a >= 0.0f ? a : (a + two_pi);
}

static void set_phase_pwm(float ua, float ub, float uc)
{
  // Clip phase voltages to valid bus range before duty conversion.
  ua = constrain(ua, 0.0f, VBUS_VOLTS);
  ub = constrain(ub, 0.0f, VBUS_VOLTS);
  uc = constrain(uc, 0.0f, VBUS_VOLTS);

  float duty_a = constrain(ua / VBUS_VOLTS, 0.0f, 1.0f);
  float duty_b = constrain(ub / VBUS_VOLTS, 0.0f, 1.0f);
  float duty_c = constrain(uc / VBUS_VOLTS, 0.0f, 1.0f);

  ledcWrite(0, (uint32_t)(duty_a * 255.0f));
  ledcWrite(1, (uint32_t)(duty_b * 255.0f));
  ledcWrite(2, (uint32_t)(duty_c * 255.0f));
}

static void set_openloop_torque(float uq, float angle_el)
{
  // SVPWM-style inverse Park/Clarke with fixed Uq in open-loop.
  uq = constrain(uq, -(VBUS_VOLTS * 0.5f), VBUS_VOLTS * 0.5f);
  float angle = normalize_angle(angle_el);

  float s = sinf(angle);
  float c = cosf(angle);
  float u_alpha = -uq * s;
  float u_beta = uq * c;

  float half_vbus = VBUS_VOLTS * 0.5f;
  float ua = u_alpha + half_vbus;
  float ub = (1.73205080757f * u_beta - u_alpha) * 0.5f + half_vbus;
  float uc = (-u_alpha - 1.73205080757f * u_beta) * 0.5f + half_vbus;

  set_phase_pwm(ua, ub, uc);
}

static void set_motor_enabled(bool enabled)
{
  digitalWrite(PIN_ENABLE, enabled ? HIGH : LOW);
}

static bool read_status_and_raw(uint8_t *status, uint16_t *raw)
{
  // Read status and raw angle together for coherent sampling.
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_STATUS_REG);
  if (Wire.endTransmission(false) != 0)
    return false;

  if (Wire.requestFrom((int)AS5600_ADDR, 3) != 3)
    return false;

  *status = Wire.read();
  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  *raw = (uint16_t)(((msb & 0x0F) << 8) | lsb);
  return true;
}

static bool read_register8(uint8_t reg, uint8_t *value)
{
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0)
    return false;

  if (Wire.requestFrom((int)AS5600_ADDR, 1) != 1)
    return false;

  *value = Wire.read();
  return true;
}

static bool read_register16(uint8_t reg, uint16_t *value)
{
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0)
    return false;

  if (Wire.requestFrom((int)AS5600_ADDR, 2) != 2)
    return false;

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  *value = (uint16_t)((msb << 8) | lsb);
  return true;
}

static bool read_encoder_diagnostics(uint16_t *raw, uint8_t *status, uint8_t *agc, uint16_t *magnitude)
{
  bool ok = true;
  ok &= read_status_and_raw(status, raw);
  ok &= read_register8(AS5600_AGC_REG, agc);
  ok &= read_register16(AS5600_MAGNITUDE_REG, magnitude);
  return ok;
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

static void print_help()
{
  Serial.println("H,commands: V,<seq> | E,<seq> | R,<seq>[,<turns>] | S,<seq>");
  Serial.println("H,turns: 0=no movement+motor disabled, 1=one turn, 2=two turns in 2s");
  Serial.println("H,log stream: LOG_BEGIN/LOG_HEADER/LOG_DATA/LOG_END");
}

static void dump_log()
{
  // Machine-parseable framing for host scripts.
  Serial.print("LOG_BEGIN,");
  Serial.println(SAMPLE_COUNT);
  Serial.println("LOG_HEADER,t_us,openloop_el_angle_rad,sensor_raw,error,status,agc,magnitude");

  for (uint32_t i = 0; i < SAMPLE_COUNT; ++i)
  {
    Serial.print("LOG_DATA,");
    Serial.print(samples[i].t_us);
    Serial.print(',');
    Serial.print(samples[i].openloop_el_angle_rad, 6);
    Serial.print(',');
    Serial.print(samples[i].sensor_raw);
    Serial.print(',');
    Serial.print(samples[i].sensor_error);
    Serial.print(',');
    Serial.print(samples[i].status);
    Serial.print(',');
    Serial.print(samples[i].agc);
    Serial.print(',');
    Serial.println(samples[i].magnitude);
  }

  Serial.print("LOG_END,");
  Serial.print(SAMPLE_COUNT);
  Serial.print(',');
  Serial.println(run_turns_last);
}

static void run_capture_once(uint8_t turns)
{
  running = true;
  log_ready = false;
  run_turns_last = turns;

  bool movement_enabled = (turns > 0);

  // One mechanical turn corresponds to pole_pairs electrical turns.
  const float total_electrical_span = 6.28318530718f * (float)turns * (float)MOTOR_POLE_PAIRS;
  const float electrical_step = (SAMPLE_COUNT > 1) ? (total_electrical_span / (float)(SAMPLE_COUNT - 1)) : 0.0f;

  set_openloop_torque(0.0f, 0.0f);
  set_motor_enabled(movement_enabled);

  float openloop_el = 0.0f;
  uint32_t start_us = micros();
  uint32_t next_tick = start_us;

  for (uint32_t i = 0; i < SAMPLE_COUNT; ++i)
  {
    // Fixed-rate scheduler: each iteration targets one 1250 us slot.
    next_tick += LOOP_PERIOD_US;

    float commanded_el = movement_enabled ? ((float)ELECTRICAL_DIRECTION * openloop_el) : 0.0f;
    if (movement_enabled)
      set_openloop_torque(OPEN_LOOP_UQ_VOLTS, commanded_el);

    uint16_t raw = 0;
    uint8_t status = 0;
    uint8_t agc = 0;
    uint16_t magnitude = 0;
    bool ok = read_encoder_diagnostics(&raw, &status, &agc, &magnitude);

    samples[i].t_us = micros() - start_us;
    samples[i].openloop_el_angle_rad = commanded_el;
    samples[i].sensor_raw = raw;
    samples[i].sensor_error = ok ? 0 : 1;
    samples[i].status = status;
    samples[i].agc = agc;
    samples[i].magnitude = magnitude;

    openloop_el += electrical_step;

    while ((int32_t)(micros() - next_tick) < 0)
    {
      // Hold fixed-rate loop timing for consistent sampling and control cadence.
    }
  }

  // Reset outputs and re-enable driver so the next command can move immediately.
  set_openloop_torque(0.0f, 0.0f);
  set_motor_enabled(true);

  running = false;
  log_ready = true;
}

static void handle_command(char *line)
{
  // Protocol: <CMD>,<seq> with minimal ack/status responses.
  char *token = strtok(line, ",");
  if (!token)
    return;

  if (strcmp(token, "H") == 0 || strcmp(token, "?") == 0)
  {
    print_help();
    return;
  }

  if (strcmp(token, "RUN") == 0)
    token = (char *)"R";

  char *seq_token = strtok(nullptr, ",");
  uint32_t seq = 0;
  if (!parse_uint32_field(seq_token, &seq))
  {
    Serial.println("ERR,bad_seq");
    return;
  }

  if (strcmp(token, "V") == 0)
  {
    Serial.print("V,");
    Serial.print(seq);
    Serial.print(',');
    Serial.println(FW_VERSION);
    return;
  }

  if (strcmp(token, "E") == 0)
  {
    Serial.print("E,");
    Serial.println(seq);
    return;
  }

  if (strcmp(token, "S") == 0)
  {
    Serial.print("S,");
    Serial.print(seq);
    Serial.print(',');
    Serial.print(running ? "RUNNING" : "IDLE");
    Serial.print(',');
    Serial.print(log_ready ? "LOG_READY" : "NO_LOG");
    Serial.print(',');
    Serial.println(run_turns_last);
    return;
  }

  if (strcmp(token, "R") == 0)
  {
    if (running || run_requested)
    {
      Serial.print("R,");
      Serial.print(seq);
      Serial.println(",BUSY");
      return;
    }

    uint8_t turns = DEFAULT_MECHANICAL_TURNS;
    char *turns_token = strtok(nullptr, ",");
    if (turns_token && turns_token[0] != '\0')
    {
      uint32_t parsed_turns = 0;
      if (!parse_uint32_field(turns_token, &parsed_turns) || parsed_turns > 2)
      {
        Serial.print("R,");
        Serial.print(seq);
        Serial.println(",ERR,bad_turns");
        return;
      }
      turns = (uint8_t)parsed_turns;
    }

    run_requested = true;
    run_seq = seq;
    run_turns_requested = turns;
    Serial.print("R,");
    Serial.print(seq);
    Serial.print(",START,");
    Serial.println(run_turns_requested);
    return;
  }

  Serial.println("ERR,unknown_cmd");
}

static void process_serial()
{
  // Simple line-based parser with CRLF tolerance.
  while (Serial.available() > 0)
  {
    char c = (char)Serial.read();

    if (c == '\r')
      continue;

    if (c == '\n')
    {
      serial_buffer[serial_buffer_index] = '\0';
      if (serial_buffer_index > 0)
        handle_command(serial_buffer);
      serial_buffer_index = 0;
      continue;
    }

    if (serial_buffer_index < (SERIAL_BUFFER_SIZE - 1))
      serial_buffer[serial_buffer_index++] = c;
    else
      serial_buffer_index = 0;
  }
}

void setup()
{
  Serial.begin(230400);
  delay(200);

  // Keep driver enabled to allow open-loop actuation during capture.
  pinMode(PIN_ENABLE, OUTPUT);
  set_motor_enabled(true);

  pinMode(PIN_PWM_A, OUTPUT);
  pinMode(PIN_PWM_B, OUTPUT);
  pinMode(PIN_PWM_C, OUTPUT);

  ledcSetup(0, 30000, 8);
  ledcSetup(1, 30000, 8);
  ledcSetup(2, 30000, 8);
  ledcAttachPin(PIN_PWM_A, 0);
  ledcAttachPin(PIN_PWM_B, 1);
  ledcAttachPin(PIN_PWM_C, 2);

  // Fast I2C to keep encoder transactions inside the 800 Hz budget.
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000UL);
  Wire.setTimeOut(2);

  set_openloop_torque(0.0f, 0.0f);

  Serial.println("BOOT,debug_encoder_ready");
  print_help();
}

void loop()
{
  process_serial();

  if (run_requested && !running)
  {
    // Capture is intentionally blocking so sample cadence remains deterministic.
    run_requested = false;

    run_capture_once(run_turns_requested);

    Serial.print("R,");
    Serial.print(run_seq);
    Serial.print(",DONE,");
    Serial.println(run_turns_last);

    dump_log();
  }
}
