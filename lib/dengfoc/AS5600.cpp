#include "AS5600.h"
#include "Wire.h"
#include <Arduino.h>

#define _2PI 6.28318530718f

// AS5600 device address and register map
static constexpr uint8_t AS5600_ADDR = 0x36;
static constexpr uint8_t AS5600_STATUS_REG = 0x0B;
static constexpr uint8_t AS5600_AGC_REG = 0x1A;

// 12-bit AS5600 raw-angle decode constants
// Angle is reported as MSB[3:0] + LSB[7:0].
static constexpr float CPR = 4096.0f;            // 2^12 counts per mechanical turn
static constexpr uint8_t LSB_MASK = 0xFF;        // Lower 8 bits of the raw angle
static constexpr uint8_t MSB_MASK = 0x0F;        // Upper 4 bits of the raw angle
static constexpr int LSB_USED = 8;               // Bit shift for the upper nibble
static constexpr float CPR_INV_2PI = _2PI / CPR; // Counts to radians scale factor

// Read len bytes from a register address in one I2C transaction.
static bool read_as5600_registers(TwoWire *wire, uint8_t reg, uint8_t *buffer, uint8_t len)
{
    wire->beginTransmission(AS5600_ADDR);
    wire->write(reg);
    if (wire->endTransmission(false) != 0)
        return false;

    if (wire->requestFrom(AS5600_ADDR, len) != len)
        return false;

    for (uint8_t i = 0; i < len; ++i)
        buffer[i] = wire->read();

    return true;
}

// Sample status + raw angle + AGC and convert angle into radians.
static bool sample_sensor_angle_status_agc(TwoWire *wire, float *angle_out, uint8_t *status_out, uint8_t *agc_out)
{
    // Read status + raw angle in one burst (status at 0x0B, angle at 0x0C/0x0D).
    uint8_t status_raw[3] = {0};
    if (!read_as5600_registers(wire, AS5600_STATUS_REG, status_raw, 3))
        return false;

    // Read AGC in the same cycle; this has been more robust on ESP32 in bench tests.
    uint8_t agc = 0;
    if (!read_as5600_registers(wire, AS5600_AGC_REG, &agc, 1))
        return false;

    // Decode 12-bit raw angle: [MSB low nibble | LSB].
    uint8_t msb = status_raw[1];
    uint8_t lsb = status_raw[2];
    uint16_t raw_angle_counts = (lsb & LSB_MASK) + ((msb & MSB_MASK) << LSB_USED);

    *status_out = status_raw[0];
    *agc_out = agc;
    *angle_out = raw_angle_counts * CPR_INV_2PI;
    return true;
}

// Construct sensor wrapper for a given motor index.
Sensor_AS5600::Sensor_AS5600(int Mot_Num)
{
    _Mot_Num = Mot_Num;
}

// Initialize I2C binding and seed cached state for angle and velocity calculations.
void Sensor_AS5600::Sensor_init(TwoWire *_wire)
{
    wire = _wire;
    // Caller must configure and begin the I2C bus before calling Sensor_init.
    // Do not call wire->begin() here to avoid resetting bus settings.

    auto sample_sensor_cache = [this]() {
        // On failure, keep the last valid angle but mark diagnostics invalid.
        if (sample_sensor_angle_status_agc(wire, &last_valid_angle, &last_status, &last_agc))
            last_diag_valid = true;
        else
            last_diag_valid = false;
    };

    // Prime cache and historical timestamps used by unwrap/velocity logic.
    delay(500);
    sample_sensor_cache();
    delayMicroseconds(1);
    sample_sensor_cache();
    vel_angle_prev = getSensorAngle();
    vel_angle_prev_ts = micros();

    delay(1);
    sample_sensor_cache();
    delayMicroseconds(1);
    sample_sensor_cache();
    angle_prev = getSensorAngle();
    angle_prev_ts = micros();
}

// Perform one periodic sample and update wrap-aware angle tracking state.
void Sensor_AS5600::Sensor_update()
{
    // On read failure, keep last_valid_angle and continue with prior angle cache.
    if (sample_sensor_angle_status_agc(wire, &last_valid_angle, &last_status, &last_agc))
        last_diag_valid = true;
    else
        last_diag_valid = false;

    float val = getSensorAngle();
    angle_prev_ts = micros();
    float d_angle = val - angle_prev;

    // Detect wrap-around and track full rotations.
    if (fabsf(d_angle) > (0.8f * _2PI))
        full_rotations += (d_angle > 0) ? -1 : 1;

    angle_prev = val;
}

// Return the latest sampled sensor angle (radians, single-turn [0, 2PI)).
float Sensor_AS5600::getSensorAngle()
{
    return last_valid_angle;
}

// Return cached mechanical angle used by FOC commutation.
float Sensor_AS5600::getMechanicalAngle()
{
    return angle_prev;
}

// Return continuous angle including full-turn wrap tracking.
double Sensor_AS5600::getAngle()
{
    return (double)full_rotations * _2PI + angle_prev;
}

// Return wrap-aware angular velocity from cached angle/time state.
float Sensor_AS5600::getVelocity()
{
    // Compute elapsed sample time.
    float Ts = (float)(uint32_t)(angle_prev_ts - vel_angle_prev_ts) * 1e-6f;
    // Guard against timestamp underflow or near-zero dt.
    if (Ts <= 0)
        Ts = 1e-3f;
    // Compute angular velocity.
    float vel = ((float)(full_rotations - vel_full_rotations) * _2PI + (angle_prev - vel_angle_prev)) / Ts;
    // Save state for the next velocity calculation.
    vel_angle_prev = angle_prev;
    vel_full_rotations = full_rotations;
    vel_angle_prev_ts = angle_prev_ts;
    return vel;
}

// Return the most recently sampled AS5600 status register.
uint8_t Sensor_AS5600::getLastStatus() const
{
    return last_status;
}

// Return the most recently sampled AS5600 AGC register.
uint8_t Sensor_AS5600::getLastAGC() const
{
    return last_agc;
}

// Return whether the latest status/raw/AGC sample was read successfully.
bool Sensor_AS5600::hasValidDiagnostics() const
{
    return last_diag_valid;
}
