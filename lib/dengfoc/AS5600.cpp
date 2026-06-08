#include "AS5600.h"
#include "Wire.h"
#include <Arduino.h>

#define _2PI 6.28318530718f

static constexpr uint8_t AS5600_ADDR = 0x36;
static constexpr uint8_t AS5600_STATUS_REG = 0x0B;
static constexpr uint8_t AS5600_AGC_REG = 0x1A;

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

// AS5600 相关
// Precomputed constants for 12-bit AS5600 angle register decoding
static constexpr float CPR = 4096.0f;            // 2^12
static constexpr uint8_t LSB_MASK = 0xFF;        // (2 << 8) - 1 = 0xFF for lower 8 bits
static constexpr uint8_t MSB_MASK = 0x0F;        // (2 << 4) - 1 = 0x0F for upper 4 bits
static constexpr int LSB_USED = 8;               // 12 - 4
static constexpr float CPR_INV_2PI = _2PI / CPR; // precomputed multiplier

static bool sample_sensor_angle_status_agc(TwoWire *wire, float *angle_out, uint8_t *status_out, uint8_t *agc_out)
{
    // Read status + raw angle + AGC in one logical sampling step.
    uint8_t status_raw[3] = {0};
    if (!read_as5600_registers(wire, AS5600_STATUS_REG, status_raw, 3))
        return false;

    uint8_t agc = 0;
    if (!read_as5600_registers(wire, AS5600_AGC_REG, &agc, 1))
        return false;

    uint8_t msb = status_raw[1];
    uint8_t lsb = status_raw[2];
    uint16_t readValue = (lsb & LSB_MASK) + ((msb & MSB_MASK) << LSB_USED);

    *status_out = status_raw[0];
    *agc_out = agc;
    *angle_out = readValue * CPR_INV_2PI;
    return true;
}

float Sensor_AS5600::getSensorAngle()
{
    // Memory-only getter. Returns the latest sampled angle from Sensor_update().
    return last_valid_angle;
}

uint8_t Sensor_AS5600::getLastStatus() const
{
    return last_status;
}

uint8_t Sensor_AS5600::getLastAGC() const
{
    return last_agc;
}

bool Sensor_AS5600::hasValidDiagnostics() const
{
    return last_diag_valid;
}

// AS5600 相关

//=========角度处理相关=============
Sensor_AS5600::Sensor_AS5600(int Mot_Num)
{
    _Mot_Num = Mot_Num; // 使得 Mot_Num 可以统一在该文件调用
}
void Sensor_AS5600::Sensor_init(TwoWire *_wire)
{
    wire = _wire;
    // Note: wire->begin() must be called by the caller before Sensor_init.
    // Do NOT call wire->begin() here as it re-initializes the I2C peripheral
    // and resets timeout settings.
    auto sample_sensor_cache = [this]() {
        if (sample_sensor_angle_status_agc(wire, &last_valid_angle, &last_status, &last_agc))
            last_diag_valid = true;
        else
            last_diag_valid = false;
    };

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
void Sensor_AS5600::Sensor_update()
{
    // Perform one sensor sample at the control-loop cadence.
    // On read failure, keep last_valid_angle and continue timing/turn updates.
    if (sample_sensor_angle_status_agc(wire, &last_valid_angle, &last_status, &last_agc))
        last_diag_valid = true;
    else
        last_diag_valid = false;

    float val = getSensorAngle();
    angle_prev_ts = micros();
    float d_angle = val - angle_prev;
    // 圈数检测
    if (fabsf(d_angle) > (0.8f * _2PI))
        full_rotations += (d_angle > 0) ? -1 : 1;
    angle_prev = val;
}

float Sensor_AS5600::getMechanicalAngle()
{
    return angle_prev;
}

double Sensor_AS5600::getAngle()
{
    return (double)full_rotations * _2PI + angle_prev;
}

float Sensor_AS5600::getVelocity()
{
    // 计算采样时间
    float Ts = (float)(uint32_t)(angle_prev_ts - vel_angle_prev_ts) * 1e-6f;
    // 快速修复奇怪的情况（微溢出）
    if (Ts <= 0)
        Ts = 1e-3f;
    // 速度计算
    float vel = ((float)(full_rotations - vel_full_rotations) * _2PI + (angle_prev - vel_angle_prev)) / Ts;
    // 保存变量以待将来使用
    vel_angle_prev = angle_prev;
    vel_full_rotations = full_rotations;
    vel_angle_prev_ts = angle_prev_ts;
    return vel;
}
