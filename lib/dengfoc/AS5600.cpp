#include "AS5600.h"
#include "Wire.h"
#include <Arduino.h>

#define _2PI 6.28318530718f

// AS5600 相关
// Precomputed constants for 12-bit AS5600 angle register decoding
static constexpr float CPR = 4096.0f;            // 2^12
static constexpr uint8_t LSB_MASK = 0xFF;        // (2 << 8) - 1 = 0xFF for lower 8 bits
static constexpr uint8_t MSB_MASK = 0x0F;        // (2 << 4) - 1 = 0x0F for upper 4 bits
static constexpr int LSB_USED = 8;               // 12 - 4
static constexpr float CPR_INV_2PI = _2PI / CPR; // precomputed multiplier

float Sensor_AS5600::getSensorAngle()
{
    // Use STOP between write and read (endTransmission(true)) so the ESP32 Wire
    // library dispatches two separate I2C transactions instead of the combined
    // i2cWriteReadNonStop() path, which fails when both I2C peripherals are active.
    wire->beginTransmission(0x36);
    wire->write((uint8_t)0x0C);
    uint8_t err = wire->endTransmission(true);
    if (err != 0)
    {
        i2c_error_count++;
        return last_valid_raw_angle;
    }

    uint8_t count = wire->requestFrom((uint8_t)0x36, (uint8_t)2);
    if (count < 2)
    {
        while (wire->available())
            wire->read();
        i2c_error_count++;
        return last_valid_raw_angle;
    }

    uint8_t msb = wire->read();
    uint8_t lsb = wire->read();

    uint16_t readValue = (lsb & LSB_MASK) + ((msb & MSB_MASK) << LSB_USED);
    last_valid_raw_angle = readValue * CPR_INV_2PI;
    return last_valid_raw_angle;
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
    delay(500);
    getSensorAngle();
    delayMicroseconds(1);
    vel_angle_prev = getSensorAngle();
    vel_angle_prev_ts = micros();
    delay(1);
    getSensorAngle();
    delayMicroseconds(1);
    angle_prev = getSensorAngle();
    angle_prev_ts = micros();
}
void Sensor_AS5600::Sensor_update()
{
    float val = getSensorAngle();
    angle_prev_ts = micros();
    float d_angle = val - angle_prev;
    // 圈数检测
    if (abs(d_angle) > (0.8f * _2PI))
        full_rotations += (d_angle > 0) ? -1 : 1;
    // Avoid overflow
    if (abs(full_rotations) > 100)
    {
        full_rotations = 0;
    }
    angle_prev = val;
}

float Sensor_AS5600::getMechanicalAngle()
{
    return angle_prev;
}

float Sensor_AS5600::getAngle()
{
    return (float)full_rotations * _2PI + angle_prev;
}

float Sensor_AS5600::getVelocity()
{
    // 计算采样时间
    float Ts = (angle_prev_ts - vel_angle_prev_ts) * 1e-6;
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
