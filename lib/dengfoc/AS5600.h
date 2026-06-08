#include <Arduino.h>
#include "Wire.h"

class Sensor_AS5600
{
public:
  Sensor_AS5600(int Mot_Num);
  void Sensor_init(TwoWire *_wire = &Wire);
  void Sensor_update();
  double getAngle();
  float getVelocity();
  float getMechanicalAngle();
  float getSensorAngle();
  uint8_t getLastStatus() const;
  uint8_t getLastAGC() const;
  bool hasValidDiagnostics() const;

private:
  int _Mot_Num;
  // AS5600 变量定义
  // int sensor_direction=1;       //编码器旋转方向定义
  float angle_prev = 0;           // Latest sampled sensor angle used for turns and velocity
  uint32_t angle_prev_ts = 0;     // 上次调用 getAngle 的时间戳
  float vel_angle_prev = 0;       // 最后一次调用 getVelocity 时的角度
  uint32_t vel_angle_prev_ts = 0; // 最后速度计算时间戳
  int32_t full_rotations = 0;     // 总圈数计数
  int32_t vel_full_rotations = 0; // 用于速度计算的先前完整旋转圈数
  float last_valid_angle = 0;     // Last successful sensor reading (returned on I2C error)
  uint8_t last_status = 0;        // Last successful AS5600 status register value (0x0B)
  uint8_t last_agc = 0;           // Last successful AS5600 AGC register value (0x1A)
  bool last_diag_valid = false;   // True when status/raw/agc came from a successful combined read path
  TwoWire *wire;
};
