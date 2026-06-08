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
  // AS5600 tracking state
  float angle_prev = 0;           // Latest sampled sensor angle used for turns and velocity
  uint32_t angle_prev_ts = 0;     // Timestamp of the latest angle update
  float vel_angle_prev = 0;       // Angle used in the previous velocity calculation
  uint32_t vel_angle_prev_ts = 0; // Timestamp used in the previous velocity calculation
  int32_t full_rotations = 0;     // Cumulative full-rotation counter
  int32_t vel_full_rotations = 0; // Rotation count used in the previous velocity calculation
  float last_valid_angle = 0;     // Last successful sensor reading (returned on I2C error)
  uint8_t last_status = 0;        // Last successful AS5600 status register value (0x0B)
  uint8_t last_agc = 0;           // Last successful AS5600 AGC register value (0x1A)
  bool last_diag_valid = false;   // True when status/raw/agc came from a successful combined read path
  TwoWire *wire;
};
