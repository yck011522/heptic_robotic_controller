/**
 * @file DengFOC.cpp
 * @brief Dual Motor Field-Oriented Control (FOC) Driver
 *
 * This library implements Field-Oriented Control for two BLDC motors with:
 * - AS5600 magnetic angle encoders for commutation feedback
 * - Inline current sensing for torque control
 * - Three-phase PWM generation
 * - Cascaded PID control loops (velocity, angle, current)
 * - Low-pass filtering for sensor noise reduction
 *
 * Architecture: Dual independent motor control with synchronized sensor updates
 * Target: ESP32 microcontroller with dual motor FOC capability
 */

#include <Arduino.h>
#include "AS5600.h"
#include "lowpass_filter.h"
#include "pid.h"
#include "InlineCurrent.h"
#include "DengFOC.h"

// ==================== UTILITY MACROS ====================
/// Constrains value to range [low, high]
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

// ==================== POWER SUPPLY ====================
/// System bus voltage (set via DFOC_Vbus())
float voltage_power_supply;

// ==================== MOTOR 0 PARAMETERS ====================
/// Zero electrical angle offset for sensor alignment (radians)
float M0_zero_electric_angle = 0;
/// Pole pairs and rotation direction
int M0_PP = 1;  ///< Number of magnetic pole pairs
int M0_DIR = 1; ///< Rotation direction: 1 or -1
/// PWM output pins (three-phase)
int M0_pwmA = 32; ///< Phase A PWM pin (GPIO 32)
int M0_pwmB = 33; ///< Phase B PWM pin (GPIO 33)
int M0_pwmC = 25; ///< Phase C PWM pin (GPIO 25)

// ==================== MOTOR 1 PARAMETERS ====================
/// Zero electrical angle offset for sensor alignment (radians)
float M1_zero_electric_angle = 0;
/// Pole pairs and rotation direction
int M1_PP = 1;  ///< Number of magnetic pole pairs
int M1_DIR = 1; ///< Rotation direction: 1 or -1
/// PWM output pins (three-phase)
int M1_pwmA = 26; ///< Phase A PWM pin (GPIO 26)
int M1_pwmB = 27; ///< Phase B PWM pin (GPIO 27)
int M1_pwmC = 14; ///< Phase C PWM pin (GPIO 14)

// ==================== CONTROL ENABLE ====================
/// Master enable pin for motor driver (GPIO 12)
int enable = 12;

// ==================== SENSOR FILTERS ====================
/// Low-pass filters for velocity (cutoff ~1 kHz at 100 Hz sample rate)
LowPassFilter M0_Vel_Flt = LowPassFilter(0.01); ///< Motor 0 velocity filter
LowPassFilter M1_Vel_Flt = LowPassFilter(0.01); ///< Motor 1 velocity filter
/// Low-pass filters for current (cutoff ~500 Hz at 100 Hz sample rate)
LowPassFilter M0_Curr_Flt = LowPassFilter(0.05); ///< Motor 0 current filter
LowPassFilter M1_Curr_Flt = LowPassFilter(0.05); ///< Motor 1 current filter

// ==================== PID CONTROLLERS ====================
/**
 * CONTROL ARCHITECTURE:
 * Three selectable control modes using cascaded/independent PID loops:
 *
 * 1. ANGLE LOOP (Primary setpoint):
 *    Input: angle error (degrees)  → Output: velocity command (rad/s)
 *    limit=100 rad/s max velocity command
 *    Used in: set_Velocity_Angle(), set_Force_Angle()
 *
 * 2. VELOCITY LOOP (Primary feedback):
 *    Input: velocity error (rad/s) → Output: voltage command (V)
 *    limit=Vbus/2 max motor voltage
 *    Used in: set_Velocity_Angle(), setVelocity()
 *    Alternative standalone use: setVelocity()
 *
 * 3. CURRENT LOOP (Torque feedback):
 *    Input: current error (A) → Output: voltage command (V)
 *    K_p=1.2 V/A (voltage per amp of error)
 *    limit=12.6V max motor voltage
 *    Used in: setTorque_current()
 *    Implements voltage-source current control for precise torque regulation
 *
 * CONTROL SIGNAL FLOW (examples):
 *  - set_Velocity_Angle(target):  angle_error → ANGLE_PID → vel_cmd → VELOCITY_PID → voltage → motor
 *  - setVelocity(target):          vel_error → VELOCITY_PID → voltage → motor
 *  - set_Force_Angle(target):      angle_error → ANGLE_PID → voltage → motor
 *  - setTorque_current(target):    current_error → CURRENT_PID → voltage → motor
 */

// Motor 0: Velocity loop, Angle loop, Current loop
PIDController M0_vel_loop(2, 0, 0, 100000, voltage_power_supply / 2);
PIDController M0_angle_loop(2, 0, 0, 100000, 100);
PIDController M0_current_loop(1.2, 0, 0, 100000, 12.6);

// Motor 1: Velocity loop, Angle loop, Current loop
PIDController M1_vel_loop(2, 0, 0, 100000, voltage_power_supply / 2);
PIDController M1_angle_loop(2, 0, 0, 100000, 100);
PIDController M1_current_loop(1.2, 0, 0, 100000, 12.6);

// ==================== SENSORS ====================
/// AS5600 I2C magnetic encoders for commutation feedback
Sensor_AS5600 S0 = Sensor_AS5600(0); ///< Motor 0 angle encoder
Sensor_AS5600 S1 = Sensor_AS5600(1); ///< Motor 1 angle encoder
/// I2C buses (different ports for simultaneous dual motor operation)
TwoWire S0_I2C = TwoWire(0); ///< I2C bus 0 (GPIO 19=SDA, 18=SCL)
TwoWire S1_I2C = TwoWire(1); ///< I2C bus 1 (GPIO 23=SDA, 5=SCL)

// ==================== CLARK/PARK TRANSFORM CONSTANTS ====================
/// Inverse square root of 3 for abc-to-dq transformation
#define _1_SQRT3 0.57735026919f
/// 2/sqrt(3) for abc-to-dq transformation
#define _2_SQRT3 1.15470053838f
/// sqrt(3) precomputed constant
#define _SQRT3 1.73205080757f
/// Pi constant
#define _PI 3.14159265359f
/// 2*Pi constant
#define _2PI_F 6.28318530718f
/// 3*Pi/2 constant
#define _3PI_2 4.71238898038f

// ==================== CURRENT SENSORS ====================
/// Inline current sensing for direct torque feedback
CurrSense CS_M0 = CurrSense(0); ///< Motor 0 phase current sensor
CurrSense CS_M1 = CurrSense(1); ///< Motor 1 phase current sensor

// ==================== PID CONFIGURATION FUNCTIONS ====================
// Motor 0 PID setters
/// @brief Configure Motor 0 velocity loop PID parameters
/// Input: velocity error (rad/s), Output: voltage command (V) to motor
/// @param P Proportional gain (V per rad/s error)
/// @param I Integral gain
/// @param D Derivative gain
/// @param ramp Output ramp rate (V/s) - limits voltage acceleration
/// @param limit Maximum output voltage (V) clamping
void DFOC_M0_SET_VEL_PID(float P, float I, float D, float ramp, float limit)
{
  M0_vel_loop.P = P;
  M0_vel_loop.I = I;
  M0_vel_loop.D = D;
  M0_vel_loop.output_ramp = ramp;
  M0_vel_loop.limit = limit;
}

/// @brief Configure Motor 0 angle/position loop PID parameters
/// Input: angle error (degrees), Output: velocity command (rad/s) to velocity loop
/// @param P Proportional gain (rad/s per degree error)
/// @param I Integral gain
/// @param D Derivative gain
/// @param ramp Output ramp rate (rad/s²) - limits velocity acceleration
/// @param limit Maximum velocity command (rad/s) clamping
void DFOC_M0_SET_ANGLE_PID(float P, float I, float D, float ramp, float limit)
{
  M0_angle_loop.P = P;
  M0_angle_loop.I = I;
  M0_angle_loop.D = D;
  M0_angle_loop.output_ramp = ramp;
  M0_angle_loop.limit = limit;
}

/// @brief Configure Motor 0 current/torque loop PID parameters
/// Input: current error (A), Output: voltage command (V) to motor (voltage-source current control)
/// @param P Proportional gain (V per amp of current error)
/// @param I Integral gain
/// @param D Derivative gain
/// @param ramp Output ramp rate (V/s) - limits voltage acceleration
void DFOC_M0_SET_CURRENT_PID(float P, float I, float D, float ramp)
{
  M0_current_loop.P = P;
  M0_current_loop.I = I;
  M0_current_loop.D = D;
  M0_current_loop.output_ramp = ramp;
}

// Motor 1 PID setters
/// @brief Configure Motor 1 velocity loop PID parameters
/// Input: velocity error (rad/s), Output: voltage command (V) to motor
/// @param P Proportional gain (V per rad/s error)
/// @param I Integral gain
/// @param D Derivative gain
/// @param ramp Output ramp rate (V/s) - limits voltage acceleration
/// @param limit Maximum output voltage (V) clamping
void DFOC_M1_SET_VEL_PID(float P, float I, float D, float ramp, float limit)
{
  M1_vel_loop.P = P;
  M1_vel_loop.I = I;
  M1_vel_loop.D = D;
  M1_vel_loop.output_ramp = ramp;
  M1_vel_loop.limit = limit;
}

/// @brief Configure Motor 1 angle/position loop PID parameters
/// Input: angle error (degrees), Output: velocity command (rad/s) to velocity loop
/// @param P Proportional gain (rad/s per degree error)
/// @param I Integral gain
/// @param D Derivative gain
/// @param ramp Output ramp rate (rad/s²) - limits velocity acceleration
/// @param limit Maximum velocity command (rad/s) clamping
void DFOC_M1_SET_ANGLE_PID(float P, float I, float D, float ramp, float limit)
{
  M1_angle_loop.P = P;
  M1_angle_loop.I = I;
  M1_angle_loop.D = D;
  M1_angle_loop.output_ramp = ramp;
  M1_angle_loop.limit = limit;
}

/// @brief Configure Motor 1 current/torque loop PID parameters
/// Input: current error (A), Output: voltage command (V) to motor (voltage-source current control)
/// @param P Proportional gain (V per amp of current error)
/// @param I Integral gain
/// @param D Derivative gain
/// @param ramp Output ramp rate (V/s) - limits voltage acceleration
void DFOC_M1_SET_CURRENT_PID(float P, float I, float D, float ramp)
{
  M1_current_loop.P = P;
  M1_current_loop.I = I;
  M1_current_loop.D = D;
  M1_current_loop.output_ramp = ramp;
}

// ==================== PID ACCESSOR FUNCTIONS ====================
/// @brief Execute Motor 0 velocity PID controller
/// Converts velocity error into motor voltage command
/// @param error Velocity error (target - actual) in rad/s
/// @return PID output voltage command (V) for motor torque control
float DFOC_M0_VEL_PID(float error)
{
  return M0_vel_loop(error);
}

/// @brief Execute Motor 0 angle/position PID controller
/// Converts angle error into velocity command for velocity loop
/// @param error Angle error (target - actual) in degrees
/// @return PID output velocity command (rad/s) for velocity loop setpoint
float DFOC_M0_ANGLE_PID(float error)
{
  return M0_angle_loop(error);
}

/// @brief Execute Motor 1 velocity PID controller
/// Converts velocity error into motor voltage command
/// @param error Velocity error (target - actual) in rad/s
/// @return PID output voltage command (V) for motor torque control
float DFOC_M1_VEL_PID(float error)
{
  return M1_vel_loop(error);
}

/// @brief Execute Motor 1 angle/position PID controller
/// Converts angle error into velocity command for velocity loop
/// @param error Angle error (target - actual) in degrees
/// @return PID output velocity command (rad/s) for velocity loop setpoint
float DFOC_M1_ANGLE_PID(float error)
{
  return M1_angle_loop(error);
}

// ==================== BASIC UTILITY FUNCTIONS ====================
/// @brief Normalize angle to range [0, 2*PI) radians
/// @param angle Input angle in radians (can be negative or > 2*PI)
/// @return Normalized angle in [0, 2*PI) range
float _normalizeAngle(float angle)
{
  // Returns an angle in radians normalized to the range [0, 2*PI)
  // Handles both positive and negative input angles
  float a = fmodf(angle, _2PI_F);
  return a >= 0 ? a : (a + _2PI_F);
}

// ==================== PWM CONTROL FUNCTIONS ====================
/// @brief Set Motor 0 three-phase PWM outputs
/// @param Ua Phase A voltage (0 to Vbus)
/// @param Ub Phase B voltage (0 to Vbus)
/// @param Uc Phase C voltage (0 to Vbus)
/// @note Internally clips values to valid PWM range [0, Vbus] and converts to 8-bit duty cycle
void M0_setPwm(float Ua, float Ub, float Uc)
{
  Ua = _constrain(Ua, 0.0f, voltage_power_supply);
  Ub = _constrain(Ub, 0.0f, voltage_power_supply);
  Uc = _constrain(Uc, 0.0f, voltage_power_supply);

  float dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
  float dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
  float dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

  ledcWrite(0, dc_a * 255);
  ledcWrite(1, dc_b * 255);
  ledcWrite(2, dc_c * 255);
}

/// @brief Set Motor 1 three-phase PWM outputs
/// @param Ua Phase A voltage (0 to Vbus)
/// @param Ub Phase B voltage (0 to Vbus)
/// @param Uc Phase C voltage (0 to Vbus)
/// @note Internally clips values to valid PWM range [0, Vbus] and converts to 8-bit duty cycle
void M1_setPwm(float Ua, float Ub, float Uc)
{
  Ua = _constrain(Ua, 0.0f, voltage_power_supply);
  Ub = _constrain(Ub, 0.0f, voltage_power_supply);
  Uc = _constrain(Uc, 0.0f, voltage_power_supply);

  float dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
  float dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
  float dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

  ledcWrite(3, dc_a * 255);
  ledcWrite(4, dc_b * 255);
  ledcWrite(5, dc_c * 255);
}

// ==================== TORQUE CONTROL FUNCTIONS ====================
/// @brief Set Motor 0 torque using inverse Park transform
/// Converts q-axis voltage command to three-phase PWM using electrical angle
/// @param Uq q-axis voltage command (-Vbus/2 to +Vbus/2), proportional to torque
/// @param angle_el Electrical angle for FOC alignment (radians)
/// @return Constrained Uq command that was applied
/// @note Uses Park (dq -> alphabeta) and SVM transforms for commutation
float M0_setTorque(float Uq, float angle_el)
{
  Uq = _constrain(Uq, -(voltage_power_supply) / 2, (voltage_power_supply) / 2);
  angle_el = _normalizeAngle(angle_el);

  // Inverse Park: dq -> alphabeta (convert q-axis voltage to cartesian space)
  float sa = sinf(angle_el);
  float ca = cosf(angle_el);
  float Ualpha = -Uq * sa;
  float Ubeta = Uq * ca;

  // Inverse Clarke: alphabeta -> abc (convert cartesian to three-phase)
  // Adds DC offset of Vbus/2 to center PWM around mid-rail
  float half_vbus = voltage_power_supply * 0.5f;
  float Ua = Ualpha + half_vbus;
  float Ub = (_SQRT3 * Ubeta - Ualpha) * 0.5f + half_vbus;
  float Uc = (-Ualpha - _SQRT3 * Ubeta) * 0.5f + half_vbus;

  M0_setPwm(Ua, Ub, Uc);
  return Uq;
}

/// @brief Set Motor 1 torque using inverse Park transform
/// Converts q-axis voltage command to three-phase PWM using electrical angle
/// @param Uq q-axis voltage command (-Vbus/2 to +Vbus/2), proportional to torque
/// @param angle_el Electrical angle for FOC alignment (radians)
/// @return Constrained Uq command that was applied
/// @note Uses Park (dq -> alphabeta) and SVM transforms for commutation
float M1_setTorque(float Uq, float angle_el)
{
  Uq = _constrain(Uq, -(voltage_power_supply) / 2, (voltage_power_supply) / 2);
  angle_el = _normalizeAngle(angle_el);

  // Inverse Park: dq -> alphabeta (convert q-axis voltage to cartesian space)
  float sa = sinf(angle_el);
  float ca = cosf(angle_el);
  float Ualpha = -Uq * sa;
  float Ubeta = Uq * ca;

  // Inverse Clarke: alphabeta -> abc (convert cartesian to three-phase)
  // Adds DC offset of Vbus/2 to center PWM around mid-rail
  float half_vbus = voltage_power_supply * 0.5f;
  float Ua = Ualpha + half_vbus;
  float Ub = (_SQRT3 * Ubeta - Ualpha) * 0.5f + half_vbus;
  float Uc = (-Ualpha - _SQRT3 * Ubeta) * 0.5f + half_vbus;

  M1_setPwm(Ua, Ub, Uc);
  return Uq;
}

// ==================== POWER MANAGEMENT FUNCTIONS ====================
/// @brief Enable motor driver (set enable pin HIGH)
void DFOC_enable()
{
  digitalWrite(enable, HIGH);
}

/// @brief Disable motor driver (set enable pin LOW). Disables all PWM outputs
void DFOC_disable()
{
  digitalWrite(enable, LOW);
}

/// @brief Initialize FOC system with specified bus voltage
/// Initializes PWM outputs, I2C sensors, current sensors, and power supply parameter
/// @param power_supply System bus voltage (typically 12V or 24V)
/// @note Must be called before any motor control functions
/// @note Configures:
///   - Motor 0 PWM on channels 0-2 (GPIO 32, 33, 25) at 30kHz
///   - Motor 1 PWM on channels 3-5 (GPIO 26, 27, 14) at 30kHz
///   - I2C bus 0 for encoder S0 (GPIO 19=SDA, 18=SCL)
///   - I2C bus 1 for encoder S1 (GPIO 23=SDA, 5=SCL)
///   - Current sensors for both motors
void DFOC_Vbus(float power_supply)
{
  voltage_power_supply = power_supply;

  // Configure Motor 0 PWM outputs (channels 0-2)
  pinMode(M0_pwmA, OUTPUT);
  pinMode(M0_pwmB, OUTPUT);
  pinMode(M0_pwmC, OUTPUT);
  ledcAttachPin(M0_pwmA, 0);
  ledcAttachPin(M0_pwmB, 1);
  ledcAttachPin(M0_pwmC, 2);
  ledcSetup(0, 30000, 8); // 30 kHz PWM, 8-bit resolution
  ledcSetup(1, 30000, 8);
  ledcSetup(2, 30000, 8);

  // Configure Motor 1 PWM outputs (channels 3-5)
  pinMode(M1_pwmA, OUTPUT);
  pinMode(M1_pwmB, OUTPUT);
  pinMode(M1_pwmC, OUTPUT);
  ledcAttachPin(M1_pwmA, 3);
  ledcAttachPin(M1_pwmB, 4);
  ledcAttachPin(M1_pwmC, 5);
  ledcSetup(3, 30000, 8); // 30 kHz PWM, 8-bit resolution
  ledcSetup(4, 30000, 8);
  ledcSetup(5, 30000, 8);

  pinMode(enable, OUTPUT);

  // Initialize I2C buses and AS5600 encoders
  // I2C0: Motor 0 encoder at 400 kHz
  // I2C1: Motor 1 encoder at 400 kHz
  S0_I2C.begin(19, 18, 400000UL);
  S1_I2C.begin(23, 5, 400000UL);
  S0.Sensor_init(&S0_I2C);
  S1.Sensor_init(&S1_I2C);

  // Initialize built-in current sense ADC channels
  CS_M0.init();
  CS_M1.init();
}

// ==================== ANGLE CALCULATION FUNCTIONS ====================
/// @brief Calculate Motor 0 electrical angle for FOC commutation
/// Converts mechanical angle from encoder, applies pole pair scaling and direction
/// @return Electrical angle in radians [0, 2*PI), accounting for PP and direction
float M0_electricalAngle()
{
  // Electrical angle = (Mechanical angle × PP × DIR) - Zero offset
  // Accounts for motor pole pairs and calibration offset
  // Result normalized to [0, 2*PI) for commutation alignment
  return _normalizeAngle((float)(M0_DIR * M0_PP) * S0.getMechanicalAngle() - M0_zero_electric_angle);
}

/// @brief Calculate Motor 1 electrical angle for FOC commutation
/// Converts mechanical angle from encoder, applies pole pair scaling and direction
/// @return Electrical angle in radians [0, 2*PI), accounting for PP and direction
float M1_electricalAngle()
{
  // Electrical angle = (Mechanical angle × PP × DIR) - Zero offset
  // Accounts for motor pole pairs and calibration offset
  // Result normalized to [0, 2*PI) for commutation alignment
  return _normalizeAngle((float)(M1_DIR * M1_PP) * S1.getMechanicalAngle() - M1_zero_electric_angle);
}

// ==================== SENSOR ALIGNMENT FUNCTIONS ====================
/// @brief Align Motor 0 encoder to motor pole positions (calibration routine)
/// Applies fixed torque at 3pi/2 position, waits for magnetic settling, then records zero offset
/// MUST be called during initialization with unloaded motor
/// @param _PP Number of pole pairs for this motor
/// @param _DIR Rotation direction: 1=CCW, -1=CW
/// @note Blocks for 1 second during alignment. Motor will experience brief torque pulse
void DFOC_M0_alignSensor(int _PP, int _DIR)
{
  // Configure motor parameters
  M0_PP = _PP;
  M0_DIR = _DIR;

  // Apply holding torque at 3pi/2 (270°) to align rotor with known position
  M0_setTorque(3, _3PI_2);
  delay(1000); // Wait for magnetic settling

  // Read encoder angle at known rotor position to determine zero offset
  S0.Sensor_update();
  M0_zero_electric_angle = M0_electricalAngle();

  // Stop motor
  M0_setTorque(0, _3PI_2);
}

/// @brief Align Motor 1 encoder to motor pole positions (calibration routine)
/// Applies fixed torque at 3pi/2 position, waits for magnetic settling, then records zero offset
/// MUST be called during initialization with unloaded motor
/// @param _PP Number of pole pairs for this motor
/// @param _DIR Rotation direction: 1=CCW, -1=CW
/// @note Blocks for 1 second during alignment. Motor will experience brief torque pulse
void DFOC_M1_alignSensor(int _PP, int _DIR)
{
  // Configure motor parameters
  M1_PP = _PP;
  M1_DIR = _DIR;

  // Apply holding torque at 3pi/2 (270°) to align rotor with known position
  M1_setTorque(3, _3PI_2);
  delay(1000); // Wait for magnetic settling

  // Read encoder angle at known rotor position to determine zero offset
  S1.Sensor_update();
  M1_zero_electric_angle = M1_electricalAngle();

  // Stop motor
  M1_setTorque(0, _3PI_2);
}

// ==================== SENSOR GETTER FUNCTIONS ====================
/// @brief Read Motor 0 mechanical angle (post-alignment)
/// @return Mechanical angle in radians, accounting for direction setting
float DFOC_M0_Angle()
{
  // Returns the mechanical angle in radians
  return M0_DIR * S0.getAngle();
}

/// @brief Read Motor 1 mechanical angle (post-alignment)
/// @return Mechanical angle in radians, accounting for direction setting
float DFOC_M1_Angle()
{
  // Returns the mechanical angle in radians
  return M1_DIR * S1.getAngle();
}

// ==================== CURRENT CALCULATION FUNCTIONS ====================
/// @brief Transform phase currents (Ia, Ib) to q-axis current using Park transform
/// Performs abc->alphabeta->dq transformation for direct torque measurement
/// @param current_a Phase A current (A)
/// @param current_b Phase B current (A) - Phase C is calculated as -(Ia+Ib)
/// @param angle_el Electrical angle for rotation (radians)
/// @return q-axis current component (proportional to torque)
float cal_Iq_Id(float current_a, float current_b, float angle_el)
{
  // Clarke Transform: abc -> alphabeta (3-phase to 2-phase orthogonal)
  // Assumes Ic = -(Ia + Ib)
  float I_alpha = current_a;
  float I_beta = _1_SQRT3 * current_a + _2_SQRT3 * current_b;

  // Park Transform: alphabeta -> dq (cartesian to rotating reference frame)
  // Rotates by electrical angle to align with motor magnetic field
  float ct = cosf(angle_el);
  float st = sinf(angle_el);
  float I_q = I_beta * ct - I_alpha * st; // q-axis current (torque component)
  return I_q;
}

/// @brief Read Motor 0 q-axis current (torque feedback) with low-pass filtering
/// Transforms phase currents to dq frame and applies low-pass filter
/// @return Filtered q-axis current (A), proportional to motor torque
/// @note Must call runFOC() first to update sensor data
float DFOC_M0_Current()
{
  float I_q_M0_ori = cal_Iq_Id(CS_M0.current_a, CS_M0.current_b, M0_electricalAngle());
  float I_q_M0_flit = M0_Curr_Flt(I_q_M0_ori);
  return I_q_M0_flit;
}

/// @brief Read Motor 1 q-axis current (torque feedback) with low-pass filtering
/// Transforms phase currents to dq frame and applies low-pass filter
/// @return Filtered q-axis current (A), proportional to motor torque
/// @note Must call runFOC() first to update sensor data
float DFOC_M1_Current()
{
  float I_q_M1_ori = cal_Iq_Id(CS_M1.current_a, CS_M1.current_b, M1_electricalAngle());
  float I_q_M1_flit = M1_Curr_Flt(I_q_M1_ori);
  return I_q_M1_flit;
}

/// @brief Read Motor 0 velocity with low-pass filtering
/// @return Filtered angular velocity (rad/s), accounting for direction setting
/// @note Must call runFOC() first to update sensor data
float DFOC_M0_Velocity()
{
  float vel_M0_ori = S0.getVelocity();
  float vel_M0_flit = M0_Vel_Flt(M0_DIR * vel_M0_ori);
  return vel_M0_flit;
}

/// @brief Read Motor 1 velocity with low-pass filtering
/// @return Filtered angular velocity (rad/s), accounting for direction setting
/// @note Must call runFOC() first to update sensor data
float DFOC_M1_Velocity()
{
  float vel_M1_ori = S1.getVelocity();
  float vel_M1_flit = M1_Vel_Flt(M1_DIR * vel_M1_ori);
  return vel_M1_flit;
}

// ==================== SERIAL COMMUNICATION ====================
/// Global target value received from serial
float motor_target;
/// Helper variable for string parsing
int commaPosition;

/// @brief Parse incoming serial data for motor target commands
/// Expects floating-point number followed by newline
/// Updates global motor_target variable when valid command received
/// @return Received command string (may be empty if no complete command yet)
/// @note Non-blocking, returns immediately if no data available
String serialReceiveUserCommand()
{
  static String received_chars;
  String command = "";

  while (Serial.available())
  {
    char inChar = (char)Serial.read();
    received_chars += inChar;

    if (inChar == '\n')
    {
      command = received_chars;
      commaPosition = command.indexOf('\n');
      if (commaPosition != -1)
      {
        motor_target = command.substring(0, commaPosition).toDouble();
      }
      received_chars = "";
    }
  }
  return command;
}

/// @brief Get last motor target command received from serial
/// @return Target value parsed from most recent serial command
float serial_motor_target()
{
  return motor_target;
}

// ==================== HIGH-LEVEL TORQUE CONTROL ====================
/// @brief Set Motor 0 torque command (simplified wrapper)
/// Automatically applies current electrical angle for FOC
/// @param Target q-axis voltage command (-Vbus/2 to +Vbus/2)
void DFOC_M0_setTorque(float Target)
{
  M0_setTorque(Target, M0_electricalAngle());
}

/// @brief Set Motor 1 torque command (simplified wrapper)
/// Automatically applies current electrical angle for FOC
/// @param Target q-axis voltage command (-Vbus/2 to +Vbus/2)
void DFOC_M1_setTorque(float Target)
{
  M1_setTorque(Target, M1_electricalAngle());
}

// ==================== CASCADED PID CONTROL LOOPS ====================
/// @brief Motor 0 position control with velocity feedforward (cascaded loops)
/// Signal flow: angle_error -> ANGLE_PID -> velocity_target -> VELOCITY_PID -> voltage_command -> motor
/// @param Target Target mechanical angle (radians)
/// @note ANGLE_PID output (velocity) is subtracted from actual velocity as error input to VEL_PIД
void DFOC_M0_set_Velocity_Angle(float Target)
{
  DFOC_M0_setTorque(DFOC_M0_VEL_PID(DFOC_M0_ANGLE_PID((Target - DFOC_M0_Angle()) * 180 / PI) - DFOC_M0_Velocity()));
}

/// @brief Motor 1 position control with velocity feedforward (cascaded loops)
/// Signal flow: angle_error -> ANGLE_PID -> velocity_target -> VELOCITY_PID -> voltage_command -> motor
/// @param Target Target mechanical angle (radians)
/// @note ANGLE_PID output (velocity) is subtracted from actual velocity as error input to VEL_PID
void DFOC_M1_set_Velocity_Angle(float Target)
{
  DFOC_M1_setTorque(DFOC_M1_VEL_PID(DFOC_M1_ANGLE_PID((Target - DFOC_M1_Angle()) * 180 / PI) - DFOC_M1_Velocity()));
}

/// @brief Motor 0 velocity control (standalone velocity loop)
/// Signal flow: velocity_error -> VELOCITY_PID -> voltage_command -> motor
/// @param Target Target angular velocity (rad/s)
/// @note Direct velocity control without position loop
void DFOC_M0_setVelocity(float Target)
{
  DFOC_M0_setTorque(DFOC_M0_VEL_PID((Target - DFOC_M0_Velocity()) * 180 / PI));
}

/// @brief Motor 1 velocity control (standalone velocity loop)
/// Signal flow: velocity_error -> VELOCITY_PID -> voltage_command -> motor
/// @param Target Target angular velocity (rad/s)
/// @note Direct velocity control without position loop
void DFOC_M1_setVelocity(float Target)
{
  DFOC_M1_setTorque(DFOC_M1_VEL_PID((Target - DFOC_M1_Velocity()) * 180 / PI));
}

/// @brief Motor 0 position control (stiff direct mode)
/// Signal flow: angle_error -> ANGLE_PID -> voltage_command -> motor (no velocity limiting)
/// @param Target Target mechanical angle (radians)
/// @note Proportional-only position control - applies voltage proportional to angle error
void DFOC_M0_set_Force_Angle(float Target)
{
  DFOC_M0_setTorque(DFOC_M0_ANGLE_PID((Target - DFOC_M0_Angle()) * 180 / PI));
}

/// @brief Motor 1 position control (stiff direct mode)
/// Signal flow: angle_error -> ANGLE_PID -> voltage_command -> motor (no velocity limiting)
/// @param Target Target mechanical angle (radians)
/// @note Proportional-only position control - applies voltage proportional to angle error
void DFOC_M1_set_Force_Angle(float Target)
{
  DFOC_M1_setTorque(DFOC_M1_ANGLE_PID((Target - DFOC_M1_Angle()) * 180 / PI));
}

// ==================== CURRENT/TORQUE CONTROL ====================
/// @brief Motor 0 current/torque control (voltage-source current controller)
/// Signal flow: current_error -> CURRENT_PID -> voltage_command -> motor
/// Maintains precise motor current (and thus torque) by adjusting applied voltage
/// @param Target Target q-axis current (A)
/// @note Current loop K_p = 1.2 V/A => 1.2V applied per amp of current error
void DFOC_M0_setTorque_current(float Target)
{
  // Compute electrical angle once and reuse for both current measurement and torque output
  float angle_el = M0_electricalAngle();
  float I_q = M0_Curr_Flt(cal_Iq_Id(CS_M0.current_a, CS_M0.current_b, angle_el));
  M0_setTorque(M0_current_loop(Target - I_q), angle_el);
}

/// @brief Motor 1 current/torque control (voltage-source current controller)
/// Signal flow: current_error -> CURRENT_PID -> voltage_command -> motor
/// Maintains precise motor current (and thus torque) by adjusting applied voltage
/// @param Target Target q-axis current (A)
/// @note Current loop K_p = 1.2 V/A => 1.2V applied per amp of current error
void DFOC_M1_setTorque_current(float Target)
{
  // Compute electrical angle once and reuse for both current measurement and torque output
  float angle_el = M1_electricalAngle();
  float I_q = M1_Curr_Flt(cal_Iq_Id(CS_M1.current_a, CS_M1.current_b, angle_el));
  M1_setTorque(M1_current_loop(Target - I_q), angle_el);
}

// ==================== SENSOR UPDATE (MAIN LOOP) ====================
/// @brief Update sensor data for M0
/// MUST be called every control loop iteration to refresh cached sensor values
/// Updates encoder positions/velocities and phase currents for both motors
/// @note Non-blocking, completes in ~5-10ms depending on I2C speed
/// @note Single runFOC() call provides fresh data for all subsequent getter functions
void runFOC_M0()
{
  S0.Sensor_update();
  CS_M0.getPhaseCurrents();
}

/// @brief Update sensor data for M1
/// MUST be called every control loop iteration to refresh cached sensor values
/// Updates encoder positions/velocities and phase currents for both motors
/// @note Non-blocking, completes in ~5-10ms depending on I2C speed
/// @note Single runFOC() call provides fresh data for all subsequent getter functions
void runFOC_M1()
{
  S1.Sensor_update();
  CS_M1.getPhaseCurrents();
}