#ifndef DIAL_H
#define DIAL_H

#include <stdint.h>

/// Enable/disable individual torque contributions
struct DialConfig
{
    bool enable_detent = false;            // Enable detent mode (spring-like torque towards nearest detent positions)
    bool enable_vibration = false;         // Enable periodic vibration pulses (for testing/debugging)
    bool enable_bounds_restoration = true; // Enable strong corrective torque when exceeding angle bounds
    bool enable_oob_kick = true;           // Enable out-of-bounds (OOB) kicking mode (pulsed corrective force when outside bounds)
    bool enable_tracking = true;           // Enable position tracking towards a target

    // Detent mode parameters
    float detent_distance = 10 * 3.1415926 / 180.0; // One detent position every ~10 degrees
    float detent_kp = 5.0;                          // Proportional gain for detent spring effect
    float detent_max_torque = 0.2;                  // Maximum absolute torque magnitude (A)

    // Bounds restoration parameters
    float bounds_min_angle = -10.1415926; // Minimum allowed angle (radians, ~-180°)
    float bounds_max_angle = 10.1415926;  // Maximum allowed angle (radians, ~+180°)
    float bounds_kp = 10.0;              // Proportional gain for bounds correction (larger than detent)
    float bounds_max_torque = 0.3;       // Maximum absolute torque magnitude (A)

    // Tracking position parameters
    float tracking_position = 0.0;   // Target position to track towards (radians)
    float tracking_kp = 2.5;         // Proportional gain for position tracking
    float tracking_kd = 0.002;         // Derivative gain for position tracking (damping)
    float tracking_max_torque = 6.0; // Maximum absolute torque magnitude (A)

    // Vibration/debug parameters
    float vibration_amplitude = 0.3;                  // Amplitude of vibration test pulse (A)
    unsigned long vibration_pulse_interval_ms = 1000; // Interval between vibration pulses (ms)

    // Out-of-bounds (OOB) kicking mode
    float oob_kick_amplitude = 3.0;                // Amplitude of OOB kick (A)
    unsigned long oob_kick_pulse_interval_ms = 40; // Interval between OOB kicks (ms)
};

// Dial class encapsulates per-motor state and behavior
class Dial
{
private:
    unsigned long last_vibration_time_local; // Timestamp of last vibration pulse
    unsigned long last_kick_time_local;      // Timestamp of last out-of-bounds kick
    bool kick_state;                         // State flag for kick pulse (unused in current implementation)
    double logical_angle_offset;             // Offset between raw sensor angle and logical dial angle

public:
    DialConfig *cfg;   // Pointer to configuration settings
    float last_torque; // Last calculated torque value (A)
    double last_angle; // Last logical dial angle (radians)
    float last_speed;  // Last measured physical motor speed (rad/s)

    Dial(DialConfig *c = nullptr);

    void begin();

    double get_motor_angle();

    float get_motor_speed();

    double get_logical_angle();

    bool is_out_of_bounds() const;

    void reset_runtime_state(unsigned long now);

    void set_current_position(double logical_angle, bool update_tracking_target);

    float calculate_detent_torque(double current_angle);

    float calculate_tracking_torque(double current_angle, float current_speed);

    float calculate_oob_kick_torque(unsigned long now);

    float calculate_vibration_torque(unsigned long now);

    float calculate_bounds_torque(double current_angle);

    float calculate_composite_torque(unsigned long now);

    void apply_torque(float torque, bool use_current_control);

    void calculate_and_apply_composite_torque(bool use_current_control);
};

#endif // DIAL_H
