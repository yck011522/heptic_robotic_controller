#include "Dial.h"
#include "DengFOC.h"

Dial::Dial(DialConfig *c)
{
    cfg = c;                       // Bind to configuration structure
    last_vibration_time_local = 0; // Initialize vibration timer
    last_angle = 0.0;              // Initialize angle tracking
    last_speed = 0.0f;
    last_torque = 0.0f;
    logical_angle_offset = 0.0f;
}

void Dial::begin()
{
    reset_runtime_state(millis());
}

double Dial::get_motor_angle()
{
    return DFOC_M0_Angle();
}

float Dial::get_motor_speed()
{
    return DFOC_M0_Velocity();
}

double Dial::get_logical_angle()
{
    return get_motor_angle() + logical_angle_offset;
}

bool Dial::is_out_of_bounds() const
{
    return last_angle < cfg->bounds_min_angle || last_angle > cfg->bounds_max_angle;
}

void Dial::reset_runtime_state(unsigned long now)
{
    last_vibration_time_local = now;
    last_kick_time_local = now;
    kick_state = false;
}

void Dial::set_current_position(double logical_angle, bool update_tracking_target)
{
    logical_angle_offset = logical_angle - get_motor_angle();
    last_angle = logical_angle;
    last_speed = 0.0f;
    reset_runtime_state(millis());

    if (update_tracking_target)
        cfg->tracking_position = logical_angle;
}

float Dial::calculate_detent_torque(double current_angle)
{
    // Find nearest detent position and apply spring-like restoration force
    double nearest_detent_angle = round(current_angle / cfg->detent_distance) * cfg->detent_distance;
    double error = nearest_detent_angle - current_angle;
    // Compute torque using spring formula: torque = kp * error, where kp is the detent stiffness
    float torque = (float)(cfg->detent_kp * error);

    // Clamp torque to maximum allowed magnitude
    if (torque > cfg->detent_max_torque)
        torque = cfg->detent_max_torque;
    else if (torque < -cfg->detent_max_torque)
        torque = -cfg->detent_max_torque;
    return torque;
}

float Dial::calculate_tracking_torque(double current_angle, float current_speed)
{
    double error = cfg->tracking_position - current_angle;

    // Spring
    float torque = (float)(cfg->tracking_kp * error);

    // Damping
    torque -= cfg->tracking_kd * current_speed;

    // Clamp
    if (torque > cfg->tracking_max_torque)
        torque = cfg->tracking_max_torque;
    else if (torque < -cfg->tracking_max_torque)
        torque = -cfg->tracking_max_torque;

    return torque;
}

float Dial::calculate_oob_kick_torque(unsigned long now)
{
    // Out-of-bounds (OOB) kicking: click at configured frequency, direction towards corrective side
    // Only generate kick when outside bounds
    if (last_angle <= cfg->bounds_max_angle && last_angle >= cfg->bounds_min_angle)
        return 0.0; // Within bounds, no corrective kicks needed

    // Determine direction: negative if above max, positive if below min
    float sign = (last_angle > cfg->bounds_max_angle) ? -1.0f : 1.0f;

    // Flip sign
    sign = -sign;

    // Apply pulsed kick at configured interval
    if (now - last_kick_time_local >= cfg->oob_kick_pulse_interval_ms)
    {
        last_kick_time_local = now;
        kick_state = !kick_state; // Toggle for alternating kicks (currently unused)
    }

    if (kick_state)
        return sign * cfg->oob_kick_amplitude;
    return 0.0;
}

float Dial::calculate_vibration_torque(unsigned long now)
{
    // Generate periodic vibration pulses at configured interval
    if (now - last_vibration_time_local >= cfg->vibration_pulse_interval_ms)
    {
        last_vibration_time_local = now;
        return cfg->vibration_amplitude; // Send pulse
    }
    return 0.0; // No pulse in this cycle
}

float Dial::calculate_bounds_torque(double current_angle)
{
    // Apply corrective torque when angle exceeds bounds (stronger than detent)
    if (current_angle > cfg->bounds_max_angle)
    {
        // Above maximum: apply negative (restoring) torque
        double error = current_angle - cfg->bounds_max_angle;
        float torque = (float)(-cfg->bounds_kp * error);
        if (torque < -cfg->bounds_max_torque)
            torque = -cfg->bounds_max_torque;
        return torque;
    }
    else if (current_angle < cfg->bounds_min_angle)
    {
        // Below minimum: apply positive (restoring) torque
        double error = cfg->bounds_min_angle - current_angle;
        float torque = (float)(cfg->bounds_kp * error);
        if (torque > cfg->bounds_max_torque)
            torque = cfg->bounds_max_torque;
        return torque;
    }
    return 0.0; // Within bounds
}

float Dial::calculate_composite_torque(unsigned long now)
{
    // Combine all enabled torque effects into single control value
    double current_angle = get_logical_angle();
    last_angle = current_angle;
    last_speed = get_motor_speed();
    float total = 0.0;

    // Add individual torque contributions based on enabled features
    if (cfg->enable_detent)
        total += calculate_detent_torque(current_angle);
    if (cfg->enable_tracking)
        total += calculate_tracking_torque(current_angle, last_speed);
    if (cfg->enable_vibration)
        total += calculate_vibration_torque(now);
    if (cfg->enable_bounds_restoration)
        total += calculate_bounds_torque(current_angle);

    // Out-of-bounds kicking: additional pulsed corrective force when outside bounds
    if (cfg->enable_oob_kick)
        total += calculate_oob_kick_torque(now);

    last_torque = total;
    return total;
}

void Dial::apply_torque(float torque, bool use_current_control)
{
    if (use_current_control)
        DFOC_M0_setTorque_current(torque);
    else
        DFOC_M0_setTorque(torque);

    last_torque = torque;
}

void Dial::calculate_and_apply_composite_torque(bool use_current_control)
{
    // Main control loop: calculate all torques and apply to motor
    unsigned long now = millis();
    float t = calculate_composite_torque(now);
    apply_torque(t, use_current_control); // false indicates torque control, true would indicate current control
}
