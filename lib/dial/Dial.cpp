#include "Dial.h"
#include "DengFOC.h"

Dial::Dial(int idx, DialConfig *c)
{
    motor_index = idx;             // Store motor index referencing
    cfg = c;                       // Bind to configuration structure
    last_vibration_time_local = 0; // Initialize vibration timer
    last_angle = 0.0;              // Initialize angle tracking
    max_torque = 1.0;             // Initialize torque tracking

}

void Dial::begin()
{
    // Initialize timing for vibration and kick effects
    last_vibration_time_local = millis();
    last_kick_time_local = 0;
    kick_state = false;
}

float Dial::get_motor_angle(int motor_index)
{
    // Retrieve angle from appropriate motor based on index
    return (motor_index == 0) ? DFOC_M0_Angle() : DFOC_M1_Angle();
}

float Dial::get_motor_speed(int motor_index)
{
    // Retrieve speed from appropriate motor based on index
    return (motor_index == 0) ? DFOC_M0_Velocity() : DFOC_M1_Velocity();
}

float Dial::calculate_detent_torque(float current_angle)
{
    // Find nearest detent position and apply spring-like restoration force
    float nearest_detent_angle = round(current_angle / cfg->detent_distance) * cfg->detent_distance;
    float error = nearest_detent_angle - current_angle;
    // Compute torque using spring formula: torque = kp * error, where kp is the detent stiffness
    float torque = cfg->detent_kp * error;

    // Clamp torque to maximum allowed magnitude
    if (torque > cfg->detent_max_torque)
        torque = cfg->detent_max_torque;
    else if (torque < -cfg->detent_max_torque)
        torque = -cfg->detent_max_torque;
    return torque;
}

float Dial::calculate_tracking_torque(float current_angle, float current_speed)
{
    float error = cfg->tracking_position - current_angle;

    // Spring
    float torque = cfg->tracking_kp * error;

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

float Dial::calculate_bounds_torque(float current_angle)
{
    // Apply corrective torque when angle exceeds bounds (stronger than detent)
    if (current_angle > cfg->bounds_max_angle)
    {
        // Above maximum: apply negative (restoring) torque
        float error = current_angle - cfg->bounds_max_angle;
        float torque = -cfg->bounds_kp * error;
        if (torque < -cfg->bounds_max_torque)
            torque = -cfg->bounds_max_torque;
        return torque;
    }
    else if (current_angle < cfg->bounds_min_angle)
    {
        // Below minimum: apply positive (restoring) torque
        float error = cfg->bounds_min_angle - current_angle;
        float torque = cfg->bounds_kp * error;
        if (torque > cfg->bounds_max_torque)
            torque = cfg->bounds_max_torque;
        return torque;
    }
    return 0.0; // Within bounds
}

float Dial::calculate_composite_torque(unsigned long now)
{
    // Combine all enabled torque effects into single control value
    last_angle = get_motor_angle(motor_index);
    last_speed = get_motor_speed(motor_index);
    float total = 0.0;

    // Add individual torque contributions based on enabled features
    if (cfg->enable_detent)
        total += calculate_detent_torque(last_angle);
    if (cfg->enable_tracking)
        total += calculate_tracking_torque(last_angle, last_speed);
    if (cfg->enable_vibration)
        total += calculate_vibration_torque(now);
    if (cfg->enable_bounds_restoration)
        total += calculate_bounds_torque(last_angle);

    // Out-of-bounds kicking: additional pulsed corrective force when outside bounds
    if (cfg->enable_oob_kick)
        total += calculate_oob_kick_torque(now);

    last_torque = total;
    return total;
}

void Dial::apply_torque(float torque)
{
    // Send calculated torque command to appropriate motor
    if (motor_index == 0)
        // DFOC_M0_setTorque_current(torque);
        DFOC_M0_setTorque(torque);

    else
        // DFOC_M1_setTorque_current(torque);
        DFOC_M1_setTorque(torque);
    last_torque = torque;
}

void Dial::calculate_and_apply_composite_torque()
{
    // Main control loop: calculate all torques and apply to motor
    unsigned long now = millis();
    float t = calculate_composite_torque(now);

    // Clamp final torque to maximum allowed magnitude before applying
    if (t > max_torque)
        t = max_torque;
    else if (t < -max_torque)
        t = -max_torque;

    // Apply the final torque command to the motor
    apply_torque(t);
}
