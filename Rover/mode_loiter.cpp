#include "Rover.h"

bool ModeLoiter::_enter()
{
    // set initial location to current position
    if (!rover.current_loc.is_valid()) {
        return false;
    }

    // Set _destination to current position (acts as position hold point)
    _destination = rover.current_loc;

    // Initialise desired speed to 0
    _desired_speed = 0.0f;

    // Initialise heading to current heading
    _desired_yaw_cd = ahrs.yaw_sensor;
    _last_pilot_heading = _desired_yaw_cd;

    return true;
}

void ModeLoiter::update()
{
    // Read pilot input
    const float throttle_in = channel_throttle->get_control_in();
    const float steer_in = channel_steer->get_control_in();

    const bool has_throttle_input = fabsf(throttle_in) > _input_threshold;
    const bool has_steer_input = fabsf(steer_in) > _input_threshold;

    // Convert inputs to body-frame velocity commands
    Vector2f body_rates;
    body_rates.x = rover.get_pilot_desired_forward_rate(channel_throttle);
    body_rates.y = rover.get_pilot_desired_lateral_rate(channel_steer);

    // Convert to earth frame
    Vector2f earth_rates = ahrs.body_to_earth2D(body_rates);

    // Position Hold Logic
    if (!has_throttle_input && !has_steer_input) {
        // No input: hold position and heading
        wp_nav.set_desired_velocity(Vector2f(0, 0));
        calc_throttle(0.0f, true);

        // Simple heading hold
        attitude_control.input_yaw_angle_cd(_last_pilot_heading);
    } else {
        // Pilot input: allow movement, update target velocity and heading
        wp_nav.set_desired_velocity(earth_rates);

        // Set desired speed based on forward input
        _desired_speed = constrain_float(body_rates.x, -g2.wp_nav.get_default_speed(), g2.wp_nav.get_default_speed());

        // Set yaw: if there’s yaw input, update heading
        if (has_steer_input) {
            _desired_yaw_cd = ahrs.yaw_sensor;
            _last_pilot_heading = _desired_yaw_cd;
        }

        // Update steering and throttle
        calc_steering_to_heading(_last_pilot_heading, 0.0f);
        calc_throttle(_desired_speed, true);
    }
}

// get desired location
bool ModeLoiter::get_desired_location(Location& destination) const
{
    destination = _destination;
    return true;
}
