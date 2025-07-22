#include "Rover.h"

bool ModeLoiter::_enter()
{
    // set _destination to reasonable stopping point
    if (!g2.wp_nav.get_stopping_location(_destination)) {
        return false;
    }

    // initialise desired speed to current speed
    if (!attitude_control.get_forward_speed(_desired_speed)) {
        _desired_speed = 0.0f;
    }

    // initialise heading to current heading
    _desired_yaw_cd = ahrs.yaw_sensor;

    return true;
}

void ModeLoiter::update()
{
    const int16_t steer_input = channel_steer->get_control_in();
    const int16_t throttle_input = channel_throttle->get_control_in();

    const bool manual_steering = fabsf(steer_input) > 100;
    const bool manual_throttle = fabsf(throttle_input) > 100;

    // Very small loiter radius (~20 cm)
    const float loiter_radius = 0.2f;

    _distance_to_destination = rover.current_loc.get_distance(_destination);

    float desired_yaw_cd = ahrs.yaw_sensor;
    float desired_speed = 0.0f;

    if (manual_steering || manual_throttle) {
        // --- Manual control ---
        const float input_throttle = throttle_input / 1000.0f;
        const float max_speed = g2.wp_nav.get_default_speed();
        desired_speed = constrain_float(input_throttle * max_speed, -max_speed, max_speed);
        desired_yaw_cd = wrap_360_cd(ahrs.yaw_sensor + steer_input * 0.01f);
        _desired_yaw_cd = desired_yaw_cd;
    } else {
        // --- Emulated Position Hold ---

        if (_distance_to_destination > loiter_radius) {
            // Move straight to target
            desired_yaw_cd = rover.current_loc.get_bearing_to(_destination);
            desired_speed = g2.wp_nav.get_default_speed();

            float yaw_error_cd = wrap_180_cd(desired_yaw_cd - ahrs.yaw_sensor);

            // Reverse if target is behind and reverse is allowed
            if ((fabsf(yaw_error_cd) > 9000 && g2.loit_type == 0) || g2.loit_type == 2) {
                desired_yaw_cd = wrap_180_cd(desired_yaw_cd + 18000);
                desired_speed = -desired_speed;
            }

        } else {
            // Close enough — stop completely
            desired_speed = 0.0f;
            desired_yaw_cd = _desired_yaw_cd;
        }

        _desired_yaw_cd = desired_yaw_cd;
        _desired_speed = desired_speed;
    }

    // Send control commands
    calc_steering_to_heading(desired_yaw_cd, 0.0f);
    calc_throttle(desired_speed, true);
}


// get desired location
bool ModeLoiter::get_desired_location(Location& destination) const
{
    destination = _destination;
    return true;
}
