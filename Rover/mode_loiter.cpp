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
    // Get pilot manual input
    const int16_t steer_input = channel_steer->get_control_in();
    const int16_t throttle_input = channel_throttle->get_control_in();

    const bool manual_steering = fabsf(steer_input) > 100;   // ~2% deadzone
    const bool manual_throttle = fabsf(throttle_input) > 100; // ~2% deadzone

    // Use standard loiter radius
    const float loiter_radius = g2.loit_radius;

    // Distance to loiter center
    _distance_to_destination = rover.current_loc.get_distance(_destination);

    // Desired yaw and speed
    float desired_yaw_cd = ahrs.yaw_sensor;
    float desired_speed = 0.0f;

    if (manual_steering || manual_throttle) {
        // --- Manual override active ---

        // Convert throttle input to m/s
        const float input_throttle = throttle_input / 1000.0f;
        const float max_speed = g2.wp_nav.get_default_speed();
        desired_speed = constrain_float(input_throttle * max_speed, -max_speed, max_speed);

        // Convert steering input to yaw change
        desired_yaw_cd = wrap_360_cd(ahrs.yaw_sensor + steer_input * 0.01f);

        // Remember this heading
        _desired_yaw_cd = desired_yaw_cd;

    } else {
        // --- Autonomous loiter control ---

        if (_distance_to_destination <= loiter_radius) {
            // Close to destination, slow or stop
            desired_speed = attitude_control.get_desired_speed_accel_limited(0.0f, rover.G_Dt);
            desired_yaw_cd = _desired_yaw_cd;  // hold previous heading
        } else {
            // Far from destination, move toward it
            desired_speed = MIN((_distance_to_destination - loiter_radius) * g2.loiter_speed_gain,
                                g2.wp_nav.get_default_speed());

            desired_yaw_cd = rover.current_loc.get_bearing_to(_destination);
            float yaw_error_cd = wrap_180_cd(desired_yaw_cd - ahrs.yaw_sensor);

            // Reverse if destination is behind us or reverse mode is forced
            if ((fabsf(yaw_error_cd) > 9000 && g2.loit_type == 0) || g2.loit_type == 2) {
                desired_yaw_cd = wrap_180_cd(desired_yaw_cd + 18000);
                yaw_error_cd = wrap_180_cd(desired_yaw_cd - ahrs.yaw_sensor);
                desired_speed = -desired_speed;
            }

            // Reduce speed if turning sharply
            float yaw_error_ratio = 1.0f - constrain_float(fabsf(yaw_error_cd / 9000.0f), 0.0f, 1.0f) * 0.5f;
            desired_speed *= yaw_error_ratio;
        }

        _desired_yaw_cd = desired_yaw_cd;
        _desired_speed = desired_speed;
    }

    // Final control commands
    calc_steering_to_heading(desired_yaw_cd, 0.0f);
    calc_throttle(desired_speed, true);
}

// get desired location
bool ModeLoiter::get_desired_location(Location& destination) const
{
    destination = _destination;
    return true;
}
