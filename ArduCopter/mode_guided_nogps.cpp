#include "Copter.h"
#include <cmath>
#include <algorithm>
#include <iostream>

using namespace std;

#if MODE_GUIDED_NOGPS_ENABLED

/*
 * Initialization and run calls for the guided_nogps flight mode
 */

float ModeGuidedNoGPS::normalize_radians(float radians) {
    return std::fmod(radians + M_PI, 2.0f * M_PI) - M_PI;
}

float ModeGuidedNoGPS::degrees_to_radians(float degrees) {
    return degrees * M_PI / 180.0f;
}

float ModeGuidedNoGPS::radians_to_degrees(float radians) {
    return radians * 180.0f / M_PI;
}

float ModeGuidedNoGPS::normalize_angle_deg(float angle) {
    return std::fmod(std::fmod(angle, 360.0f) + 360.0f, 360.0f);
}

// Initialize the guided_nogps controller
bool ModeGuidedNoGPS::init(bool ignore_checks)
{
    // Start the angle control mode
    ModeGuided::angle_control_start();

    // Set parameters
    fly_angle = copter.aparm.angle_max / 100.0f; // maximum tilt angle in radians (angle_max in hundredths of a degree)
    interval_ms = 100.0f;                        // update interval in milliseconds

    // Minimum height and yaw
    fly_alt_min = g.rtl_altitude / 100.0f;       // minimum height above the home
    home_yaw = g.dr_home_yaw < 1 ? copter.azimuth_to_home : static_cast<float>(g.dr_home_yaw);
    home_yaw = degrees_to_radians(normalize_angle_deg(home_yaw)); // convert home_yaw to radians
    timeout = g2.failsafe_dr_timeout;

    // Initial value of climb_rate
    climb_rate = 0.0f;

    // Information message
    gcs().send_text(MAV_SEVERITY_INFO, "DR Start");

    return true;
}

// Run the guided_nogps controller logic
void ModeGuidedNoGPS::run()
{
    // Run the angle control logic
    ModeGuided::angle_control_run();

    // Calculate the current altitude below home
    float curr_alt_below_home = 0.0f;
    AP::ahrs().get_relative_position_D_home(curr_alt_below_home);

    // Calculate the target altitude above the vehicle
    float target_alt_above_vehicle = fly_alt_min + curr_alt_below_home;
    float climb_rate_chg_max = interval_ms * 0.001f * (wp_nav->get_accel_z() * 0.01f);
    
    climb_rate = min(target_alt_above_vehicle * 0.1f, min(wp_nav->get_default_speed_up() * 0.01f, climb_rate + climb_rate_chg_max));

    // Calculate body to home azimuth
    float body_azimuth = radians_to_degrees(AP::ahrs().get_yaw());
    float home_azimuth = radians_to_degrees(home_yaw);
    float body_to_home_azimuth = home_azimuth - body_azimuth;

    float body_to_home_azimuth_rad = degrees_to_radians(body_to_home_azimuth);

    // Create vector for body to home azimuth needed to apply correct body angle
    Vector2f home_vector = Vector2f(sin(body_to_home_azimuth_rad), cos(body_to_home_azimuth_rad));

    // Create quaternion from 
    q.from_euler(radians(fly_angle * home_vector.x), -radians(fly_angle * home_vector.y), AP::ahrs().get_yaw());

    // Set angle for moving to home azimuth
    ModeGuided::set_angle(q, Vector3f{}, climb_rate * 100.0f, false);
}

#endif
