#include "Copter.h"
#include <cmath>
#include <algorithm>

using namespace std;

#if MODE_GUIDED_NOGPS_ENABLED

/*
 * Initialization and run calls for the guided_nogps flight mode
 */

float ModeGuidedNoGPS::degrees_to_radians(float degrees) {
    return degrees * M_PI / 180.0f;
}

float ModeGuidedNoGPS::normalize_angle_deg(float angle) {
    return fmod(fmod(angle, 360.0f) + 360.0f, 360.0f);
}

void ModeGuidedNoGPS::normalize_vector(Vector2f& vector) {
    float length = sqrtf(vector.x * vector.x + vector.y * vector.y);

    vector.x = vector.x / length;
    vector.y = vector.y / length;
}

// Initialize the guided_nogps controller
bool ModeGuidedNoGPS::init(bool ignore_checks)
{
    // Start the angle control mode
    ModeGuided::angle_control_start();

    // Set parameters
    fly_angle = copter.aparm.angle_max / 100.0f;    // maximum tilt angle in radians (angle_max in hundredths of a degree)
    interval_ms = 100.0f;                           // update interval in milliseconds

    // Minimum height and yaw
    fly_alt_min = g.rtl_altitude / 100.0f;       // minimum height above the home
    home_yaw = g.dr_home_yaw < 1 ? copter.azimuth_to_home : static_cast<float>(g.dr_home_yaw);
    home_yaw = degrees_to_radians(normalize_angle_deg(home_yaw)); // convert home_yaw to radians

    // Initial value of climb_rate
    climb_rate = 0.0f;

    // Initial flow quality
    quality_filtered = 0;

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
    float current_yaw = AP::ahrs().get_yaw();
    float target_yaw = current_yaw + get_pilot_desired_yaw_rate() * 0.00003f;
    float body_to_home_azimuth_rad = home_yaw - target_yaw;

    // Create vector for body to home azimuth needed to apply correct body angle
    Vector2f target_vector = Vector2f(
        sinf(body_to_home_azimuth_rad) * g.gngp_home_vector_multiplier,
        cosf(body_to_home_azimuth_rad) * g.gngp_home_vector_multiplier
    );

#if AP_OPTICALFLOW_ENABLED
    if (copter.optflow.healthy()) {
        const float filter_constant = 0.95;
        quality_filtered = filter_constant * quality_filtered + (1-filter_constant) * copter.optflow.quality();

        if (quality_filtered >= 10) {
            Vector2f diff_vector = (copter.optflow.flowRate() - target_vector) * g.gngp_optflow_multiplier;
            
            target_vector.x = target_vector.x - diff_vector.x;
            target_vector.y = target_vector.y - diff_vector.y;
        }
    } else {
        quality_filtered = 0;
    }
#endif

    normalize_vector(target_vector);

    target_vector *= g.gngp_speed_multiplier;

    // Create quaternion for movement
    q.from_euler(
        radians(fly_angle * target_vector.x),
        -radians(fly_angle * target_vector.y),
        target_yaw
    );

    // Start movement
    ModeGuided::set_angle(q, Vector3f{}, climb_rate * 100.0f, false);
}

#endif
