#include "Copter.h"
#include <cmath>
#include <algorithm>
#include <iostream>

using namespace std;

#if MODE_GUIDED_NOGPS_ENABLED

/*
 * Initialization and run calls for the guided_nogps flight mode
 */

const AP_Param::GroupInfo ModeGuidedNoGPS::var_info[] = {
    // @Param: _XY_P
    // @DisplayName: GNGP P gain
    // @Description: GNGP (horizontal) P gain.
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: _XY_I
    // @DisplayName: GNGP I gain
    // @Description: GNGP (horizontal) I gain
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _XY_IMAX
    // @DisplayName: GNGP Integrator Max
    // @Description: GNGP (horizontal) integrator maximum
    // @Range: 0 4500
    // @Increment: 10
    // @Units: cdeg
    // @User: Advanced

    // @Param: _XY_FILT_HZ
    // @DisplayName: GNGP filter on input to control
    // @Description: GNGP (horizontal) filter on input to control
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced
    AP_SUBGROUPINFO(flow_pi_xy, "_XY_",  1, ModeGuidedNoGPS, AC_PI_2D),

    // @Param: _FLOW_MAX
    // @DisplayName: GNGP Flow Rate Max
    // @Description: Controls maximum apparent flow rate in flowhold
    // @Range: 0.1 2.5
    // @User: Standard
    AP_GROUPINFO("_FLOW_MAX", 2, ModeGuidedNoGPS, flow_max, 0.6),

    // @Param: _FILT_HZ
    // @DisplayName: GNGP Filter Frequency
    // @Description: Filter frequency for flow data
    // @Range: 1 100
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("_FILT_HZ", 3, ModeGuidedNoGPS, flow_filter_hz, 5),

    // @Param: _QUAL_MIN
    // @DisplayName: GNGP Flow quality minimum
    // @Description: Minimum flow quality to use flow position hold
    // @Range: 0 255
    // @User: Standard
    AP_GROUPINFO("_QUAL_MIN", 4, ModeGuidedNoGPS, flow_min_quality, 10),

    // 5 was FLOW_SPEED

    AP_GROUPEND
};

float ModeGuidedNoGPS::degrees_to_radians(float degrees) {
    return degrees * M_PI / 180.0f;
}

float ModeGuidedNoGPS::normalize_angle_deg(float angle) {
    return fmod(fmod(angle, 360.0f) + 360.0f, 360.0f);
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

    // Optical flow
    flow_filter.set_cutoff_frequency(copter.scheduler.get_loop_rate_hz(), 5);

    quality_filtered = 0;
    flow_pi_xy.reset_I();

    flow_pi_xy.set_dt(1.0/copter.scheduler.get_loop_rate_hz());

    // start with INS height
    last_ins_height = copter.inertial_nav.get_position_z_up_cm() * 0.01;
    height_offset = 0;

    // Information message
    gcs().send_text(MAV_SEVERITY_INFO, "DR Start");

    return true;
}

// Run the guided_nogps controller logic
void ModeGuidedNoGPS::run()
{
    // Calculate the current altitude below home
    float curr_alt_below_home = 0.0f;
    AP::ahrs().get_relative_position_D_home(curr_alt_below_home);

    // Calculate the target altitude above the vehicle
    float target_alt_above_vehicle = fly_alt_min + curr_alt_below_home;
    float climb_rate_chg_max = interval_ms * 0.001f * (wp_nav->get_accel_z() * 0.01f);
    
    climb_rate = min(target_alt_above_vehicle * 0.1f, min(wp_nav->get_default_speed_up() * 0.01f, climb_rate + climb_rate_chg_max));

    copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    climb_rate = get_avoidance_adjusted_climbrate(climb_rate);

    pos_control->set_pos_target_z_from_climb_rate_cm(climb_rate);

    // Calculate body to home azimuth
    float current_yaw = AP::ahrs().get_yaw();
    float body_to_home_azimuth_rad = home_yaw - current_yaw;

    // Create vector for body to home azimuth needed to apply correct body angle
    Vector2f home_vector = Vector2f(sinf(body_to_home_azimuth_rad), -cosf(body_to_home_azimuth_rad));
    Vector2f bf_angles = Vector2f(home_vector.x, home_vector.y);

    float angle_max = copter.aparm.angle_max;
    bf_angles *= angle_max;

#if AP_OPTICALFLOW_ENABLED
    if (copter.optflow.healthy()) {
        const float filter_constant = 0.95;
        quality_filtered = filter_constant * quality_filtered + (1-filter_constant) * copter.optflow.quality();
    } else {
        quality_filtered = 0;
    }

    if (quality_filtered >= 10) {
        Vector2f flow_angles;

        // Flow correction
        Vector2f raw_flow = copter.optflow.flowRate() - copter.optflow.bodyRate();

        // limit sensor flow, this prevents oscillation at low altitudes
        raw_flow.x = constrain_float(raw_flow.x, -0.6f, 0.6f);
        raw_flow.y = constrain_float(raw_flow.y, -0.6f, 0.6f);

        // filter the flow rate
        Vector2f sensor_flow = flow_filter.apply(raw_flow);

        // scale by height estimate, limiting it to height_min to height_max
        float height = copter.inertial_nav.get_position_z_up_cm() * 0.01;

        // compensate for height, this converts to (approx) m/s
        sensor_flow *= constrain_float(height, height_min, height_max);

        // rotate controller input to earth frame
        Vector2f input_ef = copter.ahrs.body_to_earth2D(sensor_flow);

        // run PI controller
        flow_pi_xy.set_input(input_ef);

        // get earth frame controller attitude in centi-degrees
        Vector2f ef_output;

        // get P term
        ef_output = flow_pi_xy.get_p() * copter.aparm.angle_max;

        // convert to body frame
        flow_angles += copter.ahrs.earth_to_body2D(ef_output);

        // constrain to angle limit
        flow_angles.x = constrain_float(flow_angles.x, -copter.aparm.angle_max, copter.aparm.angle_max);
        flow_angles.y = constrain_float(flow_angles.y, -copter.aparm.angle_max, copter.aparm.angle_max);

        bf_angles += flow_angles;
    // }
#endif

    bf_angles.x = constrain_float(bf_angles.x, -angle_max, angle_max);
    bf_angles.y = constrain_float(bf_angles.y, -angle_max, angle_max);

    // call attitude controller
    copter.attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(bf_angles.x, bf_angles.y, get_pilot_desired_yaw_rate());

    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();
}

#endif
