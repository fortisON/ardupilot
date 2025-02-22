#include "Copter.h"
#include <cmath>
#include <algorithm>

using namespace std;

#if MODE_GUIDED_NOGPS_ENABLED

const AP_Param::GroupInfo ModeGuidedNoGPS::var_info[] = {
    // @Param: _XY_P
    // @DisplayName: GuidedNoGPS P gain
    // @Description: GuidedNoGPS (horizontal) P gain.
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: _XY_I
    // @DisplayName: GuidedNoGPS I gain
    // @Description: GuidedNoGPS (horizontal) I gain
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _XY_IMAX
    // @DisplayName: GuidedNoGPS Integrator Max
    // @Description: GuidedNoGPS (horizontal) integrator maximum
    // @Range: 0 4500
    // @Increment: 10
    // @Units: cdeg
    // @User: Advanced

    // @Param: _XY_FILT_HZ
    // @DisplayName: GuidedNoGPS filter on input to control
    // @Description: GuidedNoGPS (horizontal) filter on input to control
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced
    AP_SUBGROUPINFO(flow_pi_xy, "_XY_",  1, ModeGuidedNoGPS, AC_PI_2D),

    // @Param: _FLOW_MAX
    // @DisplayName: GuidedNoGPS Flow Rate Max
    // @Description: Controls maximum apparent flow rate in GuidedNoGPS
    // @Range: 0.1 2.5
    // @User: Standard
    AP_GROUPINFO("_FLOW_MAX", 2, ModeGuidedNoGPS, flow_max, 0.6),

    // @Param: _FILT_HZ
    // @DisplayName: GuidedNoGPS Filter Frequency
    // @Description: Filter frequency for flow data
    // @Range: 1 100
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("_FILT_HZ", 3, ModeGuidedNoGPS, flow_filter_hz, 5),

    // @Param: _QUAL_MIN
    // @DisplayName: GuidedNoGPS Flow quality minimum
    // @Description: Minimum flow quality to use flow position hold
    // @Range: 0 255
    // @User: Standard
    AP_GROUPINFO("_QUAL_MIN", 4, ModeGuidedNoGPS, flow_min_quality, 10),

    // 5 was FLOW_SPEED

    AP_GROUPEND
};

ModeGuidedNoGPS::ModeGuidedNoGPS(void) : ModeGuided()
{
    AP_Param::setup_object_defaults(this, var_info);
}

float ModeGuidedNoGPS::normalize_angle_deg(float angle) {
    return fmod(fmod(angle, 360.0f) + 360.0f, 360.0f);
}

// Initialize the guided_nogps controller
bool ModeGuidedNoGPS::init(bool ignore_checks)
{
    // initialise the vertical position controller
    if (!copter.pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // Set parameters
    fly_angle = copter.aparm.angle_max / 100.0f;    // maximum tilt angle in radians (angle_max in hundredths of a degree)

    // Minimum height and yaw
    fly_alt_min = g.rtl_altitude / 100.0f;          // minimum height above the home
    home_yaw = normalize_angle_deg(g.dr_home_yaw < 1 ? copter.azimuth_to_home : static_cast<float>(g.dr_home_yaw));

    flow_filter.set_cutoff_frequency(copter.scheduler.get_loop_rate_hz(), 5);

    flow_pi_xy.reset_I();
    flow_pi_xy.set_dt(1.0/copter.scheduler.get_loop_rate_hz());

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
    float current_yaw = AP::ahrs().get_yaw() * (180 / M_PI);
    float body_to_home_azimuth = home_yaw + (-current_yaw);

    body_to_home_azimuth = body_to_home_azimuth / (180 / M_PI);

    // Create vector for body to home azimuth needed to apply correct body angle
    Vector2f home_vector = Vector2f(sinf(body_to_home_azimuth), -cosf(body_to_home_azimuth));
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

        // scale by height estimate
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
    }
#endif

    bf_angles.x = constrain_float(bf_angles.x, -angle_max, angle_max);
    bf_angles.y = constrain_float(bf_angles.y, -angle_max, angle_max);

    // call attitude controller
    copter.attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(
        bf_angles.x,
        bf_angles.y,
        get_pilot_desired_yaw_rate()
    );

    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();
}

#endif
