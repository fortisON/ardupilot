#include "Copter.h"
#include <cmath>
#include <algorithm>

using namespace std;

#if MODE_GUIDED_NOGPS_ENABLED

const AP_Param::GroupInfo ModeGuidedNoGPS::var_info[] = {
    // @Param: _YAW_RATE
    // @DisplayName: GuidedNoGPS yaw rate
    // @Description: Yaw rate for YAW state (in degrees per second)
    // @Range: 0.0 10.0
    // @User: Standard
    AP_GROUPINFO("_YAW_RATE", 1, ModeGuidedNoGPS, yaw_rate, 3),

    // @Param: _CLMB_RATE
    // @DisplayName: GuidedNoGPS climb rate
    // @Description: Climb rate for FLY state
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("_CLMB_RATE", 2, ModeGuidedNoGPS, climb_rate, 3),

#ifdef AP_OPTICALFLOW_ENABLED
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
    AP_SUBGROUPINFO(flow_pi_xy, "_XY_",  3, ModeGuidedNoGPS, AC_PI_2D),

    // @Param: _FLOW_MAX
    // @DisplayName: GuidedNoGPS Flow Rate Max
    // @Description: Controls maximum apparent flow rate in GuidedNoGPS
    // @Range: 0.1 2.5
    // @User: Standard
    AP_GROUPINFO("_FLOW_MAX", 4, ModeGuidedNoGPS, flow_max, 0.6),

    // @Param: _FILT_HZ
    // @DisplayName: GuidedNoGPS Filter Frequency
    // @Description: Filter frequency for flow data
    // @Range: 1 100
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("_FILT_HZ", 5, ModeGuidedNoGPS, flow_filter_hz, 5),

    // @Param: _QUAL_MIN
    // @DisplayName: GuidedNoGPS Flow quality minimum
    // @Description: Minimum flow quality to use flow position hold
    // @Range: 0 255
    // @User: Standard
    AP_GROUPINFO("_QUAL_MIN", 6, ModeGuidedNoGPS, flow_min_quality, 10),

    // @Param: _FLOW_IMP
    // @DisplayName: GuidedNoGPS Flow impact
    // @Description: Optical flow impact
    // @Range: 0.0 1.0
    // @User: Standard
    AP_GROUPINFO("_FLOW_IMP", 7, ModeGuidedNoGPS, flow_impact, 0.5f),

    // @Param: _FLOW_SMPL
    // @DisplayName: GuidedNoGPS Flow filter samples
    // @Description: Optical flow filter samples
    // @Range: 0 255
    // @User: Standard
    AP_GROUPINFO("_FLOW_SMPL", 8, ModeGuidedNoGPS, flow_filter_samples, 15),
#endif

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

    _state = State::YAW;

    // Set parameters
    fly_angle = copter.aparm.angle_max / 100.0f;    // maximum tilt angle in radians (angle_max in hundredths of a degree)

    // Minimum height and yaw
    fly_alt_min = g.rtl_altitude / 100.0f;          // minimum height above the home
    home_yaw = normalize_angle_deg(g.dr_home_yaw < 1 ? copter.azimuth_to_home : static_cast<float>(g.dr_home_yaw));

#ifdef AP_OPTICALFLOW_ENABLED
    flow_filter.set_cutoff_frequency(copter.scheduler.get_loop_rate_hz(), flow_filter_hz);

    flow_pi_xy.reset_I();
    flow_pi_xy.set_dt(1.0/copter.scheduler.get_loop_rate_hz());

    flow_samples_count = 0;
    flow_error.zero();
#endif

    // Information message
    gcs().send_text(MAV_SEVERITY_INFO, "DR Start");

    return true;
}

// Run the guided_nogps controller logic
void ModeGuidedNoGPS::run()
{
    // Calculate the current altitude below home
    float curr_alt_below_home = 0.0f;
    copter.ahrs.get_relative_position_D_home(curr_alt_below_home);

    // Calculate the target altitude above the vehicle
    float target_alt_above_vehicle = fly_alt_min + curr_alt_below_home;

    copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    
    // Climb rate control
    float target_climb_rate = target_alt_above_vehicle > 0 ? climb_rate * 100 : -climb_rate * 100;

    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

    if (abs(target_alt_above_vehicle) > 1.0f) {
        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
    } else {
        pos_control->set_pos_target_z_from_climb_rate_cm(0);
    }

    switch (_state) {
        case State::YAW:
            yaw_run();
            break;

        case State::FLY:
            fly_run();
            break;
    }

    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();
}

void ModeGuidedNoGPS::yaw_run()
{
    // Calculate the yaw error
    float yaw_error = fmod(normalize_angle_deg(home_yaw - degrees(copter.ahrs.get_yaw())), 90);

    // Calculate the yaw rate
    float target_yaw_rate = yaw_rate * 1000;

    if (yaw_error > 0) {
        target_yaw_rate = -target_yaw_rate;
    }

    if (abs(yaw_error) > 1.0f) {
        copter.attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, target_yaw_rate);
    } else {
        _state = State::FLY;
    }
}

void ModeGuidedNoGPS::fly_run()
{
    // Calculate body to home azimuth
    float current_yaw = degrees(copter.ahrs.get_yaw());
    float body_to_home_azimuth = radians(home_yaw + (-current_yaw));

    // Create vector for body to home azimuth needed to apply correct body angle
    Vector2f home_vector = Vector2f(sinf(body_to_home_azimuth), -cosf(body_to_home_azimuth));
    Vector2f bf_angles = Vector2f(home_vector.x, home_vector.y);

    float angle_max = copter.aparm.angle_max;
    bf_angles = bf_angles.normalized() * angle_max;

#if AP_OPTICALFLOW_ENABLED
    optflow_correction(bf_angles);
#endif // AP_OPTICALFLOW_ENABLED

    bf_angles.x = constrain_float(bf_angles.x, -angle_max, angle_max);
    bf_angles.y = constrain_float(bf_angles.y, -angle_max, angle_max);

    // call attitude controller
    copter.attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(
        bf_angles.x,
        bf_angles.y,
        0
    );
}

#if AP_OPTICALFLOW_ENABLED
void ModeGuidedNoGPS::optflow_correction(Vector2f& target_angles)
{
    if (copter.optflow.healthy()) {
        const float filter_constant = 0.95;
        quality_filtered = filter_constant * quality_filtered + (1-filter_constant) * copter.optflow.quality();
    } else {
        quality_filtered = 0;
    }

    if (quality_filtered >= flow_min_quality) {
        // Flow correction
        Vector2f raw_flow = copter.optflow.flowRate() - copter.optflow.bodyRate();

        flow_samples_count++;

        // Subtract the previous error
        flow_error_buff.x = (raw_flow.x - flow_error_buff.x) / 25;
        flow_error_buff.y = (raw_flow.y - flow_error_buff.y) / 25;

        if (flow_samples_count == flow_filter_samples) {
            flow_samples_count = 0;
            flow_error = flow_error_buff;
        }

        // limit sensor flow, this prevents oscillation at low altitudes
        flow_error.x = constrain_float(flow_error.x, -flow_max, flow_max);
        flow_error.y = constrain_float(flow_error.y, -flow_max, flow_max);

        // filter the flow rate
        Vector2f sensor_flow = flow_filter.apply(flow_error);

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

        // get I term
        if (limited) {
            // only allow I term to shrink in length
            xy_I = flow_pi_xy.get_i_shrink();
        } else {
            // normal I term operation
            xy_I = flow_pi_xy.get_pi();
        }

        // get P term
        ef_output = flow_pi_xy.get_p();
        ef_output += xy_I;
        ef_output *= copter.aparm.angle_max;

#ifdef HAL_LOGGING_ENABLED
        copter.Log_Write_Optflow_PI(flow_pi_xy.get_p(), xy_I);
#endif // HAL_LOGGING_ENABLED

        Vector2f flow_angles;

        // convert to body frame
        flow_angles += copter.ahrs.earth_to_body2D(ef_output);

        // set limited flag to prevent integrator windup
        limited = fabsf(target_angles.x) > copter.aparm.angle_max || fabsf(target_angles.y) > copter.aparm.angle_max;

        // constrain to angle limit
        flow_angles.x = constrain_float(flow_angles.x, -copter.aparm.angle_max * flow_impact, copter.aparm.angle_max * flow_impact);
        flow_angles.y = constrain_float(flow_angles.y, -copter.aparm.angle_max * flow_impact, copter.aparm.angle_max * flow_impact);

        target_angles.x += flow_angles.x + target_angles.x * flow_impact;
        target_angles.y += flow_angles.y + target_angles.y * flow_impact;
    }
}
#endif // AP_OPTICALFLOW_ENABLED
#endif // MODE_GUIDED_NOGPS_ENABLED