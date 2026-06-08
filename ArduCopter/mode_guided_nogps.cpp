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
    AP_GROUPINFO("_YAW_RATE", 1, ModeGuidedNoGPS, yaw_rate, 2),

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
    // @Range: 0.01 6.0
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _XY_I
    // @DisplayName: GuidedNoGPS I gain
    // @Description: GuidedNoGPS (horizontal) I gain
    // @Range: 0.01 1.00
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

    // @Param: _FLOW_ERMP
    // @DisplayName: GuidedNoGPS Flow error multiplier
    // @Description: Optical flow error multiplier
    // @Range: 0.0 1.0
    // @User: Standard
    AP_GROUPINFO("_FLOW_ERMP", 9, ModeGuidedNoGPS, flow_error_multiplier, 0.1f),
#endif

    // @Param: _HOME_YAW_CH
    // @DisplayName: GuidedNoGPS home yaw source channel
    // @Description: Home yaw source channel for HOME state
    // @Range: 0 16
    // @User: Standard
    AP_GROUPINFO("_HOME_YAW_CH", 10, ModeGuidedNoGPS, home_yaw_channel, 0),

    // @Param: _ALT_CH
    // @DisplayName: GuidedNoGPS Altitude Channel
    // @Description: Altitude source channel
    // @Range: 0 16
    // @User: Standard
    AP_GROUPINFO("_ALT_CH", 11, ModeGuidedNoGPS, altitude_channel, 0),

    // @Param: _RC_TYPE
    // @DisplayName: GuidedNoGPS RC input type
    // @Description: How the home-yaw and altitude RC channels change DR_HOME_YAW / RTL_ALT. 0: absolute, stick position maps directly to the value. 1: incremental, stick deflection ramps the value up/down over time (faster towards the stick extremes, nothing within the deadzone).
    // @Values: 0:Absolute,1:Incremental
    // @User: Standard
    AP_GROUPINFO("_RC_TYPE", 12, ModeGuidedNoGPS, rc_input_type, 0),

    // @Param: _RC_DZ
    // @DisplayName: GuidedNoGPS RC incremental deadzone
    // @Description: Normalised neutral deadzone for incremental RC input. While the stick stays within this fraction of centre the value is not changed.
    // @Range: 0.0 0.95
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("_RC_DZ", 13, ModeGuidedNoGPS, rc_deadzone, 0.1f),

    // @Param: _RC_YSPD
    // @DisplayName: GuidedNoGPS incremental yaw speed
    // @Description: Maximum DR_HOME_YAW change rate at full stick deflection in incremental mode.
    // @Range: 1 180
    // @Units: deg/s
    // @User: Standard
    AP_GROUPINFO("_RC_YSPD", 14, ModeGuidedNoGPS, rc_yaw_speed, 45.0f),

    // @Param: _RC_ASPD
    // @DisplayName: GuidedNoGPS incremental altitude speed
    // @Description: Maximum RTL_ALT change rate at full stick deflection in incremental mode.
    // @Range: 0.1 20
    // @Units: m/s
    // @User: Standard
    AP_GROUPINFO("_RC_ASPD", 15, ModeGuidedNoGPS, rc_alt_speed, 5.0f),

    AP_GROUPEND
};

ModeGuidedNoGPS::ModeGuidedNoGPS(void) : ModeGuided()
{
    AP_Param::setup_object_defaults(this, var_info);
}

float ModeGuidedNoGPS::normalize_angle_deg(float angle)
{
    return fmod(fmod(angle, 360.0f) + 360.0f, 360.0f);
}

float ModeGuidedNoGPS::get_yaw_error()
{
    return fmod(normalize_angle_deg(home_yaw - degrees(copter.ahrs.get_yaw())), 180);
}

float ModeGuidedNoGPS::get_target_yaw_rate(float yaw_error)
{
    // Calculate the yaw rate
    float target_rate = yaw_rate * 1000 * max(0.1f, min(1.0f, abs(yaw_error) / 20));

    if (yaw_error > 90 && target_rate > 0) {
        target_rate = -target_rate;
    }

    return target_rate;
}

void ModeGuidedNoGPS::read_rc()
{
    if (!rc().has_valid_input()) {
        return;
    }

    if (rc_input_type.get() == (int8_t)RCInputType::INCREMENTAL) {
        read_rc_incremental();
    } else {
        read_rc_absolute();
    }
}

// Absolute mapping: stick position maps directly to the parameter value.
void ModeGuidedNoGPS::read_rc_absolute()
{
    // Home yaw
    uint8_t home_yaw_ch = home_yaw_channel.get();
    if (home_yaw_ch > 0) {
        RC_Channel* channel = RC_Channels::rc_channel(home_yaw_ch - 1);
        if (channel != nullptr) {
            const uint16_t yaw = 360.0f * ((channel->norm_input_dz() + 1.0f) / 2.0f);
            AP_Param::set_and_save_by_name_ifchanged("dr_home_yaw", yaw);
        }
    }

    // Altitude
    uint8_t alt_ch = altitude_channel.get();
    if (alt_ch > 0) {
        RC_Channel* channel = RC_Channels::rc_channel(alt_ch - 1);
        if (channel != nullptr) {
            const uint16_t altitude = 200.0f * ((channel->norm_input_dz() + 1.0f) / 2.0f);
            AP_Param::set_and_save_by_name_ifchanged("rtl_alt", altitude * 100);
        }
    }
}

// Signed increment factor in [-1..1] for the given channel.
// Returns 0 inside the deadzone; magnitude grows linearly from the deadzone
// edge towards the stick extreme (proportional speed profile).
float ModeGuidedNoGPS::rc_increment_factor(const RC_Channel* channel) const
{
    const float n = channel->norm_input();          // [-1..1], centred on trim
    const float dz = constrain_float(rc_deadzone.get(), 0.0f, 0.95f);

    if (fabsf(n) <= dz) {
        return 0.0f;
    }

    const float sign = (n > 0.0f) ? 1.0f : -1.0f;
    return sign * (fabsf(n) - dz) / (1.0f - dz);    // rescale to 0..1 beyond deadzone
}

// Incremental mapping: stick deflection ramps the value up/down over time.
void ModeGuidedNoGPS::read_rc_incremental()
{
    const float dt = 0.01f;     // read_rc() is called from rc_loop() at 100Hz

    // Home yaw (degrees, wrapped to 0..360)
    uint8_t home_yaw_ch = home_yaw_channel.get();
    if (home_yaw_ch > 0) {
        RC_Channel* channel = RC_Channels::rc_channel(home_yaw_ch - 1);
        if (channel != nullptr) {
            const float factor = rc_increment_factor(channel);
            if (is_zero(factor)) {
                yaw_increment_remainder = 0.0f;
                // stick back at centre: persist the dialled-in value once
                if (yaw_pending_save) {
                    g.dr_home_yaw.save();
                    yaw_pending_save = false;
                }
            } else {
                yaw_increment_remainder += factor * rc_yaw_speed.get() * dt;
                if (fabsf(yaw_increment_remainder) >= 1.0f) {
                    const int32_t step = (int32_t)yaw_increment_remainder;   // truncate toward zero
                    int32_t newval = (int32_t)g.dr_home_yaw + step;
                    newval = constrain_int32(newval, 0, 360);
                    g.dr_home_yaw.set(newval);   // RAM only: visible on OSD, no flash write
                    yaw_increment_remainder -= step;
                    yaw_pending_save = true;
                }
            }
        }
    }

    // Altitude (stored in centimetres, 0..200m)
    uint8_t alt_ch = altitude_channel.get();
    if (alt_ch > 0) {
        RC_Channel* channel = RC_Channels::rc_channel(alt_ch - 1);
        if (channel != nullptr) {
            const float factor = rc_increment_factor(channel);
            if (is_zero(factor)) {
                alt_increment_remainder = 0.0f;
                // stick back at centre: persist the dialled-in value once
                if (alt_pending_save) {
                    g.rtl_altitude.save();
                    alt_pending_save = false;
                }
            } else {
                alt_increment_remainder += factor * rc_alt_speed.get() * 100.0f * dt;
                if (fabsf(alt_increment_remainder) >= 1.0f) {
                    const int32_t step = (int32_t)alt_increment_remainder;   // truncate toward zero
                    int32_t newval = (int32_t)g.rtl_altitude + step;
                    newval = constrain_int32(newval, 0, 200 * 100);
                    g.rtl_altitude.set(newval);   // RAM only: visible on OSD, no flash write
                    alt_increment_remainder -= step;
                }
            }
        }
    }
}

bool ModeGuidedNoGPS::adjust_altitude()
{
    // Calculate the current altitude below home
    float curr_alt_below_home = 0.0f;
    copter.ahrs.get_relative_position_D_home(curr_alt_below_home);

    // Calculate the target altitude above the vehicle
    float target_alt_above_vehicle = fly_alt_min + curr_alt_below_home;

    copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    float target_climb_rate = 0;

    if (target_alt_above_vehicle > 0.5f) {
        target_climb_rate = constrain_float(climb_rate * 100, -get_pilot_speed_dn(), g.pilot_speed_up);
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);
    }

    pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);

    if (target_climb_rate > 0) {
        return false;
    }

    return true;
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
    flow_pi_xy.set_dt(1.0 / copter.scheduler.get_loop_rate_hz() * flow_filter_samples);

    flow_samples_count = 0;
    flow_error.zero();
    flow_error_buff.zero();
#endif

    // Information message
    gcs().send_text(MAV_SEVERITY_INFO, "DR Start");

    return true;
}

// Run the guided_nogps controller logic
void ModeGuidedNoGPS::run()
{
    switch (_state) {
        case State::YAW:
            yaw_run();
            break;

        case State::ALT:
            alt_run();
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
    float error = get_yaw_error();
    float rate = get_target_yaw_rate(error);

    if (abs(error) < 5.0f) {
        copter.attitude_control->get_rate_yaw_pid().reset_filter();
        copter.attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0);
        _state = State::ALT;
        return;
    }

    copter.attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, rate);
}

void ModeGuidedNoGPS::alt_run()
{
    bool altitude_reached = adjust_altitude();

    Vector2f angles = Vector2f(0, 0);
    
#if AP_OPTICALFLOW_ENABLED
    optflow_correction(angles);
#endif // AP_OPTICALFLOW_ENABLED

    angles.x = constrain_float(angles.x, -copter.aparm.angle_max, copter.aparm.angle_max);
    angles.y = constrain_float(angles.y, -copter.aparm.angle_max, copter.aparm.angle_max);

    copter.attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(angles.x, angles.y, 0);
    
    if(altitude_reached) {
        _state = State::FLY;

        // Reset optical flow error
        flow_samples_count = 0;
        flow_error.zero();
        flow_error_buff.zero();
    }
}

void ModeGuidedNoGPS::fly_run()
{
    adjust_altitude();

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

    // Maybe apply yaw correction
    float yaw_error = get_yaw_error();

    // call attitude controller
    copter.attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(
        bf_angles.x,
        bf_angles.y,
        yaw_error > 0.5f ? get_target_yaw_rate(yaw_error) : 0
    );
}

#if AP_OPTICALFLOW_ENABLED
void ModeGuidedNoGPS::optflow_correction(Vector2f& target_angles)
{
    if (copter.optflow.healthy()) {
        const float filter_constant = 0.95;
        quality_filtered = filter_constant * quality_filtered + (1 - filter_constant) * copter.optflow.quality();
    } else {
        quality_filtered = 0;
    }

    if (quality_filtered >= flow_min_quality) {
        // Flow correction
        Vector2f raw_flow = copter.optflow.flowRate() - copter.optflow.bodyRate();

        flow_samples_count++;
        flow_error_buff += raw_flow;

        int ffs = flow_filter_samples;

        if (flow_samples_count == ffs) {
            flow_error_buff /= ffs;
            flow_error_buff *= flow_error_multiplier;

            flow_error = flow_error_buff;

            flow_samples_count = 0;
            flow_error_buff.zero();
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