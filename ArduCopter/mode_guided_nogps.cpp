#include "Copter.h"
#include <cmath>
#include <algorithm>

using namespace std;

#if MODE_GUIDED_NOGPS_ENABLED

const AP_Param::GroupInfo ModeGuidedNoGPS::var_info[] = {
    // @Param: _YAW_RATE
    // @DisplayName: GuidedNoGPS yaw rate
    // @Description: Yaw rate for YAW state (in degrees per second)
    // @Range: 0.0 60.0
    // @User: Standard
    AP_GROUPINFO("_YAW_RATE", 1, ModeGuidedNoGPS, yaw_rate, 20),

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
    // @Description: How the home-yaw and altitude RC channels change GNGP_HOME_YAW / RTL_ALT_M. 0: absolute, stick position maps directly to the value. 1: incremental, stick deflection ramps the value up/down over time (faster towards the stick extremes, nothing within the deadzone).
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
    // @Description: Maximum GNGP_HOME_YAW change rate at full stick deflection in incremental mode.
    // @Range: 1 180
    // @Units: deg/s
    // @User: Standard
    AP_GROUPINFO("_RC_YSPD", 14, ModeGuidedNoGPS, rc_yaw_speed, 45.0f),

    // @Param: _RC_ASPD
    // @DisplayName: GuidedNoGPS incremental altitude speed
    // @Description: Maximum RTL_ALT_M change rate at full stick deflection in incremental mode.
    // @Range: 0.1 20
    // @Units: m/s
    // @User: Standard
    AP_GROUPINFO("_RC_ASPD", 15, ModeGuidedNoGPS, rc_alt_speed, 5.0f),

    // @Param: _HOME_YAW
    // @DisplayName: GuidedNoGPS home yaw
    // @Description: Return azimuth to home in degrees. Values below 1 mean automatic: the bearing to home captured while GPS was still healthy is used.
    // @Range: 0 360
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("_HOME_YAW", 16, ModeGuidedNoGPS, dr_home_yaw, DR_HOME_YAW_DEFAULT),

    // @Param: _ANGLE
    // @DisplayName: GuidedNoGPS FLY state lean angle
    // @Description: Maximum lean angle towards home in the FLY state. Zero means use ANGLE_MAX. Regardless of this value the lean angle is also reduced whenever the altitude controller runs short of thrust headroom, so altitude is held in preference to speed.
    // @Range: 0 45
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("_ANGLE", 17, ModeGuidedNoGPS, fly_angle, 0),

    // @Param: _HDG_MODE
    // @DisplayName: GuidedNoGPS heading mode
    // @Description: Heading behaviour while returning. 0: keep the heading the vehicle had when the mode engaged and crab home without turning. 1: turn nose towards the home azimuth (legacy behaviour). 2: turn tail towards home (nose away), e.g. for a rear-facing directional antenna.
    // @Values: 0:HoldHeading,1:NoseToHome,2:TailToHome
    // @User: Standard
    AP_GROUPINFO("_HDG_MODE", 18, ModeGuidedNoGPS, hdg_mode, (int8_t)HeadingMode::NOSE_TO_HOME),

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

// signed shortest-path yaw error to the desired heading in degrees, [-180..180)
float ModeGuidedNoGPS::get_yaw_error()
{
    return wrap_180(target_heading - degrees(copter.ahrs.get_yaw()));
}

float ModeGuidedNoGPS::get_target_yaw_rate(float yaw_error)
{
    // proportional rate in rad/s: full speed above 20 deg of error, 10% floor
    float target_rate = radians(yaw_rate * constrain_float(fabsf(yaw_error) / 20, 0.1f, 1.0f));

    if (yaw_error < 0) {
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
            dr_home_yaw.set_and_save_ifchanged(yaw);
        }
    }

    // Altitude
    uint8_t alt_ch = altitude_channel.get();
    if (alt_ch > 0) {
        RC_Channel* channel = RC_Channels::rc_channel(alt_ch - 1);
        if (channel != nullptr) {
            const float altitude_m = 200.0f * ((channel->norm_input_dz() + 1.0f) / 2.0f);
            copter.mode_rtl.set_and_save_altitude_m_ifchanged(altitude_m);
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
                    dr_home_yaw.save();
                    yaw_pending_save = false;
                }
            } else {
                yaw_increment_remainder += factor * rc_yaw_speed.get() * dt;
                if (fabsf(yaw_increment_remainder) >= 1.0f) {
                    const int32_t step = (int32_t)yaw_increment_remainder;   // truncate toward zero
                    int32_t newval = (int32_t)dr_home_yaw + step;
                    newval = constrain_int32(newval, 0, 360);
                    dr_home_yaw.set(newval);   // RAM only: visible on OSD, no flash write
                    yaw_increment_remainder -= step;
                    yaw_pending_save = true;
                }
            }
        }
    }

    // Altitude (RTL_ALT_M, metres, 0..200m)
    uint8_t alt_ch = altitude_channel.get();
    if (alt_ch > 0) {
        RC_Channel* channel = RC_Channels::rc_channel(alt_ch - 1);
        if (channel != nullptr) {
            const float factor = rc_increment_factor(channel);
            if (is_zero(factor)) {
                // stick back at centre: persist the dialled-in value once
                if (alt_pending_save) {
                    copter.mode_rtl.save_altitude_m();
                    alt_pending_save = false;
                }
            } else {
                const float newval = constrain_float(
                    copter.mode_rtl.get_altitude_m() + factor * rc_alt_speed.get() * dt, 0.0f, 200.0f);
                copter.mode_rtl.set_altitude_m(newval);   // RAM only: visible on OSD, no flash write
                alt_pending_save = true;
            }
        }
    }
}

bool ModeGuidedNoGPS::adjust_altitude()
{
    // Calculate the current altitude below home
    float curr_alt_below_home = 0.0f;
    copter.ahrs.get_relative_position_D_home(curr_alt_below_home);

    copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // Command the climb against the position-controller TARGET, not the vehicle:
    // if the vehicle lags behind (headwind, thrust limit, baro offset at speed) a
    // vehicle-based check keeps integrating the target upwards, inflating the
    // position error and with it the throttle. AltHold holds a fixed target and
    // sits at hover throttle; once the target reaches fly_alt_min this does too.
    const float target_above_home_m = pos_control->get_pos_target_U_m()
                                      - pos_control->get_pos_estimate_U_m()
                                      - curr_alt_below_home;

    // proportional approach to the return altitude, capped at GNGP_CLMB_RATE
    float target_climb_rate_ms = constrain_float(fly_alt_min - target_above_home_m, 0.0f,
                                                 constrain_float(climb_rate, 0.0f, get_pilot_speed_up_ms()));
    if (target_climb_rate_ms > 0) {
        target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms);
    }

    pos_control->D_set_pos_target_from_climb_rate_ms(target_climb_rate_ms);

    // reached once the vehicle itself is within 0.5 m of the return altitude
    return (fly_alt_min + curr_alt_below_home) <= 0.5f;
}

// Initialize the guided_nogps controller
bool ModeGuidedNoGPS::init(bool ignore_checks)
{
    // initialise the vertical position controller
    if (!pos_control->D_is_active()) {
        pos_control->D_init_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->D_set_max_speed_accel_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_D_mss());
    pos_control->D_set_correction_speed_accel_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_D_mss());

    // Minimum height and ground track to home
    fly_alt_min = copter.mode_rtl.get_altitude_m(); // minimum height above the home
    home_yaw = normalize_angle_deg(dr_home_yaw < 1 ? copter.azimuth_to_home : static_cast<float>(dr_home_yaw));

    // desired heading while returning; the ground track stays home_yaw in all
    // modes - the lean vector and the flow lateral projection are heading-independent
    switch ((HeadingMode)hdg_mode.get()) {
    case HeadingMode::HOLD_HEADING:
        // keep the heading we entered the mode with and crab home without turning
        target_heading = normalize_angle_deg(degrees(copter.ahrs.get_yaw()));
        break;
    case HeadingMode::TAIL_TO_HOME:
        target_heading = normalize_angle_deg(home_yaw + 180.0f);
        break;
    case HeadingMode::NOSE_TO_HOME:
    default:
        target_heading = home_yaw;
        break;
    }

    _state = State::YAW;

#ifdef AP_OPTICALFLOW_ENABLED
    flow_filter.set_cutoff_frequency(copter.scheduler.get_loop_rate_hz(), flow_filter_hz);

    flow_pi_xy.reset_I();
    // the PI controller runs every loop iteration (optflow_correction), even
    // though the averaged flow input only refreshes every flow_filter_samples
    // loops, so dt must be a single loop period
    flow_pi_xy.set_dt(1.0f / copter.scheduler.get_loop_rate_hz());

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
    pos_control->D_update_controller();
}

void ModeGuidedNoGPS::yaw_run()
{
    // Calculate the yaw error
    float error = get_yaw_error();
    float rate = get_target_yaw_rate(error);

    if (fabsf(error) < 5.0f) {
        copter.attitude_control->get_rate_yaw_pid().reset_filter();
        copter.attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(0, 0, 0);
        _state = State::ALT;
        return;
    }

    copter.attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(0, 0, rate);
}

void ModeGuidedNoGPS::alt_run()
{
    bool altitude_reached = adjust_altitude();

    // respect the altitude controller's thrust-headroom limit so flow
    // corrections cannot tilt away the thrust needed to climb
    const float angle_max_rad = MIN(copter.attitude_control->lean_angle_max_rad(),
                                    copter.attitude_control->get_althold_lean_angle_max_rad());
    Vector2f angles_rad;

#if AP_OPTICALFLOW_ENABLED
    optflow_correction(angles_rad, false);
#endif // AP_OPTICALFLOW_ENABLED

    angles_rad.x = constrain_float(angles_rad.x, -angle_max_rad, angle_max_rad);
    angles_rad.y = constrain_float(angles_rad.y, -angle_max_rad, angle_max_rad);

    copter.attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(angles_rad.x, angles_rad.y, 0);
    
    if(altitude_reached) {
        _state = State::FLY;

#if AP_OPTICALFLOW_ENABLED
        // Reset optical flow error
        flow_samples_count = 0;
        flow_error.zero();
        flow_error_buff.zero();

        // drop the position-hold integrator: in FLY the PI input is projected to
        // the cross-track axis, so an along-track I component accumulated here
        // would freeze in place and permanently brake the flight home
        flow_pi_xy.reset_I();
#endif
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

    // lean limit: ANGLE_MAX, reduced by the altitude controller's thrust-headroom
    // limit (hold altitude in preference to speed home on low-margin airframes)
    // and by the optional GNGP_ANGLE cruise cap
    float angle_max_rad = MIN(copter.attitude_control->lean_angle_max_rad(),
                              copter.attitude_control->get_althold_lean_angle_max_rad());
    if (is_positive(fly_angle.get())) {
        angle_max_rad = MIN(angle_max_rad, radians(fly_angle.get()));
    }
    bf_angles = bf_angles.normalized() * angle_max_rad;

#if AP_OPTICALFLOW_ENABLED
    optflow_correction(bf_angles, true);
#endif // AP_OPTICALFLOW_ENABLED

    // renormalise the base+correction sum to the full lean budget: the flow
    // correction steers the lean DIRECTION while the magnitude stays fixed.
    // Per-axis clamping of the sum rotated the command arbitrarily and starved
    // the lateral correction whenever the base lean already sat at the limit
    // (always the case while crabbing in HOLD_HEADING mode).
    if (!bf_angles.is_zero()) {
        bf_angles = bf_angles.normalized() * angle_max_rad;
    }

    // Maybe apply yaw correction
    float yaw_error = get_yaw_error();

    // call attitude controller
    copter.attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(
        bf_angles.x,
        bf_angles.y,
        fabsf(yaw_error) > 0.5f ? get_target_yaw_rate(yaw_error) : 0
    );
}

#if AP_OPTICALFLOW_ENABLED
void ModeGuidedNoGPS::optflow_correction(Vector2f& target_angles, bool lateral_only)
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
        const float height_m = pos_control->get_pos_estimate_U_m();

        // compensate for height, this converts to (approx) m/s
        sensor_flow *= constrain_float(height_m, height_min, height_max);

        // flow axes are velocity rotated 90 deg: flow = (-v_by, +v_bx)/range
        const Vector2f vel_bf(sensor_flow.y, -sensor_flow.x);

        // Reconstruct horizontal earth-frame velocity with the full attitude,
        // assuming near-level flight (vD ~ 0). A yaw-only rotation leaks the
        // along-track speed into the lateral axis at cruise pitch
        // (v_by = v_lat*cos(roll) + v_along*sin(pitch)*sin(roll)), so the PI
        // trimmed to a constant lateral drift instead of zero.
        const Matrix3f &rot = copter.ahrs.get_rotation_body_to_ned();
        const float det = rot.a.x * rot.b.y - rot.b.x * rot.a.y;
        Vector2f vel_ef;
        if (fabsf(det) > 0.1f) {
            vel_ef.x = ( rot.b.y * vel_bf.x - rot.b.x * vel_bf.y) / det;
            vel_ef.y = (-rot.a.y * vel_bf.x + rot.a.x * vel_bf.y) / det;
        } else {
            vel_ef = copter.ahrs.body_to_earth2D(vel_bf);
        }

        // back to the flow/angle-space convention used by the PI pipeline
        Vector2f input_ef(-vel_ef.y, vel_ef.x);

        if (lateral_only) {
            // strip the along-track (towards home_yaw) component before the PI so
            // the integrator never accumulates the deliberate flight speed home
            // and only damps cross-track drift. Flow vectors live in angle space
            // (rotated 90 degrees from motion), hence (sin, -cos) for the track.
            const float track_rad = radians(home_yaw);
            const Vector2f track_dir{sinf(track_rad), -cosf(track_rad)};
            input_ef -= track_dir * (input_ef * track_dir);
        }

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
        const float angle_max_rad = copter.attitude_control->lean_angle_max_rad();

        ef_output = flow_pi_xy.get_p();
        ef_output += xy_I;
        ef_output *= angle_max_rad;

#ifdef HAL_LOGGING_ENABLED
        copter.Log_Write_Optflow_PI(flow_pi_xy.get_p(), xy_I);
#endif // HAL_LOGGING_ENABLED

        Vector2f flow_angles;

        // convert to body frame
        flow_angles += copter.ahrs.earth_to_body2D(ef_output);

        // anti-windup: freeze the integrator only when the CORRECTION saturates
        // its own authority clamp. Judging by the combined base+correction vector
        // stalled the I term whenever the base lean already sat at ANGLE_MAX
        // (always true while crabbing), leaving cross-track drift uncorrected.
        const float corr_limit = angle_max_rad * flow_impact;
        limited = fabsf(flow_angles.x) > corr_limit || fabsf(flow_angles.y) > corr_limit;

        // constrain to angle limit
        flow_angles.x = constrain_float(flow_angles.x, -corr_limit, corr_limit);
        flow_angles.y = constrain_float(flow_angles.y, -corr_limit, corr_limit);

        target_angles += flow_angles;
    }
}
#endif // AP_OPTICALFLOW_ENABLED
#endif // MODE_GUIDED_NOGPS_ENABLED