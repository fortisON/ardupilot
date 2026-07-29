# GUIDED_NOGPS — custom GPS-less return-to-home mode (Dead Reckoning)

Custom implementation of the `GUIDED_NOGPS` mode (class `ModeGuidedNoGPS`,
`ArduCopter/mode_guided_nogps.cpp`). It inherits from `ModeGuided` but does not
use any guided logic — instead it runs its own state machine
**YAW → ALT → FLY**: turn to the home azimuth, climb, then fly towards home at
maximum tilt while stabilizing with optical flow. On mode entry the message
`DR Start` is sent to the GCS.

The mode does not require GPS (`requires_position() = false`), throttle is
automatic, and it is treated as an autopilot mode.

---

## 1. How the mode is entered and exited

The mode is designed for automatic entry from failsafes:

1. **Fallback instead of LAND** — `Copter::set_mode_RTL_or_land_with_pause()`
   (`events.cpp`) is modified: if switching to RTL fails (no position
   estimate), `set_mode_guided_nogps()` is called instead of landing. This
   path activates the mode from radio/battery/deadreckon failsafes whose
   action is RTL.
2. **EKF failsafe** (`ekf_check.cpp`): with `FS_EKF_ACTION = 1` (AltHold), if
   the radio failsafe is active at the same time, GUIDED_NOGPS is engaged
   instead of ALT_HOLD.
3. **Deadreckon failsafe** (`FS_DR_ENABLE`, default action RTL) — on EKF
   position loss the RTL action falls through to GUIDED_NOGPS via item 1.

**Automatic exit**: in the 1 Hz loop (`Copter.cpp`, `one_hz_loop`) — if the
vehicle is in GUIDED_NOGPS, the radio failsafe is active and GPS HDOP has
recovered (`hdop <= GPS_HDOP_GOOD`), the vehicle switches to RTL
(`set_mode_RTL_or_land_with_pause`, reason `EKF_FAILSAFE_RECOVERY`).
So when GPS recovers in flight, the copter transitions to a normal RTL on its
own.

---

## 2. State machine

### 2.1 YAW — turning to the return azimuth

* Heading error: `get_yaw_error() = wrap_180(home_yaw − yaw)` — signed
  shortest-path error in the range [−180°; 180°).
* Turn rate: `GNGP_YAW_RATE` deg/s (default 20), scaled proportionally to the
  error: full speed at |error| ≥ 20°, below that it slows down linearly with a
  10 % floor. The rate sign equals the error sign (always turning the short
  way).
* When |error| < 5° — the yaw rate PID filter is reset, the command is zeroed
  and the state advances to **ALT**.

During YAW the vertical controller is already running
(`D_update_controller()` is called in `run()` for all states).

### 2.2 ALT — climbing

`adjust_altitude()`:
* Current altitude relative to home is taken from AHRS
  (`get_relative_position_D_home`).
* Target altitude = `RTL_ALT_M` (metres) **above home** (`fly_alt_min`).
* While more than 0.5 m short, the vehicle climbs at `GNGP_CLMB_RATE` (m/s),
  limited by `PILOT_SPEED_UP`/`PILOT_SPEED_DN` and avoidance.
* Descent is never commanded: if the vehicle is already above `RTL_ALT_M`, the
  climb rate is 0 and the current altitude is simply held.

Horizontal angles in ALT: base roll/pitch = 0 plus the optical flow correction
(see §4), constrained to ±`ANGLE_MAX`.

Once the altitude is reached, the state advances to **FLY** and the optical
flow error accumulators are reset.

### 2.3 FLY — flying home

* `adjust_altitude()` keeps running (if the altitude sags, the vehicle climbs
  back).
* Home azimuth in the body frame: `body_to_home = home_yaw − current_yaw`;
  the tilt vector is `(sin, −cos)` of that angle (x → roll, y → pitch: with
  home straight ahead this yields pure forward pitch).
* The vector is normalized to the **maximum** tilt angle `ANGLE_MAX`
  (centidegrees) — the copter flies home at maximum tilt.
* The optical flow correction (§4) is applied to the vector; the result is
  constrained to ±`ANGLE_MAX` per axis.
* Heading keeps tracking home: if |yaw error| > 0.5°, the same turn rate as in
  the YAW state is commanded.

---

## 3. Selecting the return azimuth (`home_yaw`)

Computed once on mode entry (`init()`):

```
home_yaw = (GNGP_HOME_YAW < 1) ? copter.azimuth_to_home : GNGP_HOME_YAW
```

Two sources:

1. **Automatic** (`GNGP_HOME_YAW = 0`): the variable `copter.azimuth_to_home` is
   continuously updated in `failsafe_deadreckon_check()` (`events.cpp`) while
   the EKF is **not** dead reckoning and `HDOP ≤ GPS_HDOP_GOOD`:
   `bearing(current position → home)`. I.e. the last "healthy" bearing to home
   captured before GPS loss is used.
2. **Manual** (`GNGP_HOME_YAW ≥ 1`): the parameter value in degrees (0–360), set
   from the ground, via OSD, or from an RC channel (see §5).

⚠️ `GNGP_HOME_YAW` values from 0 to 0.99 are treated as "auto", so a heading of
true north must be set as `360`.

---

## 4. Optical flow correction (`optflow_correction`)

Active in the ALT and FLY states (compiled with `AP_OPTICALFLOW_ENABLED`).
The idea comes from ModeFlowHold: convert the measured flow (≈ horizontal
velocity) into tilt angles that cancel drift.

Processing pipeline (every main loop cycle):

1. **Quality**: `quality_filtered` — IIR filter (constant 0.95) of
   `optflow.quality()`; zeroed when the sensor is unhealthy. The correction is
   applied only when `quality_filtered ≥ GNGP_QUAL_MIN`.
2. **Raw flow**: `raw_flow = flowRate − bodyRate` (rad/s, gyro rotation
   compensation).
3. **Sample averaging**: `GNGP_FLOW_SMPL` samples are accumulated (default
   15), the average is multiplied by `GNGP_FLOW_ERMP` (0.1) and becomes the
   new `flow_error`. So the controller input updates once every N cycles (at
   400 Hz and SMPL=15 — every ~37.5 ms); the PI controller `dt` is set
   accordingly (`loop_dt * GNGP_FLOW_SMPL`).
4. **Limiting**: `flow_error` is clamped to ±`GNGP_FLOW_MAX` (protection
   against oscillations at low altitude).
5. **Low-pass filter**: LowPass at `GNGP_FILT_HZ` (5 Hz).
6. **Height scaling**: multiplied by the position controller height estimate
   (`get_pos_estimate_U_m`) constrained to
   [0.1 m; 200 m] — converting angular flow to ≈ m/s.
7. **PI controller** `AC_PI_2D` (input rotated to the earth frame):
   * P = `GNGP_XY_P`, I = `GNGP_XY_I`, IMAX = `GNGP_XY_IMAX` (centidegrees),
     input filter `GNGP_XY_FILT_HZ`.
   * **Anti-windup**: if the final angles hit `ANGLE_MAX` on the previous
     cycle, the integrator is only allowed to shrink (`get_i_shrink()`).
   * Output `(P + I) × ANGLE_MAX` → back to the body frame.
8. **Contribution limit**: the correction is clamped to
   ±`ANGLE_MAX × GNGP_FLOW_IMP` per axis.
9. **Mixing with the target angles**:

   ```
   target = target × (1 + GNGP_FLOW_IMP) + flow_correction
   ```

   after which the caller constrains the result to ±`ANGLE_MAX`.

The point of the mixing: the base "towards home" vector is pre-boosted by
`(1 + FLOW_IMP)`, so on the saturated axis (tilt towards home = `ANGLE_MAX`)
the correction cannot "eat into" the pull towards home — after clamping it
still ends up at `ANGLE_MAX`. On the lateral axis (base angle ≈ 0) the
correction works at full strength and cancels sideways drift. In the ALT state
(base = 0) the correction is fully responsible for position hold.

**Logging**: the P and I terms are written to the log as message **`GOPI`**
(`TimeUS, flow_p_x, flow_p_y, flow_i_x, flow_i_y`).

---

## 5. Controlling GNGP_HOME_YAW and RTL_ALT_M from the transmitter

`ModeGuidedNoGPS::read_rc()` is called from `Copter::rc_loop()` at **100 Hz
permanently**, regardless of the current flight mode (the transmitter can
adjust the parameters at any time; the values are visible in the OSD widgets).
With invalid RC input it does nothing.

Channels: `GNGP_HOME_YAW_CH` and `GNGP_ALT_CH` (1–16, 0 = disabled).

### 5.1 Absolute mode (`GNGP_RC_TYPE = 0`, default)

The stick position maps directly to the value:

* yaw: `360 × (norm_input_dz + 1) / 2` → `GNGP_HOME_YAW` = 0–360°;
* altitude: `200 × (norm_input_dz + 1) / 2` m → `RTL_ALT_M` = 0–200 m.

Written via `set_and_save_by_name_ifchanged` — every change is immediately
saved to EEPROM (a noisy channel may wear the flash — fine for a knob/slider,
prefer INCREMENTAL for a non-latching stick).

### 5.2 Incremental mode (`GNGP_RC_TYPE = 1`)

Stick deflection sets the **rate of change** of the value:

* Inside the deadzone `GNGP_RC_DZ` (normalized, default 0.1) the value does
  not change.
* Beyond it the rate grows linearly from 0 to the maximum at full deflection:
  `GNGP_RC_YSPD` deg/s for yaw (default 45), `GNGP_RC_ASPD` m/s for altitude
  (default 5).
* Yaw accumulates fractional increments and is applied in whole degrees;
  altitude changes smoothly. Limits: yaw 0–360°, altitude 0–200 m.
* While the stick is deflected, the parameter changes **in RAM only** (`set()`
  — the value is visible on the OSD, no flash writes); when the stick returns
  to centre the value is saved to EEPROM once. This applies to both
  `GNGP_HOME_YAW` and `RTL_ALT_M`.

---

## 6. Parameters

### GNGP_ group (the mode's own parameters, `Parameters.cpp`, subgroup 60)

| Parameter | Default | Description |
|---|---|---|
| `GNGP_YAW_RATE` | 20 | Turn rate in the YAW state, deg/s; slows down linearly below 20° of error (10 % floor). |
| `GNGP_CLMB_RATE` | 3 | Climb rate (m/s) in ALT/FLY, limited by `PILOT_SPEED_UP/_DN`. |
| `GNGP_XY_P` | 0.2 | Optical flow P gain (AC_PI_2D). |
| `GNGP_XY_I` | 0.8 | Optical flow I gain. |
| `GNGP_XY_IMAX` | 3000 | Integrator limit, centidegrees. |
| `GNGP_XY_FILT_HZ` | 5 | PI controller input filter, Hz. |
| `GNGP_FLOW_MAX` | 0.6 | Averaged flow limit, rad/s (anti-oscillation near the ground). |
| `GNGP_FILT_HZ` | 5 | Flow low-pass filter frequency, Hz. |
| `GNGP_QUAL_MIN` | 10 | Minimum filtered flow quality for the correction to apply (0–255). |
| `GNGP_FLOW_IMP` | 0.5 | Flow "impact": correction limit = ±`ANGLE_MAX × IMP`; the base vector is boosted by `(1+IMP)` before mixing. |
| `GNGP_FLOW_SMPL` | 15 | Number of flow averaging samples. |
| `GNGP_FLOW_ERMP` | 0.1 | Averaged flow error multiplier. |
| `GNGP_HOME_YAW_CH` | 0 | RC channel for `GNGP_HOME_YAW` (0 = off). |
| `GNGP_ALT_CH` | 0 | RC channel for `RTL_ALT_M` (0 = off). |
| `GNGP_RC_TYPE` | 0 | 0 = absolute stick mapping, 1 = incremental. |
| `GNGP_RC_DZ` | 0.1 | Incremental mode deadzone (0–0.95). |
| `GNGP_RC_YSPD` | 45 | Max `GNGP_HOME_YAW` change rate, deg/s. |
| `GNGP_RC_ASPD` | 5 | Max `RTL_ALT_M` change rate, m/s. |
| `GNGP_HOME_YAW` | 0 | Return azimuth to home, deg (0–360). Values < 1 = automatic bearing to home (§3). Formerly the global `DR_HOME_YAW`; a stored old value is converted automatically on boot. |

### Related external parameters

| Parameter | Role in the mode |
|---|---|
| `RTL_ALT_M` | Target altitude above home (m) for the ALT state; no descent. Formerly `RTL_ALT` in cm — auto-converted on boot. |
| `GPS_HDOP_GOOD` | HDOP threshold: while GPS is "good", `azimuth_to_home` is updated; HDOP recovery in flight → automatic switch to RTL. |
| `ANGLE_MAX` | Maximum tilt: the copter flies home with it; also the PI output scale and the flow correction limit. |
| `PILOT_SPEED_UP` / `PILOT_SPEED_DN` | Vertical speed limits in `adjust_altitude()`. |
| `FS_EKF_ACTION` | =1 (AltHold): EKF failsafe + radio failsafe → enter GUIDED_NOGPS. |
| `FS_DR_ENABLE` / `FS_DR_TIMEOUT` | Deadreckon failsafe; its RTL action falls back to GUIDED_NOGPS when RTL is unavailable. |
| `PSC_*`, vertical PIDs | The stock `AC_PosControl` D-frame controller handles altitude (`D_update_controller()`). |

---

## 7. Fix history and remaining quirks

Fixed (2026-07-29):

1. `RTL_ALT` in the incremental RC mode was never saved to flash
   (`alt_pending_save` was never set) — it is now saved when the stick
   returns to centre, same as `GNGP_HOME_YAW`.
2. The YAW rate used a multiplier of 1000 (effectively parameter × 10 deg/s).
   `GNGP_YAW_RATE` is now true deg/s and the default was raised from 2 to 20
   so the default behaviour is unchanged.
   ⚠️ **Migration**: if an old `GNGP_YAW_RATE` value is stored in EEPROM,
   multiply it by 10 (was 2 → set 20).
3. `get_yaw_error()` via `fmod(…, 180)` collapsed a 180° error into 0 —
   replaced with signed `wrap_180`; the turn is now always the short way.
4. Removed the unused `fly_angle` field (dead code).
5. The global `DR_HOME_YAW` parameter was moved into the mode's group as
   `GNGP_HOME_YAW` so everything lives next to the mode. A value stored in
   EEPROM under the old name is converted automatically on the first boot.

2026-07-29: the fork was updated onto ArduPilot **Copter-4.7.0 stable**.
`RTL_ALT` became `RTL_ALT_M` (metres, upstream auto-conversion on boot) and the
mode was migrated to the new radian/metre controller APIs; behaviour is
unchanged.

Remaining quirks:

* In the absolute RC mode every change is written to EEPROM immediately — a
  noisy channel may wear the flash (use INCREMENTAL for non-latching sticks).
