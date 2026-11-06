#include "dusk/gyro.h"
#include "dusk/action_bindings.h"
#include "dusk/ui/ui.hpp"
#include "d/actor/d_a_alink.h"

namespace dusk::gyro {
namespace {
constexpr s32   kRollgoalTableMaxOffset = 6500;
constexpr float kGyroEmaAlphaMin = 0.05f;
constexpr float kGyroEmaAlphaMax = 1.0f;

bool  s_sensor_enabled = false;
bool  s_was_aiming     = false;
float s_smooth_gx      = 0.0f;
float s_smooth_gy      = 0.0f;
float s_smooth_gz      = 0.0f;
float s_yaw_rad        = 0.0f;
float s_pitch_rad      = 0.0f;
float s_roll_rad       = 0.0f;
s32   s_rollgoal_ax    = 0;
s32   s_rollgoal_az    = 0;

void reset_filter_state() {
    s_smooth_gx = s_smooth_gy = s_smooth_gz = 0.0f;
    s_was_aiming = false;
    s_yaw_rad = s_pitch_rad = s_roll_rad = 0.0f;
    s_rollgoal_ax = s_rollgoal_az = 0;
}

float apply_deadband(float v, float deadband_rad_s) {
    if (v > -deadband_rad_s && v < deadband_rad_s) {
        return 0.0f;
    }
    return v;
}

void disable_pad_sensors() {
    if (s_sensor_enabled) {
        PADSetSensorEnabled(PAD_CHAN0, PAD_SENSOR_GYRO, FALSE);
        s_sensor_enabled = false;
    }
}
}  // namespace

bool s_sensor_keep_alive = false;
bool get_sensor_keep_alive() { return s_sensor_keep_alive; }
void set_sensor_keep_alive(bool value) { s_sensor_keep_alive = value; }

bool rollgoal_gyro_enabled() {
    return getSettings().game.enableGyroRollgoal;
}

bool queryGyroAimContext() {
    return getSettings().game.enableGyroAim.getValue() && dCamera_c::isAimActive();
}

void recenter() {
    // Zero filter state so the next samples start from a clean neutral hold.
    s_smooth_gx = s_smooth_gy = s_smooth_gz = 0.0f;
    s_yaw_rad = s_pitch_rad = s_roll_rad = 0.0f;
    s_rollgoal_ax = s_rollgoal_az = 0;
}

void read(float dt) {
    const bool aim_active = queryGyroAimContext();
    s_was_aiming = aim_active;

    // Allow Reset Gyro outside aim (bind must still be polled this frame).
    if (getActionBindTrig(ActionBinds::RESET_GYRO, PAD_CHAN0)) {
        recenter();
    }

    if (!s_sensor_keep_alive && !aim_active) {
        disable_pad_sensors();
        reset_filter_state();
        return;
    }

    if (!s_sensor_enabled) {
        if (!PADHasSensor(PAD_CHAN0, PAD_SENSOR_GYRO)) {
            return;
        }
        if (!PADSetSensorEnabled(PAD_CHAN0, PAD_SENSOR_GYRO, TRUE)) {
            return;
        }
        s_sensor_enabled = true;
    }

    f32 gyro[3];
    if (!PADGetSensorData(PAD_CHAN0, PAD_SENSOR_GYRO, gyro, 3)) {
        return;
    }

    const float smooth_alpha = kGyroEmaAlphaMax + getSettings().game.gyroSmoothing * (kGyroEmaAlphaMin - kGyroEmaAlphaMax);
    const float deadband = getSettings().game.gyroDeadband;

    s_smooth_gx += smooth_alpha * (gyro[0] - s_smooth_gx);
    s_smooth_gy += smooth_alpha * (gyro[1] - s_smooth_gy);
    s_smooth_gz += smooth_alpha * (gyro[2] - s_smooth_gz);

    const float pitch_rate = apply_deadband(s_smooth_gx, deadband);
    const float yaw_rate = apply_deadband(s_smooth_gy, deadband);
    const float roll_rate = apply_deadband(s_smooth_gz, deadband);

    // Body-frame mapping: X -> pitch, Y -> horizontal look. No gravity yaw/roll remap —
    // that blend made upright holds drive L/R from roll ("rocking") instead of yaw.
    s_pitch_rad = -pitch_rate * dt * getSettings().game.gyroSensitivityY;
    s_roll_rad = roll_rate * dt * getSettings().game.gyroSensitivityX; // rollgoal only
    s_yaw_rad = yaw_rate * dt * getSettings().game.gyroSensitivityX;

    s_pitch_rad = getSettings().game.gyroInvertPitch ? -s_pitch_rad : s_pitch_rad;
    s_yaw_rad = getSettings().game.gyroInvertYaw ? -s_yaw_rad : s_yaw_rad;
    s_yaw_rad = getSettings().game.enableMirrorMode ? -s_yaw_rad : s_yaw_rad;
}

void getAimDeltas(float& out_yaw, float& out_pitch) {
    out_yaw = s_yaw_rad;
    out_pitch = s_pitch_rad;
}

void rollgoalTick(bool play_active, s16 camera_yaw) {
    if (!play_active) {
        reset_filter_state();
        return;
    }

    float pitch_rad = -s_pitch_rad * getSettings().game.gyroSensitivityRollgoal;
    float roll_rad = s_roll_rad * getSettings().game.gyroSensitivityRollgoal;
    roll_rad = getSettings().game.enableMirrorMode ? -roll_rad : roll_rad;

    s_rollgoal_az += cM_rad2s(roll_rad);
    cXyz in(roll_rad, 0.0f, pitch_rad);
    cXyz out;
    cMtx_YrotS(*calc_mtx, -camera_yaw);
    MtxPosition(&in, &out);

    s_rollgoal_ax += cM_rad2s(out.z);
    s_rollgoal_az += cM_rad2s(out.x);
    s_rollgoal_ax = std::clamp(s_rollgoal_ax, -kRollgoalTableMaxOffset, kRollgoalTableMaxOffset);
    s_rollgoal_az = std::clamp(s_rollgoal_az, -kRollgoalTableMaxOffset, kRollgoalTableMaxOffset);
}

void rollgoalTableOffset(s16& out_ax, s16& out_az) {
    out_ax = static_cast<s16>(s_rollgoal_ax);
    out_az = static_cast<s16>(s_rollgoal_az);
}
}  // namespace dusk::gyro
