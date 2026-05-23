#include "AP_Choreo.h"

#if AP_CHOREO_ENABLED

#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_Math/AP_Math.h>

// ArduCopter mode number for GUIDED (see ArduCopter/mode.h: Mode::Number::GUIDED = 4)
// Hard-coded as a magic number to avoid including ArduCopter/Copter.h.
static constexpr uint8_t CHOREO_VEHICLE_MODE_GUIDED = 4;

AP_Choreo* AP_Choreo::_singleton = nullptr;

const AP_Param::GroupInfo AP_Choreo::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: AP_Choreo enable
    // @Description: 0:Disabled 1:Enabled
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 0, AP_Choreo, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: NONCE
    // @DisplayName: Choreo arm nonce
    // @Description: Increment to (re)arm choreo start
    // @User: Standard
    AP_GROUPINFO("NONCE",    1, AP_Choreo, _nonce,      0),

    // @Param: LEAD
    // @DisplayName: Choreo lead time
    // @Description: Seconds between arm and start
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("LEAD",     2, AP_Choreo, _lead,       2.0f),

    // @Param: CYCLE
    // @DisplayName: Choreo cycle time
    // @Description: Full loop duration in seconds
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("CYCLE",    3, AP_Choreo, _cycle,      10.0f),

    // @Param: MLAT
    // @DisplayName: Master latitude
    // @Description: Master drone latitude in degrees
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("MLAT",     4, AP_Choreo, _master_lat, 0),

    // @Param: MLON
    // @DisplayName: Master longitude
    // @Description: Master drone longitude in degrees
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("MLON",     5, AP_Choreo, _master_lon, 0),

    // @Param: BASE_ALT
    // @DisplayName: Base altitude
    // @Description: Base altitude above HOME for choreo plane
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("BASE_ALT", 6, AP_Choreo, _base_alt,   0),

    // @Param: MIN_ALT
    // @DisplayName: Minimum altitude
    // @Description: Minimum AGL altitude required to start choreo
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("MIN_ALT",  7, AP_Choreo, _min_alt,    3.0f),

    // @Param: LOOP
    // @DisplayName: Loop mode
    // @Description: 0:OneShot 1:Loop forever
    // @Values: 0:OneShot,1:Loop
    // @User: Standard
    AP_GROUPINFO("LOOP",     8, AP_Choreo, _loop,       1),

    AP_GROUPEND
};

AP_Choreo::AP_Choreo()
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

// =====================================================================
// init() — startup, load CSV
// =====================================================================
void AP_Choreo::init()
{
    if (!_enable.get()) {
        return;
    }
    gcs().send_text(MAV_SEVERITY_INFO, "AP_Choreo: init starting");

    // Load CSV from SD card (in SITL maps to <cwd>/choreo.csv via AP_Filesystem)
    if (!_load_csv("/APM/choreo.csv")) {
        gcs().send_text(MAV_SEVERITY_WARNING,
            "AP_Choreo: /APM/choreo.csv not found or empty");
    }
}

// =====================================================================
// Time source helpers (docs/TIME_SYNC.md section 3)
// =====================================================================
bool AP_Choreo::_read_utc_now(uint64_t &utc_usec_out) const
{
    // 1) Prefer RTC (covers GPS / internal RTC / system time / NTP, etc)
    if (AP::rtc().get_utc_usec(utc_usec_out)) {
        return true;
    }
    // 2) Fall back to GPS directly
    const uint64_t t = AP::gps().time_epoch_usec();
    if (t > 0) {
        utc_usec_out = t;
        return true;
    }
    // 3) No UTC available
    return false;
}

// =====================================================================
// _check_arm() — react to nonce changes (docs/TIME_SYNC.md section 4)
// =====================================================================
void AP_Choreo::_check_arm()
{
    int32_t nonce = _nonce.get();
    if (nonce == _last_seen_nonce) {
        return;
    }
    _last_seen_nonce = nonce;

    if (nonce <= 0) {
        _arm_utc_usec = 0;
        _arm_millis   = 0;
        _started      = false;
        return;
    }

    // Record both time sources (whichever is available at run-time wins)
    const uint32_t lead_ms    = uint32_t(_lead.get() * 1000.0f);
    const uint64_t lead_usec  = uint64_t(_lead.get() * 1e6f);

    uint64_t utc;
    if (_read_utc_now(utc)) {
        _arm_utc_usec = utc + lead_usec;
        gcs().send_text(MAV_SEVERITY_INFO,
            "AP_Choreo: armed nonce=%ld UTC source (lead=%.1fs)",
            (long)nonce, (double)_lead.get());
    } else {
        _arm_utc_usec = 0;
        gcs().send_text(MAV_SEVERITY_INFO,
            "AP_Choreo: armed nonce=%ld millis source (lead=%.1fs)",
            (long)nonce, (double)_lead.get());
    }
    // millis is always recorded as fallback
    _arm_millis = AP_HAL::millis() + lead_ms;
    _started    = false;
}

// =====================================================================
// _elapsed_s() — auto pick UTC or millis (docs/TIME_SYNC.md section 5)
// =====================================================================
float AP_Choreo::_elapsed_s() const
{
    uint64_t utc;
    if (_arm_utc_usec > 0 && _read_utc_now(utc)) {
        if (utc < _arm_utc_usec) {
            return -float(_arm_utc_usec - utc) * 1e-6f;     // countdown
        }
        return float(utc - _arm_utc_usec) * 1e-6f;
    }

    if (_arm_millis > 0) {
        const uint32_t now = AP_HAL::millis();
        if (now < _arm_millis) {
            return -float(_arm_millis - now) * 1e-3f;
        }
        return float(now - _arm_millis) * 1e-3f;
    }

    return -1e9f;       // not armed
}

// =====================================================================
// _to_abs() — Waypoint -> AbsPos (docs/WAYPOINT_TYPES.md section 4)
// =====================================================================
bool AP_Choreo::_to_abs(const Waypoint &wp, AbsPos &out) const
{
    if (wp.type == 'L') {
        out.lat_1e7 = wp.pos.lla.lat_1e7;
        out.lng_1e7 = wp.pos.lla.lng_1e7;
        out.up_m    = wp.pos.lla.up_m;
        return true;
    }

    // type 'N' — NED metre offset from master HOME
    const float mlat = _master_lat.get();
    const float mlon = _master_lon.get();
    if (is_zero(mlat) || is_zero(mlon)) {
        // No master reference — fall back to own HOME
        Location my_home = AP::ahrs().get_home();
        my_home.offset(wp.pos.ned.north_m, wp.pos.ned.east_m);
        out.lat_1e7 = my_home.lat;
        out.lng_1e7 = my_home.lng;
        out.up_m    = wp.pos.ned.up_m;
    } else {
        Location master {};
        master.lat = int32_t(mlat * 1e7f);
        master.lng = int32_t(mlon * 1e7f);
        master.offset(wp.pos.ned.north_m, wp.pos.ned.east_m);
        out.lat_1e7 = master.lat;
        out.lng_1e7 = master.lng;
        out.up_m    = wp.pos.ned.up_m;
    }
    return true;
}

// =====================================================================
// _sample_at_t_norm() — interpolate at given normalized time
// =====================================================================
bool AP_Choreo::_sample_at_t_norm(float t_norm,
                                  AbsPos &out,
                                  uint8_t &R, uint8_t &G, uint8_t &B) const
{
    if (_num_wps == 0) {
        return false;
    }
    if (_num_wps == 1) {
        if (!_to_abs(_wps[0], out)) return false;
        R = _wps[0].r; G = _wps[0].g; B = _wps[0].b;
        return true;
    }

    // Find bracketing segment [i, i+1] s.t. wps[i].t_norm <= t_norm <= wps[i+1].t_norm
    if (t_norm <= _wps[0].t_norm) {
        if (!_to_abs(_wps[0], out)) return false;
        R = _wps[0].r; G = _wps[0].g; B = _wps[0].b;
        return true;
    }
    if (t_norm >= _wps[_num_wps-1].t_norm) {
        const Waypoint &w = _wps[_num_wps-1];
        if (!_to_abs(w, out)) return false;
        R = w.r; G = w.g; B = w.b;
        return true;
    }

    uint16_t i = 0;
    for (uint16_t k = 0; k + 1 < _num_wps; k++) {
        if (t_norm >= _wps[k].t_norm && t_norm <= _wps[k+1].t_norm) {
            i = k;
            break;
        }
    }

    const Waypoint &a = _wps[i];
    const Waypoint &b = _wps[i+1];
    const float span = b.t_norm - a.t_norm;
    const float alpha = (span > 1e-6f) ? (t_norm - a.t_norm) / span : 0.0f;

    AbsPos pa, pb;
    if (!_to_abs(a, pa) || !_to_abs(b, pb)) {
        return false;
    }

    // Interpolate in absolute LLA + up_m
    const double lat_d = double(pa.lat_1e7) + double(pb.lat_1e7 - pa.lat_1e7) * alpha;
    const double lng_d = double(pa.lng_1e7) + double(pb.lng_1e7 - pa.lng_1e7) * alpha;
    out.lat_1e7 = int32_t(lat_d);
    out.lng_1e7 = int32_t(lng_d);
    out.up_m    = pa.up_m + (pb.up_m - pa.up_m) * alpha;

    // Colour interpolation (uint8 saturated)
    R = uint8_t(constrain_int16(int(a.r + (int(b.r) - int(a.r)) * alpha + 0.5f), 0, 255));
    G = uint8_t(constrain_int16(int(a.g + (int(b.g) - int(a.g)) * alpha + 0.5f), 0, 255));
    B = uint8_t(constrain_int16(int(a.b + (int(b.b) - int(a.b)) * alpha + 0.5f), 0, 255));

    return true;
}

// =====================================================================
// update() — 50 Hz main loop
// =====================================================================
void AP_Choreo::update()
{
    if (!_enable.get()) {
        return;
    }

    AP_Vehicle *vehicle = AP::vehicle();
    if (vehicle == nullptr) {
        return;
    }

    // (1) GUIDED check
    if (vehicle->get_mode() != CHOREO_VEHICLE_MODE_GUIDED) {
        _state = State::IDLE;
        _started = false;
        return;
    }

    // (2) Altitude check (NED z is down → -z = up)
    Vector3f pos_ned;
    if (!AP::ahrs().get_relative_position_NED_home(pos_ned)) {
        return;
    }
    if (-pos_ned.z < _min_alt.get()) {
        _state = State::WAIT_ALT;
        return;
    }

    // (3) Nonce-driven arm
    _check_arm();
    if (_arm_utc_usec == 0 && _arm_millis == 0) {
        _state = State::WAIT_ALT;
        return;
    }

    // (4) Elapsed (auto picks UTC or millis source)
    const float elapsed_s = _elapsed_s();
    if (elapsed_s < 0) {
        _state = State::ARMED_WAIT;
        return;
    }

    // (5) Open ceremony banner
    if (!_started) {
        gcs().send_text(MAV_SEVERITY_INFO,
            "AP_Choreo: START cycle=%.1fs base=%.1fm wps=%u",
            (double)_cycle.get(), (double)_base_alt.get(), (unsigned)_num_wps);
        _started = true;
    }
    _state = State::RUNNING;

    // (6) Compute t_norm
    const float cycle_s = (_cycle.get() > 0.1f) ? _cycle.get() : 10.0f;
    float t_norm = elapsed_s / cycle_s;
    if (_loop.get()) {
        t_norm -= floorf(t_norm);              // wrap into [0,1)
    } else if (t_norm > 1.0f) {
        t_norm = 1.0f;                          // clamp at last frame
    }

    // (7) Sample absolute target
    AbsPos abs;
    uint8_t R = 0, G = 0, B = 0;
    if (!_sample_at_t_norm(t_norm, abs, R, G, B)) {
        return;
    }

    // (8) Dispatch as absolute LLA
    Location target {};
    target.lat = abs.lat_1e7;
    target.lng = abs.lng_1e7;
    target.set_alt_cm(int32_t((abs.up_m + _base_alt.get()) * 100.0f),
                      Location::AltFrame::ABOVE_HOME);

    vehicle->set_target_location(target);

    // (9) LED
    const uint32_t now_ms = AP_HAL::millis();
    _drive_led(R, G, B);
    _report_led(now_ms, R, G, B);
}

namespace AP {
AP_Choreo* choreo() { return AP_Choreo::get_singleton(); }
}

#endif // AP_CHOREO_ENABLED
