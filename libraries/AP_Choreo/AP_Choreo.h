#pragma once

#include "AP_Choreo_config.h"

#if AP_CHOREO_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>

class AP_Choreo {
public:
    AP_Choreo();
    CLASS_NO_COPY(AP_Choreo);

    static AP_Choreo* get_singleton() { return _singleton; }
    void init();
    void update();

    static const struct AP_Param::GroupInfo var_info[];

    // ------------------------------------------------------------------
    // Waypoint storage (see docs/WAYPOINT_TYPES.md section 3)
    // sizeof(Waypoint) == 20 bytes  →  500 * 20 = 10 KB
    // ------------------------------------------------------------------
    struct Waypoint {
        float   t_norm;       // 0..1 normalized time (frame or t_s after load)
        uint8_t type;         // 'N' or 'L'
        uint8_t r, g, b;      // 0..255
        union {
            struct { float   north_m, east_m, up_m;   } ned;
            struct { int32_t lat_1e7, lng_1e7;
                     float   up_m;                      } lla;
        } pos;
    };
    static constexpr uint16_t MAX_WPS = 500;

    // Absolute LLA + relative altitude, used for sample/dispatch
    struct AbsPos {
        int32_t lat_1e7;
        int32_t lng_1e7;
        float   up_m;          // metres above HOME
    };

private:
    static AP_Choreo* _singleton;

    // ---------------- Parameters ----------------
    AP_Int8  _enable;
    AP_Int32 _nonce;
    AP_Float _lead;
    AP_Float _cycle;
    AP_Float _master_lat;
    AP_Float _master_lon;
    AP_Float _base_alt;
    AP_Float _min_alt;
    AP_Int8  _loop;

    // ---------------- State machine ----------------
    enum class State : uint8_t {
        IDLE, WAIT_ALT, ARMED_WAIT, RUNNING
    } _state = State::IDLE;

    int32_t  _last_seen_nonce = -1;
    uint64_t _arm_utc_usec = 0;        // GPS UTC start (primary)
    uint32_t _arm_millis   = 0;         // millis fallback start
    bool     _started = false;

    // ---------------- Waypoint table ----------------
    Waypoint _wps[MAX_WPS];
    uint16_t _num_wps = 0;

    // ---------------- LED report throttling ----------------
    uint32_t _last_led_report_ms = 0;
    static constexpr uint32_t LED_REPORT_INTERVAL_MS = 200;   // 5 Hz

    // ---------------- Private methods ----------------
    bool  _load_csv(const char* path);

    bool  _read_utc_now(uint64_t &utc_usec_out) const;
    void  _check_arm();
    float _elapsed_s() const;

    bool  _to_abs(const Waypoint &wp, AbsPos &out) const;
    bool  _sample_at_t_norm(float t_norm,
                            AbsPos &out,
                            uint8_t &R, uint8_t &G, uint8_t &B) const;

    void  _drive_led(uint8_t R, uint8_t G, uint8_t B);
    void  _report_led(uint32_t now_ms, uint8_t R, uint8_t G, uint8_t B);
};

namespace AP { AP_Choreo* choreo(); }

#endif // AP_CHOREO_ENABLED
