#include "AP_Choreo.h"

#if AP_CHOREO_ENABLED

#include <GCS_MAVLink/GCS.h>

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

void AP_Choreo::init()
{
    if (!_enable.get()) {
        return;
    }
    gcs().send_text(MAV_SEVERITY_INFO, "AP_Choreo: init (Phase 6.1 skeleton)");
}

void AP_Choreo::update()
{
    if (!_enable.get()) {
        return;
    }
    // TODO: Phase 6.4 主循环
}

namespace AP {
AP_Choreo* choreo() { return AP_Choreo::get_singleton(); }
}

#endif // AP_CHOREO_ENABLED
