#pragma once

#include "AP_Choreo_config.h"

#if AP_CHOREO_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

class AP_Choreo {
public:
    AP_Choreo();
    CLASS_NO_COPY(AP_Choreo);

    static AP_Choreo* get_singleton() { return _singleton; }
    void init();
    void update();

    static const struct AP_Param::GroupInfo var_info[];

private:
    static AP_Choreo* _singleton;

    AP_Int8  _enable;
    AP_Int32 _nonce;
    AP_Float _lead;
    AP_Float _cycle;
    AP_Float _master_lat;
    AP_Float _master_lon;
    AP_Float _base_alt;
    AP_Float _min_alt;
    AP_Int8  _loop;

    int32_t _last_seen_nonce = -1;
};

namespace AP { AP_Choreo* choreo(); }

#endif // AP_CHOREO_ENABLED
