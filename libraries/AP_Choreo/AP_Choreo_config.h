#pragma once

#ifndef AP_CHOREO_ENABLED
// Phase 6.6: default ON for SITL development.
// Production builds can override via ./waf configure --extra-hwdef=... or
// by defining AP_CHOREO_ENABLED=0 in the board hwdef.
#define AP_CHOREO_ENABLED 1
#endif
