#include "AP_Choreo.h"

#if AP_CHOREO_ENABLED

#include <GCS_MAVLink/GCS.h>
#include <AP_SerialLED/AP_SerialLED.h>
#include <AP_Notify/AP_Notify.h>

// Drive NeoPixel + onboard notify LED.
// Either subsystem may be absent / disabled on a given build; failures are
// silently ignored so the main loop is never disturbed.
void AP_Choreo::_drive_led(uint8_t R, uint8_t G, uint8_t B)
{
    // NeoPixel strip on SERVO1 channel (chan=1, led index -1 = all leds)
    AP_SerialLED *sled = AP_SerialLED::get_singleton();
    if (sled != nullptr) {
        sled->set_RGB(1, -1, R, G, B);
        sled->send(1);
    }

    // Onboard notify LED (also broadcasts to GCS via MAV_CMD_DO_SET_LED)
    AP_Notify::handle_rgb(R, G, B, 255);
}

// 5 Hz report to GCS: pack RGB into a single float via 24-bit integer.
void AP_Choreo::_report_led(uint32_t now_ms, uint8_t R, uint8_t G, uint8_t B)
{
    if (now_ms - _last_led_report_ms < LED_REPORT_INTERVAL_MS) {
        return;
    }
    _last_led_report_ms = now_ms;

    const uint32_t packed = (uint32_t(R) << 16) | (uint32_t(G) << 8) | uint32_t(B);
    gcs().send_named_float("LED_RGB", float(packed));
}

#endif // AP_CHOREO_ENABLED
