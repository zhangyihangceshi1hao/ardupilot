#include "Blimp.h"

#define ARM_DELAY               20  // called at 10hz so 2 seconds
#define DISARM_DELAY            20  // called at 10hz so 2 seconds
#define LOST_VEHICLE_DELAY      10  // called at 10hz so 1 second

// arm_motors_check - checks for pilot input to arm or disarm the blimp
// called at 10hz
void Blimp::arm_motors_check()
{
    static int16_t arming_counter;

    // check if arming/disarm using rudder is allowed
    AP_Arming::RudderArming arming_rudder = arming.get_rudder_arming_type();
    if (arming_rudder == AP_Arming::RudderArming::IS_DISABLED) {
        return;
    }

    // ensure throttle is down
    if (channel_up->get_control_in() > 0) { //MIR what dow we do with this?
        arming_counter = 0;
        return;
    }

    int16_t yaw_in = channel_yaw->get_control_in();

    // full right
    if (yaw_in > 900) {

        // increase the arming counter to a maximum of 1 beyond the auto trim counter
        if (arming_counter < ARM_DELAY) {
            arming_counter++;
        }

        // arm the motors and configure for flight
        if (arming_counter == ARM_DELAY && !motors->armed()) {
            // reset arming counter if arming fail
            if (!arming.arm(AP_Arming::Method::RUDDER)) {
                arming_counter = 0;
            }
        }

        // full left and rudder disarming is enabled
    } else if ((yaw_in < -900) && (arming_rudder == AP_Arming::RudderArming::ARMDISARM)) {
        if (!flightmode->has_manual_throttle() && !ap.land_complete) {
            arming_counter = 0;
            return;
        }

        // increase the counter to a maximum of 1 beyond the disarm delay
        if (arming_counter <= DISARM_DELAY) {
            arming_counter++;
        }

        // disarm the motors
        if (arming_counter == DISARM_DELAY && motors->armed()) {
            arming.disarm(AP_Arming::Method::RUDDER);
        }

        // Yaw is centered so reset arming counter
    } else {
        arming_counter = 0;
    }
}


// motors_output - send output to motors library which will adjust and send to ESCs and servos
void Blimp::motors_output()
{
    // output any servo channels
    SRV_Channels::calc_pwm();

    auto &srv = AP::srv();

    // cork now, so that all channel outputs happen at once
    srv.cork();

    // update output on any aux channels, for manual passthru
    SRV_Channels::output_ch_all();

    // send output signals to motors
    motors->output();

    // push all channels
    srv.push();
}
