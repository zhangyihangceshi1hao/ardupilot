#include <AP_Math/AP_Math.h>
#include "FD_DATA.h"
#include <FD_CAN/FD_CAN.h>

extern const AP_HAL::HAL& hal;

void FD_DATA::send_motor_rpm(mavlink_channel_t chan)
{
    motor_rpm_packet.time_boot_ms = AP_HAL::millis();

    mavlink_msg_motor_rpm_send_struct(chan, &motor_rpm_packet);
}

void FD_DATA::send_battery_error(mavlink_channel_t chan)
{
    battery_error_packet.time_boot_ms = AP_HAL::millis();

    mavlink_msg_battery_error_send_struct(chan, &battery_error_packet);
}

void FD_DATA::send_battery_info(mavlink_channel_t chan)
{
    battery_info_packet.time_boot_ms = AP_HAL::millis();

    mavlink_msg_battery_info_send_struct(chan, &battery_info_packet);
}

void FD_DATA::handle_message_power_control(const mavlink_message_t &msg)
{
    uint8_t final_cmd = 233;

    if (msg.msgid == MAVLINK_MSG_ID_POWER_CONTROL) {
        mavlink_power_control_t packet;
        mavlink_msg_power_control_decode(&msg, &packet);

        if (packet.target_system == 0 || packet.target_system == gcs().sysid_this_mav()) {
            if (packet.command == 1) {
                final_cmd = 1;
            }
            if (packet.command == 2) {
                final_cmd = 0;
            }
        }
    }

    if (final_cmd == 233) {
        return;
    }

    for (uint8_t i = 0; i < AP::can().get_num_drivers(); i++) {
        if (AP::can().get_driver_type(i) == AP_CANManager::Driver_Type_FDCAN) {
            FD_CAN *fd_can = FD_CAN::get_can_fd(i);
            if (fd_can == nullptr) {
                // send_text(MAV_SEVERITY_INFO, "%d| fd_can == nullptr", i);
                continue;
            }
            if (fd_can->_bms_ptr == nullptr) {
                if (fd_can->_print.get()) {
                    gcs().send_text(MAV_SEVERITY_INFO, "%d| fd_can->_bms_ptr", i);
                    continue;
                }
            }
            fd_can->_bms_ptr->set_switch(final_cmd);
            break;
        }
    }
}
