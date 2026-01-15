#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS.h>

struct PACKED Serial_number {
    uint32_t serial_number;
};

class FD_DATA
{

public:
    FD_DATA();

    /* Do not allow copies */
    FD_DATA(const FD_DATA &other) = delete;
    FD_DATA &operator=(const FD_DATA&) = delete;

    static FD_DATA *get_singleton() {
        return _singleton;
    }

    void update();

    void handle_message(const mavlink_message_t &msg);
    void handle_message_txhy_sn(const mavlink_message_t &msg);
    void handle_message_command_long_txhy_sn(const mavlink_message_t &msg);
    void handle_message_power_control(const mavlink_message_t &msg);

    bool get_serial_number(uint32_t& serial_number);
    bool set_serial_number(uint32_t serial_number);
    void send_mav_serial_number(uint32_t serial_number);
    void send_motor_rpm(mavlink_channel_t chan);
    void send_battery_error(mavlink_channel_t chan);
    void send_battery_info(mavlink_channel_t chan);

    mavlink_motor_rpm_t motor_rpm_packet;
    mavlink_battery_error_t battery_error_packet;
    mavlink_battery_info_t battery_info_packet;
private:
    static FD_DATA *_singleton;

    static StorageAccess _storage;
};


namespace AP {
    FD_DATA &fd_data();
};
