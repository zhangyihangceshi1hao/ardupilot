#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#define POS_FIXED_FRAME_LENGTH 100 // 帧长

class AP_POS {
public:
    AP_POS();

    /* Do not allow copies */
    AP_POS(const AP_POS &other) = delete;
    AP_POS &operator=(const AP_POS&) = delete;

    void init(const AP_SerialManager& serial_manager);

    bool pos_update(uint8_t UTC_year, uint8_t UTC_month, uint8_t UTC_day,
                    uint8_t UTC_hour, uint8_t UTC_minute, uint16_t UTC_second,
                    int32_t longitude, int32_t latitude, int16_t alt_sealevel,
                    int16_t pitch_cd, int16_t roll_cd, uint16_t yaw_cd,
                    uint16_t speed, int16_t v_speed, uint16_t course_cd,
                    uint8_t gps_fixed, uint32_t gps_tow, int16_t alt_home);

    bool BDS_upstream_update(uint8_t *tx_buf, int data_size);

    bool is_ap_pos() { return _is_ap_pos; }
    bool is_BDS_upsteam() { return _is_BDS_upstream; }

private:
    AP_HAL::UARTDriver *_port;              // UART used to send data to receiver

    struct PACKED pva {
        uint8_t sync_1;
        uint8_t sync_2;
        uint8_t UTC_year;
        uint8_t UTC_month;
        uint8_t UTC_day;
        uint8_t UTC_hour;
        uint8_t UTC_minute;
        uint16_t UTC_second;
        int32_t longitude;
        int32_t latitude;
        int16_t alt_sealevel;
        int16_t pitch_cd;
        int16_t roll_cd;
        uint16_t yaw_cd;
        uint16_t speed;
        int16_t v_speed;
        uint16_t course_cd;
        uint8_t gps_fixed;
        uint32_t gps_tow;
        int16_t alt_home;
        uint8_t rsv[60];
        uint8_t check1;
        uint8_t check2;
    };

    union PACKED msgbuffer {
        pva pva_u;
        uint8_t bytes[POS_FIXED_FRAME_LENGTH];
    };

    msgbuffer _tx_data;

    bool _is_ap_pos = false;
    bool _is_BDS_upstream = false;
};
