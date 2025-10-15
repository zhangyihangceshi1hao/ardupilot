#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#define POS_PVA_FRAME_ID 0x21

class AP_POS {
public:
    AP_POS();

    /* Do not allow copies */
    AP_POS(const AP_POS &other) = delete;
    AP_POS &operator=(const AP_POS&) = delete;

    void init(const AP_SerialManager& serial_manager);

    bool update(float px_cm, float py_cm, float pz_cm, float vx_cms, float vy_cms, float vz_cms, float roll_rad, float pitch_rad, float yaw_rad);

private:
    AP_HAL::UARTDriver *_port;              // UART used to send data to receiver

    struct PACKED pva {
        float px_m;
        float py_m;
        float pz_m;
        float vx_ms;
        float vy_ms;
        float vz_ms;
        float roll_rad;
        float pitch_rad;
        float yaw_rad;  // 0 ~ 2π
    };

    union PACKED msgbuffer {
        pva pva_u;
        uint8_t bytes[512];
    };

    msgbuffer _tx_data;
    uint8_t _tx_frame_num;

    uint32_t pos_crc32(uint32_t crc, uint8_t *buf, uint32_t size);
};
