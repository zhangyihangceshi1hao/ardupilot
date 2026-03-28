#pragma once
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

class AP_Serial_LIDE
{
public:
    void init(const AP_SerialManager &serial_manager);
    void send_heartbeat_pck();
    bool send_controller(int enable, int32_t frequency);
    void get_telem_data();

private:
    AP_HAL::UARTDriver *_serial_port;
    void loop(void);
};