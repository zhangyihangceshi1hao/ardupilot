#pragma once
//修改
#include "AP_PPS_Serial.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

class AP_PPS_Serial{
    public:
        AP_PPS_Serial();
        void init(const AP_SerialManager& serial_manager);
        bool send_controller(int enable,int32_t frequency);
        static AP_PPS_Serial* get_singleton() {
            return _singleton;
        }
        static AP_PPS_Serial* _singleton;
    private:
        AP_HAL::UARTDriver* _serial_port;
};
