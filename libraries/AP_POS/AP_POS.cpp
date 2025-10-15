#include "AP_POS.h"

extern const AP_HAL::HAL& hal;

//constructor
AP_POS::AP_POS(void)
{
    _port = NULL;
    memset(_tx_data.bytes, 0, sizeof(_tx_data.bytes));
    _tx_data.pva_u.sync_1 = 0xEE;
    _tx_data.pva_u.sync_2 = 0x90;
}

void AP_POS::init(const AP_SerialManager& serial_manager) {
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_POS, 0))) {
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        uint32_t baudrate = serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_POS, 0);
        _port->begin(baudrate, 256, 512);
    }
}

bool AP_POS::update(uint8_t UTC_year, uint8_t UTC_month, uint8_t UTC_day,
                    uint8_t UTC_hour, uint8_t UTC_minute, uint16_t UTC_second,
                    int32_t longitude, int32_t latitude, int16_t alt_sealevel,
                    int16_t pitch_cd, int16_t roll_cd, uint16_t yaw_cd,
                    uint16_t speed, int16_t v_speed, uint16_t course_cd,
                    uint8_t gps_fixed, uint32_t gps_tow, int16_t alt_home) 
{
    if(_port == NULL)
        return false;

    _tx_data.pva_u.UTC_year = UTC_year;
    _tx_data.pva_u.UTC_month = UTC_month;
    _tx_data.pva_u.UTC_day = UTC_day;
    _tx_data.pva_u.UTC_hour = UTC_hour;
    _tx_data.pva_u.UTC_minute = UTC_minute;
    _tx_data.pva_u.UTC_second = UTC_second;
    _tx_data.pva_u.longitude = longitude;
    _tx_data.pva_u.latitude = latitude;
    _tx_data.pva_u.alt_sealevel = alt_sealevel;
    _tx_data.pva_u.pitch_cd = pitch_cd;
    _tx_data.pva_u.roll_cd = roll_cd;
    _tx_data.pva_u.yaw_cd = yaw_cd;
    _tx_data.pva_u.speed = speed;
    _tx_data.pva_u.v_speed = v_speed;
    _tx_data.pva_u.course_cd = course_cd;
    _tx_data.pva_u.gps_fixed = gps_fixed;
    _tx_data.pva_u.gps_tow = gps_tow;
    _tx_data.pva_u.alt_home = alt_home;

    _tx_data.pva_u.check1 = _tx_data.bytes[0];
    for (int i = 1; i < 98; i++)
    {
        _tx_data.pva_u.check1 ^= _tx_data.bytes[i];
    }
    
    _tx_data.pva_u.check2 = _tx_data.bytes[0];
    for (int i = 1; i < 99; i++)
    {
        _tx_data.pva_u.check2 += _tx_data.bytes[i];
    }
    
    _port->write(_tx_data.bytes, POS_FIXED_FRAME_LENGTH);

    return true;
}
