#include "AP_PPS_Serial.h"
extern const AP_HAL::HAL& hal;
// 定义静态成员变量
AP_PPS_Serial* AP_PPS_Serial::_singleton = nullptr;
//修改
// 构造函数
AP_PPS_Serial::AP_PPS_Serial() :
    _serial_port(nullptr)
{
    _singleton = this;
}

// 初始化函数
void AP_PPS_Serial::init(const AP_SerialManager& serial_manager)
{
    _serial_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_PPS_Serial, 0);
    
    if (_serial_port != nullptr) 
    {
        _serial_port->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_PPS_Serial, 0));
        _serial_port->printf("_serial_port OK\r\n");
        gcs().send_text(MAV_SEVERITY_WARNING,"_serial_port OK");
    }
    else
    {
        gcs().send_text(MAV_SEVERITY_WARNING,"_serial_port null");
    }
}

// send_controller 函数实现
bool AP_PPS_Serial::send_controller(int enable, int32_t frequency)
{
    if (_serial_port == nullptr) {
        gcs().send_text(MAV_SEVERITY_ERROR, "PPS Serial port not initialized");
        return false;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "send_receive_pps.....");
   
    char message[64];
    
    // 先构造消息，然后再发送
    if (enable == 0) {
        hal.util->snprintf(message, sizeof(message), "TX:Stop:%" PRId32 "\r\n", frequency);
    } else if (enable == 1) {
        hal.util->snprintf(message, sizeof(message), "TX:Init:%" PRId32 "\r\n", frequency);
    } else {
        gcs().send_text(MAV_SEVERITY_ERROR, "Invalid enable value");
        return false;
    }
    
    const uint8_t* buf = reinterpret_cast<const uint8_t*>(message);
    size_t len = strlen(message);

    auto ret = _serial_port->write(buf, len);

    if (ret == len) {
         gcs().send_text(MAV_SEVERITY_ERROR, "PPS Serial send true");
        return true;
    } else {
        gcs().send_text(MAV_SEVERITY_ERROR, "PPS Serial send failed");
        return false;
    }
}