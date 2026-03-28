#include "AP_Serial_LIDE.h"

extern const AP_HAL::HAL &hal;
AP_Serial_LIDE *AP_Serial_LIDE::singleton = nullptr;

void AP_Serial_LIDE::init(const AP_SerialManager &serial_manager)
{
    _serial_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_LIDE, 0);
    if (_serial_port != nullptr)
    {
        _serial_port->begin(115200);
        //  _serial_port->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_LIDE, 0));
        if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Serial_LIDE::loop, void),
                                          "LIDE",
                                          1024, AP_HAL::Scheduler::PRIORITY_RCIN, 1))
        {
            gcs().send_text(MAV_SEVERITY_ERROR, "Failed to create LIDE thread");
        }
    }
}

void AP_Serial_LIDE::loop(void)
{
    while (true)
    {
        hal.scheduler->delay(500);
        send_heartbeat_pck();
    }
}

void AP_Serial_LIDE::send_heartbeat_pck()
{
    // 检查串口是否有效
    if (_serial_port == nullptr)
    {
        return;
    }

    // 检查串口是否打开
    if (!_serial_port->is_initialized())
    {
        // 尝试重新初始化
        _serial_port->begin(115200);
        hal.scheduler->delay(1000);
    }

    uint8_t heartbeat_pck[10] = {0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    auto ret = _serial_port->write(heartbeat_pck, sizeof(heartbeat_pck));
    if (ret == 10)
    {
        gcs().send_text(MAV_SEVERITY_ERROR, "LIDE Serial send true");
    }
    else
    {
        gcs().send_text(MAV_SEVERITY_ERROR, "LIDE Serial send failed");
    }
}

AP_Serial_LIDE::AP_Serial_LIDE()
{
    singleton = this;
}
