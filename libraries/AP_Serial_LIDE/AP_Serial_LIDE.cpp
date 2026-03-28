#include "AP_Serial_LIDE.h"


extern const AP_HAL::HAL& hal;

AP_Serial_LIDE* AP_Serial_LIDE::singleton = nullptr;

void AP_Serial_LIDE::init(const AP_SerialManager& serial_manager){
    _serial_port  = serial_manager.find_serial(AP_SerialManager::SerialProtocol_LIDE, 0);
    if(_serial_port == nullptr){
        hal.console->printf("LIDE: No Serial Port found for LIDE\n");
    }else{
        hal.console->printf("LIDE: Serial Port found for LIDE\n");
        // _serial_port->begin(115200);
         _serial_port->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_LIDE, 0));
        // register thread
        // if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Serial_LIDE::loop, void),
        //                                   "LIDE",
        //                                   1024, AP_HAL::Scheduler::PRIORITY_RCIN, 1)) {
        //     DEV_PRINTF("Failed to create LIDE thread\n");
        // }
    }
}

// void AP_Serial_LIDE::loop(void)
// {
//     // initialise uart (this must be called from within tick b/c the UART begin must be called from the same thread as it is used from)
//     // port->begin(115200);
    
//     while (true) {
//         hal.scheduler->delay(50);
//         send_heartbeat_pck();
//     }
// }

void AP_Serial_LIDE::send_heartbeat_pck(){
     // 检查串口是否有效
    if (_serial_port == nullptr) {
        hal.console->printf("LIDE: Serial port lost!\n");
        return;
    }

    // 检查串口是否打开
    if (!_serial_port->is_initialized()) {
        hal.console->printf("LIDE: Serial port not initialized!\n");
        // 尝试重新初始化
        _serial_port->begin(115200);
        hal.scheduler->delay(1000);
    }
        
    // uint8_t heartbeat_pck[10] = {0x55,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    _serial_port->write(1);
    _serial_port->write(2);
    _serial_port->write(3);
    _serial_port->write(4);
    _serial_port->write(5);
    _serial_port->write(6);
    _serial_port->write(7);
    _serial_port->write(8);
    _serial_port->write(9);
    auto ret = _serial_port->write(10);
    if (ret == 1) {
         gcs().send_text(MAV_SEVERITY_ERROR, "LIDE Serial send true");
    } else {
        gcs().send_text(MAV_SEVERITY_ERROR, "LIDE Serial send failed");
    }
    gcs().send_text(MAV_SEVERITY_WARNING,"LIDE serial port OK");
}


AP_Serial_LIDE::AP_Serial_LIDE()
{
    singleton = this;
}

