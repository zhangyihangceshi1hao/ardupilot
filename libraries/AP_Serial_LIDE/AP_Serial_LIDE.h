#pragma once
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <stdio.h>

// 砺德发动机数据结构
#pragma pack(1)   // 按1字节对齐，避免编译器填充
typedef struct
{
    uint8_t controller_id;     // 字节1：控制器标识 (0xA5)
    uint8_t aircraft_id;       // 字节2：飞机标识 (0x5A)
    uint8_t data_length;       // 字节3：数据长度 (14)
    uint8_t frame_counter;     // 字节4：帧计数器 (0~255循环)
    uint16_t throttle;     // 字节5-6：油门请求 (Uint16, 系数1/10, 实际值=raw*0.1)
    uint16_t altitude;         // 字节7-8：当前海拔 (单位 m)
    uint8_t airspeed;          // 字节9：空速 (系数1/4, 实际值=raw*0.25 m/s)
    uint8_t cmd_controll;       //字节10：控制指令
    uint8_t reserve1;          // 字节11：预留
    uint8_t reserve2;          // 字节12：预留
    uint8_t reserve3;          // 字节13：预留
    uint8_t checksum;          // 字节14：校验和（前面所有字节异或）
} LIDE_Engine_Control_t;

#pragma pack()

class AP_Serial_LIDE
{
public:
    static AP_Serial_LIDE* _singleton;
    AP_Serial_LIDE() ;
     static AP_Serial_LIDE* get_singleton() {
            return _singleton;
        }
    void init(const AP_SerialManager &serial_manager);
    void send_heartbeat_pck();
    bool send_controller(int enable, int32_t frequency);
    void get_telem_data();

      // ====== set 方法 ======
    void set_cmd_controll(uint8_t cmd);
    void set_throttle(uint16_t throttle);
    void set_altitude(uint16_t altitude);
    void set_airspeed(uint8_t airspeed);

    // ====== get 方法 ======
    uint8_t  get_cmd_controll(void) const;
    uint16_t get_throttle(void) const;
    uint16_t get_altitude(void) const;
    uint8_t  get_airspeed(void) const;
private:
    
    LIDE_Engine_Control_t _engine_control;
    AP_HAL::UARTDriver *_serial_port;
    void loop(void);

};

