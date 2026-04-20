#pragma once

#include <AP_HAL/AP_HAL.h>
#include "FD_CAN.h"

class FD_CAN;

class FD_MOT {
public:
    friend class FD_CAN;

    FD_MOT(FD_CAN* frotend);
    ~FD_MOT();

    /* Do not allow copies */
    FD_MOT(const FD_MOT &other) = delete;
    FD_MOT &operator=(const FD_MOT&) = delete;

    void handle_info(AP_HAL::CANFrame &in_frame, bool do_print = false);
    void set_id(uint8_t id_in);
    // void set_pwm(uint16_t pwm_in);   // 已废弃，油门由FD_CAN::loop()统一管理
    // void update();                    // 已废弃
    // void update_cmd();                // 已废弃
    // void update_status();             // 已废弃
    void send_cmd(uint32_t id, uint8_t *data);
    FD_CAN* _frotend_ptr;

    struct status_t {
        uint8_t id;
        uint8_t order;
        uint8_t group;
        uint16_t thr_in;
        uint16_t rpm;
        int16_t temp;  // 修改: 改为int16_t，因为温度减去50后可能为负值
        uint32_t last_status_ms;
        uint32_t last_mot_ms;
        uint32_t last_rpm_ms;
        uint32_t last_temp_ms;
        uint32_t last_print_ms;
    };

    status_t status{};  // 修复: 使用值初始化，所有成员置0
    uint8_t _data[8]{};  // 修复: 初始化数组
};