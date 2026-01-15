#pragma once

#include <AP_HAL/AP_HAL.h>
#include "FD_CAN.h"

class FD_CAN;

class FD_FAN {
public:
    friend class FD_CAN;

    FD_FAN(FD_CAN* frotend);
    ~FD_FAN();

    struct fan_info_t {
        uint16_t status_code;
        uint16_t rpm;
        uint16_t sampletime;
    };

    enum class Fan_state {
        Status = 0,
        RPM,
        SampleTime,
    };

    /* Do not allow copies */
    FD_FAN(const FD_FAN &other) = delete;
    FD_FAN &operator=(const FD_FAN&) = delete;

    void handle_info(AP_HAL::CANFrame &in_frame, bool do_print = false);
    void update();
    void do_start(bool start_in);
    void update_state();
    void set_state(Fan_state state_in);
    void send_cmd(uint32_t id, uint8_t *data);
    void log_status(void);

    FD_CAN* _frotend_ptr{nullptr};  // 修复: 初始化为 nullptr

    fan_info_t fan_info{};  // 修复: 初始化结构体
    Fan_state ask_state{Fan_state::Status};  // 修复: 初始化枚举
    uint8_t msg_data[8]{};  // 修复: 初始化数组
    uint32_t _state_time{0};  // 修复: 初始化
    bool _print_enable{false};  // 修复: 初始化
};