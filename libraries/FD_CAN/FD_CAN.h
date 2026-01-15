#pragma once

#include <AP_HAL/AP_HAL.h>
// #include <AP_CANManager/AP_CANDriver.h>
#include <AP_CANManager/AP_CANManager.h>

#include <AP_Param/AP_Param.h>

#include <FD_CAN/FD_FAN.h>
#include <FD_CAN/FD_MOT.h>
#include <FD_CAN/FD_BMS.h>

#define FD_CAN_MAX_MOT_NUM 16

class FD_FAN;
class FD_MOT;
class FD_BMS;

class FD_CAN : public AP_CANDriver
{
public:
    friend class FD_FAN;
    friend class FD_MOT;
    friend class FD_BMS;

    FD_CAN();
    ~FD_CAN();

    /* Do not allow copies */
    FD_CAN(const FD_CAN &other) = delete;
    FD_CAN &operator=(const FD_CAN&) = delete;

    static const struct AP_Param::GroupInfo var_info[];

    // Return CAN_ESC from @driver_index or nullptr if it's not ready or doesn't exist
    static FD_CAN *get_can_fd(uint8_t driver_index);

    // initialize CAN_ESC bus
    void init(uint8_t driver_index, bool enable_filters) override;
    bool add_interface(AP_HAL::CANIface* can_iface) override;

    // called from high level code
    void update();

    // test if the CAN driver is ready to be armed
    bool pre_arm_check(char* reason, uint8_t reason_len);

    FD_FAN *_fan_ptr{nullptr};  // 修复: 初始化为 nullptr
    FD_MOT *_mot_ptr[FD_CAN_MAX_MOT_NUM]{};  // 修复: 初始化所有元素为 nullptr
    FD_BMS *_bms_ptr{nullptr};  // 修复: 初始化为 nullptr

    AP_Int32 _print;
    AP_Int16 _out;
    AP_Int8  _enable_fan;
    AP_Int8  _enable_mot;
    AP_Int8  _enable_bms;

    struct {
        uint16_t rpm[FD_CAN_MAX_MOT_NUM];
        int8_t temp[FD_CAN_MAX_MOT_NUM];
    } _mot_state{};  // 修复: 初始化结构体

    struct {
        uint16_t error_code;
        uint8_t battery_id;
        uint16_t voltage;
        int16_t current;
    } _bms_state{};  // 修复: 初始化结构体

private:

    // loop to send output to ESCs in background thread
    void loop();

    // write frame on CAN bus, returns true on success
    bool write_frame(AP_HAL::CANFrame &out_frame, uint64_t timeout);

    // read frame on CAN bus, returns true on succses
    bool read_frame(AP_HAL::CANFrame &recv_frame, uint64_t timeout);

    bool _initialized{false};  // 修复: 初始化
    char _thread_name[16]{};   // 修复: 初始化
    uint8_t _driver_index{0};  // 修复: 初始化
    AP_HAL::CANIface* _can_iface{nullptr};  // 修复: 初始化为 nullptr
    HAL_EventHandle sem_handle;
};