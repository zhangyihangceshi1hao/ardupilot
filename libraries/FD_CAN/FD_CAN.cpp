#include "FD_CAN.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Param/AP_Param.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>
#include <FD_DATA/FD_DATA.h>
#include <stdio.h>

extern const AP_HAL::HAL &hal;

#if HAL_CANMANAGER_ENABLED
#define debug_can(level_debug, fmt, args...)                     \
    do {                                                         \
        AP::can().log_text(level_debug, "CAN_FD", fmt, ##args); \
    } while (0)
#else
#define debug_can(level_debug, fmt, args...)
#endif

// table of user-configurable Piccolo CAN bus parameters
const AP_Param::GroupInfo FD_CAN::var_info[] = {

    // No use, reserved
    AP_GROUPINFO("PRINT", 1, FD_CAN, _print, 0),

    // No use, reserved
    AP_GROUPINFO("OUT", 2, FD_CAN, _out, 0),

    // No use, reserved
    AP_GROUPINFO("FAN", 3, FD_CAN, _enable_fan, 0),

    AP_GROUPINFO("MOT", 4, FD_CAN, _enable_mot, 1),

    AP_GROUPINFO("BMS", 5, FD_CAN, _enable_bms, 1),

    AP_GROUPEND};

FD_CAN::FD_CAN() {
    AP_Param::setup_object_defaults(this, var_info);

    _fan_ptr = new FD_FAN(this);

    for (uint8_t i_mot = 0; i_mot < FD_CAN_MAX_MOT_NUM; i_mot++)
    {
        _mot_ptr[i_mot] = new FD_MOT(this);
        if (_mot_ptr[i_mot] != nullptr)
        {
            _mot_ptr[i_mot]->set_id(i_mot+1);
        }
    }

    _bms_ptr = new FD_BMS(this);

    debug_can(AP_CANManager::LOG_INFO, "CAN_FD: constructed\n\r");
}

FD_CAN *FD_CAN::get_can_fd(uint8_t driver_index) {
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_driver_type(driver_index) !=
            AP_CANManager::Driver_Type_FDCAN) {
        return nullptr;
    }

    return static_cast<FD_CAN *>(AP::can().get_driver(driver_index));
}

bool FD_CAN::add_interface(AP_HAL::CANIface *can_iface) {
    gcs().send_text(MAV_SEVERITY_INFO, "add_interface");
    if (_can_iface != nullptr) {
        debug_can(AP_CANManager::LOG_ERROR,
                  "CAN_FD: Multiple Interface not supported\n\r");
        return false;
    }

    _can_iface = can_iface;

    if (_can_iface == nullptr) {
        debug_can(AP_CANManager::LOG_ERROR, "FDCAN: CAN driver not found\n\r");
        return false;
    }

    if (!_can_iface->is_initialized()) {
        debug_can(AP_CANManager::LOG_ERROR, "FDCAN: Driver not initialized\n\r");
        return false;
    }

    if (!_can_iface->set_event_handle(&sem_handle)) {
        debug_can(AP_CANManager::LOG_ERROR, "FDCAN: Cannot add event handle\n\r");
        return false;
    }
    return true;
}

// initialize CAN_FD bus
void FD_CAN::init(uint8_t driver_index, bool enable_filters) {
    gcs().send_text(MAV_SEVERITY_INFO, "CAN_FD: starting init\n\r");
    _driver_index = driver_index;

    debug_can(AP_CANManager::LOG_DEBUG, "CAN_FD: starting init\n\r");

    if (_initialized) {
        debug_can(AP_CANManager::LOG_ERROR, "CAN_FD: already initialized\n\r");
        return;
    }

    hal.util->snprintf(_thread_name, sizeof(_thread_name), "FD_CAN_%u", driver_index);
    // start calls to loop in separate thread
    if (!hal.scheduler->thread_create(
            FUNCTOR_BIND_MEMBER(&FD_CAN::loop, void), _thread_name, 4096,
            AP_HAL::Scheduler::PRIORITY_MAIN, 1)) {
        debug_can(AP_CANManager::LOG_ERROR,
                  "CAN_FD: couldn't create thread\n\r");
        return;
    }

    _initialized = true;

    debug_can(AP_CANManager::LOG_DEBUG, "CAN_FD: init done\n\r");
    gcs().send_text(MAV_SEVERITY_INFO, "CAN_FD: init done\n\r");
}

// loop to send output to CAN devices in background thread
void FD_CAN::loop() {
    AP_HAL::CANFrame txFrame{};
    AP_HAL::CANFrame rxFrame{};

    // 新增：用于定时发送MAVLink消息
    uint32_t last_mav_send_ms = 0;

    while (true) {
        if (!_initialized) {
            debug_can(AP_CANManager::LOG_ERROR, "CAN_FD: not initialized\n\r");
            hal.scheduler->delay_microseconds(10000);
            continue;
        }

        while (read_frame(rxFrame, 0)) {
            if (_print.get()) {
                gcs().send_text(MAV_SEVERITY_INFO, "rxFrame.id 0x%lX", (unsigned long)(rxFrame.id & 0x1FFFFFFF));
            }
            
            if (_fan_ptr != nullptr && _enable_fan.get() != 0) {
                _fan_ptr->handle_info(rxFrame, _print.get());
            }

            for (uint8_t i_mot = 0; i_mot < FD_CAN_MAX_MOT_NUM; i_mot++)
            {
                if (_mot_ptr[i_mot] != nullptr)
                {
                    _mot_ptr[i_mot]->handle_info(rxFrame, _print.get());
                }
            }

            // 新增：处理BMS数据
            if (_bms_ptr != nullptr && _enable_bms.get() != 0) {
                _bms_ptr->handle_info(rxFrame, _print.get());
            }
        }

        if (_fan_ptr != nullptr && _enable_fan.get() != 0) {
            _fan_ptr->update();
        }

        if (_enable_mot.get()) {   
            for (uint8_t i_mot = 0; i_mot < FD_CAN_MAX_MOT_NUM; i_mot++) { 
                uint16_t mot_output = 0;
                if (SRV_Channels::get_output_pwm(SRV_Channel::k_motor1, mot_output)) {
                    ;
                }
                _mot_ptr[i_mot]->set_pwm(mot_output);
                _mot_ptr[i_mot]->update();
            }
        }

        // =====================================================
        // 新增：周期性发送自定义MAVLink消息 (MOTOR_RPM, BATTERY_ERROR, BATTERY_INFO)
        // 发送频率：2Hz (每500ms发送一次)
        // =====================================================
        if (AP_HAL::millis() - last_mav_send_ms > 500) {
            last_mav_send_ms = AP_HAL::millis();
            
            // ==================== 调试输出区域 ====================
            
            // ---------- 电机1-4转速调试(全部) ----------
            gcs().send_text(MAV_SEVERITY_INFO, "RPM 1-4: %d %d %d %d", 
                AP::fd_data().motor_rpm_packet.motor_rpm[0],
                AP::fd_data().motor_rpm_packet.motor_rpm[1],
                AP::fd_data().motor_rpm_packet.motor_rpm[2],
                AP::fd_data().motor_rpm_packet.motor_rpm[3]);
            
            // ---------- 电机5-8转速调试 ----------
            // gcs().send_text(MAV_SEVERITY_INFO, "RPM 5-8: %d %d %d %d", 
            //     AP::fd_data().motor_rpm_packet.motor_rpm[4],
            //     AP::fd_data().motor_rpm_packet.motor_rpm[5],
            //     AP::fd_data().motor_rpm_packet.motor_rpm[6],
            //     AP::fd_data().motor_rpm_packet.motor_rpm[7]);
            
            // ---------- 电机9-12转速调试 ----------
            // gcs().send_text(MAV_SEVERITY_INFO, "RPM 9-12: %d %d %d %d", 
            //     AP::fd_data().motor_rpm_packet.motor_rpm[8],
            //     AP::fd_data().motor_rpm_packet.motor_rpm[9],
            //     AP::fd_data().motor_rpm_packet.motor_rpm[10],
            //     AP::fd_data().motor_rpm_packet.motor_rpm[11]);
            
            // ---------- 电机13-16转速调试 ----------
            // gcs().send_text(MAV_SEVERITY_INFO, "RPM 13-16: %d %d %d %d", 
            //     AP::fd_data().motor_rpm_packet.motor_rpm[12],
            //     AP::fd_data().motor_rpm_packet.motor_rpm[13],
            //     AP::fd_data().motor_rpm_packet.motor_rpm[14],
            //     AP::fd_data().motor_rpm_packet.motor_rpm[15]);
            
            // ---------- 电机1-4温度调试 ----------
            gcs().send_text(MAV_SEVERITY_INFO, "TEMP 1-4: %d %d %d %d", 
                AP::fd_data().motor_rpm_packet.motor_temp[0],
                AP::fd_data().motor_rpm_packet.motor_temp[1],
                AP::fd_data().motor_rpm_packet.motor_temp[2],
                AP::fd_data().motor_rpm_packet.motor_temp[3]);
            
            // ---------- 电机5-8温度调试 ----------
            // gcs().send_text(MAV_SEVERITY_INFO, "TEMP 5-8: %d %d %d %d", 
            //     AP::fd_data().motor_rpm_packet.motor_temp[4],
            //     AP::fd_data().motor_rpm_packet.motor_temp[5],
            //     AP::fd_data().motor_rpm_packet.motor_temp[6],
            //     AP::fd_data().motor_rpm_packet.motor_temp[7]);
            
            // ---------- 电机9-12温度调试 ----------
            // gcs().send_text(MAV_SEVERITY_INFO, "TEMP 9-12: %d %d %d %d", 
            //     AP::fd_data().motor_rpm_packet.motor_temp[8],
            //     AP::fd_data().motor_rpm_packet.motor_temp[9],
            //     AP::fd_data().motor_rpm_packet.motor_temp[10],
            //     AP::fd_data().motor_rpm_packet.motor_temp[11]);
            
            // ---------- 电机13-16温度调试 ----------
            // gcs().send_text(MAV_SEVERITY_INFO, "TEMP 13-16: %d %d %d %d", 
            //     AP::fd_data().motor_rpm_packet.motor_temp[12],
            //     AP::fd_data().motor_rpm_packet.motor_temp[13],
            //     AP::fd_data().motor_rpm_packet.motor_temp[14],
            //     AP::fd_data().motor_rpm_packet.motor_temp[15]);
            
            // ---------- BMS电压电流调试 ----------
            gcs().send_text(MAV_SEVERITY_INFO, "BMS: V=%dmV I=%dmA SOC=%d%%", 
                AP::fd_data().battery_info_packet.voltage,
                AP::fd_data().battery_info_packet.current,
                AP::fd_data().battery_info_packet.battery_remaining);
            
            // ---------- BMS故障码调试 ----------
            gcs().send_text(MAV_SEVERITY_INFO, "BMS ERR: code=0x%04X id=%d", 
                AP::fd_data().battery_error_packet.error_code,
                AP::fd_data().battery_error_packet.battery_id);
            
            // ---------- BMS详细状态调试 (需要_bms_ptr有效) ----------
            if (_bms_ptr != nullptr) {
                gcs().send_text(MAV_SEVERITY_INFO, "BMS: SOC=%d SOH=%d V=%.1f I=%.1f", 
                    _bms_ptr->status.SOC,
                    _bms_ptr->status.SOH,
                    _bms_ptr->status.Volt,
                    _bms_ptr->status.Curr);
                gcs().send_text(MAV_SEVERITY_INFO, "BMS: chg=%d dischg=%d chg_err=%d dischg_err=%d", 
                    _bms_ptr->status.allow_charge,
                    _bms_ptr->status.allow_discharge,
                    _bms_ptr->status.error_charge_code,
                    _bms_ptr->status.error_discharge_code);
            }
            
            // ==================== 调试输出区域结束 ====================
            
            // 遍历所有MAVLink通道并发送消息
            for (uint8_t i = 0; i < gcs().num_gcs(); i++) {
                mavlink_channel_t chan = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
                
                // 发送电机转速和温度数据 (MSG ID: 13000)
                if (_enable_mot.get()) {
                    if (HAVE_PAYLOAD_SPACE(chan, MOTOR_RPM)) {
                        AP::fd_data().send_motor_rpm(chan);
                    }
                }
                
                // 发送电池错误信息 (MSG ID: 13001)
                if (_enable_bms.get()) {
                    if (HAVE_PAYLOAD_SPACE(chan, BATTERY_ERROR)) {
                        AP::fd_data().send_battery_error(chan);
                    }
                }
                
                // 发送电池电压电流信息 (MSG ID: 13002)
                if (_enable_bms.get()) {
                    if (HAVE_PAYLOAD_SPACE(chan, BATTERY_INFO)) {
                        AP::fd_data().send_battery_info(chan);
                    }
                }
            }
        }
        // =====================================================

        // 1ms loop delay
        hal.scheduler->delay_microseconds(1000);
    }
}

// write frame on CAN bus, returns true on success
bool FD_CAN::write_frame(AP_HAL::CANFrame &out_frame, uint64_t timeout) {
    if (!_initialized) {
        debug_can(AP_CANManager::LOG_ERROR,
                  "CAN_FD: Driver not initialized for write_frame\n\r");
        return false;
    }

    bool read_select = false;
    bool write_select = true;

    bool ret = _can_iface->select(read_select, write_select, &out_frame, timeout);

    if (!ret || !write_select) {
        return false;
    }

    return (_can_iface->send(out_frame, timeout, 0));
}

// read frame on CAN bus, returns true on succses
bool FD_CAN::read_frame(AP_HAL::CANFrame &recv_frame, uint64_t timeout) {
    if (!_initialized) {
        debug_can(AP_CANManager::LOG_ERROR,
                  "CAN_FD: Driver not initialized for read_frame\n\r");
        return false;
    }
    bool read_select = true;
    bool write_select = false;
    bool ret = _can_iface->select(read_select, write_select, nullptr, timeout);

    if (!ret || !read_select) {
        // No frame available
        return false;
    }

    uint64_t time;
    AP_HAL::CANIface::CanIOFlags flags{};

    return (_can_iface->receive(recv_frame, time, flags) == 1);
}

// called from high level code
void FD_CAN::update() {
    ;
}

bool FD_CAN::pre_arm_check(char *reason, uint8_t reason_len) {
    snprintf(reason, reason_len, "FD CAN");
    return true;
}