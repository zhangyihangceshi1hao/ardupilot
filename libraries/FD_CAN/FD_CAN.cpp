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

// =====================================================
// 使用TUNNEL消息发送电机数据
// payload_type = 13000 (MOTOR_RPM)
// 载荷结构: time_boot_ms(4) + motor_rpm[16](32) + motor_temp[16](16) = 52字节
// =====================================================
void FD_CAN::send_motor_rpm_via_tunnel(mavlink_channel_t chan)
{
    // TUNNEL消息payload固定128字节，必须使用完整大小的数组
    uint8_t payload[128];
    memset(payload, 0, sizeof(payload));  // 先清零
    uint8_t idx = 0;
    
    // 1. 时间戳 (4字节, 小端序)
    uint32_t time_ms = AP_HAL::millis();
    payload[idx++] = (uint8_t)(time_ms & 0xFF);
    payload[idx++] = (uint8_t)((time_ms >> 8) & 0xFF);
    payload[idx++] = (uint8_t)((time_ms >> 16) & 0xFF);
    payload[idx++] = (uint8_t)((time_ms >> 24) & 0xFF);
    
    // 2. 16个电机RPM (每个2字节, 小端序)
    for (int i = 0; i < 16; i++) {
        uint16_t rpm = AP::fd_data().motor_rpm_packet.motor_rpm[i];
        payload[idx++] = (uint8_t)(rpm & 0xFF);
        payload[idx++] = (uint8_t)((rpm >> 8) & 0xFF);
    }
    
    // 3. 16个电机温度 (每个1字节, 有符号)
    for (int i = 0; i < 16; i++) {
        payload[idx++] = (uint8_t)AP::fd_data().motor_rpm_packet.motor_temp[i];
    }
    
    // 发送TUNNEL消息, payload_length指定实际有效数据长度
    mavlink_msg_tunnel_send(
        chan,
        0,                              // target_system (0 = 广播)
        0,                              // target_component (0 = 广播)
        TUNNEL_PAYLOAD_TYPE_MOTOR_RPM,  // payload_type = 13000
        52,                             // payload_length (实际数据长度)
        payload
    );
}

// =====================================================
// 使用TUNNEL消息发送电池错误信息
// payload_type = 13001 (BATTERY_ERROR)
// 载荷结构: time_boot_ms(4) + error_code(2) + battery_id(1) = 7字节
// =====================================================
void FD_CAN::send_battery_error_via_tunnel(mavlink_channel_t chan)
{
    // TUNNEL消息payload固定128字节
    uint8_t payload[128];
    memset(payload, 0, sizeof(payload));
    uint8_t idx = 0;
    
    // 1. 时间戳 (4字节)
    uint32_t time_ms = AP_HAL::millis();
    payload[idx++] = (uint8_t)(time_ms & 0xFF);
    payload[idx++] = (uint8_t)((time_ms >> 8) & 0xFF);
    payload[idx++] = (uint8_t)((time_ms >> 16) & 0xFF);
    payload[idx++] = (uint8_t)((time_ms >> 24) & 0xFF);
    
    // 2. 错误码 (2字节)
    uint16_t error_code = AP::fd_data().battery_error_packet.error_code;
    payload[idx++] = (uint8_t)(error_code & 0xFF);
    payload[idx++] = (uint8_t)((error_code >> 8) & 0xFF);
    
    // 3. 电池ID (1字节)
    payload[idx++] = AP::fd_data().battery_error_packet.battery_id;
    
    // 发送TUNNEL消息
    mavlink_msg_tunnel_send(
        chan,
        0,
        0,
        TUNNEL_PAYLOAD_TYPE_BATTERY_ERROR,  // payload_type = 13001
        7,                                   // payload_length (实际数据长度)
        payload
    );
}

// =====================================================
// 使用TUNNEL消息发送电池信息
// payload_type = 13002 (BATTERY_INFO)
// 载荷结构: time_boot_ms(4) + voltage(2) + current(2) + battery_remaining(1) = 9字节
// =====================================================
void FD_CAN::send_battery_info_via_tunnel(mavlink_channel_t chan)
{
    // TUNNEL消息payload固定128字节
    uint8_t payload[128];
    memset(payload, 0, sizeof(payload));
    uint8_t idx = 0;
    
    // 1. 时间戳 (4字节)
    uint32_t time_ms = AP_HAL::millis();
    payload[idx++] = (uint8_t)(time_ms & 0xFF);
    payload[idx++] = (uint8_t)((time_ms >> 8) & 0xFF);
    payload[idx++] = (uint8_t)((time_ms >> 16) & 0xFF);
    payload[idx++] = (uint8_t)((time_ms >> 24) & 0xFF);
    
    // 2. 电压 (2字节, mV)
    uint16_t voltage = AP::fd_data().battery_info_packet.voltage;
    payload[idx++] = (uint8_t)(voltage & 0xFF);
    payload[idx++] = (uint8_t)((voltage >> 8) & 0xFF);
    
    // 3. 电流 (2字节, mA, 有符号)
    int16_t current = AP::fd_data().battery_info_packet.current;
    payload[idx++] = (uint8_t)(current & 0xFF);
    payload[idx++] = (uint8_t)((current >> 8) & 0xFF);
    
    // 4. 剩余电量 (1字节, %)
    payload[idx++] = AP::fd_data().battery_info_packet.battery_remaining;
    
    // 发送TUNNEL消息
    mavlink_msg_tunnel_send(
        chan,
        0,
        0,
        TUNNEL_PAYLOAD_TYPE_BATTERY_INFO,  // payload_type = 13002
        9,                                  // payload_length (实际数据长度)
        payload
    );
}

// =====================================================
// 处理接收到的TUNNEL消息 (新增)
// 用于接收地面站发来的POWER_CONTROL命令
// =====================================================
void FD_CAN::handle_mavlink_tunnel(const mavlink_tunnel_t& tunnel)
{
    // 只处理 POWER_CONTROL 消息 (payload_type = 13003)
    if (tunnel.payload_type != TUNNEL_PAYLOAD_TYPE_POWER_CONTROL) {
        return;
    }
    
    // 检查载荷长度
    // POWER_CONTROL载荷结构 (8字节):
    // [0-3] time_boot_ms: uint32 (小端序)
    // [4]   target_system: uint8
    // [5]   target_component: uint8
    // [6]   command: uint8 (0=无操作, 1=上电, 2=断电)
    // [7]   channel: uint8 (0-15, 255=所有通道)
    if (tunnel.payload_length < 8) {
        gcs().send_text(MAV_SEVERITY_WARNING, "POWER_CTRL: invalid len=%d", tunnel.payload_length);
        return;
    }
    
    // 解析命令
    uint8_t command = tunnel.payload[6];
    uint8_t channel = tunnel.payload[7];
    
    gcs().send_text(MAV_SEVERITY_INFO, "POWER_CTRL RX: cmd=%d ch=%d", command, channel);
    
    // 获取FD_CAN实例
    FD_CAN* fd_can = FD_CAN::get_can_fd(0);
    if (fd_can == nullptr) {
        gcs().send_text(MAV_SEVERITY_ERROR, "POWER_CTRL: FD_CAN not found");
        return;
    }
    
    // 检查BMS指针
    if (fd_can->_bms_ptr == nullptr) {
        gcs().send_text(MAV_SEVERITY_ERROR, "POWER_CTRL: BMS not init");
        return;
    }
    
    // 检查BMS是否启用
    if (fd_can->_enable_bms.get() == 0) {
        gcs().send_text(MAV_SEVERITY_WARNING, "POWER_CTRL: BMS disabled");
        return;
    }
    
    // 执行命令
    switch (command) {
        case 1:  // 上电/闭合
            fd_can->_bms_ptr->set_switch(1);
            gcs().send_text(MAV_SEVERITY_INFO, "BMS: Power ON sent");
            break;
            
        case 2:  // 断电/断开
            fd_can->_bms_ptr->set_switch(0);
            gcs().send_text(MAV_SEVERITY_INFO, "BMS: Power OFF sent");
            break;
            
        case 0:  // 无操作
        default:
            gcs().send_text(MAV_SEVERITY_WARNING, "POWER_CTRL: no action cmd=%d", command);
            break;
    }
}

// loop to send output to CAN devices in background thread
void FD_CAN::loop() {
    AP_HAL::CANFrame txFrame{};
    AP_HAL::CANFrame rxFrame{};

    // 用于定时发送MAVLink消息
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

            // 处理BMS数据
            if (_bms_ptr != nullptr && _enable_bms.get() != 0) {
                _bms_ptr->handle_info(rxFrame, _print.get());
            }
        }

        if (_fan_ptr != nullptr && _enable_fan.get() != 0) {
            _fan_ptr->update();
        }

        if (_enable_mot.get()) {
            uint16_t pwm[4] = {};
            SRV_Channels::get_output_pwm(SRV_Channel::k_motor1, pwm[0]);
            SRV_Channels::get_output_pwm(SRV_Channel::k_motor2, pwm[1]);
            SRV_Channels::get_output_pwm(SRV_Channel::k_motor3, pwm[2]);
            SRV_Channels::get_output_pwm(SRV_Channel::k_motor4, pwm[3]);

            if (AP_HAL::millis() - _last_mot_ms > 20) {
                _last_mot_ms = AP_HAL::millis();

                // k_motor1(右前)→电机3/4, k_motor2(左前)→电机1/2
                // k_motor3(左后)→电机7/8, k_motor4(右后)→电机5/6
                // group1: 电机1/2(bytes0-3)=k_motor2(左前), 电机3/4(bytes4-7)=k_motor1(右前)
                // group2: 电机5/6(bytes0-3)=k_motor4(右后), 电机7/8(bytes4-7)=k_motor3(左后)
                uint8_t data1[8], data2[8];

                data1[0] = (uint8_t)((pwm[1] >> 8) & 0xFF);  // 电机1/2 = k_motor2(后右)
                data1[1] = (uint8_t)(pwm[1] & 0xFF);
                data1[2] = (uint8_t)((pwm[1] >> 8) & 0xFF);
                data1[3] = (uint8_t)(pwm[1] & 0xFF);
                data1[4] = (uint8_t)((pwm[0] >> 8) & 0xFF);  // 电机3/4 = k_motor1(前右)
                data1[5] = (uint8_t)(pwm[0] & 0xFF);
                data1[6] = (uint8_t)((pwm[0] >> 8) & 0xFF);
                data1[7] = (uint8_t)(pwm[0] & 0xFF);

                data2[0] = (uint8_t)((pwm[3] >> 8) & 0xFF);  // 电机5/6 = k_motor4(右后)
                data2[1] = (uint8_t)(pwm[3] & 0xFF);
                data2[2] = (uint8_t)((pwm[3] >> 8) & 0xFF);
                data2[3] = (uint8_t)(pwm[3] & 0xFF);
                data2[4] = (uint8_t)((pwm[2] >> 8) & 0xFF);  // 电机7/8 = k_motor3(左后)
                data2[5] = (uint8_t)(pwm[2] & 0xFF);
                data2[6] = (uint8_t)((pwm[2] >> 8) & 0xFF);
                data2[7] = (uint8_t)(pwm[2] & 0xFF);

                if (_mot_ptr[0] != nullptr) {
                    _mot_ptr[0]->send_cmd(0x14661C27 | AP_HAL::CANFrame::FlagEFF, data1);
                    _mot_ptr[0]->send_cmd(0x14671C27 | AP_HAL::CANFrame::FlagEFF, data2);
                }
            }
        }

        // =====================================================
        // 周期性发送数据 (使用TUNNEL消息)
        // 发送频率：2Hz (每500ms发送一次)
        // =====================================================
        if (AP_HAL::millis() - last_mav_send_ms > 500) {
            last_mav_send_ms = AP_HAL::millis();
            
            // ==================== 调试输出 ====================

            // gcs().send_text(MAV_SEVERITY_INFO, "RPM 1-4: %d %d %d %d", 
            //     AP::fd_data().motor_rpm_packet.motor_rpm[0],
            //     AP::fd_data().motor_rpm_packet.motor_rpm[1],
            //     AP::fd_data().motor_rpm_packet.motor_rpm[2],
            //     AP::fd_data().motor_rpm_packet.motor_rpm[3]);
            
            // gcs().send_text(MAV_SEVERITY_INFO, "TEMP 1-4: %d %d %d %d", 
            //     AP::fd_data().motor_rpm_packet.motor_temp[0],
            //     AP::fd_data().motor_rpm_packet.motor_temp[1],
            //     AP::fd_data().motor_rpm_packet.motor_temp[2],
            //     AP::fd_data().motor_rpm_packet.motor_temp[3]);
            
            // gcs().send_text(MAV_SEVERITY_INFO, "BMS: V=%dmV I=%dmA SOC=%d%%", 
            //     AP::fd_data().battery_info_packet.voltage,
            //     AP::fd_data().battery_info_packet.current,
            //     AP::fd_data().battery_info_packet.battery_remaining);
            
            // gcs().send_text(MAV_SEVERITY_INFO, "BMS ERR: code=0x%04X id=%d", 
            //     AP::fd_data().battery_error_packet.error_code,
            //     AP::fd_data().battery_error_packet.battery_id);

            
            // ==================== 使用TUNNEL消息发送数据 ====================
            for (uint8_t i = 0; i < gcs().num_gcs(); i++) {
                mavlink_channel_t chan = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
                
                // 发送电机转速和温度数据 (via TUNNEL, payload_type=13000)
                if (_enable_mot.get()) {
                    if (HAVE_PAYLOAD_SPACE(chan, TUNNEL)) {
                        send_motor_rpm_via_tunnel(chan);
                    }
                }
                
                // 发送电池错误信息 (via TUNNEL, payload_type=13001)
                if (_enable_bms.get()) {
                    if (HAVE_PAYLOAD_SPACE(chan, TUNNEL)) {
                        send_battery_error_via_tunnel(chan);
                    }
                }
                
                // 发送电池电压电流信息 (via TUNNEL, payload_type=13002)
                if (_enable_bms.get()) {
                    if (HAVE_PAYLOAD_SPACE(chan, TUNNEL)) {
                        send_battery_info_via_tunnel(chan);
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