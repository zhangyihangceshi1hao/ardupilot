#include "AP_CAN_LIDE.h"

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
#include <stdio.h>

extern const AP_HAL::HAL &hal;

// 辅助函数：发送GCS调试信息（中文）
static void send_gcs_text(MAV_SEVERITY severity, const char *fmt, ...) {
    char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1];
    va_list ap;
    va_start(ap, fmt);
    hal.util->vsnprintf(text, sizeof(text), fmt, ap);
    va_end(ap);
    gcs().send_text(severity, "%s", text);
}

// 构造函数
AP_CAN_LIDE::AP_CAN_LIDE() {
    // 这行很重要：设置参数表的指针
    AP_Param::setup_object_defaults(this, var_info);

    // 初始化发动机数组
    for (int i = 0; i < LIDE_NODE_ID_MAX; i++) {
        _engines[i].node_id = i + 1 + _node_id_offset;
        _engines[i].enabled = (_engine_bm & (1 << i));
        _engines[i].is_online = false;
        _engines[i].is_running = false;
        _engines[i].is_heating = false;
        _engines[i].throttle_request = 0;
        _engines[i].engine_rpm = 0;
        _engines[i].engine_status = 0;
        _engines[i].last_update_ms = 0;
        
        // 初始化控制命令
        _engines[i].control_cmd.bits.stop_cmd = 0;
        _engines[i].control_cmd.bits.heating_cmd = 0;
        _engines[i].control_cmd.bits.start_cmd = 0;
        _engines[i].control_cmd.bits.start_valid = 0;
        _engines[i].control_cmd.bits.heating_valid = 0;
        _engines[i].control_cmd.bits.altitude_valid = 0;
        _engines[i].control_cmd.bits.airspeed_valid = 0;
        
        // 初始化故障字节
        for (int j = 0; j < 8; j++) {
            _engines[i].fault_bytes[j] = 0;
        }
    }

    send_gcs_text(MAV_SEVERITY_INFO, "砺德CAN驱动: 构造完成");
}


// ============ 重要：添加静态成员变量的定义 ============
const AP_Param::GroupInfo AP_CAN_LIDE::var_info[] = {
    // @Param: ENG_BM
    // @DisplayName: 发动机通道
    // @Description: 位掩码定义哪些发动机通道要通过CAN传输
    // @Bitmask: 0: 发动机1, 1: 发动机2, 2: 发动机3, 3: 发动机4, 4: 发动机5, 5: 发动机6, 6: 发动机7, 7: 发动机8
    // @User: Advanced
    AP_GROUPINFO("ENG_BM", 1, AP_CAN_LIDE, _engine_bm, 0xFF),

    // @Param: ENG_RT
    // @DisplayName: 发动机命令更新率
    // @Description: 发动机命令消息的输出频率
    // @Units: Hz
    // @User: Advanced
    // @Range: 10 100
    AP_GROUPINFO("ENG_RT", 2, AP_CAN_LIDE, _update_hz, LIDE_MSG_RATE_HZ_DEFAULT),

    // @Param: NODE_OFFSET
    // @DisplayName: 节点ID偏移
    // @Description: 发动机的CAN节点ID偏移量
    // @User: Advanced
    // @Range: 0 240
    AP_GROUPINFO("NODE_OFFSET", 3, AP_CAN_LIDE, _node_id_offset, 0),

    AP_GROUPEND
};

AP_CAN_LIDE::~AP_CAN_LIDE() {
    // 清理资源
}

// 获取实例
AP_CAN_LIDE *AP_CAN_LIDE::get_can_lide(uint8_t driver_index) {
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_driver_type(driver_index) !=
            AP_CANManager::Driver_Type_CAN_LIDE) {
        return nullptr;
    }

    return static_cast<AP_CAN_LIDE *>(AP::can().get_driver(driver_index));
}

// 添加CAN接口
bool AP_CAN_LIDE::add_interface(AP_HAL::CANIface *can_iface) {
    if (_can_iface != nullptr) {
        send_gcs_text(MAV_SEVERITY_ERROR, "砺德CAN: 不支持多个接口");
        return false;
    }

    _can_iface = can_iface;

    if (_can_iface == nullptr) {
        send_gcs_text(MAV_SEVERITY_ERROR, "砺德CAN: 未找到CAN驱动");
        return false;
    }

    if (!_can_iface->is_initialized()) {
        send_gcs_text(MAV_SEVERITY_ERROR, "砺德CAN: 驱动未初始化");
        return false;
    }

    if (!_can_iface->set_event_handle(&_event_handle)) {
        send_gcs_text(MAV_SEVERITY_ERROR, "砺德CAN: 无法添加事件句柄");
        return false;
    }
    
    return true;
}

// 初始化
void AP_CAN_LIDE::init(uint8_t driver_index, bool enable_filters) {
    _driver_index = driver_index;

    send_gcs_text(MAV_SEVERITY_DEBUG, "砺德CAN: 开始初始化");

    if (_initialized) {
        send_gcs_text(MAV_SEVERITY_ERROR, "砺德CAN: 已经初始化");
        return;
    }
    
    // 创建后台线程
    if (!hal.scheduler->thread_create(
            FUNCTOR_BIND_MEMBER(&AP_CAN_LIDE::loop, void), 
            _thread_name, 
            4096,
            AP_HAL::Scheduler::PRIORITY_MAIN, 
            1)) {
        send_gcs_text(MAV_SEVERITY_ERROR, "砺德CAN: 无法创建线程");
        return;
    }

    _initialized = true;
    snprintf(_thread_name, sizeof(_thread_name), "CAN_LIDE_%u", driver_index);

    send_gcs_text(MAV_SEVERITY_DEBUG, "砺德CAN: 初始化完成");
}

// 后台线程循环
void AP_CAN_LIDE::loop() {
    AP_HAL::CANFrame rxFrame{};
    uint32_t loop_counter = 0;
    uint32_t debug_counter = 0;
    bool _status_printed = false;  // 添加状态打印标志
    
    while (true) {
        if (!_initialized) {
            send_gcs_text(MAV_SEVERITY_ERROR, "砺德CAN: 未初始化");
            hal.scheduler->delay_microseconds(10000);
            continue;
        }
        
        // 初始化完成后打印状态
        if (!_status_printed) {
            _status_printed = true;
            send_gcs_text(MAV_SEVERITY_INFO, "砺德CAN: 线程循环开始运行");
        }
        
        // 获取当前时间
        uint64_t now_us = AP_HAL::micros64();
        uint32_t now_ms = now_us / 1000;  // 转换为毫秒
        
        // 接收处理CAN帧
        
       
        static uint32_t last_frame_print_ms = 0;
        static uint8_t frame_counter_since_last_print = 0;

 
        // 接收处理CAN帧
        // 接收处理CAN帧
        bool received_frame = false;

        while (read_frame(rxFrame, 0)) {
            received_frame = true;
            
            // 打印帧头信息
            send_gcs_text(MAV_SEVERITY_INFO, 
                        "=== CAN数据帧 ===");
            send_gcs_text(MAV_SEVERITY_INFO,
                        "帧ID: 0x%03lX", (unsigned long)rxFrame.id);
            send_gcs_text(MAV_SEVERITY_INFO,
                        "数据长度: %lu字节", (unsigned long)rxFrame.dlc);
            
            // 逐个打印每个字节
            if (rxFrame.dlc > 0) {
                send_gcs_text(MAV_SEVERITY_INFO, "数据内容:");
                
                for (uint8_t i = 0; i < rxFrame.dlc; i++) {
                    send_gcs_text(MAV_SEVERITY_INFO,
                                "  字节[%u]: 0x%02X (十进制: %3u)",
                                (unsigned int)i,
                                (unsigned int)rxFrame.data[i],                                                                                              
                                (unsigned int)rxFrame.data[i]);
                }
            } else {
                send_gcs_text(MAV_SEVERITY_INFO, "无数据内容");
            }
            
            // 打印时间
            send_gcs_text(MAV_SEVERITY_INFO,
                        "接收时间戳: %lu微秒", 
                        (unsigned long)(now_us % 1000000));
            
            // 处理状态帧
            process_status_frame(rxFrame);
        }

        // 每秒重置计数器
        if (now_ms - last_frame_print_ms >= 1000) {
            last_frame_print_ms = now_ms;
            if (frame_counter_since_last_print > 0) {
                send_gcs_text(MAV_SEVERITY_DEBUG, "砺德CAN: 收到%d帧", frame_counter_since_last_print);
                frame_counter_since_last_print = 0;
            }
        }
        
        // 定时发送控制命令 (根据配置的频率)
        _update_hz.set(constrain_int16(_update_hz, LIDE_MSG_RATE_HZ_MIN, LIDE_MSG_RATE_HZ_MAX));
        uint32_t update_period_ms = 1000 / _update_hz;
        
        if (loop_counter >= update_period_ms) {
            loop_counter = 0;
            
            // 发送所有已使能的发动机控制命令
            for (int i = 0; i < LIDE_NODE_ID_MAX; i++) {
                if (_engines[i].enabled) {
                    send_control_command(i);
                }
            }
        }
        loop_counter++;
        
        // 更新在线状态 - 添加防抖逻辑
        static uint32_t last_status_change_ms[LIDE_NODE_ID_MAX] = {0};
        for (int i = 0; i < LIDE_NODE_ID_MAX; i++) {
            if (_engines[i].enabled) {
                bool was_online = _engines[i].is_online;
                
                // 如果有收到帧，更新时间为当前时间
                if (received_frame) {
                    _engines[i].last_update_ms = now_ms;
                }
                
                // 检查是否超时
                uint32_t time_since_update = now_ms - _engines[i].last_update_ms;
                _engines[i].is_online = (time_since_update < LIDE_ONLINE_TIMEOUT_MS);
                
                // 防抖逻辑，避免频繁切换
                if (was_online != _engines[i].is_online) {
                    uint32_t time_since_last_change = now_ms - last_status_change_ms[i];
                    if (time_since_last_change > 1000) {  // 至少1秒才允许状态变化
                        last_status_change_ms[i] = now_ms;
                        send_gcs_text(MAV_SEVERITY_INFO, "发动机 %d %s", 
                                   i + 1, _engines[i].is_online ? "在线" : "离线");
                    } else {
                        // 1秒内频繁变化，保持原状态
                        _engines[i].is_online = was_online;
                    }
                }
            }
        }
        
        // 定时记录日志 (10Hz)
        if (now_ms - _last_log_ms >= LIDE_STATUS_LOG_PERIOD_MS) {
            _last_log_ms = now_ms;
            log_engine_status();
        }
        
        // 定期打印调试信息（每秒一次）
        debug_counter++;
        if (debug_counter >= 1000) {
            debug_counter = 0;
            // 只打印第一个使能的发动机状态
            for (int i = 0; i < LIDE_NODE_ID_MAX; i++) {
                if (_engines[i].enabled && _engines[i].is_online) {
                    send_gcs_text(MAV_SEVERITY_DEBUG, "砺德 发动机%d: 转速=%d 油门=%.1f%%", 
                               i + 1, _engines[i].engine_rpm, _engines[i].throttle_request * 0.1f);
                    break;
                }
            }
        }
        
        // 1ms循环延迟
        hal.scheduler->delay_microseconds(1000);
    }
}

// 发送控制命令
void AP_CAN_LIDE::send_control_command(uint8_t engine_id) {
    // 修复：只发送到实际使能的发动机
    if (engine_id >= LIDE_NODE_ID_MAX || 
        !_engines[engine_id].enabled || 
        engine_id >= 1) {  // 如果你只配置了发动机1
        return;
    }
    
    LIDE_Engine_t &engine = _engines[engine_id];
    AP_HAL::CANFrame txFrame{};
    
    // 设置CAN ID
    txFrame.id = LIDE_CAN_ID_CONTROL;
    txFrame.dlc = 8;  // 根据协议，控制帧为8字节
    
    // 油门请求 (0-1000对应0-100%)
    uint16_t throttle_raw = engine.throttle_request;
    txFrame.data[0] = (throttle_raw >> 8) & 0xFF;  // 高字节
    txFrame.data[1] = throttle_raw & 0xFF;         // 低字节
    
    // 海拔高度 (米)
    txFrame.data[2] = (engine.altitude >> 8) & 0xFF;
    txFrame.data[3] = engine.altitude & 0xFF;
    
    // 空速 (m/s)
    uint8_t airspeed_raw = constrain_int16(engine.airspeed * 4, 0, 252);
    txFrame.data[4] = airspeed_raw;
    
    // 控制命令字节
    txFrame.data[5] = engine.control_cmd.value;
    
    // 预留字节 (设置为0)
    txFrame.data[6] = 0;
    txFrame.data[7] = 0;
    
   
    
    // 根据协议文档的bit定义，完整解析控制命令字节
    // uint8_t cmd = engine.control_cmd.value;

    // // 打印发送的数据帧信息
    // gcs().send_text(MAV_SEVERITY_INFO, "砺德发送: ID:0x%03lX 油门:%.1f%%", 
    //                 (unsigned long)txFrame.id,
    //                 throttle_raw * 0.1f);

   
    // gcs().send_text(MAV_SEVERITY_INFO, "控制字节: 0x%02X", (unsigned int)cmd);

    // // 控制指令位
    // gcs().send_text(MAV_SEVERITY_INFO, "停机:%d 加热:%d 启动:%d 操作:%d",
    //                 (cmd & 0x01) ? 1 : 0,
    //                 (cmd & 0x02) ? 1 : 0,
    //                 (cmd & 0x04) ? 1 : 0,
    //                 (cmd & 0x08) ? 1 : 0);

    // // 有效标志位
    // gcs().send_text(MAV_SEVERITY_INFO, "启效:%d 热效:%d 海效:%d 空效:%d",
    //                 (cmd & 0x10) ? 1 : 0,
    //                 (cmd & 0x20) ? 1 : 0,
    //                 (cmd & 0x40) ? 1 : 0,
    //                 (cmd & 0x80) ? 1 : 0);
    // 发送帧 - 增加重试机制
    bool send_success = false;
    static uint32_t last_fail_msg_ms = 0;
    uint32_t now_ms = AP_HAL::millis();
    
    for (int retry = 0; retry < 3; retry++) {
        if (write_frame(txFrame, AP_HAL::micros64() + 1000ULL)) {
            send_success = true;
            break;
        }
        hal.scheduler->delay_microseconds(100);
    }
    
    if (!send_success && (now_ms - last_fail_msg_ms > 5000)) {
        last_fail_msg_ms = now_ms;
        send_gcs_text(MAV_SEVERITY_WARNING, "砺德CAN: 发送控制命令到发动机 %d 失败", engine_id + 1);
    }
}


// 处理状态帧
void AP_CAN_LIDE::process_status_frame(const AP_HAL::CANFrame &frame) {
    uint32_t now_ms = AP_HAL::millis();
    static uint32_t last_rx_debug_ms = 0;
    static uint32_t frame_count = 0;
    
    frame_count++;
    
    // 定期打印接收统计
    if (now_ms - last_rx_debug_ms > 5000) {
        last_rx_debug_ms = now_ms;
        send_gcs_text(MAV_SEVERITY_DEBUG, "砺德CAN: 已处理 %d 帧状态数据", frame_count);
    }
    
    // 根据CAN ID分发处理
    switch (frame.id) {
        case LIDE_CAN_ID_STATUS1:
        case LIDE_CAN_ID_STATUS2:
        case LIDE_CAN_ID_STATUS3:
        case LIDE_CAN_ID_STATUS4:
        case LIDE_CAN_ID_STATUS5:
        case LIDE_CAN_ID_STATUS6:
        case LIDE_CAN_ID_STATUS7:
            // 找到对应的发动机
            for (int i = 0; i < LIDE_NODE_ID_MAX; i++) {
                if (_engines[i].enabled) {
                    _engines[i].last_update_ms = now_ms;
                    
                    // 根据帧ID处理不同数据
                    switch (frame.id) {
                        case LIDE_CAN_ID_STATUS1:
                            // 发动机系统状态
                            if (frame.dlc >= 8) {
                                // 解析数据
                                _engines[i].engine_status = (frame.data[0] >> 6) & 0x03;
                                _engines[i].is_running = ((frame.data[0] >> 4) & 0x01) != 0;
                                _engines[i].maintenance_status = frame.data[0] & 0x07;
                                
                                // 状态
                                const char* status_str = "未知";
                                switch (_engines[i].engine_status) {
                                    case LIDE_STATUS_NORMAL: status_str = "正常"; break;
                                    case LIDE_STATUS_ABNORMAL: status_str = "异常"; break;
                                    case LIDE_STATUS_WARNING: status_str = "警告"; break;
                                }
                                gcs().send_text(MAV_SEVERITY_INFO, "状态: %s 运行: %s", 
                                                status_str, 
                                                _engines[i].is_running ? "是" : "否");
                                
                                // 总运行时间
                                _engines[i].engine_runtime_hours = (frame.data[1] << 8 | frame.data[2]) * 0.1f;
                                gcs().send_text(MAV_SEVERITY_INFO, "总运行: %.1fh", (double)_engines[i].engine_runtime_hours);
                                
                                // 当前运行时间
                                _engines[i].engine_runtime_minutes = frame.data[3] << 8 | frame.data[4];
                                gcs().send_text(MAV_SEVERITY_INFO, "本次运行: %um", _engines[i].engine_runtime_minutes);
                                
                                // 油耗
                                _engines[i].fuel_consumption = (frame.data[5] << 8 | frame.data[6]) * 2;
                                _engines[i].fuel_rate_instant = frame.data[7] * 0.1f;
                                gcs().send_text(MAV_SEVERITY_INFO, "油耗: %uml 瞬时: %.1fL/h", 
                                                _engines[i].fuel_consumption, 
                                                (double)_engines[i].fuel_rate_instant);
                            }
                            break;
                            
                        case LIDE_CAN_ID_STATUS2:
                            // 缸头温度
                            if (frame.dlc >= 8) {
                                // 打印原始数据
                                gcs().send_text(MAV_SEVERITY_INFO, "缸头温度帧:");
                                gcs().send_text(MAV_SEVERITY_INFO, "原始数据: %02X %02X %02X %02X %02X %02X %02X %02X",
                                            frame.data[0], frame.data[1], frame.data[2], frame.data[3],
                                            frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
                                
                                // 油门反馈
                                uint16_t throttle_feedback = (frame.data[0] << 8) | frame.data[1];
                                float throttle_percent = throttle_feedback * 0.1f;
                                gcs().send_text(MAV_SEVERITY_INFO, "油门反馈: %u (%.1f%%)", throttle_feedback, (double)throttle_percent);
                                
                                // 转速
                                _engines[i].engine_rpm = (frame.data[2] << 8) | frame.data[3];
                                gcs().send_text(MAV_SEVERITY_INFO, "转速: %u RPM", _engines[i].engine_rpm);
                                
                                // 缸头温度
                                for (int cyl = 0; cyl < 4; cyl++) {
                                    if (frame.data[4 + cyl] != 0xFF) {  // 检查有效值
                                        float temp = convert_to_physical(frame.data[4 + cyl], 2.0f, -50.0f);
                                        _engines[i].cylinder_head_temp[cyl] = temp;
                                        gcs().send_text(MAV_SEVERITY_INFO, "缸%d温度: %.1f°C (原始:0x%02X)", 
                                                    cyl + 1, (double)temp, frame.data[4 + cyl]);
                                    } else {
                                        gcs().send_text(MAV_SEVERITY_INFO, "缸%d温度: 无效", cyl + 1);
                                    }
                                }
                            }
                            break;
                            
                        case LIDE_CAN_ID_STATUS3:
                            // 排气温度
                            if (frame.dlc >= 8) {
                                gcs().send_text(MAV_SEVERITY_INFO, "排气温度帧:");
                                gcs().send_text(MAV_SEVERITY_INFO, "原始数据: %02X %02X %02X %02X %02X %02X %02X %02X",
                                            frame.data[0], frame.data[1], frame.data[2], frame.data[3],
                                            frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
                                
                                // 排气温度
                                for (int cyl = 0; cyl < 4; cyl++) {
                                    if (frame.data[cyl] != 0xFF) {  // 检查有效值
                                        float temp = convert_to_physical(frame.data[cyl], 4.0f, -50.0f);
                                        _engines[i].exhaust_temp[cyl] = temp;
                                        gcs().send_text(MAV_SEVERITY_INFO, "缸%d排温: %.1f°C (原始:0x%02X)", 
                                                    cyl + 1, (double)temp, frame.data[cyl]);
                                    } else {
                                        gcs().send_text(MAV_SEVERITY_INFO, "缸%d排温: 无效", cyl + 1);
                                    }
                                }
                                
                                // 冷风门占空比
                                for (int cool = 0; cool < 4; cool++) {
                                    if (frame.data[4 + cool] != 0xFF) {
                                        float duty = convert_to_physical(frame.data[4 + cool], 0.5f, 0);
                                        gcs().send_text(MAV_SEVERITY_INFO, "冷风门%d: %.1f%% (原始:0x%02X)", 
                                                    cool + 1, (double)duty, frame.data[4 + cool]);
                                    }
                                }
                            }
                            break;
                            
                        case LIDE_CAN_ID_STATUS4:
                            // 燃油压力
                            if (frame.dlc >= 8) {
                                gcs().send_text(MAV_SEVERITY_INFO, "燃油系统帧:");
                                gcs().send_text(MAV_SEVERITY_INFO, "原始数据: %02X %02X %02X %02X %02X %02X %02X %02X",
                                            frame.data[0], frame.data[1], frame.data[2], frame.data[3],
                                            frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
                                
                                // 燃油压力
                                _engines[i].fuel_pressure_target = frame.data[0] * 1.6f;
                                _engines[i].fuel_pressure_actual = frame.data[1] * 1.6f;
                                gcs().send_text(MAV_SEVERITY_INFO, "低压燃油压力: 设定=%.1fkPa 实际=%.1fkPa", 
                                            (double)_engines[i].fuel_pressure_target, 
                                            (double)_engines[i].fuel_pressure_actual);
                                
                                // 低压油泵转速
                                uint16_t pump_rpm = frame.data[2] * 100;
                                gcs().send_text(MAV_SEVERITY_INFO, "低压油泵转速: %u RPM", pump_rpm);
                                
                                // 轨压
                                _engines[i].rail_pressure_target = frame.data[3] * 2.0f;
                                _engines[i].rail_pressure_actual = frame.data[4] * 2.0f;
                                gcs().send_text(MAV_SEVERITY_INFO, "轨压: 设定=%.1fBar 实际=%.1fBar", 
                                            (double)_engines[i].rail_pressure_target, 
                                            (double)_engines[i].rail_pressure_actual);
                                
                                // 系统电压
                                _engines[i].system_voltage = frame.data[5] * 0.125f;
                                gcs().send_text(MAV_SEVERITY_INFO, "系统电压: %.2fV", (double)_engines[i].system_voltage);
                                
                                // 滑油消耗
                                uint16_t oil_consumption = (frame.data[6] << 8) | frame.data[7];
                                gcs().send_text(MAV_SEVERITY_INFO, "滑油消耗: %u ml", oil_consumption);
                            }
                            break;
                            
                        case LIDE_CAN_ID_STATUS5:
                            // 节气门和环境参数
                            if (frame.dlc >= 8) {
                                gcs().send_text(MAV_SEVERITY_INFO, "节气门环境帧:");
                                gcs().send_text(MAV_SEVERITY_INFO, "原始数据: %02X %02X %02X %02X %02X %02X %02X %02X",
                                            frame.data[0], frame.data[1], frame.data[2], frame.data[3],
                                            frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
                                
                                // 节气门1
                                _engines[i].throttle1_deviation = convert_to_physical(frame.data[0], 0.5f, -60.0f);
                                _engines[i].throttle1_pos = convert_to_physical(frame.data[1], 0.5f, 0);
                                gcs().send_text(MAV_SEVERITY_INFO, "节气门1: 偏差=%.1f%% 开度=%.1f%%", 
                                            (double)_engines[i].throttle1_deviation, 
                                            (double)_engines[i].throttle1_pos);
                                
                                // 节气门2
                                _engines[i].throttle2_deviation = convert_to_physical(frame.data[2], 0.5f, -60.0f);
                                _engines[i].throttle2_pos = convert_to_physical(frame.data[3], 0.5f, 0);
                                gcs().send_text(MAV_SEVERITY_INFO, "节气门2: 偏差=%.1f%% 开度=%.1f%%", 
                                            (double)_engines[i].throttle2_deviation, 
                                            (double)_engines[i].throttle2_pos);
                                
                                // 进气温度
                                _engines[i].intake_temp = convert_to_physical(frame.data[4], 2.0f, -50.0f);
                                gcs().send_text(MAV_SEVERITY_INFO, "进气温度: %.1f°C", (double)_engines[i].intake_temp);
                                
                                // 环境压力
                                float env_pressure = convert_to_physical(frame.data[5], 0.5f, 0);
                                gcs().send_text(MAV_SEVERITY_INFO, "环境压力: %.1fkPa", (double)env_pressure);
                                
                                // 油位
                                _engines[i].oil_level = frame.data[6] * 0.5f;
                                gcs().send_text(MAV_SEVERITY_INFO, "油位: %.1f%%", (double)_engines[i].oil_level);
                                
                                // 预留字节
                                if (frame.data[7] != 0xFF) {
                                    gcs().send_text(MAV_SEVERITY_INFO, "预留字节: 0x%02X", frame.data[7]);
                                }
                            }
                            break;
                            
                        case LIDE_CAN_ID_STATUS6:
                            // 故障字节1-6
                            if (frame.dlc >= 6) {
                                gcs().send_text(MAV_SEVERITY_INFO, "故障状态帧1-6:");
                                gcs().send_text(MAV_SEVERITY_INFO, "原始数据: %02X %02X %02X %02X %02X %02X",
                                            frame.data[0], frame.data[1], frame.data[2],
                                            frame.data[3], frame.data[4], frame.data[5]);
                                
                                for (int fb = 0; fb < 6; fb++) {
                                    // uint8_t old_fault = _engines[i].fault_bytes[fb];
                                    _engines[i].fault_bytes[fb] = frame.data[fb];
                                    
                                    // 打印每个故障字节
                                    if (_engines[i].fault_bytes[fb] != 0) {
                                        gcs().send_text(MAV_SEVERITY_WARNING, "故障字节%d: 0x%02X", 
                                                    fb + 1, _engines[i].fault_bytes[fb]);
                                        
                                        // 解析具体故障位
                                        uint8_t fault_byte = _engines[i].fault_bytes[fb];
                                        for (int bit = 0; bit < 8; bit++) {
                                            if (fault_byte & (1 << bit)) {
                                                
                                                    gcs().send_text(MAV_SEVERITY_WARNING, "  故障: %d", bit);
                                                
                                            }
                                        }
                                    } else {
                                        gcs().send_text(MAV_SEVERITY_INFO, "故障字节%d: 正常", fb + 1);
                                    }
                                }
                            }
                            break;
                            
                        case LIDE_CAN_ID_STATUS7:
                            // 故障字节7-8
                            if (frame.dlc >= 2) {
                                gcs().send_text(MAV_SEVERITY_INFO, "故障状态帧7-8:");
                                gcs().send_text(MAV_SEVERITY_INFO, "原始数据: %02X %02X",
                                            frame.data[0], frame.data[1]);
                                
                                // uint8_t old_fault7 = _engines[i].fault_bytes[6];
                                // uint8_t old_fault8 = _engines[i].fault_bytes[7];
                                
                                _engines[i].fault_bytes[6] = frame.data[0];
                                _engines[i].fault_bytes[7] = frame.data[1];
                                
                                // 打印故障字节7
                                if (_engines[i].fault_bytes[6] != 0) {
                                    gcs().send_text(MAV_SEVERITY_WARNING, "故障字节7: 0x%02X", _engines[i].fault_bytes[6]);
                                    uint8_t fault_byte = _engines[i].fault_bytes[6];
                                    for (int bit = 0; bit < 8; bit++) {
                                        if (fault_byte & (1 << bit)) {
                                           
                                                gcs().send_text(MAV_SEVERITY_WARNING, "  故障: %d", bit);
                                            
                                        }
                                    }
                                } else {
                                    gcs().send_text(MAV_SEVERITY_INFO, "故障字节7: 正常");
                                }
                                
                                // 打印故障字节8
                                if (_engines[i].fault_bytes[7] != 0) {
                                    gcs().send_text(MAV_SEVERITY_WARNING, "故障字节8: 0x%02X", _engines[i].fault_bytes[7]);
                                    uint8_t fault_byte = _engines[i].fault_bytes[7];
                                    for (int bit = 0; bit < 8; bit++) {
                                        if (fault_byte & (1 << bit)) {
                                            
                                            gcs().send_text(MAV_SEVERITY_WARNING, "  故障: %d", bit);
                                            
                                        }
                                    }
                                } else {
                                    gcs().send_text(MAV_SEVERITY_INFO, "故障字节8: 正常");
                                }
                            }
                            break;
                    }
                    break;  // 处理第一个匹配的发动机
                }
            }
            break;
    }
}

// 写入CAN帧
bool AP_CAN_LIDE::write_frame(AP_HAL::CANFrame &out_frame, uint64_t timeout) {
    if (!_initialized) {
        static uint32_t last_error_ms = 0;
        uint32_t now_ms = AP_HAL::millis();
        if (now_ms - last_error_ms > 5000) {
            last_error_ms = now_ms;
            send_gcs_text(MAV_SEVERITY_ERROR, "砺德CAN: 驱动未初始化");
        }
        return false;
    }

    if (_can_iface == nullptr) {
        static uint32_t last_error_ms = 0;
        uint32_t now_ms = AP_HAL::millis();
        if (now_ms - last_error_ms > 5000) {
            last_error_ms = now_ms;
            send_gcs_text(MAV_SEVERITY_ERROR, "砺德CAN: CAN接口为空");
        }
        return false;
    }

    bool read_select = false;
    bool write_select = true;

    bool ret = _can_iface->select(read_select, write_select, &out_frame, timeout);

    if (!ret) {
        // select失败是正常现象，不打印错误
        return false;
    }
    
    if (!write_select) {
        // 不能立即发送，正常返回false
        return false;
    }

    int send_result = _can_iface->send(out_frame, timeout, AP_HAL::CANIface::AbortOnError);
    
    if (send_result != 1) {
        static uint32_t last_error_ms = 0;
        uint32_t now_ms = AP_HAL::millis();
        if (now_ms - last_error_ms > 10000) {
            last_error_ms = now_ms;
            send_gcs_text(MAV_SEVERITY_DEBUG, "砺德CAN: 发送失败, 结果=%d", send_result);
        }
        return false;
    }
    
    return true;
}

// 读取CAN帧
bool AP_CAN_LIDE::read_frame(AP_HAL::CANFrame &recv_frame, uint64_t timeout) {
    if (!_initialized) {
        return false;
    }
    
    if (_can_iface == nullptr) {
        return false;
    }
    
    bool read_select = true;
    bool write_select = false;
    bool ret = _can_iface->select(read_select, write_select, nullptr, timeout);

    if (!ret || !read_select) {
        return false;
    }

    uint64_t time;
    AP_HAL::CANIface::CanIOFlags flags{};

    return (_can_iface->receive(recv_frame, time, flags) == 1);
}

// 更新函数（从主线程调用）
void AP_CAN_LIDE::update() {
    _last_update_ms = AP_HAL::millis();
    
    // 这里可以添加从SRV_Channels获取油门值的逻辑
    // 例如：_engines[0].throttle_request = SRV_Channels::get_output_norm(SRV_Channel::k_throttle) * 1000;
    
    // 定期打印更新状态
    static uint32_t last_update_debug_ms = 0;
    if (_last_update_ms - last_update_debug_ms > 10000) {
        last_update_debug_ms = _last_update_ms;
        send_gcs_text(MAV_SEVERITY_DEBUG, "砺德CAN: 更新函数被调用");
    }
}

// 日志记录
void AP_CAN_LIDE::log_engine_status() {
    AP_Logger *logger = AP_Logger::get_singleton();
    
    if (!logger || !logger->logging_enabled()) {
        return;
    }
    
    // 定期打印日志状态
    static uint32_t last_log_debug_ms = 0;
    if (_last_log_ms - last_log_debug_ms > 10000) {
        last_log_debug_ms = _last_log_ms;
        send_gcs_text(MAV_SEVERITY_DEBUG, "砺德CAN: 记录发动机状态日志");
    }
    
    for (int i = 0; i < LIDE_NODE_ID_MAX; i++) {
        if (_engines[i].enabled) {
            // 记录发动机状态到日志
            // 需要根据实际的日志结构实现
        }
    }
}

// 物理值转换
float AP_CAN_LIDE::convert_to_physical(uint16_t raw_value, float scale, float offset) {
    return raw_value * scale + offset;
}

uint16_t AP_CAN_LIDE::convert_to_raw(float physical_value, float scale, float offset) {
    return static_cast<uint16_t>((physical_value - offset) / scale);
}

// 公共API实现
void AP_CAN_LIDE::set_throttle(uint8_t engine_id, uint16_t throttle) {
    if (engine_id < LIDE_NODE_ID_MAX) {
        uint16_t old_throttle = _engines[engine_id].throttle_request;
        _engines[engine_id].throttle_request = constrain_int16(throttle, 
                                                              LIDE_THROTTLE_MIN, 
                                                              LIDE_THROTTLE_MAX);
        
        // 打印油门变化（大变化时）
        if (abs(old_throttle - _engines[engine_id].throttle_request) > 100) {
            send_gcs_text(MAV_SEVERITY_INFO, "发动机 %d 油门: %.1f%% -> %.1f%%",
                       engine_id + 1, old_throttle * 0.1f, _engines[engine_id].throttle_request * 0.1f);
        }
    }
}

void AP_CAN_LIDE::set_start_cmd(uint8_t engine_id, bool start) {
    if (engine_id < LIDE_NODE_ID_MAX) {
        bool old_start = _engines[engine_id].control_cmd.bits.start_cmd;
        _engines[engine_id].control_cmd.bits.start_cmd = start ? 1 : 0;
        _engines[engine_id].control_cmd.bits.start_valid = 1;
        
        if (old_start != _engines[engine_id].control_cmd.bits.start_cmd) {
            send_gcs_text(MAV_SEVERITY_INFO, "发动机 %d 启动命令: %s",
                       engine_id + 1, start ? "开启" : "关闭");
        }
    }
}

void AP_CAN_LIDE::set_stop_cmd(uint8_t engine_id, bool stop) {
    if (engine_id < LIDE_NODE_ID_MAX) {
        bool old_stop = _engines[engine_id].control_cmd.bits.stop_cmd;
        _engines[engine_id].control_cmd.bits.stop_cmd = stop ? 1 : 0;
        
        if (old_stop != _engines[engine_id].control_cmd.bits.stop_cmd) {
            send_gcs_text(MAV_SEVERITY_WARNING, "发动机 %d 停止命令: %s",
                       engine_id + 1, stop ? "开启" : "关闭");
        }
    }
}

void AP_CAN_LIDE::set_heating_cmd(uint8_t engine_id, bool heating) {
    if (engine_id < LIDE_NODE_ID_MAX) {
        bool old_heating = _engines[engine_id].is_heating;
        _engines[engine_id].control_cmd.bits.heating_cmd = heating ? 1 : 0;
        _engines[engine_id].control_cmd.bits.heating_valid = 1;
        _engines[engine_id].is_heating = heating;
        
        if (old_heating != _engines[engine_id].is_heating) {
            send_gcs_text(MAV_SEVERITY_INFO, "发动机 %d 加热: %s",
                       engine_id + 1, heating ? "开启" : "关闭");
        }
    }
}

void AP_CAN_LIDE::set_altitude(uint8_t engine_id, uint16_t altitude) {
    if (engine_id < LIDE_NODE_ID_MAX) {
        _engines[engine_id].altitude = altitude;
        _engines[engine_id].control_cmd.bits.altitude_valid = 1;
        
        static uint32_t last_alt_debug_ms = 0;
        if (AP_HAL::millis() - last_alt_debug_ms > 30000) {
            last_alt_debug_ms = AP_HAL::millis();
            send_gcs_text(MAV_SEVERITY_DEBUG, "发动机 %d 海拔设置: %d 米",
                       engine_id + 1, altitude);
        }
    }
}

void AP_CAN_LIDE::set_airspeed(uint8_t engine_id, uint8_t airspeed) {
    if (engine_id < LIDE_NODE_ID_MAX) {
        _engines[engine_id].airspeed = airspeed;
        _engines[engine_id].control_cmd.bits.airspeed_valid = 1;
        
        static uint32_t last_airspeed_debug_ms = 0;
        if (AP_HAL::millis() - last_airspeed_debug_ms > 30000) {
            last_airspeed_debug_ms = AP_HAL::millis();
            send_gcs_text(MAV_SEVERITY_DEBUG, "发动机 %d 空速设置: %d 米/秒",
                       engine_id + 1, airspeed);
        }
    }
}

bool AP_CAN_LIDE::is_engine_online(uint8_t engine_id) {
    if (engine_id >= LIDE_NODE_ID_MAX) {
        return false;
    }
    return _engines[engine_id].is_online;
}

bool AP_CAN_LIDE::is_engine_running(uint8_t engine_id) {
    if (engine_id >= LIDE_NODE_ID_MAX) {
        return false;
    }
    return _engines[engine_id].is_running;
}

uint16_t AP_CAN_LIDE::get_engine_rpm(uint8_t engine_id) {
    if (engine_id >= LIDE_NODE_ID_MAX) {
        return 0;
    }
    return _engines[engine_id].engine_rpm;
}

float AP_CAN_LIDE::get_engine_temperature(uint8_t engine_id, uint8_t cylinder) {
    if (engine_id >= LIDE_NODE_ID_MAX || cylinder >= 4) {
        return 0.0f;
    }
    return _engines[engine_id].cylinder_head_temp[cylinder];
}

float AP_CAN_LIDE::get_fuel_consumption(uint8_t engine_id) {
    if (engine_id >= LIDE_NODE_ID_MAX) {
        return 0.0f;
    }
    return _engines[engine_id].fuel_consumption;
}

uint8_t AP_CAN_LIDE::get_engine_status(uint8_t engine_id) {
    if (engine_id >= LIDE_NODE_ID_MAX) {
        return 0;
    }
    return _engines[engine_id].engine_status;
}

bool AP_CAN_LIDE::has_fault(uint8_t engine_id) {
    if (engine_id >= LIDE_NODE_ID_MAX) {
        return true;
    }
    
    // 检查是否有故障位被设置
    for (int i = 0; i < 8; i++) {
        if (_engines[engine_id].fault_bytes[i] != 0) {
            return true;
        }
    }
    
    return false;
}

bool AP_CAN_LIDE::pre_arm_check(char* reason, uint8_t reason_len) {
    // 检查所有使能的发动机是否在线
    for (int i = 0; i < LIDE_NODE_ID_MAX; i++) {
        if (_engines[i].enabled && !_engines[i].is_online) {
            snprintf(reason, reason_len, "发动机 %u 未检测到", i + 1);
            send_gcs_text(MAV_SEVERITY_ERROR, "预上电检查失败: %s", reason);
            return false;
        }
    }
    
    // 检查是否有故障
    for (int i = 0; i < LIDE_NODE_ID_MAX; i++) {
        if (_engines[i].enabled && _engines[i].is_online) {
            if (has_fault(i)) {
                snprintf(reason, reason_len, "发动机 %u 存在故障", i + 1);
                send_gcs_text(MAV_SEVERITY_ERROR, "预上电检查失败: %s", reason);
                return false;
            }
            
            // 检查发动机状态
            uint8_t status = get_engine_status(i);
            if (status == LIDE_STATUS_WARNING) {
                snprintf(reason, reason_len, "发动机 %u 状态警告", i + 1);
                send_gcs_text(MAV_SEVERITY_ERROR, "预上电检查失败: %s", reason);
                return false;
            }
            
            if (status == LIDE_STATUS_ABNORMAL) {
                send_gcs_text(MAV_SEVERITY_WARNING, "发动机 %u 异常状态, 请尽快降落", i + 1);
            }
        }
    }
    
    send_gcs_text(MAV_SEVERITY_INFO, "砺德发动机预上电检查通过");
    return true;
}