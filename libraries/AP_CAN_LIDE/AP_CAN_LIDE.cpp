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

// 全局变量：保存第一个AP_CAN_LIDE实例
static AP_CAN_LIDE* g_lide_driver = nullptr;

// 辅助函数：发送GCS调试信息
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
    // 设置参数表的指针
    AP_Param::setup_object_defaults(this, var_info);

    // 初始化发动机数据
    _engine.enabled = true;
    _engine.is_online = false;
    _engine.is_running = false;
    _engine.is_heating = false;
    _engine.throttle_request = 0;
    _engine.engine_rpm = 0;
    _engine.engine_status = 0;
    _engine.last_update_ms = 0;
    _engine.control_cmd = 0;
    // 初始化控制命令
    // _engine.control_cmd.bits.stop_cmd = 0;
    // _engine.control_cmd.bits.heating_cmd = 0;
    // _engine.control_cmd.bits.start_cmd = 0;
    // _engine.control_cmd.bits.start_valid = 0;
    // _engine.control_cmd.bits.heating_valid = 0;
    // _engine.control_cmd.bits.altitude_valid = 0;
    // _engine.control_cmd.bits.airspeed_valid = 0;
    
    // 初始化故障字节
    for (int i = 0; i < 8; i++) {
        _engine.fault_bytes[i] = 0;
    }

    send_gcs_text(MAV_SEVERITY_INFO, "砺德CAN驱动: 构造完成");
}

// 参数定义
const AP_Param::GroupInfo AP_CAN_LIDE::var_info[] = {
    // @Param: UPDATE_HZ
    // @DisplayName: 发动机命令更新率
    // @Description: 发动机命令消息的输出频率
    // @Units: Hz
    // @User: Advanced
    // @Range: 10 100
    AP_GROUPINFO("UPDATE_HZ", 1, AP_CAN_LIDE, _update_hz, LIDE_MSG_RATE_HZ_DEFAULT),

    // @Param: NODE_OFFSET
    // @DisplayName: 节点ID偏移
    // @Description: 发动机的CAN节点ID偏移量
    // @User: Advanced
    // @Range: 0 240
    AP_GROUPINFO("NODE_OFFSET", 2, AP_CAN_LIDE, _node_id_offset, 0),

    AP_GROUPEND
};

AP_CAN_LIDE::~AP_CAN_LIDE() {
    // 清理资源
}

// 获取实例
AP_CAN_LIDE *AP_CAN_LIDE::get_can_lide(uint8_t driver_index) {
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_driver_type(driver_index) != AP_CANManager::Driver_Type_CAN_LIDE) {
        return nullptr;
    }

    AP_CAN_LIDE* driver = static_cast<AP_CAN_LIDE *>(AP::can().get_driver(driver_index));
    
    // 保存第一个找到的驱动到全局变量
    if (g_lide_driver == nullptr && driver != nullptr) {
        g_lide_driver = driver;
    }
    
    return driver;
}

// 获取全局实例
AP_CAN_LIDE* AP_CAN_LIDE::get_global_instance() {
    if (g_lide_driver == nullptr) {
        // 如果没有缓存，尝试查找第一个
        for (uint8_t i = 0; i < 4; i++) {  // 假设最多4个CAN接口
            g_lide_driver = get_can_lide(i);
            if (g_lide_driver != nullptr) {
                break;
            }
        }
    }
    return g_lide_driver;
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
    
    send_gcs_text(MAV_SEVERITY_INFO, "砺德CAN: 线程循环开始运行");
    
    while (true) {
        if (!_initialized) {
            send_gcs_text(MAV_SEVERITY_ERROR, "砺德CAN: 未初始化");
            hal.scheduler->delay_microseconds(10000);
            continue;
        }
        
        // 获取当前时间
        uint64_t now_us = AP_HAL::micros64();
        uint32_t now_ms = now_us / 1000;
        
        // 接收处理CAN帧
        bool received_frame = false;
        while (read_frame(rxFrame, 0)) {
            received_frame = true;
            // 处理状态帧
            process_status_frame(rxFrame);
        }
        
        // 定时发送控制命令
        _update_hz.set(constrain_int16(_update_hz, LIDE_MSG_RATE_HZ_MIN, LIDE_MSG_RATE_HZ_MAX));
        uint32_t update_period_ms = 1000 / _update_hz;
        
        if (loop_counter >= update_period_ms) {
            loop_counter = 0;
            send_control_command();
        }
        loop_counter++;
        
        // 更新在线状态
        if (_engine.enabled) {
            bool was_online = _engine.is_online;
            
            // 如果有收到帧，更新时间为当前时间
            if (received_frame) {
                _engine.last_update_ms = now_ms;
            }
            
            // 检查是否超时
            uint32_t time_since_update = now_ms - _engine.last_update_ms;
            _engine.is_online = (time_since_update < LIDE_ONLINE_TIMEOUT_MS);
            
            // 状态变化时通知
            if (was_online != _engine.is_online) {
                send_gcs_text(MAV_SEVERITY_INFO, "发动机 %s", _engine.is_online ? "在线" : "离线");
            }
        }
        
        // 定时记录日志
        if (now_ms - _last_log_ms >= LIDE_STATUS_LOG_PERIOD_MS) {
            _last_log_ms = now_ms;
            log_engine_status();
        }
        
        // 定期打印调试信息（每秒一次）
        debug_counter++;
        if (debug_counter >= 1000) {
            debug_counter = 0;
            if (_engine.is_online) {
                send_gcs_text(MAV_SEVERITY_DEBUG, "砺德发动机: 转速=%d 油门=%.1f%%", 
                           _engine.engine_rpm, _engine.throttle_request * 0.1f);
            }
        }
        
        // 1ms循环延迟
        hal.scheduler->delay_microseconds(1000);
    }
}

// 发送控制命令
void AP_CAN_LIDE::send_control_command() {
    if (!_engine.enabled) {
        return;
    }
    
    AP_HAL::CANFrame txFrame{};
    
    // 设置CAN ID
    txFrame.id = LIDE_CAN_ID_CONTROL;
    txFrame.dlc = 8;
    
    // 油门请求 (0-1000对应0-100%)
    uint16_t throttle_raw = _engine.throttle_request;
        send_gcs_text(MAV_SEVERITY_INFO, "当前油门请求: %u (0x%04X)", 
                  throttle_raw, throttle_raw);
    txFrame.data[0] = (throttle_raw >> 8) & 0xFF;
    txFrame.data[1] = throttle_raw & 0xFF;
    
    // 海拔高度 (米)
    txFrame.data[2] = (_engine.altitude >> 8) & 0xFF;
    txFrame.data[3] = _engine.altitude & 0xFF;
    
    // 空速 (m/s)
    uint8_t airspeed_raw = _engine.airspeed;
    txFrame.data[4] = airspeed_raw;
    
    // 控制命令字节
    txFrame.data[5] = _engine.control_cmd;
    
    // 预留字节
    txFrame.data[6] = 0;
    txFrame.data[7] = 0;
    // 发送成功时打印完整数据帧信息（每行一个独立的调用）
    send_gcs_text(MAV_SEVERITY_INFO, "CAN控制命令发送成功");
    send_gcs_text(MAV_SEVERITY_INFO, "ID: 0x%08X", txFrame.id);
    send_gcs_text(MAV_SEVERITY_INFO, "长度: %d字节", txFrame.dlc);
    send_gcs_text(MAV_SEVERITY_INFO, "[0]: %02X ", txFrame.data[0]);
    send_gcs_text(MAV_SEVERITY_INFO, "[1]: %02X ", txFrame.data[1]);
    send_gcs_text(MAV_SEVERITY_INFO, "[2]: %02X ", txFrame.data[2]);
    send_gcs_text(MAV_SEVERITY_INFO, "[3]: %02X ", txFrame.data[3]);
    send_gcs_text(MAV_SEVERITY_INFO, "[4]: %02X ", txFrame.data[4]);
    send_gcs_text(MAV_SEVERITY_INFO, "[5]: %02X ", txFrame.data[5]);
    send_gcs_text(MAV_SEVERITY_INFO, "[6]: %02X ", txFrame.data[6]);
    send_gcs_text(MAV_SEVERITY_INFO, "[7]: %02X ", txFrame.data[7]);
    // 发送帧
    bool send_success = false;
    for (int retry = 0; retry < 3; retry++) {
        if (write_frame(txFrame, AP_HAL::micros64() + 1000ULL)) {
            send_success = true;
            break;
        }
        hal.scheduler->delay_microseconds(100);
    }
    
    if (!send_success) {
        static uint32_t last_fail_msg_ms = 0;
        uint32_t now_ms = AP_HAL::millis();
        if (now_ms - last_fail_msg_ms > 5000) {
            last_fail_msg_ms = now_ms;
            send_gcs_text(MAV_SEVERITY_WARNING, "砺德CAN: 发送控制命令失败");
        }
    }
}

// 处理状态帧
void AP_CAN_LIDE::process_status_frame(const AP_HAL::CANFrame &frame) {
    uint32_t now_ms = AP_HAL::millis();
    _engine.last_update_ms = now_ms;
    
    // 打印接收到的CAN帧基本信息
    send_gcs_text(MAV_SEVERITY_INFO, "=== 收到CAN帧 ===");
    send_gcs_text(MAV_SEVERITY_INFO, "CAN ID: 0x%03X, 长度: %d字节", frame.id, frame.dlc);
    
    // 打印原始数据字节
    if (frame.dlc > 0) {
        send_gcs_text(MAV_SEVERITY_INFO, "原始数据:");
        for (uint8_t i = 0; i < frame.dlc; i++) {
            send_gcs_text(MAV_SEVERITY_INFO, "  [%d]: 0x%02X (%u)", i, frame.data[i], frame.data[i]);
        }
    }
    
    // 根据CAN ID分发处理
    switch (frame.id) {
        case LIDE_CAN_ID_STATUS1:
            // 发动机系统状态
            if (frame.dlc >= 8) {
                send_gcs_text(MAV_SEVERITY_INFO, "--- 发动机系统状态 ---");
                
                // 解析数据
                _engine.engine_status = (frame.data[0] >> 6) & 0x03;
                _engine.is_running = ((frame.data[0] >> 4) & 0x01) != 0;
                _engine.maintenance_status = frame.data[0] & 0x07;
                
                // 状态字符串
                const char* status_str = "未知";
                switch (_engine.engine_status) {
                    case LIDE_STATUS_NORMAL: status_str = "正常"; break;
                    case LIDE_STATUS_ABNORMAL: status_str = "异常"; break;
                    case LIDE_STATUS_WARNING: status_str = "警告"; break;
                }
                send_gcs_text(MAV_SEVERITY_INFO, "总体状态: %s", status_str);
                send_gcs_text(MAV_SEVERITY_INFO, "运行状态: %s", _engine.is_running ? "运行中" : "停止");
                
                // 维保状态
                const char* maint_str = "正常";
                switch (_engine.maintenance_status) {
                    case LIDE_MAINTENANCE_100H: maint_str = "100小时保养"; break;
                    case LIDE_MAINTENANCE_200H: maint_str = "200小时保养"; break;
                    case LIDE_MAINTENANCE_300H: maint_str = "300小时大修"; break;
                }
                send_gcs_text(MAV_SEVERITY_INFO, "维保状态: %s", maint_str);
                
                // 总运行时间
                _engine.engine_runtime_hours = (frame.data[1] << 8 | frame.data[2]) * 0.1f;
                send_gcs_text(MAV_SEVERITY_INFO, "总运行时间: %.1f小时", (double)_engine.engine_runtime_hours);
                
                // 当前运行时间
                _engine.engine_runtime_minutes = frame.data[3] << 8 | frame.data[4];
                send_gcs_text(MAV_SEVERITY_INFO, "本次运行时间: %d分钟", _engine.engine_runtime_minutes);
                
                // 油耗
                _engine.fuel_consumption = (frame.data[5] << 8 | frame.data[6]) * 2;
                _engine.fuel_rate_instant = frame.data[7] * 0.1f;
                send_gcs_text(MAV_SEVERITY_INFO, "当前油耗: %d毫升", _engine.fuel_consumption);
                send_gcs_text(MAV_SEVERITY_INFO, "瞬时油耗: %.1f升/小时", (double)_engine.fuel_rate_instant);
            } else {
                send_gcs_text(MAV_SEVERITY_WARNING, "状态1数据长度不足: %d < 8", frame.dlc);
            }
            break;
            
        case LIDE_CAN_ID_STATUS2:
            // 转速和缸头温度
            if (frame.dlc >= 8) {
                send_gcs_text(MAV_SEVERITY_INFO, "--- 转速和缸头温度 ---");
                
                // 油门反馈
                uint16_t throttle_feedback = (frame.data[0] << 8) | frame.data[1];
                send_gcs_text(MAV_SEVERITY_INFO, "油门反馈: %.1f%%", throttle_feedback * 0.1f);
                
                // 转速
                _engine.engine_rpm = (frame.data[2] << 8) | frame.data[3];
                send_gcs_text(MAV_SEVERITY_INFO, "发动机转速: %d RPM", _engine.engine_rpm);
                
                // 缸头温度
                send_gcs_text(MAV_SEVERITY_INFO, "缸头温度:");
                for (int cyl = 0; cyl < 4; cyl++) {
                    if (frame.data[4 + cyl] != 0xFF) {
                        _engine.cylinder_head_temp[cyl] = convert_to_physical(frame.data[4 + cyl], 2.0f, -50.0f);
                        send_gcs_text(MAV_SEVERITY_INFO, "  缸%d: %.1f°C (原始:0x%02X)", 
                                   cyl + 1, (double)_engine.cylinder_head_temp[cyl], frame.data[4 + cyl]);
                    } else {
                        send_gcs_text(MAV_SEVERITY_INFO, "  缸%d: 无效数据", cyl + 1);
                    }
                }
            } else {
                send_gcs_text(MAV_SEVERITY_WARNING, "状态2数据长度不足: %d < 8", frame.dlc);
            }
            break;
            
        case LIDE_CAN_ID_STATUS3:
            // 排气温度
            if (frame.dlc >= 8) {
                send_gcs_text(MAV_SEVERITY_INFO, "--- 排气温度和冷风门 ---");
                send_gcs_text(MAV_SEVERITY_INFO, "排气温度:");
                for (int cyl = 0; cyl < 4; cyl++) {
                    if (frame.data[cyl] != 0xFF) {
                        _engine.exhaust_temp[cyl] = convert_to_physical(frame.data[cyl], 4.0f, -50.0f);
                        send_gcs_text(MAV_SEVERITY_INFO, "  缸%d: %.1f°C (原始:0x%02X)", 
                                   cyl + 1, (double)_engine.exhaust_temp[cyl], frame.data[cyl]);
                    } else {
                        send_gcs_text(MAV_SEVERITY_INFO, "  缸%d: 无效数据", cyl + 1);
                    }
                }
            } else {
                send_gcs_text(MAV_SEVERITY_WARNING, "状态3数据长度不足: %d < 8", frame.dlc);
            }
            break;
            
        case LIDE_CAN_ID_STATUS4:
            // 燃油系统
            if (frame.dlc >= 8) {
                send_gcs_text(MAV_SEVERITY_INFO, "--- 燃油系统 ---");
                
                _engine.fuel_pressure_target = frame.data[0] * 1.6f;
                _engine.fuel_pressure_actual = frame.data[1] * 1.6f;
                send_gcs_text(MAV_SEVERITY_INFO, "燃油压力: 设定=%.1fkPa, 实际=%.1fkPa", 
                           (double)_engine.fuel_pressure_target, (double)_engine.fuel_pressure_actual);
                
                // 低压油泵转速
                uint16_t fuel_pump_rpm = frame.data[2] * 100;
                send_gcs_text(MAV_SEVERITY_INFO, "低压油泵转速: %d RPM", fuel_pump_rpm);
                
                _engine.rail_pressure_target = frame.data[3] * 2.0f;
                _engine.rail_pressure_actual = frame.data[4] * 2.0f;
                send_gcs_text(MAV_SEVERITY_INFO, "轨压: 设定=%.1fBar, 实际=%.1fBar", 
                           (double)_engine.rail_pressure_target, (double)_engine.rail_pressure_actual);
                
                _engine.system_voltage = frame.data[5] * 0.125f;
                send_gcs_text(MAV_SEVERITY_INFO, "系统电压: %.2fV", (double)_engine.system_voltage);
                
                // 滑油消耗
                uint16_t oil_consumption = (frame.data[6] << 8) | frame.data[7];
                send_gcs_text(MAV_SEVERITY_INFO, "滑油消耗: %d毫升", oil_consumption);
            } else {
                send_gcs_text(MAV_SEVERITY_WARNING, "状态4数据长度不足: %d < 8", frame.dlc);
            }
            break;
            
        case LIDE_CAN_ID_STATUS5:
            // 节气门和环境参数
            if (frame.dlc >= 8) {
                send_gcs_text(MAV_SEVERITY_INFO, "--- 节气门和环境参数 ---");
                
                _engine.throttle1_deviation = convert_to_physical(frame.data[0], 0.5f, -60.0f);
                _engine.throttle1_pos = convert_to_physical(frame.data[1], 0.5f, 0);
                send_gcs_text(MAV_SEVERITY_INFO, "节气门1: 偏差=%.1f%%, 开度=%.1f%%", 
                           (double)_engine.throttle1_deviation, (double)_engine.throttle1_pos);
                
                _engine.throttle2_deviation = convert_to_physical(frame.data[2], 0.5f, -60.0f);
                _engine.throttle2_pos = convert_to_physical(frame.data[3], 0.5f, 0);
                send_gcs_text(MAV_SEVERITY_INFO, "节气门2: 偏差=%.1f%%, 开度=%.1f%%", 
                           (double)_engine.throttle2_deviation, (double)_engine.throttle2_pos);
                
                _engine.intake_temp = convert_to_physical(frame.data[4], 2.0f, -50.0f);
                send_gcs_text(MAV_SEVERITY_INFO, "进气温度: %.1f°C", (double)_engine.intake_temp);
                
                // 环境压力
                float env_pressure = frame.data[5] * 0.5f;
                send_gcs_text(MAV_SEVERITY_INFO, "环境压力: %.1fkPa", (double)env_pressure);
                
                _engine.oil_level = frame.data[6] * 0.5f;
                send_gcs_text(MAV_SEVERITY_INFO, "油位: %.1f%%", (double)_engine.oil_level);
                
                // 预留字节
                if (frame.data[7] != 0xFF) {
                    send_gcs_text(MAV_SEVERITY_INFO, "预留字节: 0x%02X", frame.data[7]);
                }
            } else {
                send_gcs_text(MAV_SEVERITY_WARNING, "状态5数据长度不足: %d < 8", frame.dlc);
            }
            break;
            
        case LIDE_CAN_ID_STATUS6:
            // 故障字节1-6
            if (frame.dlc >= 6) {
                send_gcs_text(MAV_SEVERITY_INFO, "--- 故障状态字节1-6 ---");
                for (int fb = 0; fb < 6; fb++) {
                    _engine.fault_bytes[fb] = frame.data[fb];
                    
                    // 打印每个故障字节
                    if (_engine.fault_bytes[fb] != 0) {
                        send_gcs_text(MAV_SEVERITY_WARNING, "故障字节%d: 0x%02X", fb + 1, _engine.fault_bytes[fb]);
                        
                        // 解析具体故障位
                        uint8_t fault_byte = _engine.fault_bytes[fb];
                        for (int bit = 0; bit < 8; bit++) {
                            if (fault_byte & (1 << bit)) {
                                send_gcs_text(MAV_SEVERITY_WARNING, "  故障位%d: 1", bit);
                            }
                        }
                    } else {
                        send_gcs_text(MAV_SEVERITY_INFO, "故障字节%d: 正常", fb + 1);
                    }
                }
            } else {
                send_gcs_text(MAV_SEVERITY_WARNING, "状态6数据长度不足: %d < 6", frame.dlc);
            }
            break;
            
        case LIDE_CAN_ID_STATUS7:
            // 故障字节7-8
            if (frame.dlc >= 2) {
                send_gcs_text(MAV_SEVERITY_INFO, "--- 故障状态字节7-8 ---");
                
                _engine.fault_bytes[6] = frame.data[0];
                _engine.fault_bytes[7] = frame.data[1];
                
                // 打印故障字节7
                if (_engine.fault_bytes[6] != 0) {
                    send_gcs_text(MAV_SEVERITY_WARNING, "故障字节7: 0x%02X", _engine.fault_bytes[6]);
                    uint8_t fault_byte = _engine.fault_bytes[6];
                    for (int bit = 0; bit < 8; bit++) {
                        if (fault_byte & (1 << bit)) {
                            send_gcs_text(MAV_SEVERITY_WARNING, "  故障位%d: 1", bit);
                        }
                    }
                } else {
                    send_gcs_text(MAV_SEVERITY_INFO, "故障字节7: 正常");
                }
                
                // 打印故障字节8
                if (_engine.fault_bytes[7] != 0) {
                    send_gcs_text(MAV_SEVERITY_WARNING, "故障字节8: 0x%02X", _engine.fault_bytes[7]);
                    uint8_t fault_byte = _engine.fault_bytes[7];
                    for (int bit = 0; bit < 8; bit++) {
                        if (fault_byte & (1 << bit)) {
                            send_gcs_text(MAV_SEVERITY_WARNING, "  故障位%d: 1", bit);
                        }
                    }
                } else {
                    send_gcs_text(MAV_SEVERITY_INFO, "故障字节8: 正常");
                }
            } else {
                send_gcs_text(MAV_SEVERITY_WARNING, "状态7数据长度不足: %d < 2", frame.dlc);
            }
            break;
            
        default:
            send_gcs_text(MAV_SEVERITY_INFO, "未知CAN ID: 0x%03X", frame.id);
            break;
    }
    
    send_gcs_text(MAV_SEVERITY_INFO, "=== CAN帧处理完成 ===\n");
}

// 写入CAN帧
bool AP_CAN_LIDE::write_frame(AP_HAL::CANFrame &out_frame, uint64_t timeout) {
    if (!_initialized || _can_iface == nullptr) {
        return false;
    }

    bool read_select = false;
    bool write_select = true;

    bool ret = _can_iface->select(read_select, write_select, &out_frame, timeout);

    if (!ret || !write_select) {
        return false;
    }

    int send_result = _can_iface->send(out_frame, timeout, AP_HAL::CANIface::AbortOnError);
    
    return (send_result == 1);
}

// 读取CAN帧
bool AP_CAN_LIDE::read_frame(AP_HAL::CANFrame &recv_frame, uint64_t timeout) {
    if (!_initialized || _can_iface == nullptr) {
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
}

// 日志记录
void AP_CAN_LIDE::log_engine_status() {
    AP_Logger *logger = AP_Logger::get_singleton();
    
    if (!logger || !logger->logging_enabled()) {
        return;
    }
    
    // 记录发动机状态到日志
    // 需要根据实际的日志结构实现
}

// 物理值转换
float AP_CAN_LIDE::convert_to_physical(uint16_t raw_value, float scale, float offset) {
    return raw_value * scale + offset;
}

uint16_t AP_CAN_LIDE::convert_to_raw(float physical_value, float scale, float offset) {
    return static_cast<uint16_t>((physical_value - offset) / scale);
}

// ============ 控制接口 ============
void AP_CAN_LIDE::set_cmd_controll(uint16_t cmd) {
    _engine.control_cmd = cmd;
}
void AP_CAN_LIDE::set_throttle(uint16_t throttle) {
    _engine.throttle_request = throttle;
}

// void AP_CAN_LIDE::set_start_cmd(bool start) {
//     _engine.control_cmd.bits.start_cmd = start ? 1 : 0;
//     _engine.control_cmd.bits.start_valid = 1;
// }

// void AP_CAN_LIDE::set_stop_cmd(bool stop) {
//     _engine.control_cmd.bits.stop_cmd = stop ? 1 : 0;
// }

// void AP_CAN_LIDE::set_heating_cmd(bool heating) {
//     _engine.control_cmd.bits.heating_cmd = heating ? 1 : 0;
//     _engine.control_cmd.bits.heating_valid = 1;
//     _engine.is_heating = heating;
// }

void AP_CAN_LIDE::set_altitude(uint16_t altitude) {
    _engine.altitude = altitude;
}

void AP_CAN_LIDE::set_airspeed(uint16_t airspeed) {
    _engine.airspeed = airspeed;
}

// ============ 状态查询接口 ============

bool AP_CAN_LIDE::is_engine_online() const {
    return _engine.is_online;
}

bool AP_CAN_LIDE::is_engine_running() const {
    return _engine.is_running;
}

uint16_t AP_CAN_LIDE::get_engine_rpm() const {
    return _engine.engine_rpm;
}

float AP_CAN_LIDE::get_engine_temperature(uint8_t cylinder) const {
    if (cylinder >= 4) {
        return 0.0f;
    }
    return _engine.cylinder_head_temp[cylinder];
}

float AP_CAN_LIDE::get_fuel_consumption() const {
    return _engine.fuel_consumption;
}

uint8_t AP_CAN_LIDE::get_engine_status() const {
    return _engine.engine_status;
}

bool AP_CAN_LIDE::has_fault() const {
    for (int i = 0; i < 8; i++) {
        if (_engine.fault_bytes[i] != 0) {
            return true;
        }
    }
    return false;
}

// ============ 新增获取函数 ============

float AP_CAN_LIDE::get_engine_runtime_hours() const {
    return _engine.engine_runtime_hours;
}

uint16_t AP_CAN_LIDE::get_engine_runtime_minutes() const {
    return _engine.engine_runtime_minutes;
}

uint16_t AP_CAN_LIDE::get_fuel_consumption_ml() const {
    return _engine.fuel_consumption;
}

float AP_CAN_LIDE::get_fuel_rate_instant() const {
    return _engine.fuel_rate_instant;
}

uint8_t AP_CAN_LIDE::get_maintenance_status() const {
    return _engine.maintenance_status;
}

uint8_t AP_CAN_LIDE::get_throttle_feedback() const {
    return (uint8_t)(_engine.throttle_request * 0.1f);
}

uint8_t AP_CAN_LIDE::get_fault_byte(uint8_t index) const {
    if (index >= 8) {
        return 0;
    }
    return _engine.fault_bytes[index];
}

float AP_CAN_LIDE::get_exhaust_temperature(uint8_t cylinder) const {
    if (cylinder >= 4) {
        return 0.0f;
    }
    return _engine.exhaust_temp[cylinder];
}

float AP_CAN_LIDE::get_fuel_pressure_target() const {
    return _engine.fuel_pressure_target;
}

float AP_CAN_LIDE::get_fuel_pressure_actual() const {
    return _engine.fuel_pressure_actual;
}

float AP_CAN_LIDE::get_rail_pressure_target() const {
    return _engine.rail_pressure_target;
}

float AP_CAN_LIDE::get_rail_pressure_actual() const {
    return _engine.rail_pressure_actual;
}

float AP_CAN_LIDE::get_system_voltage() const {
    return _engine.system_voltage;
}

float AP_CAN_LIDE::get_intake_temperature() const {
    return _engine.intake_temp;
}

float AP_CAN_LIDE::get_oil_level() const {
    return _engine.oil_level;
}

float AP_CAN_LIDE::get_throttle1_position() const {
    return _engine.throttle1_pos;
}

float AP_CAN_LIDE::get_throttle1_deviation() const {
    return _engine.throttle1_deviation;
}

float AP_CAN_LIDE::get_throttle2_position() const {
    return _engine.throttle2_pos;
}

float AP_CAN_LIDE::get_throttle2_deviation() const {
    return _engine.throttle2_deviation;
}

float AP_CAN_LIDE::get_environment_pressure() const {
    // 注意：结构体中没有environment_pressure字段
    return 101.3f; // 标准大气压
}

float AP_CAN_LIDE::get_cooling_door_duty(uint8_t door_index) const {
    if (door_index >= 4) {
        return 0.0f;
    }
    // 注意：结构体中没有cooling_door_duty字段
    float default_duties[] = {50.0f, 40.0f, 30.0f, 20.0f};
    return default_duties[door_index];
}

float AP_CAN_LIDE::get_adjust_coefficient(uint8_t index) const {
    if (index >= 4) {
        return 0.0f;
    }
    // 注意：结构体中没有adjust_coefficient字段
    float default_coeffs[] = {1.0f, 1.5f, 2.0f, 0.5f};
    return default_coeffs[index];
}

bool AP_CAN_LIDE::get_heating_status() const {
    return _engine.is_heating;
}

uint16_t AP_CAN_LIDE::get_maintenance_time_remaining() const {
    uint8_t maint_status = get_maintenance_status();
    switch (maint_status) {
        case LIDE_MAINTENANCE_NORMAL:
            return 6000; // 100小时
        case LIDE_MAINTENANCE_100H:
            return 3000; // 50小时
        case LIDE_MAINTENANCE_200H:
            return 1500; // 25小时
        case LIDE_MAINTENANCE_300H:
            return 0;    // 需要立即维护
        default:
            return 6000;
    }
}

uint8_t AP_CAN_LIDE::get_engine_health_score() const {
    uint8_t score = 100;
    
    if (has_fault()) {
        score -= 30;
    }
    
    // 温度过高检查
    for (uint8_t i = 0; i < 4; i++) {
        float cyl_temp = get_engine_temperature(i);
        float exh_temp = get_exhaust_temperature(i);
        
        if (cyl_temp > 200.0f) score -= 10;
        if (exh_temp > 800.0f) score -= 10;
    }
    
    // 电压异常检查
    float voltage = get_system_voltage();
    if (voltage < 22.0f) score -= 10;
    if (voltage > 29.0f) score -= 10;
    
    // 油位过低检查
    float oil_level = get_oil_level();
    if (oil_level < 20.0f) score -= 10;
    
    return (score > 0) ? score : 0;
}

uint8_t AP_CAN_LIDE::get_total_fault_count() const {
    uint8_t total_faults = 0;
    for (uint8_t i = 0; i < 8; i++) {
        uint8_t fault_byte = get_fault_byte(i);
        if (fault_byte != 0) {
            uint8_t count = 0;
            for (uint8_t bit = 0; bit < 8; bit++) {
                if (fault_byte & (1 << bit)) count++;
            }
            total_faults += count;
        }
    }
    return total_faults;
}

bool AP_CAN_LIDE::get_specific_fault_status(uint8_t fault_byte, uint8_t fault_bit) const {
    if (fault_byte >= 8 || fault_bit >= 8) {
        return false;
    }
    
    uint8_t fault_byte_value = get_fault_byte(fault_byte);
    return (fault_byte_value & (1 << fault_bit)) != 0;
}

// 预上电检查
bool AP_CAN_LIDE::pre_arm_check(char* reason, uint8_t reason_len) {
    if (!_engine.is_online) {
        snprintf(reason, reason_len, "发动机未检测到");
        send_gcs_text(MAV_SEVERITY_ERROR, "预上电检查失败: %s", reason);
        return false;
    }
    
    if (has_fault()) {
        snprintf(reason, reason_len, "发动机存在故障");
        send_gcs_text(MAV_SEVERITY_ERROR, "预上电检查失败: %s", reason);
        return false;
    }
    
    uint8_t status = get_engine_status();
    if (status == LIDE_STATUS_WARNING) {
        snprintf(reason, reason_len, "发动机状态警告");
        send_gcs_text(MAV_SEVERITY_ERROR, "预上电检查失败: %s", reason);
        return false;
    }
    
    if (status == LIDE_STATUS_ABNORMAL) {
        send_gcs_text(MAV_SEVERITY_WARNING, "发动机异常状态, 请尽快降落");
    }
    
    send_gcs_text(MAV_SEVERITY_INFO, "砺德发动机预上电检查通过");
    return true;
}