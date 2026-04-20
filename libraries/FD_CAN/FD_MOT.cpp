#include "FD_MOT.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>
#include <FD_DATA/FD_DATA.h>

extern const AP_HAL::HAL &hal;

FD_MOT::FD_MOT(FD_CAN *frotend) {
    _frotend_ptr = frotend;
    // 修复: 显式初始化 status 结构体 (虽然头文件已使用值初始化，这里双重保险)
    memset(&status, 0, sizeof(status));
}

void FD_MOT::handle_info(AP_HAL::CANFrame &in_frame, bool do_print) {
    // 修复: 添加 group 有效性检查，防止未初始化或无效值
    if (status.group < 1 || status.group > 4) {
        return;  // group 无效，直接返回
    }

    // =====================================================
    // 电机转速解析
    // CAN ID: 0x189D271C (group1), 0x189E271C (group2), 0x189F271C (group3), 0x18A0271C (group4)
    // 每帧8字节，每2字节一个电机转速
    // 解析规则：16进制转10进制后乘以2
    // =====================================================
    bool have_rpm = false;
    uint16_t raw_rpm = 0;
    
    if (in_frame.id == ((0x189D271C + ((status.group - 1) * 0x10000)) | AP_HAL::CANFrame::FlagEFF)) {
        switch(status.order) {
        case 1:
            raw_rpm = (uint16_t)in_frame.data[0] << 8 | (uint16_t)in_frame.data[1];
            have_rpm = true;
            break;
        case 2:
            raw_rpm = (uint16_t)in_frame.data[2] << 8 | (uint16_t)in_frame.data[3];
            have_rpm = true;
            break;
        case 3:
            raw_rpm = (uint16_t)in_frame.data[4] << 8 | (uint16_t)in_frame.data[5];
            have_rpm = true;
            break;
        case 4:
            raw_rpm = (uint16_t)in_frame.data[6] << 8 | (uint16_t)in_frame.data[7];
            have_rpm = true;
            break;
        default:
            break;
        }
    }

    // 修复: 转速需要乘以2
    if (have_rpm && status.id >= 1 && status.id <= 16) {
        status.rpm = raw_rpm * 2;  // 关键修改：乘以2
        status.last_rpm_ms = AP_HAL::millis();
        AP::fd_data().motor_rpm_packet.motor_rpm[status.id - 1] = status.rpm;
        if (do_print) {
            if (AP_HAL::millis() - status.last_print_ms > 1000) {
                gcs().send_text(MAV_SEVERITY_INFO, "MOT %d, raw_rpm %d, rpm %d", 
                               status.id, raw_rpm, status.rpm);
                status.last_print_ms = AP_HAL::millis();
            }
        }
    }

    // =====================================================
    // 电机温度解析
    // CAN ID: 0x18A1271C (group1), 0x18A2271C (group2), 0x18A3271C (group3), 0x18A4271C (group4)
    // 每帧8字节，每2字节一个电机温度
    // 解析规则：16进制转10进制，除以10（单位0.1℃），再减去50
    // 例如：667 -> 667/10 = 66.7 -> 66.7 - 50 = 16.7℃
    // =====================================================
    bool have_temp = false;
    uint16_t raw_temp = 0;
    
    if (in_frame.id == ((0x18A1271C + ((status.group - 1) * 0x10000)) | AP_HAL::CANFrame::FlagEFF)) {
        switch(status.order) {
        case 1:
            raw_temp = (uint16_t)in_frame.data[0] << 8 | (uint16_t)in_frame.data[1];
            have_temp = true;
            break;
        case 2:
            raw_temp = (uint16_t)in_frame.data[2] << 8 | (uint16_t)in_frame.data[3];
            have_temp = true;
            break;
        case 3:
            raw_temp = (uint16_t)in_frame.data[4] << 8 | (uint16_t)in_frame.data[5];
            have_temp = true;
            break;
        case 4:
            raw_temp = (uint16_t)in_frame.data[6] << 8 | (uint16_t)in_frame.data[7];
            have_temp = true;
            break;
        default:
            break;
        }
    }

    // 修复: 温度解析 = (原始值 / 10) - 50
    // 单位是0.1℃，所以先除以10得到实际读数，再减50得到真实温度
    if (have_temp && status.id >= 1 && status.id <= 16) {
        // 使用浮点数计算以保持精度
        float actual_temp_f = (float)raw_temp / 10.0f - 50.0f;
        
        // 四舍五入转换为整数
        int16_t actual_temp = (int16_t)(actual_temp_f + (actual_temp_f >= 0 ? 0.5f : -0.5f));
        
        // 限制温度范围到int8_t的有效范围 [-128, 127]
        if (actual_temp < -128) actual_temp = -128;
        if (actual_temp > 127) actual_temp = 127;
        
        status.temp = actual_temp;  // 保存到status用于内部使用
        status.last_temp_ms = AP_HAL::millis();
        
        // 存储到MAVLink包，motor_temp是int8_t类型
        AP::fd_data().motor_rpm_packet.motor_temp[status.id - 1] = (int8_t)actual_temp;
        
        if (do_print) {
            if (AP_HAL::millis() - status.last_print_ms > 1000) {
                gcs().send_text(MAV_SEVERITY_INFO, "MOT %d, raw_temp %d, temp %.1f -> %d", 
                               status.id, raw_temp, (double)actual_temp_f, actual_temp);
            }
        }
    }
}

void FD_MOT::set_id(uint8_t id_in)
{
    status.id = id_in;
    
    // 修复: 添加默认值，防止 id_in 不在有效范围时 group/order 未被赋值
    status.group = 0;  // 默认无效值
    status.order = 0;  // 默认无效值
    
    if ((1 <= status.id) && (status.id <= 4)) {
        status.group = 1;
    }
    else if ((5 <= status.id) && (status.id <= 8)) {
        status.group = 2;
    }
    else if ((9 <= status.id) && (status.id <= 12)) {
        status.group = 3;
    }
    else if ((13 <= status.id) && (status.id <= 16)) {
        status.group = 4;
    }

    // 只有在 id 有效时才计算 order
    if (status.id >= 1 && status.id <= 16) {
        status.order = status.id % 4;
        if (status.order == 0) {
            status.order = 4;
        }
    }
}

// void FD_MOT::set_pwm(uint16_t pwm_in) // 1000~2000
// {
//     status.thr_in = pwm_in;
// }

// void FD_MOT::update()
// {
//     update_cmd();
//     update_status();
// }

// void FD_MOT::update_status()
// {
//     if (AP_HAL::millis() - status.last_status_ms > 500) {
//         status.last_status_ms = AP_HAL::millis();
//     }
// }

// void FD_MOT::update_cmd()
// {
//     // 已废弃：油门聚合逻辑移至 FD_CAN::loop()，由FD_CAN统一发帧
//     if (status.group < 1 || status.group > 4) {
//         return;
//     }
//     if (AP_HAL::millis() - status.last_mot_ms > 20) {
//         status.last_mot_ms = AP_HAL::millis();
//         uint16_t tmp_thr = status.thr_in;
//         _data[0] = 0xFF; _data[1] = 0xFF; _data[2] = 0xFF; _data[3] = 0xFF;
//         _data[4] = 0xFF; _data[5] = 0xFF; _data[6] = 0xFF; _data[7] = 0xFF;
//         switch(status.order) {
//         case 1: _data[0] = (uint8_t)((tmp_thr >> 8) & 0xff); _data[1] = (uint8_t)(tmp_thr & 0xff); break;
//         case 2: _data[2] = (uint8_t)((tmp_thr >> 8) & 0xff); _data[3] = (uint8_t)(tmp_thr & 0xff); break;
//         case 3: _data[4] = (uint8_t)((tmp_thr >> 8) & 0xff); _data[5] = (uint8_t)(tmp_thr & 0xff); break;
//         case 4: _data[6] = (uint8_t)((tmp_thr >> 8) & 0xff); _data[7] = (uint8_t)(tmp_thr & 0xff); break;
//         default: return;
//         }
//         uint32_t target_addr = 0x14661C27 + ((status.group - 1) * 0x10000);
//         send_cmd(target_addr | AP_HAL::CANFrame::FlagEFF, _data);
//     }
// }

void FD_MOT::send_cmd(uint32_t id, uint8_t *data) {
    if (_frotend_ptr == nullptr) {return;}
    const uint8_t data_length = 8;
    AP_HAL::CANFrame txFrame{};
    memcpy(txFrame.data, data, data_length);
    txFrame.id = id;
    txFrame.dlc = 8;
    uint64_t timeout = AP_HAL::micros64() + 10000ULL;
    _frotend_ptr->write_frame(txFrame, timeout);
}