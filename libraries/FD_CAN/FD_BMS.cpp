#include "FD_BMS.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>
#include <FD_DATA/FD_DATA.h>

extern const AP_HAL::HAL &hal;

FD_BMS::FD_BMS(FD_CAN *frotend) {
    _frotend_ptr = frotend;
}

void FD_BMS::handle_info(AP_HAL::CANFrame &in_frame, bool do_print) {
    // =====================================================
    // BMS总压及总流数据解析
    // CAN ID: 0x18102701
    // data[0]: SOC (电量百分比)
    // data[1]: SOH (健康度)
    // data[2-3]: 电压 (0.1V分辨率)
    // data[4-5]: 电流 (0.1A分辨率，正放电负充电)
    // =====================================================
    if (in_frame.id == (0x18102701 | AP_HAL::CANFrame::FlagEFF)) {
        status.SOC = (uint8_t)in_frame.data[0];
        status.SOH = (uint8_t)in_frame.data[1];
        
        // 电压解析：高字节在前，单位0.1V
        uint16_t raw_volt = (uint16_t)in_frame.data[2] << 8 | (uint16_t)in_frame.data[3];
        status.Volt = (float)raw_volt * 0.1f;  // 转换为V
        
        // 电流解析：高字节在前，单位0.1A，有符号（正放电负充电）
        int16_t raw_curr = (int16_t)((uint16_t)in_frame.data[4] << 8 | (uint16_t)in_frame.data[5]);
        status.Curr = (float)raw_curr * 0.1f;  // 转换为A

        // 转换到MAVLink包
        // voltage: mV (millivolts)
        AP::fd_data().battery_info_packet.voltage = (uint16_t)(status.Volt * 1000.0f);
        
        // current: mA (milliamps)，int16_t类型，正放电负充电
        AP::fd_data().battery_info_packet.current = (int16_t)(status.Curr * 1000.0f);
        
        // battery_remaining: 百分比
        AP::fd_data().battery_info_packet.battery_remaining = status.SOC;

        if (do_print) {
            gcs().send_text(MAV_SEVERITY_INFO, "BMS: SOC=%d%% V=%.1fV I=%.1fA", 
                           status.SOC, status.Volt, status.Curr);
        }
    }

    // =====================================================
    // BMS充放电及故障状态解析
    // CAN ID: 0x18152701
    // data[0]: 允许充电
    // data[1]: 允许放电
    // data[2]: 充电故障等级
    // data[3]: 充电故障码
    // data[4]: 放电故障等级
    // data[5]: 放电故障码
    // data[6]: 电池状态
    // data[7]: 其他故障码
    // =====================================================
    if (in_frame.id == (0x18152701 | AP_HAL::CANFrame::FlagEFF)) {
        status.allow_charge           = in_frame.data[0];
        status.allow_discharge        = in_frame.data[1];
        status.error_charge_level     = in_frame.data[2];
        status.error_charge_code      = in_frame.data[3];
        status.error_discharge_level  = in_frame.data[4];
        status.error_discharge_code   = in_frame.data[5];
        status.batter_status          = in_frame.data[6];
        status.other_error_code       = in_frame.data[7];

        // 组合错误码：高8位=充电故障码，低8位=放电故障码
        AP::fd_data().battery_error_packet.error_code = 
            (uint16_t)status.error_charge_code << 8 | (uint16_t)status.error_discharge_code;
        
        // 设置电池ID（如果有多个BMS，这里可以根据实际情况修改）
        AP::fd_data().battery_error_packet.battery_id = 0;

        if (do_print) {
            gcs().send_text(MAV_SEVERITY_INFO, "BMS ERR: chg=%d dischg=%d status=%d", 
                           status.error_charge_code, status.error_discharge_code, status.batter_status);
        }
    }
}

void FD_BMS::update()
{
    ;
}

// =====================================================
// BMS控制指令发送
// 标准帧ID: 0x00000102
// 0x02 0x01 = 闭合（上电）
// 0x02 0x00 = 断开（断电）
// =====================================================
void FD_BMS::set_switch(uint8_t switch_in)
{
    // 初始化数据帧
    _data[0] = 0x02;  // 命令字节
    _data[1] = 0x00;  // 默认断开
    _data[2] = 0x00;
    _data[3] = 0x00;
    _data[4] = 0x00;
    _data[5] = 0x00;
    _data[6] = 0x00;
    _data[7] = 0x00;

    if (switch_in == 0) {
        // 断开命令
        _data[1] = 0x00;
        gcs().send_text(MAV_SEVERITY_INFO, "BMS: Sending POWER OFF command");
    }
    else if (switch_in == 1) {
        // 闭合命令
        _data[1] = 0x01;
        gcs().send_text(MAV_SEVERITY_INFO, "BMS: Sending POWER ON command");
    }
    else {
        return;  // 无效命令，不发送
    }

    // 标准帧ID: 0x102 (不带扩展帧标志)
    uint32_t target_addr = 0x102;
    send_cmd(target_addr, _data);
}

void FD_BMS::send_cmd(uint32_t id, uint8_t *data) {
    if (_frotend_ptr == nullptr) {return;}
    const uint8_t data_length = 8;
    AP_HAL::CANFrame txFrame{};
    memcpy(txFrame.data, data, data_length);
    txFrame.id = id;  // 标准帧，不需要设置FlagEFF
    txFrame.dlc = 8;
    uint64_t timeout = AP_HAL::micros64() + 10000ULL;
    _frotend_ptr->write_frame(txFrame, timeout);
}