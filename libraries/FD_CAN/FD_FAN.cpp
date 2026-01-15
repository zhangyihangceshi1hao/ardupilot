#include "FD_FAN.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL &hal;


FD_FAN::FD_FAN(FD_CAN *frotend) {
    _frotend_ptr = frotend;
    _state_time = 0;
    ask_state = Fan_state::Status;
    _print_enable = false;
}

void FD_FAN::handle_info(AP_HAL::CANFrame &in_frame, bool do_print) {
    if (_print_enable) {gcs().send_text(MAV_SEVERITY_INFO, "in_frame.id: %x | %d", (int)in_frame.id, (int)in_frame.id);}
    if (in_frame.id != (0x00D63000| AP_HAL::CANFrame::FlagEFF)) {
        return;
    }
    if (in_frame.data[0] != 0x03) {
        return;
    }
    switch(ask_state) {
        case Fan_state::Status:
        {
            if (in_frame.data[1] == 0x01) {
                fan_info.status_code = (((uint16_t)in_frame.data[2])<<8) + in_frame.data[3];
                if (_print_enable) {gcs().send_text(MAV_SEVERITY_INFO, "Status rep: %d", fan_info.status_code);}
            }
        }
        break;
        case Fan_state::RPM:
        {
            if (in_frame.data[1] == 0x01) {
                fan_info.rpm = (((uint16_t)in_frame.data[2])<<8) + in_frame.data[3];
                if (_print_enable) {gcs().send_text(MAV_SEVERITY_INFO, "RPM rep: %d", fan_info.rpm);}
            }
        }
        break;
        case Fan_state::SampleTime:
        {
            if (in_frame.data[1] == 0x01) {
                fan_info.sampletime = (((uint16_t)in_frame.data[2])<<8) + in_frame.data[3];
                if (_print_enable) {gcs().send_text(MAV_SEVERITY_INFO, "SampleTime rep: %d", fan_info.sampletime);}
            }
        }
        break;
    }
}

void FD_FAN::do_start(bool start_in) {
    msg_data[0] = 0x06;
    msg_data[1] = 0x00;
    msg_data[2] = 20;
    msg_data[3] = 0x00;
    msg_data[4] = start_in;
    msg_data[5] = 0x00;
    msg_data[6] = 0x00;
    msg_data[7] = 0x00;
    send_cmd(0x00D62000, msg_data);
}

void FD_FAN::update() {
    update_state();

    static uint32_t _last_mav_ms = AP_HAL::millis();
    if (AP_HAL::millis() - _last_mav_ms > 500) {
        _last_mav_ms = AP_HAL::millis();
        gcs().send_message(MSG_TXHY_FAN206_STATUS);
    }

    log_status();
}

void FD_FAN::update_state() {
    uint32_t state_ms = AP_HAL::millis() - _state_time;
    switch(ask_state) {
        case Fan_state::Status:
        {
            if (state_ms > 2000) {
                set_state(Fan_state::RPM);
            }
        }
        break;
        case Fan_state::RPM:
        {
            if (state_ms > 2000) {
                set_state(Fan_state::SampleTime);
            }
        }
        break;
        case Fan_state::SampleTime:
        {
            if (state_ms > 2000) {
                set_state(Fan_state::Status);
            }
        }
        break;
    }
}

void FD_FAN::set_state(Fan_state state_in) {
    if (ask_state == state_in) {
        return;
    }
    ask_state = state_in;
    _state_time = AP_HAL::millis();
    switch(ask_state) {
        case Fan_state::Status:
        {
            msg_data[0] = 0x03;
            msg_data[1] = 0x00;
            msg_data[2] = 0x00;
            msg_data[3] = 0x00;
            msg_data[4] = 0x01;
            msg_data[5] = 0x00;
            msg_data[6] = 0x00;
            msg_data[7] = 0x00;
            send_cmd(0x00D62000| AP_HAL::CANFrame::FlagEFF, msg_data);
        }
        break;
        case Fan_state::RPM:
        {
            msg_data[0] = 0x03;
            msg_data[1] = 0x00;
            msg_data[2] = 0x01;
            msg_data[3] = 0x00;
            msg_data[4] = 0x01;
            msg_data[5] = 0x00;
            msg_data[6] = 0x00;
            msg_data[7] = 0x00;
            send_cmd(0x00D62000| AP_HAL::CANFrame::FlagEFF, msg_data);
        }
        break;
        case Fan_state::SampleTime:
        {
            msg_data[0] = 0x03;
            msg_data[1] = 0x00;
            msg_data[2] = 0x02;
            msg_data[3] = 0x00;
            msg_data[4] = 0x01;
            msg_data[5] = 0x00;
            msg_data[6] = 0x00;
            msg_data[7] = 0x00;
            send_cmd(0x00D62000| AP_HAL::CANFrame::FlagEFF, msg_data);
        }
        break;
    }
}

void FD_FAN::send_cmd(uint32_t id, uint8_t *data) {
    if (_frotend_ptr == nullptr) {return;}
    const uint8_t data_length = 8;
    AP_HAL::CANFrame txFrame{};
    memcpy(txFrame.data, data, data_length);
    txFrame.id = id;
    txFrame.dlc = 8;
    uint64_t timeout = AP_HAL::micros64() + 10000ULL;
    _frotend_ptr->write_frame(txFrame, timeout);
}

void FD_FAN::log_status(void) {
    static uint32_t _last_log_ms = AP_HAL::millis();
    if (AP_HAL::millis() - _last_log_ms < 1000) {
        return;
    }
    _last_log_ms = AP_HAL::millis();
    AP::logger().WriteStreaming("F206","TimeUS,status,rpm,smpt",
                                "s---",
                                "F---",
                                "Qfff",
                                AP_HAL::micros64(),
                                fan_info.status_code,
                                fan_info.rpm,
                                fan_info.sampletime);
}
