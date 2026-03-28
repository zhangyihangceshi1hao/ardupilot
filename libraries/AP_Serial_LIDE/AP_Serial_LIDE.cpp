#include "AP_Serial_LIDE.h"
#define LIDE_FRAME_LEN 54
#define LIDE_HEADER1 0x5A
#define LIDE_HEADER2 0xA5
extern const AP_HAL::HAL &hal;

void AP_Serial_LIDE::init(const AP_SerialManager &serial_manager)
{
    _serial_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_LIDE, 0);
    if (_serial_port != nullptr)
    {
        _serial_port->begin(115200);
        //  _serial_port->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_LIDE, 0));
        if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Serial_LIDE::loop, void),
                                          "LIDE",
                                          1024, AP_HAL::Scheduler::PRIORITY_RCIN, 1))
        {
            gcs().send_text(MAV_SEVERITY_ERROR, "Failed to create LIDE thread");
        }
    }
}

void AP_Serial_LIDE::loop(void)
{
    while (true)
    {
        hal.scheduler->delay(500);
        // send_heartbeat_pck();
    }
}

void AP_Serial_LIDE::send_heartbeat_pck()
{
    // 检查串口是否有效
    if (_serial_port == nullptr)
    {
        return;
    }

    // 检查串口是否打开
    if (!_serial_port->is_initialized())
    {
        // 尝试重新初始化
        _serial_port->begin(115200);
        hal.scheduler->delay(1000);
    }

    uint8_t heartbeat_pck[10] = {0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    auto ret = _serial_port->write(heartbeat_pck, sizeof(heartbeat_pck));
    if (ret == 10)
    {
        gcs().send_text(MAV_SEVERITY_ERROR, "LIDE Serial send true");
    }
    else
    {
        gcs().send_text(MAV_SEVERITY_ERROR, "LIDE Serial send failed");
    }
}




void AP_Serial_LIDE::get_telem_data()
{
    if (_serial_port == nullptr) {
        return;
    }

    static uint8_t buffer[LIDE_FRAME_LEN];
    static uint8_t index = 0;

    while (_serial_port->available() > 0) {

        uint8_t byte = _serial_port->read();

        // ================= 找帧头 =================
        if (index == 0) {
            if (byte == LIDE_HEADER1) {
                buffer[index++] = byte;
            }
            continue;
        }

        if (index == 1) {
            if (byte == LIDE_HEADER2) {
                buffer[index++] = byte;
            } else {
                // ❗关键：如果又来了5A，继续当帧头
                if (byte == LIDE_HEADER1) {
                    buffer[0] = LIDE_HEADER1;
                    index = 1;
                } else {
                    index = 0;
                }
            }
            continue;
        }

        // ================= 收满54字节 =================
        buffer[index++] = byte;

        if (index == LIDE_FRAME_LEN) {

            // ================= 打印验证 =================
            for (int i = 0; i < LIDE_FRAME_LEN; i++) {
                gcs().send_text(
                    MAV_SEVERITY_WARNING,
                    "[%02d] %02X",
                    i,
                    buffer[i]
                );
            }

            // ================= 解析 =================
            uint8_t *p = &buffer[3];

            uint8_t system_status = p[0];
            uint16_t total_runtime = (p[1] << 8) | p[2];
            uint16_t current_runtime = (p[3] << 8) | p[4];
            uint16_t fuel_consumption = (p[5] << 8) | p[6];
            uint8_t fuel_rate = p[7];
            uint16_t throttle_feedback = (p[8] << 8) | p[9];
            uint16_t rpm = (p[10] << 8) | p[11];

            uint8_t cht1 = p[12];
            uint8_t cht2 = p[13];
            uint8_t cht3 = p[14];
            uint8_t cht4 = p[15];

            uint8_t egt1 = p[16];
            uint8_t egt2 = p[17];
            uint8_t egt3 = p[18];
            uint8_t egt4 = p[19];

            uint8_t fuel_pressure_set = p[20];
            uint8_t fuel_pressure_actual = p[21];
            uint8_t fuel_pump_rpm = p[22];

            uint8_t rail_pressure_set = p[23];
            uint8_t rail_pressure_actual = p[24];

            uint8_t throttle1_diff = p[25];
            uint8_t throttle1_pos = p[26];
            uint8_t throttle2_diff = p[27];
            uint8_t throttle2_pos = p[28];

            uint8_t voltage = p[29];
            uint8_t cooling1 = p[30];
            uint8_t cooling2 = p[31];

            uint16_t oil_consumption = (p[32] << 8) | p[33];

            uint8_t adjust1 = p[34];
            uint8_t adjust2 = p[35];
            uint8_t adjust3 = p[36];
            uint8_t adjust4 = p[37];

            uint8_t intake_temp = p[38];
            uint8_t env_pressure = p[39];
            uint8_t fuel_level = p[40];

            uint8_t fault1 = p[41];
            uint8_t fault2 = p[42];
            uint8_t fault3 = p[43];
            uint8_t fault4 = p[44];
            uint8_t fault5 = p[45];
            uint8_t fault6 = p[46];
            uint8_t fault7 = p[47];
            uint8_t fault8 = p[48];

            // ================= MAVLink =================
            mavlink_msg_lide_engine_status_full_send(
                MAVLINK_COMM_0,
                system_status,
                total_runtime,
                current_runtime,
                fuel_consumption,
                fuel_rate,
                throttle_feedback,
                rpm,
                cht1, cht2, cht3, cht4,
                egt1, egt2, egt3, egt4,
                fuel_pressure_set,
                fuel_pressure_actual,
                fuel_pump_rpm,
                rail_pressure_set,
                rail_pressure_actual,
                throttle1_diff,
                throttle1_pos,
                throttle2_diff,
                throttle2_pos,
                voltage,
                cooling1,
                cooling2,
                oil_consumption,
                adjust1,
                adjust2,
                adjust3,
                adjust4,
                intake_temp,
                env_pressure,
                fuel_level,
                fault1,
                fault2,
                fault3,
                fault4,
                fault5,
                fault6,
                fault7,
                fault8
            );

            gcs().send_text(MAV_SEVERITY_WARNING, "LIDE frame OK");

            // ❗关键：重新找下一帧（不要残留）
            index = 0;
        }
    }
}