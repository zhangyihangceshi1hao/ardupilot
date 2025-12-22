#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANDriver.h>
#include <AP_Param/AP_Param.h>

// 砺德发动机CAN协议相关定义
#define LIDE_MSG_RATE_HZ_DEFAULT 50
#define LIDE_MSG_RATE_HZ_MIN 10
#define LIDE_MSG_RATE_HZ_MAX 100

#define LIDE_NODE_ID_START 1
#define LIDE_NODE_ID_MAX 8  // 最大支持8个发动机节点

#define LIDE_THROTTLE_MAX 1000  // 油门最大值，对应100%
#define LIDE_THROTTLE_MIN 0

#define LIDE_ONLINE_TIMEOUT_MS 500  // 掉线超时时间（毫秒）
#define LIDE_STATUS_LOG_PERIOD_MS 100  // 状态日志记录周期

// 发动机总体状态枚举
enum LIDE_EngineStatus {
    LIDE_STATUS_NORMAL = 1,
    LIDE_STATUS_ABNORMAL = 2,
    LIDE_STATUS_WARNING = 3
};

// 维保状态枚举
enum LIDE_MaintenanceStatus {
    LIDE_MAINTENANCE_NORMAL = 0,
    LIDE_MAINTENANCE_100H = 1,
    LIDE_MAINTENANCE_200H = 2,
    LIDE_MAINTENANCE_300H = 3
};

// 发动机控制命令位定义
typedef union {
    struct {
        uint8_t stop_cmd : 1;          // bit0: 停机指令
        uint8_t heating_cmd : 1;       // bit1: 加热指令
        uint8_t start_cmd : 1;         // bit2: 启动指令
        uint8_t reserved1 : 1;         // bit3: 保留
        uint8_t start_valid : 1;       // bit4: 启动指令有效位
        uint8_t heating_valid : 1;     // bit5: 加热指令有效位
        uint8_t altitude_valid : 1;    // bit6: 海拔有效位
        uint8_t airspeed_valid : 1;    // bit7: 空速有效位
    } bits;
    uint8_t value;
} LIDE_ControlCommand;

// CAN ID定义（根据文档）
#define LIDE_CAN_ID_CONTROL   0x600   // 飞控->发动机控制指令
#define LIDE_CAN_ID_STATUS1   0x602   // 发动机->飞控状态1
#define LIDE_CAN_ID_STATUS2   0x603   // 发动机->飞控状态2
#define LIDE_CAN_ID_STATUS3   0x604   // 发动机->飞控状态3
#define LIDE_CAN_ID_STATUS4   0x605   // 发动机->飞控状态4
#define LIDE_CAN_ID_STATUS5   0x606   // 发动机->飞控状态5
#define LIDE_CAN_ID_STATUS6   0x607   // 发动机->飞控状态6
#define LIDE_CAN_ID_STATUS7   0x608   // 发动机->飞控状态7

// 砺德发动机数据结构
typedef struct {
    // 配置信息
    uint8_t node_id;                   // CAN节点ID
    bool enabled;                      // 是否使能
    uint32_t last_update_ms;           // 最后更新时间
    
    // 控制相关
    uint16_t throttle_request;         // 油门请求值 (0-1000对应0-100%)
    LIDE_ControlCommand control_cmd;   // 控制命令
    uint16_t altitude;                 // 海拔高度 (米)
    uint8_t airspeed;                  // 空速 (m/s)
    
    // 状态反馈
    uint8_t engine_status;             // 发动机总体状态
    uint8_t maintenance_status;        // 维保状态
    uint16_t engine_rpm;               // 发动机转速
    float engine_runtime_hours;        // 发动机总运行时间
    uint16_t engine_runtime_minutes;   // 当前运行时间
    
    // 温度信息
    uint8_t cylinder_head_temp[4];     // 缸头温度 (1-4缸)
    uint8_t exhaust_temp[4];           // 排气温度 (1-4缸)
    uint8_t intake_temp;               // 进气温度
    
    // 燃油系统
    uint8_t fuel_pressure_target;      // 设定低压燃油压力
    uint8_t fuel_pressure_actual;      // 实际低压燃油压力
    uint8_t rail_pressure_target;      // 设定轨压
    uint8_t rail_pressure_actual;      // 实际轨压
    uint16_t fuel_consumption;         // 当前油耗
    uint8_t fuel_rate_instant;         // 瞬时油耗
    
    // 电子节气门
    uint8_t throttle1_pos;             // 节气门1开度
    uint8_t throttle1_deviation;       // 节气门1偏差
    uint8_t throttle2_pos;             // 节气门2开度 (四缸)
    uint8_t throttle2_deviation;       // 节气门2偏差 (四缸)
    
    // 电气系统
    float system_voltage;              // 系统电压
    uint8_t oil_level;                 // 油位百分比
    
    // 故障状态
    uint8_t fault_bytes[8];            // 8个故障状态字节
    
    // 标志位
    bool is_online;                    // 是否在线
    bool is_running;                   // 是否在运行
    bool is_heating;                   // 是否在加热
} LIDE_Engine_t;

class AP_CAN_LIDE : public AP_CANDriver
{
public:
    AP_CAN_LIDE();
    ~AP_CAN_LIDE();

    // 禁止拷贝
    AP_CAN_LIDE(const AP_CAN_LIDE &other) = delete;
    AP_CAN_LIDE &operator=(const AP_CAN_LIDE&) = delete;

    // 参数定义
    static const struct AP_Param::GroupInfo var_info[];

    // 获取实例
    static AP_CAN_LIDE *get_can_lide(uint8_t driver_index);

    // 初始化接口
    void init(uint8_t driver_index, bool enable_filters) override;
    bool add_interface(AP_HAL::CANIface* can_iface) override;

    // 主循环更新
    void update();

    // 发动机控制接口
    void set_throttle(uint8_t engine_id, uint16_t throttle);  // 0-1000对应0-100%
    void set_start_cmd(uint8_t engine_id, bool start);
    void set_stop_cmd(uint8_t engine_id, bool stop);
    void set_heating_cmd(uint8_t engine_id, bool heating);
    void set_altitude(uint8_t engine_id, uint16_t altitude);
    void set_airspeed(uint8_t engine_id, uint8_t airspeed);
    
    // 状态查询接口
    bool is_engine_online(uint8_t engine_id);
    bool is_engine_running(uint8_t engine_id);
    uint16_t get_engine_rpm(uint8_t engine_id);
    float get_engine_temperature(uint8_t engine_id, uint8_t cylinder);
    float get_fuel_consumption(uint8_t engine_id);
    uint8_t get_engine_status(uint8_t engine_id);
    bool has_fault(uint8_t engine_id);
    
    // 预上电检查
    bool pre_arm_check(char* reason, uint8_t reason_len);

private:
    // 后台线程循环
    void loop();
   
    // CAN通信接口
    bool write_frame(AP_HAL::CANFrame &out_frame, uint64_t timeout);
    bool read_frame(AP_HAL::CANFrame &recv_frame, uint64_t timeout);
    
    // 发送控制命令
    void send_control_command(uint8_t engine_id);
    
    // 处理接收到的状态帧
    void process_status_frame(const AP_HAL::CANFrame &frame);
    
    // 日志记录
    void log_engine_status();
    
    // 物理值转换
    float convert_to_physical(uint16_t raw_value, float scale, float offset);
    uint16_t convert_to_raw(float physical_value, float scale, float offset);
    
    // 私有成员变量
    bool _initialized;
    char _thread_name[16];
    uint8_t _driver_index;
    AP_HAL::CANIface* _can_iface;
    HAL_EventHandle _event_handle;
    
    // 发动机数组
    LIDE_Engine_t _engines[LIDE_NODE_ID_MAX];
    
    // 参数
    AP_Int32 _engine_bm;       // 发动机选择位掩码
    AP_Int16 _update_hz;       // 更新频率
    AP_Int8 _node_id_offset;   // 节点ID偏移
    
    // 状态
    uint32_t _last_log_ms;
    uint32_t _last_update_ms;
};