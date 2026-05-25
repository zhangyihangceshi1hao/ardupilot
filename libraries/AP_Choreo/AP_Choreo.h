// ============================================================================
//  AP_Choreo.h —— 无人机编队表演舞步执行库（主类声明）
// ----------------------------------------------------------------------------
//  这个库是把 master 分支 lua v1.4 (gcs/lua/template.lua) 的舞步执行逻辑迁移
//  到 ArduPilot 原生 C++ 的实现。设计完全脱钩 GCS，飞控自治执行：
//
//      ┌─────────────────────────────────────────────────────────────┐
//      │  启动 init():                                                │
//      │    从 SD 卡 /APM/choreo.csv 读 50 个航点到 _wps[]            │
//      │    归一化到 t_norm ∈ [0, 1]                                  │
//      │                                                              │
//      │  50 Hz update():                                             │
//      │    1) 看 mode == GUIDED？                                    │
//      │    2) 高度 ≥ MIN_ALT？                                       │
//      │    3) check_arm()：COMMAND_INT MAV_CMD_USER_1 触发 → 双轨记起点│
//      │    4) elapsed_s = (now - arm) ← UTC 主、millis 兜底          │
//      │       返负 → ARMED_WAIT，飞 waypoint[0] 悬停 + LED 进入第一帧 │
//      │    5) t_norm = (elapsed/cycle) mod 1                         │
//      │    6) 采样航点 → 算 absolute Location → set_target_location  │
//      │    7) 同步驱动 NeoPixel + 上报 NAMED_VALUE_FLOAT(LED_RGB)    │
//      └─────────────────────────────────────────────────────────────┘
//
//  跟 ArduCopter 4 处集成：
//    1. Copter.h         — class Copter 内加 `AP_Choreo choreo;`
//    2. Copter.cpp       — scheduler_tasks[] 加 50 Hz 调度项
//    3. Parameters.cpp   — `GOBJECT(choreo, "CHOREO_", AP_Choreo)`
//    4. system.cpp       — init_ardupilot() 末尾调 `choreo.init()`
//    5. ArduCopter/wscript — 把 'AP_Choreo' 加进库列表
//    6. ArduCopter/Parameters.h — 加 k_param_choreo 参数枚举 id
//
//  跨架时间同步精度：
//    GPS 锁定时 < 1 μs（卫星原子钟）
//    GPS 失锁退到本机 millis()，~10 ms 跨架
//  详见 docs/TIME_SYNC.md 第三-六节
//
//  作者：FormationPerformance / Zhang-Yi-Hang  2026-05
//  设计文档：D:\Study_Code\模型\docs\FIRMWARE_PLAN.md
//            D:\Study_Code\模型\docs\TIME_SYNC.md
//            D:\Study_Code\模型\docs\WAYPOINT_TYPES.md
//            D:\Study_Code\模型\docs\TIME_MODEL.md
// ============================================================================
#pragma once

#include "AP_Choreo_config.h"

#if AP_CHOREO_ENABLED

#include <AP_Param/AP_Param.h>          // AP_Float/AP_Int8 等参数类
#include <AP_Math/AP_Math.h>            // Vector3f, is_zero, constrain_*
#include <AP_Common/Location.h>         // Location 类，绝对 LLA + alt_frame
#include <GCS_MAVLink/GCS_MAVLink.h>    // mavlink_command_int_t / MAV_RESULT

class AP_Choreo {
public:
    AP_Choreo();
    CLASS_NO_COPY(AP_Choreo);           // 禁止拷贝，singleton 语义

    // 单例访问（其它库通过 AP::choreo() namespace 函数取，见文件末尾）
    static AP_Choreo* get_singleton() { return _singleton; }

    // 启动时一次性调，由 ArduCopter/system.cpp init_ardupilot() 末尾发起
    void init();

    // 50 Hz 主循环，由 ArduCopter/Copter.cpp scheduler_tasks[] 调度
    void update();

    // 处理 COMMAND_INT MAV_CMD_USER_1（=31010）—— 同步扳机入口
    //   x = int32(target_usec >> 32)
    //   y = int32(target_usec & 0xFFFFFFFF)
    //   (x=0, y=0) 表示解武装
    //   COMMAND_INT 的 x/y 是原生 int32，不像 PARAM_SET INT32 走 float→int32 强转
    //   会破坏 bit pattern（int32 bit 经 float 中转易变 NaN/denormal）。
    //   每次到达都触发扣扳机，不依赖值变化。
    MAV_RESULT handle_command_int_packet(const mavlink_command_int_t &packet);

    // AP_Param 参数表声明（在 .cpp 里定义具体的 AP_GROUPINFO）
    static const struct AP_Param::GroupInfo var_info[];

    // ------------------------------------------------------------------
    //  Waypoint 结构（一条航点记录）
    //  -----------------------------------------------------------------
    //  详细字段说明见 docs/WAYPOINT_TYPES.md 第三节。
    //  关键设计：
    //    - 一个 union 同时支持 NED（type='N'）和 LLA（type='L'）两种类型
    //    - lat/lng 用 int32 × 1e-7 度，精度 ~1cm（float32 在 lat=35° 处只有 10cm）
    //    - r,g,b 用 uint8 0~255，CSV 里的 0~1 float 在 load 时已转换
    //    - sizeof(Waypoint) = 4 + 1 + 3 + 12 = 20 字节
    //    - MAX_WPS = 500 → 共 10 KB SRAM，板载 Cube Orange / Pixhawk 6X 充足
    // ------------------------------------------------------------------
    struct Waypoint {
        float   t_norm;       // 归一化时间 0..1（CSV 里 frame 或 t_s 在 load 后统一映射到这）
        uint8_t type;         // 'N' = NED 米偏移; 'L' = 绝对经纬度
        uint8_t r, g, b;      // 航灯颜色 0..255
        union {
            // type='N' 时：相对【主机 HOME】的米偏移
            //   north_m = 北方向，east_m = 东方向，up_m = 上方向（相对 HOME 高度）
            struct { float   north_m, east_m, up_m;   } ned;
            // type='L' 时：绝对 GPS 经纬度 + 相对【本机 HOME】的高度米
            //   lat_1e7/lng_1e7 = 度 × 1e7（保 ~1cm 精度）
            struct { int32_t lat_1e7, lng_1e7;
                     float   up_m;                      } lla;
        } pos;
    };
    static constexpr uint16_t MAX_WPS = 500;    // 最大航点数 → 10 KB 内存

    // ------------------------------------------------------------------
    //  AbsPos —— 内部"绝对位置"的统一表示
    //  -----------------------------------------------------------------
    //  采样 _sample_at_t_norm() 时无论原始 type='N' 还是 'L'，都先转成 AbsPos
    //  再做插值，结果是绝对 LLA + 相对 HOME 高度。下发时直接喂给 Location。
    // ------------------------------------------------------------------
    struct AbsPos {
        int32_t lat_1e7;       // 经纬度（绝对 GPS）
        int32_t lng_1e7;
        float   up_m;          // 米，相对【本机 HOME】的高度
    };

private:
    static AP_Choreo* _singleton;   // 全局单例指针，由构造函数赋值

    // ============== 参数（暴露给 GCS / MissionPlanner 调） ==============
    AP_Int8  _enable;        // CHOREO_ENABLE       0=禁用 1=启用
    AP_Int32 _nonce;         // CHOREO_NONCE        【已废弃】保留兼容 EEPROM，_check_arm 不再用
    AP_Float _lead;          // CHOREO_LEAD         【已废弃】保留兼容 EEPROM，lead 已编进 T_HI/LO
    AP_Float _cycle;         // CHOREO_CYCLE        一圈秒数（默认 10s）
    AP_Float _master_lat;    // CHOREO_MLAT         主机 lat 度（type='N' 用）
    AP_Float _master_lon;    // CHOREO_MLON         主机 lng 度
    AP_Float _base_alt;      // CHOREO_BASE_ALT     基准高度补偿米（加到每帧 z 上）
    AP_Float _min_alt;       // CHOREO_MIN_ALT      最低开演高度米（默认 3m）
    AP_Int8  _loop;          // CHOREO_LOOP         0=单次跑完停 1=循环
    AP_Int32 _t_hi;          // CHOREO_T_HI 【已废弃】仅保留 EEPROM 兼容（idx 9），不再读取
    AP_Int32 _t_lo;          // CHOREO_T_LO 【已废弃】仅保留 EEPROM 兼容（idx 10），不再读取
                             // —— 同步扳机改走 COMMAND_INT MAV_CMD_USER_1（bit-preserve int32）
    AP_Int8  _loop_num;      // CHOREO_LOOP_NUM     循环次数限制 (idx 11)
                             //   = 0  无限制，受 CHOREO_LOOP 控制（保持旧行为）
                             //   >= 1 强制飞 N 次后 t_norm 锁定 1.0 停在最后一帧
                             //        覆盖 CHOREO_LOOP 设置（即使 LOOP=1 也只飞 N 圈）

    // ============== COMMAND_INT 武装暂存（handle_command_int_packet 写入 → _check_arm 消费）==============
    uint64_t _pending_target_usec = 0;  // 上一次 COMMAND_INT 携带的 target UTC 微秒
    bool     _pending_arm         = false; // 收到 COMMAND_INT 后置 true，_check_arm 消费完清零

    // ============== 状态机 ==============
    enum class State : uint8_t {
        IDLE,           // 未启用 / 非 GUIDED
        WAIT_ALT,       // GUIDED 但高度不够
        ARMED_WAIT,     // 已扣扳机，倒计时中
        RUNNING         // 表演中
    } _state = State::IDLE;

    int32_t  _last_seen_nonce = -1;     // 【deprecated】旧版 nonce 缓存，不再使用
    int32_t  _last_t_hi = 0;            // 【deprecated】旧 T_HI/LO 变化检测残留，新方案改用 _pending_arm
    int32_t  _last_t_lo = 0;            // 【deprecated】同上，保留只为最小化扰动

    // ============== Option B 双轨时间源（docs/TIME_SYNC.md） ==============
    uint64_t _arm_utc_usec = 0;        // GPS UTC 起点（主），微秒
    uint32_t _arm_millis   = 0;         // 本机 millis() 起点（兜底），毫秒
    // 跨架 GPS 锁定时同步 < 1μs，失锁退兜底也 ~10ms

    bool     _started = false;          // 是否已发过 "START" STATUSTEXT（防止反复发）

    // ============== 航点表（启动时从 SD 卡 CSV 装载） ==============
    Waypoint _wps[MAX_WPS];
    uint16_t _num_wps = 0;              // 实际航点数（≤ MAX_WPS）

    // ============== LED 上报限频（5 Hz） ==============
    uint32_t _last_led_report_ms = 0;
    static constexpr uint32_t LED_REPORT_INTERVAL_MS = 200;   // 200ms = 5Hz

    // ============== 私有方法 ==============

    // 从 SD 卡 CSV 装载航点（启动时调一次）
    //   支持 8 列（老格式纯 NED）和 9 列（新格式带 type 字段）
    //   只挑自己 sysid（MAV_SYS_ID）的航点存
    //   load 完按 frame / t_s 排序，归一化到 t_norm ∈ [0,1]
    bool  _load_csv(const char* path);

    // 三层降级读 UTC 时间：rtc → gps → false
    //   AP::rtc() 已经覆盖 GPS / SYSTEM_TIME / 内置 RTC 等所有来源
    //   GPS 直接读 epoch_usec 作二级兜底
    //   都没有时返 false（调用方会自动用 millis 路径）
    bool  _read_utc_now(uint64_t &utc_usec_out) const;

    // 消费 _pending_arm flag 完成武装
    //   handle_command_int_packet 收到 MAV_CMD_USER_1 后写 _pending_*，
    //   本函数在 update() 50Hz 节拍里把暂存值搬到 _arm_utc_usec / _arm_millis。
    //   target_usec=0 表示解武装（_arm_* 清零）
    //   GCS 已把 lead 提前编进 target_usec，飞控不再 +LEAD
    void  _check_arm();

    // 算自起点以来已经过去多少秒
    //   优先用 UTC 路径（精度 < 1μs 跨架）
    //   UTC 不可用退 millis 路径（精度 ~10ms 跨架）
    //   返负数 = 还在倒计时阶段
    float _elapsed_s() const;

    // 把任意 Waypoint 转成 AbsPos（绝对 LLA + 相对 HOME 高度）
    //   type='L' 直接拷贝
    //   type='N' 用 _master_lat/lon 当原点 + Location::offset(north, east) 推
    //   master 没设时退到本机自己的 HOME
    bool  _to_abs(const Waypoint &wp, AbsPos &out) const;

    // 在 [0,1] 时间相位上采样得到 (位置 + RGB)
    //   找到 t_norm 落在哪段 [wps[i], wps[i+1]]
    //   两端各转 AbsPos 再线性插值
    //   颜色 RGB 也插值
    bool  _sample_at_t_norm(float t_norm,
                            AbsPos &out,
                            uint8_t &R, uint8_t &G, uint8_t &B) const;

    // 把当前帧的 RGB 推给硬件 LED
    //   NeoPixel 灯条（SERIAL1 / SERVO1 chan，需 SERIAL1_PROTOCOL=29）
    //   板载 Notify LED（NTF_LED_TYPES）
    //   两路并行尝试，失败静默不影响主循环
    void  _drive_led(uint8_t R, uint8_t G, uint8_t B);

    // 5Hz 限频通过 MAVLink NAMED_VALUE_FLOAT 上报"当前 LED 颜色" 给 GCS
    //   packed = R*65536 + G*256 + B（uint24 编进 float32 mantissa，精确）
    //   GCS 解包后能让 3D 视图球的颜色同步变化
    void  _report_led(uint32_t now_ms, uint8_t R, uint8_t G, uint8_t B);
};

// 命名空间访问器，其它库通过 AP::choreo() 取单例
namespace AP { AP_Choreo* choreo(); }

#endif // AP_CHOREO_ENABLED
