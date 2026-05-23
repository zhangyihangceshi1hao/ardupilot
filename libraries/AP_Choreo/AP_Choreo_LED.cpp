// ============================================================================
//  AP_Choreo_LED.cpp —— 航灯输出 + GCS 回报
// ----------------------------------------------------------------------------
//  两件事：
//    1. _drive_led(R,G,B) —— 把当前帧的颜色推到硬件 LED
//         a) NeoPixel 灯条（WS281x），走 AP_SerialLED 子系统
//         b) 板载 Notify LED（NTF_LED_TYPES 配的 RGB 灯）
//       两路并行尝试，失败静默不影响主循环
//
//    2. _report_led(now_ms, R, G, B) —— 5 Hz 限频通过 MAVLink NAMED_VALUE_FLOAT
//       把当前 RGB 推回 GCS（GCS 3D 视图球颜色实时同步用）
//
//  设计文档：docs/FIRMWARE_PLAN.md 第二节、docs/TIME_SYNC.md（关于 GCS 协议）
//
//  关于 packed RGB：
//    Float32 mantissa 24 bit 能精确表示 0..2^24=16,777,215 之间任何整数。
//    R*65536 + G*256 + B = max 0xFFFFFF = 16,777,215，正好踩满上限。
//    GCS 端 `int(round(value)) >> 16 / >> 8 / & 0xFF` 解回原 RGB，精确。
// ============================================================================
#include "AP_Choreo.h"

#if AP_CHOREO_ENABLED

#include <GCS_MAVLink/GCS.h>            // gcs().send_named_float()
#include <AP_SerialLED/AP_SerialLED.h>  // NeoPixel WS281x 驱动
#include <AP_Notify/AP_Notify.h>        // 板载 Notify LED

// ----------------------------------------------------------------------------
//  _drive_led() —— 把 RGB 推到实际硬件 LED
//  ----------------------------------------------------------------------------
//  调用频率：50 Hz（跟随 update() 主循环）。
//  失败处理：两个子系统都可能没装（编译时禁用 / 配置没接），都用 nullptr 检查
//  静默跳过。主舞步不受影响。
//
//  NeoPixel 灯条接线说明：
//    硬件 SERIAL1 串口（一般 TX1 引脚）-> WS281x 灯条
//    飞控参数 SERIAL1_PROTOCOL = 29 (NeoPixel)
//    SERIAL_LED1_NUM = N (灯条上灯珠数量)
//    chan=1 对应 SERIAL1
//    led=-1 表示整条灯带统一刷成同一色
//
//  Notify LED：
//    板载小 RGB 灯（Cube Orange 顶部 / Pixhawk 前部）
//    NTF_LED_TYPES 参数控制哪些 LED 接 Notify 子系统
//    rate=255 全亮（默认）
// ----------------------------------------------------------------------------
void AP_Choreo::_drive_led(uint8_t R, uint8_t G, uint8_t B)
{
    // ① NeoPixel 灯条（chan=1 → SERIAL1，led=-1 → 全条统一）
    AP_SerialLED *sled = AP_SerialLED::get_singleton();
    if (sled != nullptr) {
        sled->set_RGB(1, -1, R, G, B);   // 设置颜色（缓存到内部 buffer）
        sled->send(1);                    // 触发把 buffer 串行送到灯条
    }

    // ② 板载 Notify LED + MAV_CMD_DO_SET_LED 广播
    // AP_Notify 是个 namespace 函数，没 singleton 检查必要
    AP_Notify::handle_rgb(R, G, B, 255);
}

// ----------------------------------------------------------------------------
//  _report_led() —— 5 Hz 限频回报 GCS
//  ----------------------------------------------------------------------------
//  调用频率：50 Hz 进入，每 200 ms 真正发一次（限频 5 Hz）
//
//  为什么 5 Hz？
//    主循环 50 Hz 太密，会占用 MAVLink 带宽（5 架 × 50Hz × 28B/msg ≈ 7 KB/s
//    每架，5 架总共 35KB/s 全是 LED 状态，挤占其它 telemetry）。
//    5 Hz 足够 GCS 3D 视图视觉同步（人眼无感的更新率）。
//
//  packed 格式（24-bit 整数 → 32-bit float）：
//    packed = R*65536 + G*256 + B   (R/G/B ∈ 0..255, max packed = 16,777,215)
//    float32 mantissa 24 bit → 能精确装下 max 2^24-1
//    GCS 端 int(round(msg.value)) 解包，再 >>16/>>8/&0xFF 拿回原 R/G/B
//    详见 GCS 端 gcs/mav/vehicle.py 解析逻辑（master 分支 Python 实现参考）
// ----------------------------------------------------------------------------
void AP_Choreo::_report_led(uint32_t now_ms, uint8_t R, uint8_t G, uint8_t B)
{
    // 限频：距上次发不到 200ms 就跳过
    if (now_ms - _last_led_report_ms < LED_REPORT_INTERVAL_MS) {
        return;
    }
    _last_led_report_ms = now_ms;

    // R/G/B 打包成单个 24-bit 整数（高 8 位 R / 中 8 位 G / 低 8 位 B）
    const uint32_t packed = (uint32_t(R) << 16) | (uint32_t(G) << 8) | uint32_t(B);

    // 通过 MAVLink NAMED_VALUE_FLOAT 消息推给 GCS
    // name = "LED_RGB", value = packed 转 float（精确无损）
    gcs().send_named_float("LED_RGB", float(packed));
}

#endif // AP_CHOREO_ENABLED
