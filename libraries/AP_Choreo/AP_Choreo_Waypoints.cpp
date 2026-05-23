// ============================================================================
//  AP_Choreo_Waypoints.cpp —— SD 卡 CSV 解析 + 航点归一化
// ----------------------------------------------------------------------------
//  启动时一次性调用 _load_csv("/APM/choreo.csv")，做这几件事：
//
//    1. 用 AP_Filesystem 打开 SD 卡上的 CSV 文件
//       （SITL 自动映到 进程 cwd）
//    2. 逐行解析，支持两种格式：
//         - 8 列  老格式：uavid, frame, x, y, z, r, g, b（master lua 兼容）
//                          → 全部当 type='N'，Blender XYZ → NED 转换
//         - 9 列  新格式：uavid, frame_or_ts, type, a, b, c, r, g, b
//                          → type='N' 或 'L'，分别走 NED 米偏移 / LLA 绝对
//    3. **只挑当前飞控 sysid（MAV_SYS_ID）匹配的那些航点存**
//         一份 CSV 包含 5 架的航点，每架自动挑自己那 50 条
//    4. r/g/b 浮点 0..1 → uint8 0..255
//    5. 按 frame 或 t_s 排序 + 归一化到 t_norm ∈ [0, 1]
//
//  设计文档：docs/STORAGE_OPTIONS.md（为啥用 CSV）
//            docs/WAYPOINT_TYPES.md（N/L 双类型）
//            docs/TIME_MODEL.md（t_norm 归一化）
// ============================================================================
#include "AP_Choreo.h"

#if AP_CHOREO_ENABLED

#include <AP_Filesystem/AP_Filesystem.h>   // AP::FS() 跨平台文件 API
#include <GCS_MAVLink/GCS.h>               // gcs().send_text()
#include <GCS_MAVLink/GCS_MAVLink.h>       // mavlink_system.sysid
#include <AP_Math/AP_Math.h>               // constrain_*

#include <stdio.h>      // sscanf
#include <string.h>     // strchr/strcmp
#include <stdlib.h>     // atof
#include <ctype.h>      // isspace, isalpha

// 某些平台 fcntl.h 没有 O_RDONLY，自己 fallback
#ifndef O_RDONLY
#define O_RDONLY 0
#endif

// ----------------------------------------------------------------------------
//  辅助：数一行有多少列（按逗号分割）
//  返 commas+1。例如 "a,b,c" 有 2 个逗号 → 3 列。
// ----------------------------------------------------------------------------
static int _count_columns(const char *line)
{
    int n = 1;
    for (const char *p = line; *p && *p != '\n' && *p != '\r'; p++) {
        if (*p == ',') n++;
    }
    return n;
}

// ----------------------------------------------------------------------------
//  辅助：跳过行首空白（空格、tab 等）
// ----------------------------------------------------------------------------
static const char *_skip_ws(const char *p)
{
    while (*p && isspace((unsigned char)*p)) p++;
    return p;
}

// ----------------------------------------------------------------------------
//  辅助：浮点 0..1 → uint8 0..255，越界饱和
//  +0.5f 是四舍五入（int truncation 默认朝零截，加 0.5 让它就近取整）
// ----------------------------------------------------------------------------
static uint8_t _f_to_u8(float v)
{
    int x = int(v * 255.0f + 0.5f);
    if (x < 0)   x = 0;
    if (x > 255) x = 255;
    return uint8_t(x);
}

// ----------------------------------------------------------------------------
//  内部用结构：Waypoint + 原始排序 key
//  -----------------------------------------------------------------
//  Load 过程分两步：
//    第一步：边读边解析，存 _RawWp[]（带原始 frame 或 t_s 值作 order_key）
//    第二步：按 order_key 排序，然后归一化到 t_norm ∈ [0,1]，存进 _wps[]
//  这么做是因为 CSV 行**不保证按 frame 升序**写（导出工具可能乱序）。
// ----------------------------------------------------------------------------
struct _RawWp {
    AP_Choreo::Waypoint wp;
    float order_key;        // 原始 frame 号 或 t_s 值（归一化前）
};

// ----------------------------------------------------------------------------
//  辅助：按 order_key 升序排序（插入排序，O(n²) 但 n≤500 无所谓）
//  使用插入排序而不是 std::sort 是为了避免引入 <algorithm> 模板膨胀
// ----------------------------------------------------------------------------
static void _sort_by_key(_RawWp *arr, uint16_t n)
{
    for (uint16_t i = 1; i < n; i++) {
        _RawWp tmp = arr[i];
        int16_t j = int16_t(i) - 1;
        while (j >= 0 && arr[j].order_key > tmp.order_key) {
            arr[j+1] = arr[j];
            j--;
        }
        arr[j+1] = tmp;
    }
}

// ============================================================================
//  _load_csv() —— 从 SD 卡装载航点表
//  ----------------------------------------------------------------------------
//  调用时机：AP_Choreo::init() 启动一次，由 ArduCopter/system.cpp 触发。
//
//  返回：true = 至少装载了 1 条航点；false = 文件没找到 / 解析空 / 没匹配 sysid
//
//  失败时不致命，主循环会因为 _num_wps==0 自动跳过采样，飞控正常待命。
//  用户可以后续通过 MAVFTP 重新上传 CSV 然后重启飞控生效。
// ============================================================================
bool AP_Choreo::_load_csv(const char *path)
{
    // AP_Filesystem 抽象层：真机走 FATFS（SD 卡），SITL 走 POSIX file（cwd）
    auto &fs = AP::FS();
    int fd = fs.open(path, O_RDONLY);
    if (fd < 0) {
        // 文件不存在 / 权限错 / 介质故障 等
        return false;
    }

    // 本机 sysid（MAV_SYS_ID 参数对应），用来挑只属于自己的航点
    const uint8_t self_sysid = mavlink_system.sysid;
    _num_wps = 0;

    char line[200];          // 每行字符 buffer（CSV 一行不会超过 200 字符）
    int  header_cols = 0;    // 8（老格式）或 9（新格式）
    bool header_seen = false;

    // 临时 buffer：先存原始航点（带 order_key 排序），稍后再归一化复制到 _wps[]
    // static 数组：避开栈上 500*sizeof(_RawWp) ~12KB 分配（嵌入式栈可能不够）
    static _RawWp raw[MAX_WPS];
    uint16_t raw_n = 0;

    // ====================== 逐行解析 ======================
    while (fs.fgets(line, sizeof(line), fd)) {
        const char *p = _skip_ws(line);

        // 跳过空行 / 注释行（# 开头）
        if (*p == '\0' || *p == '#' || *p == '\n' || *p == '\r') {
            continue;
        }

        // 第一行非空非注释 → 看是否 header
        if (!header_seen) {
            header_seen = true;
            header_cols = _count_columns(p);
            // 如果第一个字符是字母（比如 "uavid,frame,..."），是 header 行，跳过
            // 否则是数字（数据行），fall through 继续解析
            if (isalpha((unsigned char)*p)) {
                continue;
            }
            // 没有 header 行，直接进数据行（兼容某些导出器不写 header 的情况）
        }

        // 防溢出：航点超过 MAX_WPS 就停止读（多余的丢弃）
        if (raw_n >= MAX_WPS) {
            break;
        }

        Waypoint wp {};       // 当前航点（清零初始化）
        float order_key = 0.0f;
        bool  ok = false;     // 这一行是否成功解析

        // ====================== 老 8 列格式 ======================
        // uavid, frame, x, y, z, r, g, b
        // Blender 坐标系 (x=东, y=北, z=上) → NED (north=y, east=x, up=z)
        if (header_cols == 8) {
            int uavid = 0;
            int frame = 0;
            float x = 0, y = 0, z = 0, rf = 0, gf = 0, bf = 0;
            // sscanf 返回成功解析的字段数，==8 才算完整
            if (sscanf(p, "%d,%d,%f,%f,%f,%f,%f,%f",
                       &uavid, &frame, &x, &y, &z, &rf, &gf, &bf) == 8) {
                // 只挑自己 sysid 的航点
                if (uavid == int(self_sysid)) {
                    wp.type = 'N';   // 老格式全部按 NED 解释
                    // Blender (x,y,z) → ArduPilot NED:
                    //   Blender x 是东方向，但 NED 用 east_m
                    //   Blender y 是北方向，对应 north_m
                    //   Blender z 是上方向，对应 up_m
                    wp.pos.ned.north_m = y;
                    wp.pos.ned.east_m  = x;
                    wp.pos.ned.up_m    = z;
                    // 浮点 RGB → uint8（_f_to_u8 含越界饱和）
                    wp.r = _f_to_u8(rf);
                    wp.g = _f_to_u8(gf);
                    wp.b = _f_to_u8(bf);
                    order_key = float(frame);
                    ok = true;
                }
                // 不是自己 sysid 的，直接忽略（其它飞机的航点）
            }
        }
        // ====================== 新 9 列格式 ======================
        // uavid, frame_or_ts, type, a, b, c, r, g, b
        else if (header_cols == 9) {
            int uavid = 0;
            float key = 0;       // frame 或 t_s 数字
            char tch = 0;        // type char ('N' 或 'L')
            float a = 0, b = 0, c = 0, rf = 0, gf = 0, bf = 0;
            if (sscanf(p, "%d,%f,%c,%f,%f,%f,%f,%f,%f",
                       &uavid, &key, &tch, &a, &b, &c, &rf, &gf, &bf) == 9) {
                if (uavid == int(self_sysid)) {
                    if (tch == 'L' || tch == 'l') {
                        // LLA 绝对经纬度
                        // a=lat度, b=lng度, c=相对 HOME 高度米
                        wp.type = 'L';
                        wp.pos.lla.lat_1e7 = int32_t(a * 1e7);  // 度 → 1e-7 度
                        wp.pos.lla.lng_1e7 = int32_t(b * 1e7);
                        wp.pos.lla.up_m    = c;
                    } else {
                        // 默认 NED 米偏移
                        // a=north_m, b=east_m, c=up_m
                        wp.type = 'N';
                        wp.pos.ned.north_m = a;
                        wp.pos.ned.east_m  = b;
                        wp.pos.ned.up_m    = c;
                    }
                    wp.r = _f_to_u8(rf);
                    wp.g = _f_to_u8(gf);
                    wp.b = _f_to_u8(bf);
                    order_key = key;
                    ok = true;
                }
            }
        }
        // 其它列数（非 8 也非 9）→ 静默忽略

        // 解析成功，存进临时 buffer
        if (ok) {
            raw[raw_n].wp = wp;
            raw[raw_n].order_key = order_key;
            raw_n++;
        }
    }

    fs.close(fd);

    // ====================== 排序 + 归一化 ======================

    // 按原始 frame / t_s 升序排序，保证 t_norm 空间单调
    // （CSV 行不保证有序，导出工具可能按 sysid 分组）
    if (raw_n > 1) {
        _sort_by_key(raw, raw_n);
    }

    if (raw_n == 0) {
        // 没匹配到自己的航点（可能 CSV 里没这架 sysid，或者文件本身有问题）
        _num_wps = 0;
        gcs().send_text(MAV_SEVERITY_WARNING,
            "AP_Choreo: 0 wps for sysid=%u in %s",
            (unsigned)self_sysid, path);
        return false;
    }

    // 归一化到 t_norm ∈ [0, 1]
    // 最小 frame/t_s → 0.0
    // 最大 frame/t_s → 1.0
    // 中间按比例
    const float k_min = raw[0].order_key;
    const float k_max = raw[raw_n-1].order_key;
    const float span  = (k_max - k_min);

    for (uint16_t i = 0; i < raw_n; i++) {
        _wps[i] = raw[i].wp;
        if (span > 1e-6f) {
            // 正常归一化
            _wps[i].t_norm = (raw[i].order_key - k_min) / span;
        } else {
            // 所有航点 key 都一样（异常情况），统一塞 0
            _wps[i].t_norm = 0.0f;
        }
    }
    _num_wps = raw_n;

    // 报告装载结果给 GCS（开演前能在 STATUSTEXT 看到这条确认）
    gcs().send_text(MAV_SEVERITY_INFO,
        "AP_Choreo: loaded %u wps (t_norm 0..1) for sysid=%u",
        (unsigned)_num_wps, (unsigned)self_sysid);

    return _num_wps > 0;
}

#endif // AP_CHOREO_ENABLED
