// ============================================================================
//  AP_Choreo.cpp —— 主循环 + 状态机 + 时间同步 + 目标下发
// ----------------------------------------------------------------------------
//  本文件包含 AP_Choreo 库的核心运行时逻辑：
//    - 单例 + 参数表
//    - init()           启动一次，读 CSV
//    - update()         50 Hz 主循环
//    - _check_arm()     扣扳机检测
//    - _elapsed_s()     双轨时间源（GPS UTC 优先 / millis 兜底）
//    - _to_abs()        Waypoint -> AbsPos
//    - _sample_at_t_norm()  时间相位上插值采样
//
//  设计文档：docs/FIRMWARE_PLAN.md + docs/TIME_SYNC.md
// ============================================================================
#include "AP_Choreo.h"

#if AP_CHOREO_ENABLED

#include <GCS_MAVLink/GCS.h>          // gcs().send_text / send_named_float
#include <AP_HAL/AP_HAL.h>            // AP_HAL::millis()
#include <AP_Vehicle/AP_Vehicle.h>    // AP::vehicle()->set_target_location()
#include <AP_AHRS/AP_AHRS.h>          // AP::ahrs().get_relative_position_NED_home / get_home
#include <AP_GPS/AP_GPS.h>            // AP::gps().time_epoch_usec()
#include <AP_RTC/AP_RTC.h>            // AP::rtc().get_utc_usec()
#include <AP_Math/AP_Math.h>          // is_zero, constrain_int16

// ----------------------------------------------------------------------------
//  GUIDED 模式编号 = 4（见 ArduCopter/mode.h 的 Mode::Number::GUIDED）
//  这里硬编码成常量，是为了避免 #include "ArduCopter/Copter.h" 造成的库间
//  循环依赖。代价是：如果 ArduPilot upstream 改了 GUIDED 的 enum 值，
//  这里要跟着改（极少发生，10 年来一直是 4）。
// ----------------------------------------------------------------------------
static constexpr uint8_t CHOREO_VEHICLE_MODE_GUIDED = 4;

// 单例指针，构造函数赋值
AP_Choreo* AP_Choreo::_singleton = nullptr;

// ============================================================================
//  AP_Param 参数表
//  ----------------------------------------------------------------------------
//  暴露 11 个参数给 GCS（MissionPlanner / Qt GCS）调。命名 `CHOREO_*`，跟
//  master lua 方案的 `SCR_USER1..6` 不重叠，可同时存在不冲突。
//
//  注：AP_GROUPINFO 的第 2 个数字（idx）是参数 idx，**永远不能改也不能复用**
//  否则会破坏 GCS 端的参数 ID 映射（cached EEPROM 错乱）。
//  现有占用：0=ENABLE 1=NONCE(废) 2=LEAD(废) 3=CYCLE 4=MLAT 5=MLON
//          6=BASE_ALT 7=MIN_ALT 8=LOOP 9=T_HI 10=T_LO
// ============================================================================
const AP_Param::GroupInfo AP_Choreo::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: AP_Choreo 总开关
    // @Description: 0=禁用（飞控行为跟原版完全一致） 1=启用（参与 50Hz 调度）
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    // 注：AP_PARAM_FLAG_ENABLE 让 MissionPlanner 显示成"主开关"风格
    AP_GROUPINFO_FLAGS("ENABLE", 0, AP_Choreo, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: NONCE
    // @DisplayName: 同步扳机【已废弃】
    // @Description: 旧方案的同步扳机，已被 CHOREO_T_HI/LO 取代。保留只为兼容 EEPROM
    // @User: Standard
    AP_GROUPINFO("NONCE",    1, AP_Choreo, _nonce,      0),

    // @Param: LEAD
    // @DisplayName: 预热秒数【已废弃】
    // @Description: 旧方案的预热秒，新方案 GCS 已把 lead 直接编进 target_usec
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("LEAD",     2, AP_Choreo, _lead,       2.0f),

    // @Param: CYCLE
    // @DisplayName: 一圈秒数
    // @Description: 完整跑完所有航点的总时长，可在线改实现整段慢/快放
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("CYCLE",    3, AP_Choreo, _cycle,      10.0f),

    // @Param: MLAT
    // @DisplayName: 主机经度（type='N' 航点的原点 lat）
    // @Description: GCS 在扣扳机时把 UAV1 当前 lat 写到这。0=没设
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("MLAT",     4, AP_Choreo, _master_lat, 0),

    // @Param: MLON
    // @DisplayName: 主机经度（type='N' 航点的原点 lng）
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("MLON",     5, AP_Choreo, _master_lon, 0),

    // @Param: BASE_ALT
    // @DisplayName: 高度补偿
    // @Description: 加到每帧航点 z 上的基线高度米数（整队抬高/压低）
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("BASE_ALT", 6, AP_Choreo, _base_alt,   0),

    // @Param: MIN_ALT
    // @DisplayName: 最低开演高度
    // @Description: 飞机爬到这个高度才允许进表演（默认 3m，安全门槛）
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("MIN_ALT",  7, AP_Choreo, _min_alt,    3.0f),

    // @Param: LOOP
    // @DisplayName: 循环模式
    // @Description: 0=单次跑完停在最后一帧, 1=无限循环
    // @Values: 0:OneShot,1:Loop
    // @User: Standard
    AP_GROUPINFO("LOOP",     8, AP_Choreo, _loop,       1),

    // @Param: T_HI
    // @DisplayName: 目标 UTC 微秒高 32 位
    // @Description: 5 架同点起跳的目标 UTC 时刻（微秒）的高 32 位。GCS 计算 target=now+lead，
    //               然后拆成 (hi,lo) 一并发给所有飞机；T_HI 写入触发武装（必须最后写）。
    //               (hi=0 && lo=0) 表示解武装
    // @User: Standard
    AP_GROUPINFO("T_HI",     9, AP_Choreo, _t_hi,       0),

    // @Param: T_LO
    // @DisplayName: 目标 UTC 微秒低 32 位
    // @Description: 5 架同点起跳的目标 UTC 时刻（微秒）的低 32 位。GCS 必须先发 T_LO 再发 T_HI，
    //               这样飞控在看到 T_HI 变化时 T_LO 已经就位
    // @User: Standard
    AP_GROUPINFO("T_LO",    10, AP_Choreo, _t_lo,       0),

    AP_GROUPEND
};

// 构造函数：注册单例 + 加载参数默认值
AP_Choreo::AP_Choreo()
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

// ============================================================================
//  init() —— 启动一次性初始化
//  ----------------------------------------------------------------------------
//  由 ArduCopter/system.cpp::init_ardupilot() 末尾调用。在 EKF / GPS / 电机
//  等都已初始化之后运行。这里只做一件事：从 SD 卡读 /APM/choreo.csv 装载航点。
//
//  注意：在 SITL 模式下 AP_Filesystem 会把 `/APM/X` 自动映射到 SITL 进程的
//  当前工作目录的 X 文件。所以可以直接 cp choreo.csv 到 sim_vehicle 启动的
//  那个目录就行，不用走 MAVFTP。详见 docs/SITL_TESTING.md。
// ============================================================================
void AP_Choreo::init()
{
    if (!_enable.get()) {
        // CHOREO_ENABLE=0 → 啥都不做，跟没这库一样
        return;
    }
    gcs().send_text(MAV_SEVERITY_INFO, "AP_Choreo: init starting");

    // SD 卡上的 CSV 路径。SITL 自动映到 cwd
    if (!_load_csv("/APM/choreo.csv")) {
        // 失败不致命 —— 启动后可以通过 MAVFTP 传上去再重启
        gcs().send_text(MAV_SEVERITY_WARNING,
            "AP_Choreo: /APM/choreo.csv not found or empty");
    }
}

// ============================================================================
//  _read_utc_now() —— 三层降级读 UTC 时间（docs/TIME_SYNC.md 第三节）
//  ----------------------------------------------------------------------------
//  优先级：
//    ① AP::rtc()  —— ArduPilot 的 RTC 子系统，会汇总所有可能的时间源
//                   （GPS / SYSTEM_TIME / 板载 RTC 等），是最高抽象层
//    ② AP::gps()  —— 直接从 GPS 接收机读，前提是 GPS 已 3D lock
//    ③ 都失败 → 返 false，调用方应退回 millis() 路径
// ============================================================================
bool AP_Choreo::_read_utc_now(uint64_t &utc_usec_out) const
{
    // ① 先试 RTC（覆盖最广）
    if (AP::rtc().get_utc_usec(utc_usec_out)) {
        return true;
    }
    // ② 再试 GPS 直接读
    const uint64_t t = AP::gps().time_epoch_usec();
    if (t > 0) {
        utc_usec_out = t;
        return true;
    }
    // ③ UTC 不可用
    return false;
}

// ============================================================================
//  _check_arm() —— 扣扳机检测 + 同步武装（docs/TIME_SYNC.md 第四节）
//  ----------------------------------------------------------------------------
//  新方案（NONCE → T_HI/T_LO）的核心改动：
//    旧方案：GCS 发 CHOREO_NONCE++ 触发，飞控本地 _arm_utc_usec = now + LEAD
//            → 5 架收到 PARAM_SET 的时刻不同（UDP 抖动 + 串发延迟），
//              各机 _arm_utc_usec 互相差几十到几百毫秒，起跳点对不齐。
//
//    新方案：GCS 端算好 target_utc_usec = now + lead，拆成 (T_HI, T_LO) 两个
//            int32 一并发给 5 架；飞控收到后都用同一个 target_usec 当
//            _arm_utc_usec → PARAM_SET 到达时差完全不影响起跳精度。
//
//  逻辑：
//    1. 看 (T_HI, T_LO) 跟上次缓存的有没有变
//    2. 没变 → 直接返回
//    3. 变了：
//         (0, 0) → 解武装（_arm_* 清零）
//         非 0   → target_usec = (uint64(uint32(T_HI)) << 32) | uint32(T_LO)
//                  _arm_utc_usec = target_usec        （不再 +LEAD）
//                  _arm_millis 用"距 target 还有多少 ms"算兜底起点
//
//  跨架精度：
//    - 共用同一 target_usec，跨架同步 < 1μs（受限于各机 GPS UTC 误差）
//    - GPS 失锁退 millis 路径，~10ms（millis 抖动 + 晶振漂移）
// ============================================================================
void AP_Choreo::_check_arm()
{
    const int32_t hi = _t_hi.get();
    const int32_t lo = _t_lo.get();
    if (hi == _last_t_hi && lo == _last_t_lo) {
        // 没变化，啥也不做
        return;
    }
    _last_t_hi = hi;
    _last_t_lo = lo;

    if (hi == 0 && lo == 0) {
        // GCS 写 (0,0) 表示"取消武装"
        _arm_utc_usec = 0;
        _arm_millis   = 0;
        _started      = false;
        return;
    }

    // 拼回 64 位目标 UTC 微秒。注意必须先 cast 到 uint32_t 避免负数符号扩展
    const uint64_t target_usec =
        (uint64_t(uint32_t(hi)) << 32) | uint32_t(lo);

    // 5 架统一用 GCS 算好的 target_usec，不再各自 +LEAD
    _arm_utc_usec = target_usec;

    // millis 兜底：算"距 target 还有多少 ms"加到当前 millis
    // —— 这样即使中途 GPS 丢锁，_elapsed_s() 退到 millis 路径仍能近似对齐
    uint64_t utc_now;
    if (_read_utc_now(utc_now) && target_usec > utc_now) {
        const uint64_t delta_usec = target_usec - utc_now;
        _arm_millis = AP_HAL::millis() + uint32_t(delta_usec / 1000ULL);
    } else {
        // GPS 没锁，无法算偏移；只能靠 UTC 路径
        _arm_millis = 0;
    }
    _started = false;   // 重置 START banner 标志，让重新演出能再发一次

    gcs().send_text(MAV_SEVERITY_INFO,
        "AP_Choreo: armed target_utc=%llu (T_HI=%ld T_LO=%ld)",
        (unsigned long long)target_usec, (long)hi, (long)lo);
}

// ============================================================================
//  _elapsed_s() —— 算从起点到现在过去多少秒（docs/TIME_SYNC.md 第五节）
//  ----------------------------------------------------------------------------
//  自动选最优时间源：
//    UTC 可用 → 用 UTC 路径（跨架同步 < 1μs）
//    UTC 失败 → 退到 millis 路径（跨架 ~10ms）
//
//  返负数代表"倒计时中"（还在 lead 缓冲期），update() 会原地等。
//  返 -1e9 代表"未武装"。
//
//  这是双轨方案"无缝切换"的核心：飞行中 GPS 突然失锁，下一帧自动走 millis
//  分支，飞机不停。GPS 恢复后又自动切回 UTC，无感。
// ============================================================================
float AP_Choreo::_elapsed_s() const
{
    uint64_t utc;
    // ① UTC 路径
    if (_arm_utc_usec > 0 && _read_utc_now(utc)) {
        if (utc < _arm_utc_usec) {
            // 还没到武装时刻 → 倒计时（返负值）
            return -float(_arm_utc_usec - utc) * 1e-6f;
        }
        return float(utc - _arm_utc_usec) * 1e-6f;
    }

    // ② millis 兜底
    if (_arm_millis > 0) {
        const uint32_t now = AP_HAL::millis();
        if (now < _arm_millis) {
            return -float(_arm_millis - now) * 1e-3f;
        }
        return float(now - _arm_millis) * 1e-3f;
    }

    return -1e9f;       // 未武装
}

// ============================================================================
//  _to_abs() —— 把 Waypoint 转成绝对位置 AbsPos
//  ----------------------------------------------------------------------------
//  详见 docs/WAYPOINT_TYPES.md 第四节。两种类型走两条不同路径：
//
//    type='L' (LLA 直接绝对)：
//        wp.pos.lla.lat/lng/up_m 直接拷贝到 out
//        因为 lat/lng 本来就是绝对 GPS 坐标
//
//    type='N' (NED 相对原点)：
//        需要把"米偏移"换成"绝对 lat/lng"
//        用 ArduPilot 标准的 Location::offset(north_m, east_m) 函数
//        （内部用球面三角法把米数加到经纬度上）
//        原点选 _master_lat/lon（GCS 写的 UAV1 HOME）
//        如果 master 没设（=0），就退到本机自己的 HOME（会有 HOME 漂移）
// ============================================================================
bool AP_Choreo::_to_abs(const Waypoint &wp, AbsPos &out) const
{
    if (wp.type == 'L') {
        // 直接拷绝对 GPS 坐标
        out.lat_1e7 = wp.pos.lla.lat_1e7;
        out.lng_1e7 = wp.pos.lla.lng_1e7;
        out.up_m    = wp.pos.lla.up_m;
        return true;
    }

    // type='N'，需要主机 HOME 当原点
    const float mlat = _master_lat.get();
    const float mlon = _master_lon.get();
    if (is_zero(mlat) || is_zero(mlon)) {
        // 兜底：GCS 没设 master，用本机自己的 HOME（队形会有 HOME 漂移）
        Location my_home = AP::ahrs().get_home();
        my_home.offset(wp.pos.ned.north_m, wp.pos.ned.east_m);
        out.lat_1e7 = my_home.lat;
        out.lng_1e7 = my_home.lng;
        out.up_m    = wp.pos.ned.up_m;
    } else {
        // 正常：用 GCS 写的 master HOME 当编队原点
        // 5 架共用同一个原点 → 队形不受各机 HOME 漂移影响
        Location master {};
        master.lat = int32_t(mlat * 1e7f);
        master.lng = int32_t(mlon * 1e7f);
        // ArduPilot 标准 API，原地把 lat/lng 加上 (north 米, east 米) 偏移
        master.offset(wp.pos.ned.north_m, wp.pos.ned.east_m);
        out.lat_1e7 = master.lat;
        out.lng_1e7 = master.lng;
        out.up_m    = wp.pos.ned.up_m;
    }
    return true;
}

// ============================================================================
//  _sample_at_t_norm() —— 在 [0,1] 时间相位上插值采样
//  ----------------------------------------------------------------------------
//  详见 docs/TIME_MODEL.md 第四节。算法：
//    1. 找到 t_norm 落在哪段 [wps[i].t_norm, wps[i+1].t_norm]
//    2. 算插值系数 alpha = (t_norm - left) / (right - left)
//    3. 两端 Waypoint 各转成 AbsPos
//    4. 在【绝对 LLA 空间】线性插值（lat/lng 用 double 算避免 int32 截断）
//    5. 颜色 RGB 也线性插值
//
//  为什么在 AbsPos 空间插值而不是原始 Waypoint 空间？
//    因为相邻两个 Waypoint 可能是 N+L 混合类型。先各自转 AbsPos 再插值，
//    跨类型边界自动平滑，没坑。
// ============================================================================
bool AP_Choreo::_sample_at_t_norm(float t_norm,
                                  AbsPos &out,
                                  uint8_t &R, uint8_t &G, uint8_t &B) const
{
    // 边界处理：空航点表 / 单个航点
    if (_num_wps == 0) {
        return false;
    }
    if (_num_wps == 1) {
        if (!_to_abs(_wps[0], out)) return false;
        R = _wps[0].r; G = _wps[0].g; B = _wps[0].b;
        return true;
    }

    // 边界处理：相位在第一帧之前 / 最后一帧之后
    if (t_norm <= _wps[0].t_norm) {
        if (!_to_abs(_wps[0], out)) return false;
        R = _wps[0].r; G = _wps[0].g; B = _wps[0].b;
        return true;
    }
    if (t_norm >= _wps[_num_wps-1].t_norm) {
        const Waypoint &w = _wps[_num_wps-1];
        if (!_to_abs(w, out)) return false;
        R = w.r; G = w.g; B = w.b;
        return true;
    }

    // 找两端航点 (线性扫描 —— 500 个以内可以接受，不用二分)
    uint16_t i = 0;
    for (uint16_t k = 0; k + 1 < _num_wps; k++) {
        if (t_norm >= _wps[k].t_norm && t_norm <= _wps[k+1].t_norm) {
            i = k;
            break;
        }
    }

    const Waypoint &a = _wps[i];
    const Waypoint &b = _wps[i+1];
    const float span = b.t_norm - a.t_norm;
    // 防 span=0 除以零
    const float alpha = (span > 1e-6f) ? (t_norm - a.t_norm) / span : 0.0f;

    // 两端各转绝对位置
    AbsPos pa, pb;
    if (!_to_abs(a, pa) || !_to_abs(b, pb)) {
        return false;
    }

    // 经纬度插值 —— 必须用 double 算
    //   原因：int32 相减可能溢出 int32 范围（例如跨赤道），
    //         先 cast 到 double 算完再 cast 回 int32 才安全
    const double lat_d = double(pa.lat_1e7) + double(pb.lat_1e7 - pa.lat_1e7) * alpha;
    const double lng_d = double(pa.lng_1e7) + double(pb.lng_1e7 - pa.lng_1e7) * alpha;
    out.lat_1e7 = int32_t(lat_d);
    out.lng_1e7 = int32_t(lng_d);
    out.up_m    = pa.up_m + (pb.up_m - pa.up_m) * alpha;

    // RGB 插值 + 饱和到 0..255
    R = uint8_t(constrain_int16(int(a.r + (int(b.r) - int(a.r)) * alpha + 0.5f), 0, 255));
    G = uint8_t(constrain_int16(int(a.g + (int(b.g) - int(a.g)) * alpha + 0.5f), 0, 255));
    B = uint8_t(constrain_int16(int(a.b + (int(b.b) - int(a.b)) * alpha + 0.5f), 0, 255));

    return true;
}

// ============================================================================
//  update() —— 50 Hz 主循环（核心）
//  ----------------------------------------------------------------------------
//  由 ArduCopter scheduler 每 20ms 调一次。9 个步骤：
//
//    (1) GUIDED 检查         —— 不在 GUIDED 不发位置目标
//    (2) 高度检查             —— 飞机得爬过 MIN_ALT 才开演
//    (3) T_HI/LO 触发检测     —— GCS 写 CHOREO_T_HI/T_LO 触发武装
//    (4) 计算 elapsed_s       —— 自动选 UTC 或 millis
//    (5) START 横幅           —— 第一次进 RUNNING 发一条 STATUSTEXT
//    (6) 算 t_norm            —— elapsed/cycle 后 mod 1（或 clamp 1）
//    (7) 采样目标             —— 在 t_norm 处插值得 (位置 + RGB)
//    (8) 下发绝对 Location    —— 用 set_target_location() 飞控自闭环
//    (9) LED 驱动 + 上报      —— NeoPixel + Notify + GCS 回报
//
//  设计原则：
//    - 每步早返回（fail-fast），不在错误状态做后续工作
//    - 用 AP::vehicle() 抽象层，不直接依赖 Copter 类
//    - 时间源走 _elapsed_s()，不直接读 millis（屏蔽双轨切换细节）
// ============================================================================
void AP_Choreo::update()
{
    // 总开关
    if (!_enable.get()) {
        return;
    }

    // 取车辆抽象（Copter / Plane 都实现了 AP_Vehicle 接口）
    AP_Vehicle *vehicle = AP::vehicle();
    if (vehicle == nullptr) {
        return;
    }

    // ============= (1) 模式检查 =============
    // 不在 GUIDED 模式不发位置，避免跟其它模式打架
    if (vehicle->get_mode() != CHOREO_VEHICLE_MODE_GUIDED) {
        _state = State::IDLE;
        _started = false;     // 重置 START 横幅标志，再次进 GUIDED 会重发
        return;
    }

    // ============= (2) 高度检查 =============
    // 飞机得爬到 MIN_ALT 以上才开演（安全门槛，免得飞机刚起飞就被拉地）
    // NED 坐标系：z 朝下为正，-z 就是上方向高度
    Vector3f pos_ned;
    if (!AP::ahrs().get_relative_position_NED_home(pos_ned)) {
        // EKF 还没出位置，等
        return;
    }
    if (-pos_ned.z < _min_alt.get()) {
        _state = State::WAIT_ALT;
        return;
    }

    // ============= (3) 武装检测 =============
    _check_arm();
    if (_arm_utc_usec == 0 && _arm_millis == 0) {
        // 还没扣扳机
        _state = State::WAIT_ALT;
        return;
    }

    // ============= (4) 算 elapsed =============
    // _elapsed_s() 自动选 UTC 或 millis 路径
    // 返负 = 还在倒计时
    const float elapsed_s = _elapsed_s();
    if (elapsed_s < 0) {
        _state = State::ARMED_WAIT;
        return;
    }

    // ============= (5) 第一次进 RUNNING 发横幅 =============
    if (!_started) {
        gcs().send_text(MAV_SEVERITY_INFO,
            "AP_Choreo: START cycle=%.1fs base=%.1fm wps=%u",
            (double)_cycle.get(), (double)_base_alt.get(), (unsigned)_num_wps);
        _started = true;
    }
    _state = State::RUNNING;

    // ============= (6) 算时间相位 t_norm ∈ [0, 1] =============
    // cycle <= 0 时用默认 10s 防除零
    const float cycle_s = (_cycle.get() > 0.1f) ? _cycle.get() : 10.0f;
    float t_norm = elapsed_s / cycle_s;
    if (_loop.get()) {
        // LOOP 模式：mod 1 永远循环
        t_norm -= floorf(t_norm);              // wrap 到 [0,1)
    } else if (t_norm > 1.0f) {
        // ONESHOT 模式：跑完停在最后一帧
        t_norm = 1.0f;
    }

    // ============= (7) 采样得到目标位置 + 颜色 =============
    AbsPos abs;
    uint8_t R = 0, G = 0, B = 0;
    if (!_sample_at_t_norm(t_norm, abs, R, G, B)) {
        // 采样失败（_num_wps==0 或 _to_abs 返 false 等）
        return;
    }

    // ============= (8) 下发绝对 Location =============
    // Location 类：ArduPilot 标准位置表示，含 lat/lng/alt + alt_frame
    Location target {};
    target.lat = abs.lat_1e7;
    target.lng = abs.lng_1e7;
    // 高度：m → cm，并用 ABOVE_HOME 帧（让飞控自动处理"相对本机 HOME"）
    // base_alt 是全局基线补偿，加在每帧 z 上
    target.set_alt_cm(int32_t((abs.up_m + _base_alt.get()) * 100.0f),
                      Location::AltFrame::ABOVE_HOME);

    // 把目标交给车辆，由 ModeGuided 内部处理实际跟踪 PID/速度规划
    vehicle->set_target_location(target);

    // ============= (9) LED 驱动 + 5Hz 回报 GCS =============
    const uint32_t now_ms = AP_HAL::millis();
    _drive_led(R, G, B);                  // 实时驱动硬件 LED
    _report_led(now_ms, R, G, B);          // 5Hz 推 NAMED_VALUE_FLOAT 给 GCS
}

// ----------------------------------------------------------------------------
// 命名空间访问器：其它库通过 AP::choreo() 拿单例
// ----------------------------------------------------------------------------
namespace AP {
AP_Choreo* choreo() { return AP_Choreo::get_singleton(); }
}

#endif // AP_CHOREO_ENABLED
