#!/bin/bash
# ============================================================================
#  start_5_sitl_stable.sh —— 稳定起 5 架 SITL（用本仓库 ardupilot，含 AP_Choreo）
# ----------------------------------------------------------------------------
#  关键：
#    1. 强制 cd 到 /home/zyh/ardupilot（含 AP_Choreo 的那个 fork，不是 study_ardupilot）
#    2. 每架间隔 5 秒启动（避免并发抢 SHM / 端口）
#    3. 每架包 tmux 独立 session，方便 attach 观察
#    4. UAV1..5 用 --sysid 1..5 + 不同 --instance + 不同 --use-dir
#    5. 端口分配:
#         UAV1 - udp:127.0.0.1:14550
#         UAV2 - udp:127.0.0.1:14551
#         ...
#         UAV5 - udp:127.0.0.1:14554
# ============================================================================
set -e

ARDUPILOT_DIR=/home/zyh/ardupilot
CSV_TEMPLATE=${1:-/home/zyh/ardupilot/sitl_test/choreo.csv}

cd "$ARDUPILOT_DIR"

# 先杀掉所有残留进程
echo "==== 清场 ===="
pkill -f arducopter 2>/dev/null || true
pkill -f sim_vehicle.py 2>/dev/null || true
pkill -f mavproxy.py 2>/dev/null || true
tmux kill-server 2>/dev/null || true
sleep 2

# 确认 AP_Choreo 在
if ! strings "$ARDUPILOT_DIR/build/sitl/bin/arducopter" | grep -q "AP_Choreo"; then
    echo "❌ $ARDUPILOT_DIR/build/sitl/bin/arducopter 没有 AP_Choreo，需要重编"
    echo "    cd $ARDUPILOT_DIR && ./waf copter"
    exit 1
fi
echo "✓ arducopter 含 AP_Choreo 模块"

# 给每架准备工作目录 + 灌 CSV
echo ""
echo "==== 准备 5 架工作目录 + CSV ===="
for i in 1 2 3 4 5; do
    DIR=/tmp/sim/uav${i}
    mkdir -p "$DIR/APM"
    # 把 CSV 拷过去（用户后续可以通过 GCS MAVFTP 覆盖）
    if [ -f "$CSV_TEMPLATE" ]; then
        cp "$CSV_TEMPLATE" "$DIR/APM/choreo.csv"
    fi
    echo "  $DIR/  ($(test -f $DIR/APM/choreo.csv && echo 'CSV 就位' || echo '⚠ 无 CSV'))"
done

# 逐架启动（间隔 5 秒避免竞争）
echo ""
echo "==== 启动 5 架 SITL (tmux 包装，间隔 5s) ===="
for i in 1 2 3 4 5; do
    INST=$((i - 1))
    SYSID=$i
    PORT=$((14549 + i))
    DIR=/tmp/sim/uav${i}
    SESSION=uav${i}

    echo "  [$i] sysid=$SYSID instance=$INST out=udp:127.0.0.1:$PORT use-dir=$DIR"

    # tmux new-session detached
    tmux new-session -d -s "$SESSION" "
        cd $ARDUPILOT_DIR && \
        ./Tools/autotest/sim_vehicle.py \
            -v ArduCopter \
            -I $INST \
            --sysid $SYSID \
            --location=CMAC \
            --out=udp:127.0.0.1:$PORT \
            --use-dir=$DIR \
            --no-mavproxy \
            --no-rebuild \
            2>&1 | tee /tmp/sim/uav${i}.log
    "
    sleep 5
done

echo ""
echo "==== 验证启动结果 ===="
sleep 3
echo "活着的 arducopter 进程:"
pgrep -af arducopter || echo "  ⚠ 无 arducopter 进程"
echo ""
echo "tmux sessions:"
tmux ls 2>/dev/null || echo "  无 tmux 会话"
echo ""
echo "UDP 端口监听 (应该 5 个 SITL 都在主动推):"
ss -uln | grep -E ':145(5[0-4])' || echo "  ⚠ 端口未占"

echo ""
echo "==== 下一步 ===="
echo "1. 等 30-60 秒让 5 架 EKF 收敛 + GPS 锁"
echo "2. 看 tmux capture: tmux capture-pane -t uav3 -p | tail"
echo "3. 期望每架最终出 'APM: ArduCopter ready' + 'AP_Choreo: loaded 50 wps for sysid=N'"
echo "4. GCS 端 ⚡ 默认 5 架 → 14550-14554 直连"
