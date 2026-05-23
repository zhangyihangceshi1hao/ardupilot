#!/usr/bin/env bash
# ============================================================================
#  start_5_sitl.sh  --  Launch 5 ArduCopter SITL instances for Phase 6.7
# ----------------------------------------------------------------------------
#  We bypass sim_vehicle.py and invoke the arducopter SITL binary directly.
#  Reasons:
#    - sim_vehicle.py defaults to spawning each instance inside its own xterm;
#      this is fragile on WSL2 (DISPLAY race / no DPI) and breaks our use case.
#    - sim_vehicle.py with --no-mavproxy still wraps SITL in `runsim.py` which
#      complicates clean restarts.
#  Direct invocation gives:
#    - one arducopter process per UAV, stdout/stderr captured to logs/
#    - stable TCP MAVLink ports 5760, 5770, 5780, 5790, 5800
#      (each --instance I adds 10*I to the base port)
#    - cwd = /tmp/sim/uav<sysid>/ so AP_Choreo reads ./APM/choreo.csv
#
#  After this runs, attach one MAVProxy (start_5_mavproxy.sh) to all five
#  TCP ports.
# ============================================================================
set -euo pipefail

ARDUPILOT_DIR=/home/zyh/ardupilot
ARDUCOPTER=${ARDUPILOT_DIR}/build/sitl/bin/arducopter
DEFAULTS=${ARDUPILOT_DIR}/Tools/autotest/default_params/copter.parm
CSV_SRC=${ARDUPILOT_DIR}/sitl_test/choreo_5uav.csv
LOG_DIR=/tmp/sim/logs

# CMAC = Canberra Model Aircraft Club, the standard SITL location
HOME_LOC="-35.363261,149.16523,584.0,353.0"

mkdir -p "${LOG_DIR}"

if [[ ! -x "${ARDUCOPTER}" ]]; then
    echo "[ERR] missing SITL binary: ${ARDUCOPTER}"
    exit 1
fi

if [[ ! -f "${CSV_SRC}" ]]; then
    echo "[ERR] choreo CSV missing: ${CSV_SRC}"
    echo "      run: powershell tools/gen_sitl_5uav_csv.ps1 -CopyToSimDirs"
    exit 1
fi

echo "[..] kill any leftover arducopter / mavproxy"
pkill -9 -f 'arducopter' 2>/dev/null || true
pkill -9 -f 'sim_vehicle' 2>/dev/null || true
pkill -9 -f 'mavproxy'    2>/dev/null || true
sleep 1

echo "[..] ensure /tmp/sim/uav{1..5}/APM/choreo.csv ..."
for i in 1 2 3 4 5; do
    mkdir -p "/tmp/sim/uav${i}/APM"
    cp "${CSV_SRC}" "/tmp/sim/uav${i}/APM/choreo.csv"
done

# Each SITL runs inside its own detached tmux session ("uav1"..."uav5") so
# WSL2 doesn't reap it as an orphan once this script returns. tmux survives
# parent shell exit by design (it daemonizes itself). To stop a SITL:
#   tmux kill-session -t uavN
# To peek at a SITL console:
#   tmux attach -t uavN   (Ctrl-B D to detach)
# We pass --serial0 udpclient so SITL pushes MAVLink to MAVProxy (udpin)
# without needing a TCP client; UDP survives MAVProxy restarts.
for i in 0 1 2 3 4; do
    sysid=$((i + 1))
    uavdir=/tmp/sim/uav${sysid}
    logf=${LOG_DIR}/uav${sysid}.log
    udp_port=$((14550 + i))
    session=uav${sysid}

    # kill any prior tmux session of the same name
    tmux kill-session -t "${session}" 2>/dev/null || true

    echo "[..] spawn UAV${sysid} in tmux ${session} -> udp:127.0.0.1:${udp_port}  cwd=${uavdir}"
    # No -S: SITL free-runs wall clock (so we don't need to drive sync).
    # Redirect stdout/err to file inside tmux so we can scrape later.
    tmux new-session -d -s "${session}" -c "${uavdir}" \
        "${ARDUCOPTER} \
            --model + \
            --speedup 1 \
            --sysid ${sysid} \
            -I ${i} \
            --defaults ${DEFAULTS} \
            --home ${HOME_LOC} \
            --serial0 udpclient:127.0.0.1:${udp_port} \
            2>&1 | tee ${logf}"
    sleep 1
done

echo ""
echo "[OK] 5 SITL instances spawning in background"
echo "     logs: ${LOG_DIR}/uav{1..5}.log"
echo "     pids: /tmp/sim/uav{1..5}.pid"
echo ""
echo "[..] waiting 15s for full boot + EKF + GPS lock ..."
sleep 15

echo ""
echo "==================== STATUS ===================="
pgrep -fa arducopter | sed 's/^/  /' || echo "  (no arducopter processes!)"
echo ""
echo "==================== AP_Choreo banners (per log) ===================="
for i in 1 2 3 4 5; do
    echo "--- uav${i} ---"
    grep -E 'Choreo|choreo|loaded' "${LOG_DIR}/uav${i}.log" 2>/dev/null \
        | tail -5 \
        | sed 's/^/  /' || true
done
echo ""
echo "Next: bash ${ARDUPILOT_DIR}/sitl_test/start_5_mavproxy.sh"
