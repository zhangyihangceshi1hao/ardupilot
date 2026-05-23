#!/usr/bin/env bash
# Stop all SITL / mavproxy / sim_vehicle processes started for Phase 6.7.
for i in 1 2 3 4 5; do
    tmux kill-session -t "uav${i}" 2>/dev/null || true
done
pkill -9 -f 'arducopter' 2>/dev/null || true
pkill -9 -f 'sim_vehicle' 2>/dev/null || true
pkill -9 -f 'mavproxy'    2>/dev/null || true
sleep 1
echo "[OK] killed. remaining:"
pgrep -fa arducopter || echo "  no arducopter"
pgrep -fa sim_vehicle || echo "  no sim_vehicle"
pgrep -fa mavproxy    || echo "  no mavproxy"
tmux ls 2>/dev/null | grep -E '^uav' || echo "  no uav tmux sessions"
