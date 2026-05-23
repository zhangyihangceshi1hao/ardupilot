#!/usr/bin/env bash
# ============================================================================
#  start_5_mavproxy.sh  --  Launch one MAVProxy attached to all 5 SITL
# ----------------------------------------------------------------------------
#  Uses --master x5 to attach to all 5 UAVs over the UDP fan-out ports the
#  SITL instances open. Inside MAVProxy, use `vehicle <n>` (1..5) to switch
#  the currently controlled UAV.
#
#  Note: `--master udp:127.0.0.1:14550` here means MAVProxy *receives* on
#  that port (the SITL `--out udp:...` is the upstream sender).
# ============================================================================
set -euo pipefail

python3 /home/zyh/.local/bin/mavproxy.py \
    --master=udpin:127.0.0.1:14550 \
    --master=udpin:127.0.0.1:14551 \
    --master=udpin:127.0.0.1:14552 \
    --master=udpin:127.0.0.1:14553 \
    --master=udpin:127.0.0.1:14554 \
    --console \
    --map
