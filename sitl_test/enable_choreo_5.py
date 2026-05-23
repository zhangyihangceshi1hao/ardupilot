#!/usr/bin/env python3
"""
enable_choreo_5.py
  Bulk-set CHOREO_ENABLE=1 on all 5 SITL via the MAVProxy fan-out on port
  24550 (carries all 5 vehicles). Then prompts user to restart the SITLs
  so AP_Choreo::init() re-runs and loads /APM/choreo.csv.
"""
from __future__ import annotations
import sys
import time
from pymavlink import mavutil

# One connection is enough -- MAVProxy fan-out (24550) carries all 5 vehicles.
# We target each by setting target_system=sysid before PARAM_SET.
PORT = 24550
SYSIDS = [1, 2, 3, 4, 5]


def main() -> int:
    c = mavutil.mavlink_connection(f"udpin:127.0.0.1:{PORT}", source_system=200)
    print(f"[..] waiting for heartbeats on udpin:{PORT} ...")
    seen = set()
    t_end = time.time() + 10
    while time.time() < t_end and len(seen) < 5:
        m = c.recv_match(type="HEARTBEAT", blocking=False, timeout=0.1)
        if m is not None:
            seen.add(m.get_srcSystem())
    print(f"  seen sysids: {sorted(seen)}")
    if not seen:
        print("[ERR] no heartbeats received -- is MAVProxy running and connected?")
        return 1

    for sysid in SYSIDS:
        print(f"[..] PARAM_SET CHOREO_ENABLE=1 -> sysid={sysid}")
        c.mav.param_set_send(
            sysid, 1,            # target_system, target_component
            b"CHOREO_ENABLE",
            1.0,
            mavutil.mavlink.MAV_PARAM_TYPE_INT8,
        )
        # Wait for ack
        t_end = time.time() + 3
        confirmed = False
        while time.time() < t_end:
            m = c.recv_match(type="PARAM_VALUE", blocking=False, timeout=0.1)
            if m is None:
                continue
            name = m.param_id.strip("\x00")
            if m.get_srcSystem() == sysid and name == "CHOREO_ENABLE":
                print(f"     sysid={sysid} CHOREO_ENABLE confirmed = {m.param_value}")
                confirmed = True
                break
        if not confirmed:
            print(f"     sysid={sysid} WARN: no ack received")

    print("")
    print("Done. Now run:")
    print("  bash /home/zyh/ardupilot/sitl_test/restart_5_sitl.sh")
    print("to let AP_Choreo::init() re-run and load the CSV.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
