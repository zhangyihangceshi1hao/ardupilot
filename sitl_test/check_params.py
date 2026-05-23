#!/usr/bin/env python3
"""check_params.py -- Dump all CHOREO_* params for each of the 5 SITLs."""
from __future__ import annotations
import sys
import time
import threading
from pymavlink import mavutil

TARGETS = [(1,5760),(2,5770),(3,5780),(4,5790),(5,5800)]

def dump(sid: int, port: int) -> None:
    c = mavutil.mavlink_connection(f"tcp:127.0.0.1:{port}")
    c.wait_heartbeat(timeout=10)
    # Request all CHOREO_* via PARAM_REQUEST_LIST then filter
    c.mav.param_request_list_send(c.target_system, c.target_component)
    t_end = time.time() + 8
    found = {}
    while time.time() < t_end:
        m = c.recv_match(type="PARAM_VALUE", blocking=False)
        if m is None:
            time.sleep(0.005)
            continue
        name = m.param_id.strip("\x00")
        if name.startswith("CHOREO_"):
            found[name] = m.param_value
    print(f"\n--- UAV{sid} (tcp:{port}) ---")
    if not found:
        print("  (no CHOREO_* params found!)")
    for k in sorted(found):
        print(f"  {k:18s} = {found[k]}")

threads = [threading.Thread(target=dump, args=(s,p), daemon=True) for s,p in TARGETS]
for t in threads: t.start()
for t in threads: t.join(timeout=20)
