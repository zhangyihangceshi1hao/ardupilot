#!/usr/bin/env python3
"""
probe_5_tcp.py  --  Connect to all 5 SITL TCP ports, collect STATUSTEXT for
                    ~10s, print AP_Choreo loaded banners.
"""
from __future__ import annotations
import sys
import time
import threading
from pymavlink import mavutil

# (sysid_expected, tcp_port)
TARGETS = [
    (1, 5760),
    (2, 5770),
    (3, 5780),
    (4, 5790),
    (5, 5800),
]
SECONDS = int(sys.argv[1]) if len(sys.argv) > 1 else 15

results = {sid: [] for sid, _ in TARGETS}
lock = threading.Lock()


def reader(sysid: int, port: int, deadline: float) -> None:
    try:
        conn = mavutil.mavlink_connection(f"tcp:127.0.0.1:{port}")
        print(f"[{sysid}] connected tcp:{port}, waiting heartbeat ...")
        hb = conn.wait_heartbeat(timeout=10)
        if hb is None:
            print(f"[{sysid}] no heartbeat in 10s")
            return
        print(
            f"[{sysid}] heartbeat src_sysid={hb.get_srcSystem()} "
            f"type={hb.type} autopilot={hb.autopilot}"
        )
        while time.time() < deadline:
            m = conn.recv_match(blocking=False)
            if m is None:
                time.sleep(0.01)
                continue
            if m.get_type() == "STATUSTEXT":
                with lock:
                    results[sysid].append(
                        (time.time(), m.get_srcSystem(), m.text)
                    )
    except Exception as e:
        print(f"[{sysid}] tcp:{port} error: {e}")


def main() -> int:
    deadline = time.time() + SECONDS
    threads = []
    for sid, port in TARGETS:
        t = threading.Thread(target=reader, args=(sid, port, deadline), daemon=True)
        t.start()
        threads.append(t)
    for t in threads:
        t.join(timeout=SECONDS + 3)

    print("")
    print("================ STATUSTEXT per UAV ================")
    for sid, _ in TARGETS:
        msgs = results[sid]
        choreo_msgs = [m for m in msgs if "Choreo" in m[2] or "choreo" in m[2]]
        print(f"\n--- UAV{sid} (expected sysid) ---  total={len(msgs)}  choreo={len(choreo_msgs)}")
        for ts, src, text in msgs[:20]:
            tag = "[CHOREO]" if ("Choreo" in text or "choreo" in text) else "        "
            print(f"  {tag} src={src}: {text}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
