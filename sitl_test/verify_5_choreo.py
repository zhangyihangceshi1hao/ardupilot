#!/usr/bin/env python3
"""
verify_5_choreo.py  --  Pull STATUSTEXT from all 5 SITL UDP fan-out ports
                        for 10 seconds and print AP_Choreo-related banners.

Usage:
    python3 verify_5_choreo.py [seconds]

This is a read-only probe; safe to run while MAVProxy is also attached
(MAVProxy listens on the same UDP master ports without "stealing" packets
because UDP master is broadcast-style).

Actually, MAVProxy is also a UDP receiver -- if both bind the same port,
only one gets packets. So either run THIS or MAVProxy, not both.
"""
from __future__ import annotations
import sys
import time
from pymavlink import mavutil

PORTS = [24550, 24551, 24552, 24553, 24554]   # MAVProxy fan-out ports
SECONDS = int(sys.argv[1]) if len(sys.argv) > 1 else 10


def main() -> int:
    # mavutil.mavlink_connection with "udpin:" listens passively; the SITL
    # `--out udp:127.0.0.1:PORT` is a UDP sender pushing to us.
    conns = []
    for p in PORTS:
        try:
            c = mavutil.mavlink_connection(f"udpin:127.0.0.1:{p}")
            conns.append((p, c))
            print(f"[bind] udp:127.0.0.1:{p}")
        except Exception as e:
            print(f"[ERR] failed to bind {p}: {e}")

    t_end = time.time() + SECONDS
    seen_choreo = {p: 0 for p in PORTS}
    seen_statustext = {p: 0 for p in PORTS}
    seen_sysids = {p: set() for p in PORTS}

    print(f"[..] listening for {SECONDS}s on {len(conns)} ports ...")
    while time.time() < t_end:
        for p, c in conns:
            msg = c.recv_match(blocking=False)
            if msg is None:
                continue
            if msg.get_type() == "BAD_DATA":
                continue
            sysid = msg.get_srcSystem()
            seen_sysids[p].add(sysid)
            if msg.get_type() == "STATUSTEXT":
                seen_statustext[p] += 1
                text = msg.text
                if "Choreo" in text or "choreo" in text or "loaded" in text:
                    seen_choreo[p] += 1
                    print(f"[choreo] port={p} sysid={sysid}: {text}")
        time.sleep(0.01)

    print("")
    print("================ summary ================")
    for p in PORTS:
        srcs = sorted(seen_sysids[p])
        print(
            f"  port {p}: src_sysids={srcs} "
            f"STATUSTEXT={seen_statustext[p]} choreo_banners={seen_choreo[p]}"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
