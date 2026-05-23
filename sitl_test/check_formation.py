#!/usr/bin/env python3
"""
check_formation.py
  Sample GLOBAL_POSITION_INT for ~10s; for each pair of UAVs compute the
  distance from the master HOME (LLA -> NED relative to UAV1 HOME) and
  the angular position on the circle. Verify they sit ~72 degrees apart
  on a ~20m circle.

  Note: this must run AFTER trigger_nonce_5.py has the UAVs flying the
  circle. Run while they're mid-cycle.
"""
from __future__ import annotations
import math
import sys
import time
import threading
from pymavlink import mavutil

PORT = 24550
SYSIDS = [1, 2, 3, 4, 5]
SAMPLE_S = 8

pos: dict[int, list[tuple[float, float, float, float]]] = {sid: [] for sid in SYSIDS}
lock = threading.Lock()


def consumer(c, deadline: float) -> None:
    while time.time() < deadline:
        m = c.recv_match(blocking=False)
        if m is None:
            time.sleep(0.002)
            continue
        if m.get_type() != "GLOBAL_POSITION_INT":
            continue
        sid = m.get_srcSystem()
        if sid not in SYSIDS:
            continue
        with lock:
            pos[sid].append((time.time(), m.lat / 1e7, m.lon / 1e7, m.relative_alt / 1000.0))


def main() -> int:
    c = mavutil.mavlink_connection(f"udpin:127.0.0.1:{PORT}", source_system=211)
    print(f"[..] connected udp:{PORT}, requesting position stream ...")
    # Wait for heartbeats
    seen = set()
    t_end = time.time() + 8
    while time.time() < t_end and len(seen) < 5:
        m = c.recv_match(type="HEARTBEAT", blocking=False, timeout=0.1)
        if m is not None:
            seen.add(m.get_srcSystem())

    for sid in SYSIDS:
        c.mav.request_data_stream_send(
            sid, 1, mavutil.mavlink.MAV_DATA_STREAM_POSITION, 5, 1
        )

    deadline = time.time() + SAMPLE_S + 3
    t = threading.Thread(target=consumer, args=(c, deadline), daemon=True)
    t.start()

    print(f"[..] sampling positions for {SAMPLE_S}s ...")
    time.sleep(SAMPLE_S)

    # Use the *most recent* position per UAV
    print("\n=== last position per UAV ===")
    latest = {}
    for sid in SYSIDS:
        with lock:
            samples = pos[sid]
        if not samples:
            print(f"  UAV{sid}: no samples")
            continue
        ts, lat, lon, alt = samples[-1]
        latest[sid] = (lat, lon, alt)
        print(f"  UAV{sid}: lat={lat:.7f} lon={lon:.7f} relalt={alt:.1f}m  n={len(samples)}")

    if len(latest) < 5:
        return 1

    # Compute approximate NED offsets relative to UAV1
    lat0, lon0, _ = latest[1]
    # 1 deg lat ~ 111320m; 1 deg lon at lat ~ 111320*cos(lat)
    mlat = 111320.0
    mlon = 111320.0 * math.cos(math.radians(lat0))

    print("\n=== NED offsets relative to UAV1 ===")
    for sid in SYSIDS:
        lat, lon, alt = latest[sid]
        n = (lat - lat0) * mlat
        e = (lon - lon0) * mlon
        r = math.hypot(n, e)
        ang = (math.degrees(math.atan2(e, n)) + 360) % 360
        print(f"  UAV{sid}: north={n:+7.2f}m east={e:+7.2f}m radius={r:5.2f}m bearing={ang:5.1f}deg")

    # Pair-wise distances
    print("\n=== inter-UAV distances ===")
    for i in SYSIDS:
        for j in SYSIDS:
            if j <= i:
                continue
            li, oi, _ = latest[i]
            lj, oj, _ = latest[j]
            dn = (lj - li) * mlat
            de = (oj - oi) * mlon
            d = math.hypot(dn, de)
            print(f"  UAV{i}-UAV{j}: {d:5.2f} m")
    return 0


if __name__ == "__main__":
    sys.exit(main())
