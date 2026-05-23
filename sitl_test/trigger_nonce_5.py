#!/usr/bin/env python3
"""
trigger_nonce_5.py
  Drive all 5 SITLs through the AP_Choreo workflow:
    1. Connect to MAVProxy fan-out (port 24550 carries all 5 vehicles)
    2. For each sysid: set GUIDED mode, arm, takeoff to 15m
    3. Wait for altitude
    4. SIMULTANEOUSLY write CHOREO_NONCE=1 to all 5 (T_trigger)
    5. Listen 30s for AP_Choreo banners + NAMED_VALUE_FLOAT(LED_RGB)
    6. Report:
         - which UAVs printed "armed nonce=1 ... UTC source" / "millis source"
         - delta-T between first-armed and last-armed banner
         - LED_RGB packed value samples across UAVs in the same window
"""
from __future__ import annotations
import sys
import time
import threading
from pymavlink import mavutil

PORT = 24550   # MAVProxy fan-out (all 5 vehicles)
SYSIDS = [1, 2, 3, 4, 5]
TAKEOFF_ALT_M = 15.0
PRE_NONCE_LISTEN_S = 5
POST_NONCE_LISTEN_S = 25
ARM_WAIT_S = 3.0
TAKEOFF_WAIT_S = 25.0


banners: dict[int, list[tuple[float, str]]] = {sid: [] for sid in SYSIDS}
led_rgb: dict[int, list[tuple[float, float]]] = {sid: [] for sid in SYSIDS}
lock = threading.Lock()


def consumer(c, deadline: float) -> None:
    while time.time() < deadline:
        m = c.recv_match(blocking=False)
        if m is None:
            time.sleep(0.002)
            continue
        t = time.time()
        sid = m.get_srcSystem()
        if sid not in SYSIDS:
            continue
        mt = m.get_type()
        if mt == "STATUSTEXT":
            with lock:
                banners[sid].append((t, m.text))
            if any(k in m.text for k in ("Choreo", "choreo", "loaded", "armed", "START")):
                print(f"[{t:.3f}] UAV{sid} [BANNER] {m.text}")
        elif mt == "NAMED_VALUE_FLOAT":
            if m.name.strip("\x00") == "LED_RGB":
                with lock:
                    led_rgb[sid].append((t, m.value))


def main() -> int:
    print(f"=== connecting to MAVProxy fan-out udp:{PORT} ===")
    c = mavutil.mavlink_connection(f"udpin:127.0.0.1:{PORT}", source_system=210)
    seen = set()
    t_end = time.time() + 10
    while time.time() < t_end and len(seen) < 5:
        m = c.recv_match(type="HEARTBEAT", blocking=False, timeout=0.1)
        if m is not None:
            seen.add(m.get_srcSystem())
    print(f"  heartbeats from sysids: {sorted(seen)}")
    if len(seen) < 5:
        print("[WARN] not all 5 sysids seen; continuing anyway")

    # Request streams on each sysid (so STATUSTEXT actually flows)
    print("=== requesting data streams from all 5 ===")
    for sid in SYSIDS:
        for stream in (
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA3,
        ):
            c.mav.request_data_stream_send(sid, 1, stream, 10, 1)

    # Start consumer thread
    deadline = time.time() + 200
    t = threading.Thread(target=consumer, args=(c, deadline), daemon=True)
    t.start()

    print(f"\n=== passively listening {PRE_NONCE_LISTEN_S}s ===")
    time.sleep(PRE_NONCE_LISTEN_S)

    print("\n=== set GUIDED mode on all 5 ===")
    for sid in SYSIDS:
        c.mav.set_mode_send(
            sid,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4,  # GUIDED
        )
    time.sleep(1.5)

    print("\n=== arm all 5 ===")
    for sid in SYSIDS:
        c.mav.command_long_send(
            sid, 1,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0,
        )
    time.sleep(ARM_WAIT_S)

    print(f"\n=== takeoff to {TAKEOFF_ALT_M}m on all 5 ===")
    for sid in SYSIDS:
        c.mav.command_long_send(
            sid, 1,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, TAKEOFF_ALT_M,
        )
    print(f"    waiting {TAKEOFF_WAIT_S}s for altitude ...")
    time.sleep(TAKEOFF_WAIT_S)

    t_trigger = time.time()
    print(f"\n=== T_TRIGGER={t_trigger:.6f}  writing CHOREO_NONCE=1 to all 5 ===")
    for sid in SYSIDS:
        c.mav.param_set_send(
            sid, 1,
            b"CHOREO_NONCE",
            1.0,
            mavutil.mavlink.MAV_PARAM_TYPE_INT32,
        )

    print(f"\n=== listening {POST_NONCE_LISTEN_S}s for armed banners + LED_RGB ===")
    time.sleep(POST_NONCE_LISTEN_S)

    print("\n========== final report ==========")
    armed_times: dict[int, float] = {}
    sources: dict[int, str] = {}
    for sid in SYSIDS:
        with lock:
            msgs = list(banners[sid])
            leds = list(led_rgb[sid])

        choreo_msgs = [(t, m) for (t, m) in msgs
                       if any(k in m for k in ("Choreo", "choreo", "loaded", "armed", "START"))]
        print(f"\n--- UAV{sid} ---  status_total={len(msgs)}  choreo_banners={len(choreo_msgs)}  led_samples={len(leds)}")
        for t, text in choreo_msgs:
            print(f"  [BANNER] {t:.3f}: {text}")
            if "armed nonce" in text:
                armed_times[sid] = t
                if "UTC source" in text:
                    sources[sid] = "UTC"
                elif "millis source" in text:
                    sources[sid] = "millis"

    if len(armed_times) >= 2:
        first = min(armed_times.values())
        last = max(armed_times.values())
        spread_ms = (last - first) * 1000.0
        print(f"\n  ARMED time spread across UAVs: {spread_ms:.1f} ms")
        print(f"  trigger -> first arm: {(first - t_trigger)*1000:.1f} ms")
        print(f"  trigger -> last arm:  {(last - t_trigger)*1000:.1f} ms")
        print(f"  time sources: {sources}")
    else:
        print(f"\n  WARN: only {len(armed_times)} UAV(s) armed")

    sample_t = t_trigger + 12.0
    print(f"\n  LED samples within +/- 0.25s of t={sample_t:.3f}:")
    for sid in SYSIDS:
        with lock:
            window = [(t, v) for (t, v) in led_rgb[sid] if abs(t - sample_t) < 0.25]
        if window:
            avg = sum(v for _, v in window) / len(window)
            print(f"    UAV{sid}: n={len(window)} avg_packed_rgb={avg:.0f}")
        else:
            print(f"    UAV{sid}: no samples in window")
    return 0


if __name__ == "__main__":
    sys.exit(main())
