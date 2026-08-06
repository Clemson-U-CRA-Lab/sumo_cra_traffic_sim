#! /usr/bin/env python3
"""
Send fixed-size random UDP datagrams to the same RSU/ACME port that sumo2V_v5_nv1_demo.py sends to.

Default target:
  x2v_constants.TARGET_IP:x2v_constants.TX_UDP_PORT

Examples:
  python3 rsu_udp_random_load.py --clients 8 --pps 10 --duration 60
  python3 rsu_udp_random_load.py --clients 20 --pps 50 --packet-bytes 284
  python3 rsu_udp_random_load.py --ports 9002 9005 --clients 10 --pps 20

Use only on the controlled RSU/C-V2X testbed.
"""

import argparse
import os
import random
import socket
import struct
import threading
import time

import x2v_constants as xc


DEFAULT_TARGET_IP = getattr(xc, "TARGET_IP", getattr(xc, "RSU_IPV4", "127.0.0.1"))
DEFAULT_TARGET_PORT = int(getattr(xc, "TX_UDP_PORT", getattr(xc, "SERVER_PORT", 9002)))
DEFAULT_BIND_IP = getattr(xc, "RSPC_IPV4", "")
DEFAULT_PACKET_BYTES = int(getattr(xc, "SIM_ARRAY_SIZE", 71)) * int(getattr(xc, "BYTE_SIZE", 4))
DEFAULT_PACKET_BYTES = 100*5


PAYLOAD_MARKER = b"SIMLOAD\x00"  # 8-byte marker to distinguish from usual sim payload
STOP = threading.Event()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Multi-client fixed-size UDP random load sender for RSU/ACME forwarding"
    )
    parser.add_argument("--target-ip", default=DEFAULT_TARGET_IP,
                        help=f"RSU/ACME destination IP (default: {DEFAULT_TARGET_IP})")
    parser.add_argument("--ports", type=int, nargs="+", default=[DEFAULT_TARGET_PORT],
                        help=f"destination UDP port(s), clients are split round-robin (default: {DEFAULT_TARGET_PORT})")
    parser.add_argument("--bind-ip", default=DEFAULT_BIND_IP,
                        help=f"local source IP to bind, empty string means OS default (default: {DEFAULT_BIND_IP!r})")
    parser.add_argument("--clients", type=int, default=15,
                        help="number of UDP client threads/sockets")
    parser.add_argument("--packet-bytes", type=int, default=DEFAULT_PACKET_BYTES,
                        help=f"fixed UDP payload size in bytes (default: {DEFAULT_PACKET_BYTES}, matching SIM_ARRAY_SIZE)")
    parser.add_argument("--pps", type=float, default=10.0,
                        help="packets per second per client; use 0 for best-effort flood")
    parser.add_argument("--duration", type=float, default=80.0,
                        help="seconds to run; use <=0 to run until Ctrl-C")
    parser.add_argument("--payload-format", choices=["floats", "bytes"], default="floats",
                        help="floats packs valid little-endian float32 values; bytes uses os.urandom")
    parser.add_argument("--first-float", type=float, default=-1.0,
                        help="marker for float payloads in element 0")
    parser.add_argument("--seed", type=int, default=None,
                        help="optional base seed for repeatable float payloads")
    parser.add_argument("--sendbuf-bytes", type=int, default=1 << 20,
                        help="SO_SNDBUF size for each UDP client socket")
    return parser.parse_args()


def validate_args(args):
    if args.clients < 1:
        raise ValueError("--clients must be >= 1")
    if args.packet_bytes < 1:
        raise ValueError("--packet-bytes must be >= 1")
    if args.pps < 0:
        raise ValueError("--pps must be >= 0")
    if args.payload_format == "floats" and args.packet_bytes % 4 != 0:
        raise ValueError("--packet-bytes must be divisible by 4 when --payload-format floats")
    if not args.ports:
        raise ValueError("--ports must contain at least one UDP port")
    for port in args.ports:
        if port < 1 or port > 65535:
            raise ValueError(f"invalid UDP port: {port}")


def make_socket(bind_ip, sendbuf_bytes):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, int(sendbuf_bytes))
    if bind_ip:
        sock.bind((bind_ip, 0))
    return sock


def make_payload(packet_bytes, payload_format, rng, first_float):
    # Start with marker, fill rest with random data
    marker_len = len(PAYLOAD_MARKER)
    remaining_bytes = packet_bytes - marker_len
    
    if remaining_bytes < 0:
        raise ValueError(f"packet_bytes ({packet_bytes}) must be >= marker length ({marker_len})")
    
    if payload_format == "bytes":
        return PAYLOAD_MARKER + os.urandom(remaining_bytes)

    # For floats: marker + random floats in remaining space
    float_count = remaining_bytes // 4
    values = [rng.uniform(0.0, 100.0) for _ in range(float_count)]
    if values:
        values[0] = first_float
    float_data = struct.pack(f"<{float_count}f", *values)
    # Pad to exact packet size if needed
    float_data += os.urandom(remaining_bytes - len(float_data))
    return PAYLOAD_MARKER + float_data


def client_worker(client_id, args, counters):
    port = args.ports[client_id % len(args.ports)]
    target = (args.target_ip, port)
    seed = None if args.seed is None else args.seed + client_id
    rng = random.Random(seed)
    sock = make_socket(args.bind_ip, args.sendbuf_bytes)
    interval_s = 0.0 if args.pps == 0 else 1.0 / args.pps
    next_send = time.monotonic()
    packets = 0
    bytes_sent = 0

    try:
        while not STOP.is_set():
            payload = make_payload(args.packet_bytes, args.payload_format, rng, args.first_float)
            sock.sendto(payload, target)
            packets += 1
            bytes_sent += len(payload)

            if interval_s > 0.0:
                next_send += interval_s
                sleep_s = next_send - time.monotonic()
                if sleep_s > 0:
                    time.sleep(sleep_s)
                else:
                    next_send = time.monotonic()
    except OSError as exc:
        print(f"[client {client_id}] UDP send error to {target}: {exc}")
        STOP.set()
    finally:
        counters[client_id] = (packets, bytes_sent, port, sock.getsockname())
        sock.close()


def wait_for_stop(duration_s):
    start = time.monotonic()
    try:
        while not STOP.is_set():
            if duration_s > 0 and time.monotonic() - start >= duration_s:
                break
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        STOP.set()


def main():
    args = parse_args()
    validate_args(args)

    total_pps = "flood" if args.pps == 0 else f"{args.clients * args.pps:.1f}"
    print(f"[i] Target RSU/ACME UDP: {args.target_ip}:{args.ports}")
    print(f"[i] Local bind IP: {args.bind_ip!r}; clients={args.clients}; "
          f"packet_bytes={args.packet_bytes}; payload_format={args.payload_format}")
    print(f"[i] Per-client pps={args.pps}; aggregate pps={total_pps}; duration={args.duration}s")

    counters = {}
    threads = []
    for client_id in range(args.clients):
        thread = threading.Thread(
            target=client_worker,
            args=(client_id, args, counters),
            daemon=True,
        )
        thread.start()
        threads.append(thread)
        time.sleep(0.01)

    wait_for_stop(args.duration)
    for thread in threads:
        thread.join(timeout=1.0)

    total_packets = sum(item[0] for item in counters.values())
    total_bytes = sum(item[1] for item in counters.values())
    print(f"[i] Sent {total_packets} packets / {total_bytes} bytes")
    for client_id in sorted(counters):
        packets, bytes_sent, port, local_addr = counters[client_id]
        print(f"[i] client={client_id} local={local_addr} -> {args.target_ip}:{port} "
              f"packets={packets} bytes={bytes_sent}")


if __name__ == "__main__":
    main()
