#! /usr/bin/env python3
"""
For same machine tests.
This is the dmummy "vehicle" side recver/sender to test udp comms.


Local UDP peer for sumo2V_v5_nv1_demo.py in x2v_constants.SAMEMACHINE mode.

The SUMO demo:
  - sends SIM_ARRAY_SIZE float32 values to TARGET_IP:TX_UDP_PORT
  - listens for VEH_ARRAY_SIZE float32 values on RSPC_IPV4:RX_UDP_PORT

This dummy script binds the local TX_UDP_PORT, receives those sim messages, and
replies to RX_UDP_PORT with a simple vehicle-state array.

Run from this folder:
  python3 dummy_samemachine_vehicle.py

Then run the SUMO demo in another terminal:
  python3 sumo2V_v5_nv1_demo.py

In x2v_constants.py, the test mode string should be exactly:
  _test_type = "SAMEMACHINE"
"""

import argparse
import os
import socket
import struct
import sys
import time

SCRIPTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

from x2v_constants import (
    BYTE_SIZE,
    MESSAGE_BYTE_LENGTH,
    RSPC_IPV4,
    SIM_ARRAY_SIZE,
    TARGET_IP,
    TX_UDP_PORT,
    RX_UDP_PORT,
    VEH_ARRAY_SIZE,
)


SIM_FRAME_BYTES = SIM_ARRAY_SIZE * BYTE_SIZE
VEH_FRAME_BYTES = MESSAGE_BYTE_LENGTH


def parse_args():
    parser = argparse.ArgumentParser(
        description="Dummy same-machine UDP vehicle peer for sumo2V_v5_nv1_demo.py"
    )
    parser.add_argument("--listen-ip", default=TARGET_IP,
                        help=f"IP to bind for receiving sim data (default: {TARGET_IP})")
    parser.add_argument("--listen-port", type=int, default=TX_UDP_PORT,
                        help=f"UDP port to bind for receiving sim data (default: {TX_UDP_PORT})")
    parser.add_argument("--reply-ip", default=RSPC_IPV4,
                        help=f"IP where the SUMO demo receives vehicle data (default: {RSPC_IPV4})")
    parser.add_argument("--reply-port", type=int, default=RX_UDP_PORT,
                        help=f"UDP port where the SUMO demo receives vehicle data (default: {RX_UDP_PORT})")
    parser.add_argument("--acc-cmd", type=float, default=0.0,
                        help="dummy acceleration/MPC command placed at realCavArray[7]")
    parser.add_argument("--gps-y", type=float, default=0.0,
                        help="dummy y coordinate placed at realCavArray[5]")
    parser.add_argument("--gps-x-offset", type=float, default=0.0,
                        help="offset added to ego lane position before placing it at realCavArray[4]")
    parser.add_argument("--verbose-every", type=int, default=10,
                        help="print one status line every N received packets; use 0 to disable")
    return parser.parse_args()


def unpack_sim_payload(frame):
    if len(frame) < SIM_FRAME_BYTES:
        return None
    return struct.unpack(f"<{SIM_ARRAY_SIZE}f", frame[:SIM_FRAME_BYTES])


def make_vehicle_payload(sim_array, acc_cmd, gps_y, gps_x_offset):
    sim_time = sim_array[0]
    ego_s = sim_array[1]
    ego_v = sim_array[2]
    ego_a = sim_array[3]
    front_s = sim_array[4]
    front_v = sim_array[5]
    front_a = sim_array[6]

    veh_array = [0.0] * VEH_ARRAY_SIZE

    # Field layout consumed by sumo2V_v5_nv1_demo.py:
    # [0] vehicle/sim timestamp, [2] ego speed, [4:6] x/y, [6] elapsed time,
    # [7] acceleration/MPC command.
    veh_array[0] = sim_time
    veh_array[1] = ego_s
    veh_array[2] = ego_v
    veh_array[3] = ego_a
    veh_array[4] = ego_s + gps_x_offset
    veh_array[5] = gps_y
    veh_array[6] = sim_time
    veh_array[7] = acc_cmd

    # Not used by the current demo, but useful when printing/debugging packets.
    if VEH_ARRAY_SIZE > 10:
        veh_array[8] = front_s
        veh_array[9] = front_v
        veh_array[10] = front_a

    return struct.pack(f"<{VEH_ARRAY_SIZE}f", *veh_array), veh_array


def main():
    args = parse_args()
    reply_addr = (args.reply_ip, args.reply_port)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.listen_ip, args.listen_port))

    print(f"[dummy] listening for SUMO sim data on {sock.getsockname()}")
    print(f"[dummy] replying with {VEH_FRAME_BYTES} byte vehicle frames to {reply_addr}")
    print("[dummy] Ctrl-C to stop")

    recv_count = 0
    send_count = 0
    start = time.monotonic()

    try:
        while True:
            frame, sender = sock.recvfrom(65535)
            recv_count += 1

            sim_array = unpack_sim_payload(frame)
            if sim_array is None:
                print(f"[dummy] short frame from {sender}: {len(frame)} bytes, "
                      f"expected at least {SIM_FRAME_BYTES}")
                continue

            payload, veh_array = make_vehicle_payload(
                sim_array,
                acc_cmd=args.acc_cmd,
                gps_y=args.gps_y,
                gps_x_offset=args.gps_x_offset,
            )
            sock.sendto(payload, reply_addr)
            send_count += 1

            if args.verbose_every > 0 and recv_count % args.verbose_every == 0:
                elapsed = time.monotonic() - start
                print(f"[dummy] recv={recv_count} sent={send_count} "
                      f"sim_t={sim_array[0]:.2f} ego_s={sim_array[1]:.2f} "
                      f"ego_v={sim_array[2]:.2f} acc_cmd={veh_array[7]:.2f} "
                      f"rate={recv_count / max(elapsed, 1e-6):.1f} Hz")

    except KeyboardInterrupt:
        print("\n[dummy] stopped")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
