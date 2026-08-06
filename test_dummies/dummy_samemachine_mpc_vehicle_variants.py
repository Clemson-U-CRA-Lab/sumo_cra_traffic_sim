#! /usr/bin/env python3
"""
Same-machine UDP dummy vehicle that solves the PCC MPC.

Combines regular (packet-driven) and fallback (timed loop + delay preview
shift) variants. Toggle USE_FALLBACK in x2v_constants.py.

Run from this folder:
  python3 dummy_samemachine_mpc_vehicle_variants.py

Then run the SUMO side in another terminal.
"""

import argparse
import os
import socket
import struct
import sys
import time

import numpy as np

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SCRIPTS_DIR = os.path.join(REPO_ROOT, "scripts")
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

from _agents import PCC
from x2v_constants import (
    BOOL_TEST_WITHOUT_GPS,
    BOOL_USE_FRONT_PREVIEW,
    BYTE_SIZE,
    DELAY_THRESHOLD,
    FALLBACK_MODE,
    MESSAGE_BYTE_LENGTH,
    MPC_DT,
    REF_CYCLE_DT,
    REF_CYCLE_STAGES,
    RSPC_IPV4,
    SIM_ARRAY_SIZE,
    TARGET_IP,
    TX_UDP_PORT,
    RX_UDP_PORT,
    USE_FALLBACK,
    VEH_ARRAY_SIZE,
)


CONTROL_FREQ = 20.0

SIM_FRAME_BYTES = SIM_ARRAY_SIZE * BYTE_SIZE
VEH_FRAME_BYTES = MESSAGE_BYTE_LENGTH
TAG = "[mpc-fb-dummy]" if USE_FALLBACK else "[mpc-dummy]"


def parse_args():
    desc = ("Same-machine UDP dummy vehicle that solves PCC MPC"
            + (" with delay fallback" if USE_FALLBACK else ""))
    parser = argparse.ArgumentParser(description=desc)
    parser.add_argument("--listen-ip", default=TARGET_IP,
                        help=f"IP to bind for receiving sim data (default: {TARGET_IP})")
    parser.add_argument("--listen-port", type=int, default=TX_UDP_PORT,
                        help=f"UDP port to bind for receiving sim data (default: {TX_UDP_PORT})")
    parser.add_argument("--reply-ip", default=RSPC_IPV4,
                        help=f"IP where the SUMO side receives vehicle data (default: {RSPC_IPV4})")
    parser.add_argument("--reply-port", type=int, default=RX_UDP_PORT,
                        help=f"UDP port where the SUMO side receives vehicle data (default: {RX_UDP_PORT})")
    parser.add_argument("--verbose-every", type=int, default=10,
                        help="print one status line every N ticks/packets; use 0 to disable")
    return parser.parse_args()


def unpack_sim_payload(frame):
    if len(frame) < SIM_FRAME_BYTES:
        return None
    return struct.unpack(f"<{SIM_ARRAY_SIZE}f", frame[:SIM_FRAME_BYTES])


def reuse_siminfo(sim_array, delay_amount, fallback_mode, ref_cycle_stages=REF_CYCLE_STAGES,
                  mpc_dt=MPC_DT):
    """Shift and extrapolate stale front-vehicle preview, matching pcc_mpc_fallback_node."""
    # shift: how many MPC_DT increments have been stale.
    shift = round(delay_amount / mpc_dt)

    sim = list(sim_array)
    pv_s = sim[7 + 1 + shift]
    pv_spd = sim[7 + ref_cycle_stages + 1 + shift]
    pv_acc = sim[6]
    front_pred_s = np.roll(np.asarray(sim[7:7 + ref_cycle_stages], dtype=float), -shift)
    front_pred_v = np.roll(
        np.asarray(sim[7 + ref_cycle_stages:7 + 2 * ref_cycle_stages], dtype=float),
        -shift,
    )

    if fallback_mode == "stop":
        # assume stop after horizon
        for i in range(1, shift + 1):
            front_pred_s[-i] = front_pred_s[ref_cycle_stages - shift - 1]
            front_pred_v[-i] = 0.0 * front_pred_v[ref_cycle_stages - shift - 1]
    elif fallback_mode == "carry":
        # assume carry the last velocity after horizon
        for i in range(1, shift + 1):
            front_pred_v[-i] = front_pred_v[ref_cycle_stages - shift - 1]
        for i in range(shift, 0, -1):
            front_pred_s[ref_cycle_stages - i] = (
                front_pred_s[ref_cycle_stages - i - 1]
                + front_pred_v[ref_cycle_stages - i - 1] * mpc_dt
            )
    elif fallback_mode == "carryAcc":
        # assume carry the last acceleration after horizon
        for i in range(shift, 0, -1):
            front_pred_v[ref_cycle_stages - i] = max(
                0.0,
                front_pred_v[ref_cycle_stages - i - 1] + pv_acc * mpc_dt,
            )
            front_pred_s[ref_cycle_stages - i] = (
                front_pred_s[ref_cycle_stages - i - 1]
                + front_pred_v[ref_cycle_stages - i - 1] * mpc_dt
            )
    else:
        raise ValueError(f"unknown fallback mode: {fallback_mode}")

    return pv_s, pv_spd, pv_acc, front_pred_s, front_pred_v, shift


def extract_pv_and_preview(sim_array):
    pv_s = sim_array[4]
    pv_v = sim_array[5]
    pv_a = sim_array[6]
    front_pred_s = np.asarray(sim_array[7:7 + REF_CYCLE_STAGES], dtype=float)
    front_pred_v = np.asarray(
        sim_array[7 + REF_CYCLE_STAGES:7 + 2 * REF_CYCLE_STAGES],
        dtype=float,
    )
    return pv_s, pv_v, pv_a, front_pred_s, front_pred_v


def solve_mpc(controller, sim_t, ego_s, ego_v, ego_a, pv_s, pv_v, pv_a,
              front_pred_s, front_pred_v):
    ego_pred_s, ego_pred_v, acc, pred_t = controller.setCommand_SUMO(
        t=sim_t,
        ego_s=ego_s,
        ego_v=ego_v,
        ego_a=ego_a,
        pv_s=pv_s,
        pv_v=pv_v,
        pv_a=pv_a,
        cycle_ss=front_pred_s,
        cycle_vs=front_pred_v,
        cycle_dt=REF_CYCLE_DT,
        n_refs=REF_CYCLE_STAGES,
        preview=BOOL_USE_FRONT_PREVIEW,
    )
    return ego_pred_s, ego_pred_v, acc, pred_t


def solve_mpc_from_sim(controller, sim_array):
    """Regular path: same unpack + solve as dummy_samemachine_mpc_vehicle.solve_mpc."""
    sim_t = sim_array[0]
    ego_s = sim_array[1]
    ego_v = sim_array[2]
    ego_a = sim_array[3]
    pv_s = sim_array[4]
    pv_v = sim_array[5]
    pv_a = sim_array[6]
    front_pred_s = sim_array[7:7 + REF_CYCLE_STAGES]
    front_pred_v = sim_array[7 + REF_CYCLE_STAGES:7 + 2 * REF_CYCLE_STAGES]
    return solve_mpc(
        controller, sim_t, ego_s, ego_v, ego_a, pv_s, pv_v, pv_a,
        front_pred_s, front_pred_v,
    )


def make_vehicle_payload(sim_array, acc_cmd, pred_s, pred_v):
    sim_t = sim_array[0]
    ego_s = sim_array[1]
    ego_v = sim_array[2]
    ego_a = sim_array[3]
    veh_array = [0.0] * VEH_ARRAY_SIZE

    # Field layout consumed by the SUMO V2X scripts:
    # [0] sim timestamp, [1:4] ego state, [4:6] GPS x/y (unused without GPS → 0),
    # [6] MPC command, [7:39] pred_s, [39:71] pred_v.
    veh_array[0] = sim_t
    veh_array[1] = ego_s
    veh_array[2] = ego_v
    veh_array[3] = ego_a
    veh_array[4] = 0.0  # GPS x
    veh_array[5] = 0.0  # GPS y
    veh_array[6] = acc_cmd
    veh_array[7:7 + REF_CYCLE_STAGES] = pred_s[:REF_CYCLE_STAGES]
    veh_array[7 + REF_CYCLE_STAGES:7 + 2 * REF_CYCLE_STAGES] = pred_v[:REF_CYCLE_STAGES]

    return struct.pack(f"<{VEH_ARRAY_SIZE}f", *veh_array), veh_array


def run_regular(args, controller, sock, reply_addr):
    """Packet-driven loop (matches dummy_samemachine_mpc_vehicle.py)."""
    last_acc = 0.0
    pred_s = [0.0] * REF_CYCLE_STAGES
    pred_v = [0.0] * REF_CYCLE_STAGES
    recv_count = 0
    send_count = 0
    error_count = 0
    start = time.monotonic()

    print(f"{TAG} listening for SUMO sim data on {sock.getsockname()}")
    print(f"{TAG} replying with {VEH_FRAME_BYTES} byte vehicle frames to {reply_addr}")
    print(f"{TAG} preview={BOOL_USE_FRONT_PREVIEW}, cycle_dt={REF_CYCLE_DT}, stages={REF_CYCLE_STAGES}")
    print(f"{TAG} Ctrl-C to stop")

    try:
        while True:
            frame, sender = sock.recvfrom(65535)
            recv_count += 1

            sim_array = unpack_sim_payload(frame)
            if sim_array is None:
                print(f"{TAG} short frame from {sender}: {len(frame)} bytes, "
                      f"expected at least {SIM_FRAME_BYTES}")
                continue

            try:
                pred_s, pred_v, last_acc, _ = solve_mpc_from_sim(controller, sim_array)
            except (IndexError, RuntimeError, ValueError) as exc:
                error_count += 1
                print(f"{TAG} MPC solve failed at sim_t={sim_array[0]:.2f}: {exc}")

            payload, veh_array = make_vehicle_payload(
                sim_array, acc_cmd=last_acc, pred_s=pred_s, pred_v=pred_v,
            )
            sock.sendto(payload, reply_addr)
            send_count += 1

            if args.verbose_every > 0 and recv_count % args.verbose_every == 0:
                elapsed = time.monotonic() - start
                print(f"{TAG} recv={recv_count} sent={send_count} errors={error_count} "
                      f"sim_t={sim_array[0]:.2f} ego_s={sim_array[1]:.2f} "
                      f"ego_v={sim_array[2]:.2f} front_s={sim_array[4]:.2f} "
                      f"acc_cmd={veh_array[6]:.3f} rate={recv_count / max(elapsed, 1e-6):.1f} Hz")
    except KeyboardInterrupt:
        print(f"\n{TAG} stopped")


def run_fallback(args, controller, sock, reply_addr):
    """Timed control loop with delay fallback (matches dummy_samemachine_mpc_vehicle_fb.py)."""
    last_acc = 0.0
    pred_s = [0.0] * REF_CYCLE_STAGES
    pred_v = [0.0] * REF_CYCLE_STAGES
    period = 1.0 / CONTROL_FREQ
    sock.settimeout(period)

    print(f"{TAG} listening for SUMO sim data on {sock.getsockname()}")
    print(f"{TAG} replying with {VEH_FRAME_BYTES} byte vehicle frames to {reply_addr}")
    print(f"{TAG} preview={BOOL_USE_FRONT_PREVIEW}, cycle_dt={REF_CYCLE_DT}, "
          f"stages={REF_CYCLE_STAGES}, control_freq={CONTROL_FREQ} Hz")
    print(f"{TAG} fallback_mode={FALLBACK_MODE}, "
          f"delay_threshold={DELAY_THRESHOLD}s, mpc_dt={MPC_DT}s")
    print(f"{TAG} Ctrl-C to stop")

    last_sim_array = None
    recv_count = 0
    send_count = 0
    error_count = 0
    fallback_count = 0
    tick_count = 0
    start = time.monotonic()
    real_start_time = None
    vehicle_elapsed_time_offset = None

    try:
        while True:
            loop_start = time.monotonic()

            try:
                frame, sender = sock.recvfrom(65535)
                recv_count += 1
                sim_array = unpack_sim_payload(frame)
                if sim_array is None:
                    print(f"{TAG} short frame from {sender}: {len(frame)} bytes, "
                          f"expected at least {SIM_FRAME_BYTES}")
                else:
                    last_sim_array = sim_array
                    if real_start_time is None:
                        real_start_time = time.monotonic()
                        vehicle_elapsed_time_offset = sim_array[0]
            except socket.timeout:
                pass

            if last_sim_array is None:
                continue

            tick_count += 1
            sim_t = last_sim_array[0]
            ego_s = last_sim_array[1]
            ego_v = last_sim_array[2]
            ego_a = last_sim_array[3]

            real_now = time.monotonic()
            vehicle_elapsed_time = (
                real_now - real_start_time + vehicle_elapsed_time_offset
            )

            # Delay detection (same criterion as pcc_mpc_fallback_node)
            if (vehicle_elapsed_time - sim_t) > period:
                delay_amount = vehicle_elapsed_time - sim_t
                delay_detected = True
            else:
                delay_amount = 0.0
                delay_detected = False

            pv_s, pv_v, pv_a, front_pred_s, front_pred_v = extract_pv_and_preview(last_sim_array)
            shift = 0
            if delay_detected and delay_amount >= DELAY_THRESHOLD:
                pv_s, pv_v, pv_a, front_pred_s, front_pred_v, shift = reuse_siminfo(
                    last_sim_array, delay_amount, FALLBACK_MODE
                )
                fallback_count += 1

            try:
                pred_s, pred_v, last_acc, _ = solve_mpc(
                    controller,
                    sim_t=sim_t,
                    ego_s=ego_s,
                    ego_v=ego_v,
                    ego_a=ego_a,
                    pv_s=pv_s,
                    pv_v=pv_v,
                    pv_a=pv_a,
                    front_pred_s=front_pred_s,
                    front_pred_v=front_pred_v,
                )
            except (IndexError, RuntimeError, ValueError) as exc:
                error_count += 1
                print(f"{TAG} MPC solve failed at sim_t={sim_t:.2f}: {exc}")

            payload, veh_array = make_vehicle_payload(
                last_sim_array,
                acc_cmd=last_acc,
                pred_s=pred_s,
                pred_v=pred_v,
            )
            sock.sendto(payload, reply_addr)
            send_count += 1

            if args.verbose_every > 0 and tick_count % args.verbose_every == 0:
                elapsed = time.monotonic() - start
                print(f"{TAG} tick={tick_count} recv={recv_count} sent={send_count} "
                      f"errors={error_count} fallbacks={fallback_count} "
                      f"sim_t={sim_t:.2f} veh_t={vehicle_elapsed_time:.2f} "
                      f"delay={delay_amount:.2f} shift={shift} ego_s={ego_s:.2f} "
                      f"ego_v={ego_v:.2f} front_s={pv_s:.2f} "
                      f"acc_cmd={veh_array[6]:.3f} rate={tick_count / max(elapsed, 1e-6):.1f} Hz")

            # Pace the control loop (recv timeout already contributes wait time)
            sleep_for = period - (time.monotonic() - loop_start)
            if sleep_for > 0:
                time.sleep(sleep_for)
    except KeyboardInterrupt:
        print(f"\n{TAG} stopped")


def main():
    args = parse_args()
    if not BOOL_TEST_WITHOUT_GPS:
        print(f"{TAG} warning: BOOL_TEST_WITHOUT_GPS is False, but this dummy uses SUMO ego state.")

    controller = PCC(dirname=SCRIPTS_DIR, s=0.0, v=0.0, a=0.0, v_max=20.0)
    reply_addr = (args.reply_ip, args.reply_port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.listen_ip, args.listen_port))

    try:
        if USE_FALLBACK:
            run_fallback(args, controller, sock, reply_addr)
        else:
            run_regular(args, controller, sock, reply_addr)
    finally:
        sock.close()
        controller.api.cleanup()


if __name__ == "__main__":
    main()
