#!/usr/bin/env python3
import os
import socket
import sys
import traceback

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, os.pardir))
sys.path.insert(0, SCRIPT_DIR)

from _live_controller_adapter import LiveMPCController


HOST = "127.0.0.1"
PORT = 5555


def parse_state_line(line):
    """
    Expected line:
    sim_time,nv0_acc,nv0_speed,nv0_x,nv1_acc,nv1_speed,nv1_x
    """
    parts = [float(x) for x in line.strip().split(",")]

    if len(parts) < 7:
        raise ValueError(f"Expected 7 comma-separated values, got {len(parts)}: {line}")

    sim_time = parts[0]

    nv0_acc = parts[1]
    nv0_speed = parts[2]
    nv0_x = parts[3]

    nv1_acc = parts[4]
    nv1_speed = parts[5]
    nv1_x = parts[6]

    veh_states_matrix = [
        [0.0, nv0_acc, nv0_speed, nv0_x, 0.0],
        [0.0, nv1_acc, nv1_speed, nv1_x, 0.0],
    ]

    return sim_time, veh_states_matrix


def main():
    controller = LiveMPCController(verbose=False)

    print(f"Live MPC controller server listening on {HOST}:{PORT}", flush=True)

    request_count = 0

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((HOST, PORT))
        server.listen(16)

        while True:
            conn, addr = server.accept()

            with conn:
                try:
                    data = conn.recv(4096).decode("utf-8").strip()

                    if not data:
                        continue

                    sim_time, veh_states_matrix = parse_state_line(data)

                    acc_cmd, debug = controller.compute_accel(
                        veh_states_matrix=veh_states_matrix,
                        sim_time=sim_time,
                        vehicle_id="nv1",
                    )

                    response = f"{float(acc_cmd):.9f}\n"
                    conn.sendall(response.encode("utf-8"))

                    request_count += 1

                    if request_count % 25 == 0:
                        nv0 = veh_states_matrix[0]
                        nv1 = veh_states_matrix[1]
                        print(
                            f"[{request_count}] t={sim_time:.2f} "
                            f"nv0: x={nv0[3]:.2f}, v={nv0[2]:.2f} | "
                            f"nv1: x={nv1[3]:.2f}, v={nv1[2]:.2f} | "
                            f"acc_cmd={float(acc_cmd):.3f}",
                            flush=True,
                        )

                except Exception:
                    traceback.print_exc()
                    conn.sendall(b"0.0\n")


if __name__ == "__main__":
    main()
