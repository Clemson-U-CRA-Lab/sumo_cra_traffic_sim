
#! /usr/bin/env python3
'''
Hybrid x2vSocketInterface: behaves like periodic under normal conditions,
but exposes delay under attack by adaptively switching behavior.

Author: Prakhar Gupta + ChatGPT hybrid mode
'''

import time
import socket
import struct
import threading
from collections import deque

from x2v_constants import *

class x2vSocketInterfaceAsync:
    def __init__(self,
                 ip=SERVER_IP,
                 port=SERVER_PORT,
                 timeout=TIMEOUT,
                 recv_bytes=BYTE_SIZE * VEH_ARRAY_SIZE,
                 recv_rate_hz=COMMS_FREQ,
                 fifo_max=256,
                 verbose=True):
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self.recvd_msg_bytes = recv_bytes
        self.recv_period = 1.0 / max(1, int(recv_rate_hz))
        self.verbose = verbose

        self.simData_lock = threading.Lock()
        self.sim_out_latest = None

        self.data_lock = threading.Lock()
        self.data_cv = threading.Condition(self.data_lock)
        self.veh_queue = deque(maxlen=fifo_max)
        self.latest_veh_data = None

        self.rx_frames = 0
        self.last_arrival = None
        self.ia_jitter = None

        self.client_socket = self._wait_for_server()

        self.recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.send_thread = threading.Thread(target=self._send_loop, daemon=True)
        self.recv_thread.start()
        self.send_thread.start()

    def _wait_for_server(self):
        print("[Hybrid] Connecting to RSU...")
        count = 0
        while count <= 12:
            try:
                s = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
                s.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)
                s.connect((self.ip, self.port, 0, INTERFACE_SCOPE_ID))
                print("[Hybrid] Connected to RSU.")
                return s
            except (ConnectionRefusedError, OSError):
                print(f"[Hybrid] Server not available. Retrying ({count}) in {self.timeout}s...")
                time.sleep(self.timeout)
                count += 1
        raise RuntimeError("Unable to connect to RSU")

    def _recv_exact(self, n):
        buf = bytearray()
        while len(buf) < n:
            chunk = self.client_socket.recv(n - len(buf))
            if not chunk:
                return None
            buf.extend(chunk)
        return bytes(buf)

    def _recv_loop(self):
        while True:
            frame_bytes = self._recv_exact(self.recvd_msg_bytes)
            now = time.monotonic()
            if not frame_bytes:
                print("[Hybrid] Peer closed connection.")
                break
            try:
                veh_array = struct.unpack(f'<{VEH_ARRAY_SIZE}f', frame_bytes)
            except struct.error as e:
                print(f"[Hybrid] Unpack error: {e}")
                continue

            with self.data_cv:
                self.latest_veh_data = veh_array
                self.veh_queue.append(veh_array)
                self.rx_frames += 1
                if self.last_arrival is not None:
                    ia = now - self.last_arrival
                    self.ia_jitter = ia if self.ia_jitter is None else (0.9 * self.ia_jitter + 0.1 * ia)
                self.last_arrival = now
                self.data_cv.notify()

            if self.verbose and self.rx_frames % 50 == 0:
                print(f"[Hybrid] RX#{self.rx_frames} | Queue={len(self.veh_queue)} | Jitter={self.ia_jitter:.3f}s")

    def _send_loop(self):
        period = 1.0 / max(1, int(COMMS_FREQ))
        next_deadline = time.monotonic()
        while True:
            with self.simData_lock:
                arr = self.sim_out_latest
            if arr is not None:
                try:
                    msg = struct.pack(f'<{SIM_ARRAY_SIZE}f', *arr)
                    self.client_socket.sendall(msg)
                except Exception as e:
                    print(f"[Hybrid] Send error: {e}")
                    break
            next_deadline += period
            sleep = next_deadline - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_deadline = time.monotonic()

    def queue_sim_info(self, sim_array):
        with self.simData_lock:
            self.sim_out_latest = sim_array

    def get_veh_info(self):
        """Smart adaptive getter: behaves like periodic unless backlog is high."""
        with self.data_cv:
            qlen = len(self.veh_queue)
            delay = self.ia_jitter or 0.0
            if qlen > 50 or delay > 0.15:
                if self.verbose:
                    print(f"[Hybrid] Congestion detected: q={qlen}, jitter={delay:.3f}s → using .popleft()")
                return self.veh_queue.popleft() if self.veh_queue else self.latest_veh_data
            return self.latest_veh_data

    def get_veh_info_next(self, timeout=None):
        with self.data_cv:
            if timeout is None:
                while not self.veh_queue:
                    self.data_cv.wait()
            else:
                end = time.monotonic() + timeout
                while not self.veh_queue:
                    remaining = end - time.monotonic()
                    if remaining <= 0:
                        return None
                    self.data_cv.wait(remaining)
            return self.veh_queue.popleft()

    def get_stats(self):
        with self.data_lock:
            return {
                "rx_frames": self.rx_frames,
                "queue_len": len(self.veh_queue),
                "jitter_s": self.ia_jitter
            }
