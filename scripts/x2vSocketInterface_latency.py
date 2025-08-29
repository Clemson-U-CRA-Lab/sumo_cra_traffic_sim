#! /usr/bin/env python3
"""
Latency-aware x2v socket interface with the SAME structure as the periodic version.

Changes vs periodic:
- Adds a bounded FIFO of inbound VEH frames (vehData_queue).
- Recv loop peels EXACTLY ONE full frame per tick and sleeps to match RECV_RATE_HZ
  so kernel/userland backlog becomes visible (queue grows under congestion).
- Keeps public API and names intact: get_veh_info() still returns the latest frame.
- Adds get_veh_info_next() (FIFO pop) and get_stats() (queue_len, jitter).

Everything else (class name, thread usage, send cadence, attribute names) matches the periodic file.
"""

import time
import socket
import struct
import threading
from collections import deque

from x2v_constants import *
from utils import bcolors

# Receive cadence (Hz). Defaulting to COMMS_FREQ keeps behavior intuitive.
RECV_RATE_HZ = COMMS_FREQ
FIFO_MAX = 50  # cap to avoid unbounded growth under heavy DoS

class x2vSocketInterfaceAsync:
    '''
    x2vSocketInterfaceAsync class helps setup a socket connection that doesnt keep the whole code waiting to recv info.
    This runs a recv from socket function on a separate thread and updates the buffers.

    Same public surface as the periodic version:
      - latest_veh_data, latest_sim_data, data_lock, simData_lock
      - get_veh_info() returns the freshest frame (backwards-compatible)
      - send_sim_info(sim_array)
    Added (non-breaking):
      - vehData_queue (FIFO of received frames)
      - get_veh_info_next(timeout=None): pop oldest frame to *see* delay
      - get_stats(): {"rx_frames", "queue_len", "jitter_s"}
    '''
    def __init__(self, ip=SERVER_IP, port=SERVER_PORT, timeout=TIMEOUT, recv_bytes=MESSAGE_BYTE_LENGTH):
        print(f"Initializing socket interface to IP:{ip}, port:{port}")
        self.ip = ip
        self.port = port
        self.server_address = (ip, port)
        self.timeout = timeout
        self.recvd_msg_bytes = recv_bytes
        self.client_socket = self.wait_for_server()

        # Verbosity
        self.verbose = True

        # Store the latest received vehicle data (legacy behavior)
        self.latest_veh_data = None
        self.data_lock = threading.Lock()

        # Also keep a FIFO so we can expose backlog/delay explicitly
        self._queue_cv = threading.Condition(self.data_lock)
        self.vehData_queue = deque(maxlen=FIFO_MAX)

        # Track inter-arrival jitter (EWMA of inter-arrival times)
        self._rx_frames = 0
        self._last_arrival = None
        self._ia_jitter = None

        # Outbound sim data (periodic sender)
        self.latest_sim_data = None
        self.simData_lock = threading.Lock()

        # Start background threads (same shape as periodic)
        # Use the latency-aware recv loop (manual framing + rate limit)
        self.recv_thread = threading.Thread(target=self._recv_loop1, daemon=True)
        self.send_thread = threading.Thread(target=self._send_loop, daemon=True)

        self.recv_thread.start()
        self.send_thread.start()

    # ---------- Public API (unchanged methods) ----------
    def wait_for_server(self):
        print("SOCKET: Starting to attempt connection and wait for server..")
        count = 0
        while count <= 12:
            try:
                client_socket = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
                client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
                client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)
                client_socket.connect((self.ip, self.port, 0, INTERFACE_SCOPE_ID))
                print(f"{bcolors.OKBLUE}Connected to server (RSU)!{bcolors.ENDC}")
                return client_socket
            except (ConnectionRefusedError, OSError):
                print(f"Server not available. Retrying for {count}th time in {self.timeout} seconds...")
                time.sleep(self.timeout)
                count += 1
        return None

    def send_sim_info(self, sim_array):
        message = struct.pack(f'<{SIM_ARRAY_SIZE}f', *sim_array)
        self.client_socket.sendall(message)
        if self.verbose:
            print(f"------>RSPC sent to RSU->OBU: SimTime {sim_array[0]:.2f}")

    def get_veh_info(self):
        """Backwards-compatible: returns the freshest frame immediately."""
        with self.data_lock:
            return self.latest_veh_data

    # ---------- New (non-breaking) helpers to SEE latency ----------
    def get_veh_info_next(self, timeout=None):
        """
        Pop the OLDEST frame (FIFO) so you observe backlog/delay.
        timeout=None blocks until a frame arrives.
        """
        with self._queue_cv:
            if timeout is None:
                while not self.vehData_queue:
                    self._queue_cv.wait()
            else:
                end = time.monotonic() + timeout
                while not self.vehData_queue:
                    remaining = end - time.monotonic()
                    if remaining <= 0:
                        return None
                    self._queue_cv.wait(remaining)
            return self.vehData_queue.popleft()

    def get_stats(self):
        """Inspect backlog/jitter without changing behavior elsewhere."""
        with self.data_lock:
            return {
                "rx_frames": self._rx_frames,
                "queue_len": len(self.vehData_queue),
                "jitter_s": self._ia_jitter
            }

    # ---------- Internals ----------
    def _recv_loop1(self):
        '''
        Latency-aware manual framing.
        IMPORTANT DIFFERENCE from periodic:
          - We peel EXACTLY ONE frame per tick (RECV_RATE_HZ), then sleep.
          - Remaining bytes stay buffered in userland, making queue/backlog visible.
        '''
        buf = bytearray()
        frame_len = self.recvd_msg_bytes
        period = 1.0 / max(1, int(RECV_RATE_HZ))
        next_deadline = time.monotonic()
        try:
            while True:
                # Pull some bytes (do NOT block forever on giant recvall)
                chunk = self.client_socket.recv(4096)
                if not chunk:
                    # peer closed
                    break
                buf.extend(chunk)

                # Peel AT MOST ONE full frame this tick
                if len(buf) >= frame_len:
                    frame = bytes(buf[:frame_len])
                    del buf[:frame_len]
                    try:
                        veh_array = struct.unpack(f'<{VEH_ARRAY_SIZE}f', frame)
                    except struct.error as e:
                        if self.verbose:
                            print(f"Unpack error (VEH frame): {e}")
                        # drop this frame and continue
                        next_deadline = time.monotonic()  # avoid drift on error
                        continue

                    now = time.monotonic()
                    with self._queue_cv:
                        # Latest-for-free (keeps old code working)
                        self.latest_veh_data = veh_array
                        # Also queue the frame so delay can be observed
                        self.vehData_queue.append(veh_array)
                        # Stats
                        self._rx_frames += 1
                        if self._last_arrival is not None:
                            ia = now - self._last_arrival
                            self._ia_jitter = ia if self._ia_jitter is None else (0.9 * self._ia_jitter + 0.1 * ia)
                        self._last_arrival = now
                        self._queue_cv.notify()

                    if self.verbose and (self._rx_frames % 50 == 0):
                        print(f"<------RSPC recvd from RSU<-OBU: SimTime {veh_array[0]:.2f} "
                              f"(q={len(self.vehData_queue)} jitter~{(self._ia_jitter or 0):.3f}s)")

                # Rate-limit to expose congestion rather than drain to latest-only
                next_deadline += period
                sleep = next_deadline - time.monotonic()
                if sleep > 0:
                    time.sleep(sleep)
                else:
                    # if we've fallen behind, reset the schedule to avoid spiral
                    next_deadline = time.monotonic()
        except socket.error as e:
            print(f"Socket receive error: {e}")

    def _send_loop(self):
        next_deadline = time.monotonic()
        period = 1.0 / max(1, int(COMMS_FREQ))
        while True:
            with self.simData_lock:
                arr = self.latest_sim_data
            if arr is not None:
                try:
                    self.send_sim_info(arr)
                except Exception as e:
                    print(f"send error: {e}")
                    break

            next_deadline += period
            sleep = next_deadline - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_deadline = time.monotonic()
