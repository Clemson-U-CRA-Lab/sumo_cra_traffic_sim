#! /usr/bin/env python3
'''
UDP socket interface with periodic send and optional send/receive delay.

This matches x2vSocketInterface_Udp_periodic.py, but the send loop can delay
outgoing SIM frames by setting self.send_delay_sec, and the receive loop can optionally
delay vehicle frames by setting self.recv_delay_sec.
'''
import socket
import struct
import threading
import time
from collections import deque

from x2v_constants import *


class x2vSocketInterfaceUdpAsync:
    def __init__(
            self,
            ip=TARGET_IP,
            tx_port=TX_UDP_PORT,
            rx_port=RX_UDP_PORT,
            timeout=TIMEOUT,
            recv_bytes=MESSAGE_BYTE_LENGTH):
        print(f"Initializing delayed UDP socket interface to IP:{ip}, tx_port:{tx_port}, rx_port:{rx_port}")
        self.ip = ip
        self.send_port = tx_port
        self.recv_port = rx_port
        self.server_address = (ip, self.send_port)

        self.timeout = timeout
        self.recvd_msg_bytes = recv_bytes
        self.send_socket, self.recv_socket = self.setup_udp_sockets()

        self.verbose = False
        self.send_delay_sec = 0.0
        self.recv_delay_sec = 0.0
        self._send_delay_queue = deque()
        self._recv_delay_queue = deque()
        self._current_payload = None

        self.latest_veh_data = None
        self.data_lock = threading.Lock()
        self.latest_sim_data = None
        self.simData_lock = threading.Lock()

        self.recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.send_thread = threading.Thread(target=self._send_loop_with_delay, daemon=True)
        self.recv_thread.start()
        self.send_thread.start()

    def setup_udp_sockets(self):
        send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        send_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 5)

        recv_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        recv_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 5)

        recv_socket.bind((RSPC_IPV4, self.recv_port))
        send_socket.bind((RSPC_IPV4, 0))

        print(f"UDP sockets ready: TX->{self.ip}:{self.send_port}, RX<-*:{self.recv_port}")
        print(f"UDP sockets bound to {recv_socket.getsockname()} for receiving; sending from {send_socket.getsockname()}")
        return send_socket, recv_socket

    def send_sim_info(self, sim_array):
        message = struct.pack(f'<{SIM_ARRAY_SIZE}f', *sim_array)
        self.send_socket.sendto(message, self.server_address)
        if self.verbose:
            print(f"------>RSPC sent to RSU->OBU: SimTime {sim_array[0]:.2f}")

    def _publish_veh_data(self, veh_array):
        with self.data_lock:
            self.latest_veh_data = veh_array
        if self.verbose:
            print(f"<------RSPC recvd from RSU<-OBU: SimTime {veh_array[0]:.2f}")


    def _recv_loop(self):
        frame_len = self.recvd_msg_bytes
        try:
            while True:
                frame, _ = self.recv_socket.recvfrom(frame_len)
                if len(frame) < frame_len:
                    if self.verbose:
                        print(f"Socket receive warning: short UDP frame ({len(frame)} bytes)")
                    continue

                veh_array = struct.unpack(f'<{VEH_ARRAY_SIZE}f', frame[:frame_len])
                self._publish_veh_data(veh_array)
        except socket.error as e:
            print(f"Socket receive error: {e}")

    def _recv_loop_with_delay(self):
        frame_len = self.recvd_msg_bytes
        self.recv_socket.settimeout(0.01)
        try:
            while True:
                now = time.monotonic()
                try:
                    frame, _ = self.recv_socket.recvfrom(frame_len)
                    if len(frame) < frame_len:
                        if self.verbose:
                            print(f"Socket receive warning: short UDP frame ({len(frame)} bytes)")
                        continue

                    veh_array = struct.unpack(f'<{VEH_ARRAY_SIZE}f', frame[:frame_len])
                    if self.recv_delay_sec <= 0.0:
                        self._recv_delay_queue.clear()
                        self._publish_veh_data(veh_array)
                    else:
                        self._recv_delay_queue.append((now + self.recv_delay_sec, veh_array))
                except socket.timeout:
                    pass

                matured = None
                while self._recv_delay_queue and self._recv_delay_queue[0][0] <= time.monotonic():
                    _, matured = self._recv_delay_queue.popleft()
                if matured is not None:
                    self._publish_veh_data(matured)
                    if self.verbose and self.recv_delay_sec > 0.0:
                        print(f"    delayed recv by ~{self.recv_delay_sec:.2f}s")
        except socket.error as e:
            print(f"Socket receive error: {e}")

    def get_veh_info(self):
        with self.data_lock:
            return self.latest_veh_data

    def _send_loop_with_delay(self):
        period = 1.0 / max(1, int(COMMS_FREQ))
        next_deadline = time.monotonic()

        while True:
            with self.simData_lock:
                snapshot = self.latest_sim_data

            now = time.monotonic()
            if snapshot is not None:
                if self.send_delay_sec <= 0.0:
                    self._send_delay_queue.clear()
                    self._current_payload = snapshot
                else:
                    self._send_delay_queue.append((now + self.send_delay_sec, snapshot))
                    while self._send_delay_queue and self._send_delay_queue[0][0] <= now:
                        _, self._current_payload = self._send_delay_queue.popleft()

            if self._current_payload is not None:
                try:
                    self.send_sim_info(self._current_payload)
                    if self.verbose and self.send_delay_sec > 0.0:
                        print(f"    delayed send by ~{self.send_delay_sec:.2f}s")
                except Exception as e:
                    print(f"send error: {e}")
                    break

            next_deadline += period
            sleep = next_deadline - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_deadline = time.monotonic()


x2vSocketInterfaceAsync = x2vSocketInterfaceUdpAsync
