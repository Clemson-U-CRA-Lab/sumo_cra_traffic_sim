#! /usr/bin/env python3
'''
Socket interface class to connect with cohda

Ideally, instantiate this object in your main node to send/receive socket info. 
This way you dont need to deal with writing socket syntax all the time.

Now added periodicity and tcp messag eframing by using threaded send and recv loops in async verison
This helps to extraxt 1 full message.
And helps to send at fixed rate for repeatability.

All send delays colntrol too

Prakhar Gupta
'''
import time
import socket
import struct
from x2v_constants import *
import threading
from utils import bcolors
import queue
from collections import deque

class x2vSocketInterfaceAsync:
    '''
    x2vSocketInterfaceAsync class helps setup a socket connection that doesnt keep the whole code waiting to recv info.
    This runs a recv fom socket function on separate thread and just updates the 'self.latest_veh_data' attribute when it received new info.
    '''
    def __init__(self, ip=SERVER_IP, port=SERVER_PORT, timeout=TIMEOUT, recv_bytes=MESSAGE_BYTE_LENGTH):
        print(f"Initializing socket interface to IP:{ip}, port:{port}")
        self.ip = ip
        self.port = port
        self.server_address = (ip, port)
        self.timeout = timeout
        self.recvd_msg_bytes = recv_bytes
        self.client_socket = self.wait_for_server()

        self.verbose = False
        
        # Store the latest received data
        self.latest_veh_data = None
        self.data_lock = threading.Lock()

        self.latest_sim_data = None
        self.simData_lock = threading.Lock()

        # --- Delay line config ---
        self.send_delay_sec = 0.0 # -> no delay injection by default.
        self._delay_queue = deque()    # (release_time_monotonic, sim_array)
        self._current_payload = None   # last matured frame to actually send

        # Start receiving data in a background thread
        # self.recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.recv_thread = threading.Thread(target=self._recv_loop1, daemon=True)
        self.recv_thread.start()

        # For delayed send
        self.sim_buffer = queue.Queue()
        self.send_thread = threading.Thread(target=self._send_loop_with_delay, daemon=True)
        self.send_thread.start()

    def wait_for_server(self):
        print("SOCKET: Starting to attempt connection and wait for server..")
        count = 0
        while count <= 12:
            try:
                client_socket = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
                client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1<<20)
                client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1<<20)    

                client_socket.connect((self.ip, self.port, 0, INTERFACE_SCOPE_ID))
                print(f"{bcolors.OKBLUE}Connected to server (RSU)!{bcolors.ENDC}")
                return client_socket
            except (ConnectionRefusedError, OSError):
                print(f"Server not available. Retrying for {count}th time in {self.timeout} seconds...")
                time.sleep(self.timeout)
                count += 1
        return None


    def _recv_loop1(self):
        '''
        Combines recvall and recvloop into one function  - manual framing.
        It empties the kernel buffer faster if multiple messages are queued.
        Compared to recvall(), this doesnt wait for just 1 full message.
        '''
        """ Continuously receives data and updates the latest vehicle state. """
        buf = bytearray()
        frame_len = self.recvd_msg_bytes  # 68*4 = 272
        try:
            while True:
                chunk = self.client_socket.recv(4096)  # up to 4KB
                if not chunk:
                    break  # peer closed
                buf.extend(chunk)
                # peel complete frames; keep only the freshest (latest-only)
                while len(buf) >= frame_len:
                    frame = bytes(buf[:frame_len])
                    del buf[:frame_len]
                    veh_array = struct.unpack(f'<{VEH_ARRAY_SIZE}f', frame)
                    with self.data_lock:
                        self.latest_veh_data = veh_array
                        # self.vehData_queue.append(veh_array)
                    if self.verbose:
                        print(f"<------RSPC recvd from RSU<-OBU: SimTime {veh_array[0]:.2f}")
        except socket.error as e:
            print(f"Socket receive error: {e}")
            
    def get_veh_info(self):
        """ Returns the latest received vehicle data without waiting. """
        with self.data_lock:
            return self.latest_veh_data  # Return last received data immediately

            # if self.vehData_queue:
            #     return self.vehData_queue.pop(0)   # FIFO
            # else:
            #     return None




    # ---------- Send (Periodic + Delay Line) ----------
    def _send_loop_with_delay(self):
        """
        1) At COMMS_FREQ, snapshot latest_sim_data and enqueue it with a release time now + send_delay_sec.
        2) Promote any matured frames (release_time <= now) to _current_payload.
        3) Send _current_payload (if any) at the same COMMS_FREQ cadence.
        """
        period = 1.0 / max(1, int(COMMS_FREQ))
        next_deadline = time.monotonic()

        # Warm-up: there will be ~send_delay_sec with no payload; that's intentional.
        while True:

            if self.send_delay_sec == 0.0:

                with self.simData_lock:
                    arr = self.latest_sim_data
                if arr is not None:
                    try:
                        msg = struct.pack(f'<{SIM_ARRAY_SIZE}f', *arr)
                        self.client_socket.sendall(msg)
                        if self.verbose:
                            print(f"--> RS-PC TX (no delay) {bcolors.OKBLUE}SimTime={arr[0]:.2f}{bcolors.ENDC}")
                    except Exception as e:
                        print(f"[Send Error] {e}")
                        break
                next_deadline += 1/COMMS_FREQ
                sleep = next_deadline - time.monotonic()
                if sleep > 0:
                    time.sleep(sleep)

            else:
                # this is delayed mode
                now = time.monotonic()
                # print("Sending delayed stuff")

                # Stage a snapshot into the delay line
                with self.simData_lock:
                    snapshot = self.latest_sim_data
                if snapshot is not None:
                    release_time = now + self.send_delay_sec
                    self._delay_queue.append((release_time, snapshot))

                # Promote matured frames (keep the newest matured one)
                while self._delay_queue and self._delay_queue[0][0] <= now:
                    _, matured = self._delay_queue.popleft()
                    self._current_payload = matured

                # Send the currently matured payload (if any)
                if self._current_payload is not None:
                    try:
                        msg = struct.pack(f'<{SIM_ARRAY_SIZE}f', *self._current_payload)
                        self.client_socket.sendall(msg)
                        if self.verbose:
                            print(f"--> RS-PC TX (delayed ~{self.send_delay_sec:.1f}s) "
                                f"{bcolors.OKCYAN}SimTime={self._current_payload[0]:.2f}{bcolors.ENDC}")
                    except Exception as e:
                        print(f"Send error: {e}")
                        break

            # Periodic pacing
            next_deadline += period
            sleep = next_deadline - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                # If we overran, realign without drifting
                next_deadline = time.monotonic()