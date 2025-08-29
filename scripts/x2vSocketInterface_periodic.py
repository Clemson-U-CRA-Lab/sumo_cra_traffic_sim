#! /usr/bin/env python3
'''
Socket interface class to connect with cohda

Ideally, instantiate this object in your main node to send/receive socket info. 
This way you dont need to deal with writing socket syntax all the time.

Now added periodicity and tcp messag eframing by using threaded send and recv loops in async verison
This helps to extraxt 1 full message.
And helps to send at fixed rate for repeatability.

Prakhar Gupta
'''
import time
import socket
import struct
from x2v_constants import *
import threading
from utils import bcolors

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

        self.vehData_queue = []  # Queue to hold received vehicle data

        # Start receiving data in a background thread
        # self.recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.recv_thread = threading.Thread(target=self._recv_loop1, daemon=True)
        self.send_thread = threading.Thread(target=self._send_loop, daemon=True)

        self.recv_thread.start()
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

    def send_sim_info(self, sim_array):
        message = struct.pack(f'<{SIM_ARRAY_SIZE}f', *sim_array)
        # self.client_socket.sendto(message, self.server_address)
        self.client_socket.sendall(message)
        if self.verbose:
            print(f"------>RSPC sent to RSU->OBU: SimTime {sim_array[0]:.2f}")
        # print("-----> RSPC Sent to RSU->OBU: ", len(message), time.time())

    def _recv_loop(self):
        """ Continuously receives data and updates the latest vehicle state. """
        while True:
            try:
                # get whatever you have
                # data = self.client_socket.recv(self.recvd_msg_bytes)
                # get only when its 1 full message of expected length
                data = self.recvall(self.recvd_msg_bytes) # wait to get full frame manually.
                if data:
                    veh_array = struct.unpack(f'<{VEH_ARRAY_SIZE}f', data)
                    print("Recvd : ",  time.time())

                    with self.data_lock:
                        self.latest_veh_data = veh_array  # Store latest received data
            except socket.error as e:
                print(f"Socket receive error: {e}")
                break

    def recvall(self, n):
        """Receive exactly n bytes from a TCP socket."""
        buf = bytearray()
        while len(buf) < n:
            chunk = self.client_socket.recv(n - len(buf))
            if not chunk:
                return None  # connection closed
            buf.extend(chunk)
        return bytes(buf)
    

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

    def _send_loop(self):
        next_deadline = time.monotonic()
        while True:
            # grab the freshest array
            with self.simData_lock:
                arr = self.latest_sim_data
            if arr is not None:
                try:
                    self.send_sim_info(arr)  # non-blocking from your updated sendall path
                    
                except Exception as e:
                    print(f"send error: {e}")
                    break

            next_deadline += 1/COMMS_FREQ
            sleep = next_deadline - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
