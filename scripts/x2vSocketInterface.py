#! /usr/bin/env python3
'''
Socket interface class to connect with cohda

Ideally, instantiate this object in your main node to send/receive socket info. 
This way you dont need to deal with writing socket syntax all the time.

Prakhar Gupta
'''
import time
import socket
import struct
from x2v_constants import *
import threading
from utils import bcolors

# TCP Socket Setup
SERVER_IP = 'fe80::6e5:48ff:fe30:0820'  # RSU IP address (modify as necessary)
SERVER_PORT = 7002  # Server port
TIMEOUT = 5  # Timeout

MESSAGE_BYTE_LENGTH = BYTE_SIZE*VEH_ARRAY_SIZE

# same machine testing only
# SERVER_IP = 'localhost'
# SERVER_PORT = 7005

class x2vSocketInterface:
    def __init__(self, ip=SERVER_IP, port=SERVER_PORT, timeout=TIMEOUT, recv_bytes=MESSAGE_BYTE_LENGTH):
        print(f"Initializing socket interface to IP:{SERVER_IP}, port:{SERVER_PORT}")
        self.ip = ip
        self.port = port
        self.server_address = (ip, port)
        self.timeout = timeout
        self.recvd_msg_bytes = recv_bytes
        self.client_socket = self.wait_for_server()

    def wait_for_server(self):
        print("SOCKET: Starting to attempt connection and wait for RSU server.. ")
        count = 0
        while count <= 12:
            try:
                client_socket = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
                client_socket.connect((self.ip, self.port, 0, 2))
                print(f"{bcolors.OKBLUE}Connected to server (RSU)!{bcolors.ENDC}")
                return client_socket
            except (ConnectionRefusedError, OSError):
                print(f"Server not available. Retrying for {count}th time in {self.timeout} seconds...")
                time.sleep(self.timeout)
                count += 1
        return None

    def send_sim_info(self, sim_array):
        # sim_array is a python list
        numbers_to_send = sim_array
        # print("Sent to RSU: ", numbers_to_send)
        message = struct.pack(f'<{SIM_ARRAY_SIZE}f', *numbers_to_send)
        self.client_socket.sendto(message, self.server_address)

    def recv_veh_info(self):
        data = self.client_socket.recv(self.recvd_msg_bytes)
        veh_array = struct.unpack(f'<{VEH_ARRAY_SIZE}f', data)
        # print("Received from RSU: ", veh_array)
        return veh_array



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
        
        # Store the latest received data
        self.latest_veh_data = None
        self.data_lock = threading.Lock()

        # Start receiving data in a background thread
        self.recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.recv_thread.start()

    def wait_for_server(self):
        print("SOCKET: Starting to attempt connection and wait for server..")
        count = 0
        while count <= 12:
            try:
                client_socket = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
                client_socket.connect((self.ip, self.port, 0, 2))
                print(f"{bcolors.OKBLUE}Connected to server (RSU)!{bcolors.ENDC}")
                return client_socket
            except (ConnectionRefusedError, OSError):
                print(f"Server not available. Retrying for {count}th time in {self.timeout} seconds...")
                time.sleep(self.timeout)
                count += 1
        return None

    def send_sim_info(self, sim_array):
        message = struct.pack(f'<{SIM_ARRAY_SIZE}f', *sim_array)
        self.client_socket.sendto(message, self.server_address)

    def _recv_loop(self):
        """ Continuously receives data and updates the latest vehicle state. """
        while True:
            try:
                data = self.client_socket.recv(self.recvd_msg_bytes)
                if data:
                    veh_array = struct.unpack(f'<{VEH_ARRAY_SIZE}f', data)
                    with self.data_lock:
                        self.latest_veh_data = veh_array  # Store latest received data
            except socket.error as e:
                print(f"Socket receive error: {e}")
                break

    def recv_veh_info(self):
        """ Returns the latest received vehicle data without waiting. """
        with self.data_lock:
            return self.latest_veh_data  # Return last received data immediately
