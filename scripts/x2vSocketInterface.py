#! /usr/bin/env python3

import time
import socket
import struct
from x2v_constants import *

# TCP Socket Setup
SERVER_IP = 'fe80::6e5:48ff:fe30:0820'  # RSU IP address (modify as necessary)
SERVER_PORT = 7002  # Server port
TIMEOUT = 5  # Timeout

class x2vSocketInterface:
    def __init__(self, ip=SERVER_IP, port=SERVER_PORT, timeout=TIMEOUT):
        print(f"Initializing socket interface to IP:{SERVER_IP}, port:{SERVER_PORT}")
        self.ip = ip
        self.port = port
        self.server_address = (ip, port)
        self.timeout = timeout
        self.client_socket = self.wait_for_server()

    def wait_for_server(self):
        print("SOCKET: Starting to attempt connection and wait for server.. ")
        count = 0
        while count <= 12:
            try:
                client_socket = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
                print("HEEEEEEEEEEEE")
                client_socket.connect((self.ip, self.port, 0, 2))
                print("Connected to server (RSU)!")
                return client_socket
            except (ConnectionRefusedError, OSError):
                print(f"Server not available. Retrying for {count}th time in {self.timeout} seconds...")
                time.sleep(self.timeout)
                count += 1
        return None

    def send_sim_info(self, sim_array):
        # sim_array is a python list
        numbers_to_send = sim_array
        print("Sent to RSU: ", numbers_to_send)
        message = struct.pack(f'<{SIM_ARRAY_SIZE}f', *numbers_to_send)
        self.client_socket.sendto(message, self.server_address)

    def recv_veh_info(self):
        data = self.client_socket.recv(16)
        veh_array = struct.unpack(f'<{VEH_ARRAY_SIZE}f', data)
        print("Received from RSU: ", veh_array)
        return veh_array
