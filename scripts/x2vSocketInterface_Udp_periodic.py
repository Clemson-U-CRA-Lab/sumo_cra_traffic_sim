#! /usr/bin/env python3
'''
Socket interface class to connect with cohda

Ideally, instantiate this object in your main node to send/receive socket info. 
This way you dont need to deal with writing socket syntax all the time.

Now added periodicity by using threaded send and recv loops in async verison.
This helps to send and receive at fixed rate for repeatability.

Useds UDP

Prakhar Gupta
'''
import time
import socket
import struct
from x2v_constants import *
import threading

class x2vSocketInterfaceAsync:
    '''
    x2vSocketInterfaceAsync class helps setup a socket connection that doesnt keep the whole code waiting to recv info.
    This runs a recv fom socket function on separate thread and just updates the 'self.latest_veh_data' attribute when it received new info.
    '''
    def __init__(
            self, 
            ip=TARGET_IP, 
            tx_port=TX_UDP_PORT, 
            rx_port=RX_UDP_PORT, 
            timeout=TIMEOUT, 
            recv_bytes=MESSAGE_BYTE_LENGTH):
        print(f"Initializing UDP socket interface to IP:{ip}, tx_port:{tx_port}, rx_port:{rx_port}")
        self.ip = ip
        self.send_port = tx_port
        self.recv_port = rx_port
        self.server_address = (ip, self.send_port)

        self.timeout = timeout
        self.recvd_msg_bytes = recv_bytes
        self.send_socket, self.recv_socket = self.setup_udp_sockets()

        self.verbose = True
        
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

    def setup_udp_sockets(self):
        send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        send_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 5)

        recv_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        recv_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 5)

        # bind appropriately for IPv4
        recv_socket.bind((RSPC_IPV4, self.recv_port))
        send_socket.bind((RSPC_IPV4, 0))

        print(f"UDP sockets ready: TX->{self.ip}:{self.send_port}, RX<-*:{self.recv_port}")
        print(f"UDP Sockets bound to {recv_socket.getsockname()} for receiving; sending from {send_socket.getsockname()}")

        return send_socket, recv_socket

    def send_sim_info(self, sim_array):
        message = struct.pack(f'<{SIM_ARRAY_SIZE}f', *sim_array)
        self.send_socket.sendto(message, self.server_address)
        if self.verbose:
            print(f"------>RSPC sent to RSU->OBU: SimTime {sim_array[0]:.2f}")
            # print("-----> RSPC Sent to RSU->OBU: ", len(message), time.time())

    def _recv_loop1(self):
        """ Continuously receives data and updates the latest vehicle state. """
        frame_len = self.recvd_msg_bytes
        try:
            while True:
                frame, _ = self.recv_socket.recvfrom(frame_len)
                if len(frame) < frame_len:
                    if self.verbose:
                        print(f"Socket receive warning: short UDP frame ({len(frame)} bytes)")
                    continue

                veh_array = struct.unpack(f'<{VEH_ARRAY_SIZE}f', frame[:frame_len])
                with self.data_lock:
                    self.latest_veh_data = veh_array
                if self.verbose:
                    print(f"<------RSPC recvd from RSU<-OBU: SimTime {veh_array[0]:.2f}")

        except socket.error as e:
            print(f"Socket receive error: {e}")
            
    def get_veh_info(self):
        """ Returns the latest received vehicle data without waiting. """
        with self.data_lock:
            return self.latest_veh_data  # Return last received data immediately

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
