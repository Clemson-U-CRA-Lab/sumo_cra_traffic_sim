import socket
import struct
import threading
import rospy
from v2x_constants import *
from utils import bcolors

class v2xSocketInterface:
    def __init__(self, ip=SERVER_IP, port=SERVER_PORT, recv_bytes=MESSAGE_BYTE_LENGTH):
        print(f"Initializing socket Server on IP:{ip}, port:{port}")
        self.ip = ip
        self.port = port
        self.server_address = (ip, port)
        self.recvd_msg_bytes = recv_bytes

        self.listen_sock_fd = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
        self.listen_sock_fd.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listen_sock_fd.bind((self.ip, self.port))
        self.listen_sock_fd.listen(CLIENT_QUEUE_LEN)
        rospy.loginfo(f"Listening on {self.ip}:{self.port}")

        self.clients = []  # Store active client sockets
        self.client_data = {}  # Store received data from clients
        self.lock = threading.Lock()

        # Accept clients in a separate thread
        self.accept_thread = threading.Thread(target=self._accept_clients, daemon=True)
        self.accept_thread.start()

    def _accept_clients(self):
        """ Continuously accept multiple client connections. """
        while True:
            client_sock, client_addr = self.listen_sock_fd.accept()
            with self.lock:
                self.clients.append(client_sock)
                self.client_data[client_sock] = None  # Initialize data storage
            rospy.loginfo(f"{bcolors.OKBLUE}New connection from OBU: {client_addr[0]}:{client_addr[1]}{bcolors.ENDC}")
            threading.Thread(target=self._recv_loop, args=(client_sock,), daemon=True).start()

    def send_veh_info(self, vehArray):
        """ Send vehicle data to all connected clients. """
        message = struct.pack(f'<{VEH_ARRAY_SIZE}f', *vehArray)
        with self.lock:
            for client in self.clients:
                try:
                    client.sendall(message)
                except socket.error:
                    self.clients.remove(client)  # Remove disconnected clients

    def _recv_loop(self, client_sock):
        """ Continuously receive data from a client. """
        while True:
            try:
                data = client_sock.recv(self.recvd_msg_bytes)
                if not data:
                    break  # Client disconnected
                simArray = struct.unpack(f'<{SIM_ARRAY_SIZE}f', data)
                with self.lock:
                    self.client_data[client_sock] = simArray  # Update latest received data
            except socket.error:
                break  # Handle disconnection

        # Cleanup after disconnect
        with self.lock:
            self.clients.remove(client_sock)
            del self.client_data[client_sock]
        client_sock.close()

    def recv_sim_info(self):
        """ Get the latest received data from all clients. """
        with self.lock:
            return self.client_data.copy()

    def __del__(self):
        print(f"{bcolors.FAIL}Destructing v2xSocketInterface Object{bcolors.ENDC}")
        with self.lock:
            for client in self.clients:
                client.close()
        self.listen_sock_fd.close()
