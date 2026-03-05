import socket
import struct

UDP_IP = "0.0.0.0"   # listen on all interfaces
UDP_PORT = 9002

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening on UDP port {UDP_PORT}...")

while True:
    data, addr = sock.recvfrom(1024)

    # each float = 4 bytes
    n = len(data) // 4

    try:
        arr = struct.unpack(f'{n}f', data)
        print(f"Received from {addr}: {arr}")
    except:
        print(f"Received raw bytes from {addr}: {data}")