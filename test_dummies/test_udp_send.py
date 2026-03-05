import socket
import struct

UDP_IP = '192.168.74.200'     # destination IP
UDP_PORT = 9002          # destination port
# UDP_IP = "localhost"     # destination IP

# Example array
data = [1.1, 2.2, 3.3, 4.4]

# Pack floats into bytes
packet = struct.pack(f'{len(data)}f', *data)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock.bind(('192.168.74.170', 0))  # bind to any available port

sock.sendto(packet, (UDP_IP, UDP_PORT))

print("Sent:", data)