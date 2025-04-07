import socket
import threading

SERVER_IP = "::"         # Listen on all IPv6 interfaces
SERVER_PORT = 7005
BUFFER_SIZE = 1024       # Intended size (for simulating overflow)
MAX_CONNECTIONS = 100000    # Max queue for incoming connections

def handle_client(conn, addr):
    print(f"[+] New connection from [{addr[0]}]:{addr[1]}")

    try:
        while True:
            data = conn.recv(BUFFER_SIZE)  # Unsafe: client may send more
            if not data:
                break
            print(f"[>] Received {len(data)} bytes: {data[:32]}{'...' if len(data) > 32 else ''}")
    except Exception as e:
        print(f"[!] Error with client {addr}: {e}")
    finally:
        conn.close()
        print(f"[-] Connection closed: {addr}")

def main():
    print(f"[i] Starting vulnerable server on port {SERVER_PORT} (IPv6)...")

    server_sock = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind((SERVER_IP, SERVER_PORT))
    server_sock.listen(MAX_CONNECTIONS)

    print("[i] Server is listening for connections...")

    try:
        while True:
            conn, addr = server_sock.accept()
            client_thread = threading.Thread(target=handle_client, args=(conn, addr))
            client_thread.daemon = True
            client_thread.start()
    except KeyboardInterrupt:
        print("\n[i] Server shutting down.")
    finally:
        server_sock.close()

if __name__ == "__main__":
    main()
