"""UDP round-trip test, Pi side: echo each probe back to the base station.

Run udp_server_base_station.py on the base station at the same time.
Turn Wi-Fi off to make sure the test runs over Ethernet.
"""

import os
import socket
import time

BIND_IP = "0.0.0.0"
PI_PORT = 5006
BASE_HOST = os.environ.get("ROV_BASE_HOST", "mba.local")
BASE_PORT = 5005


def main():
    base_ip = socket.gethostbyname(BASE_HOST)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((BIND_IP, PI_PORT))
    sock.settimeout(1.0)
    print(f"PI: UDP test started, replying to {BASE_HOST} ({base_ip})")

    try:
        while True:
            try:
                data, addr = sock.recvfrom(1024)
            except TimeoutError:
                continue
            # Probe is "seq,send_time"; reply with "seq,send_time,pi_time"
            reply = f"{data.decode()},{time.time()}".encode()
            sock.sendto(reply, (base_ip, BASE_PORT))
            print(f"PI: echoed probe {data.decode().split(',')[0]} from {addr[0]}", end="\r", flush=True)
    except KeyboardInterrupt:
        print("\nPI: stopped.")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
