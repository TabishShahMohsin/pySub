# base_udp_test.py
import socket
import time

BASE_IP = "0.0.0.0"
BASE_PORT = 5005

"""
Use socket.gethostbyname('auv.local') or auv.local in terminal
Use ifconfig
!!! TURN OFF WIFI FOR ENSURING CONNECTION OVER ETHERNET NOT SAME WIFI
"""

PI_IP = socket.gethostbyname("auv.local")  # <-- Pi IP
PI_PORT = 5006

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((BASE_IP, BASE_PORT))
sock.settimeout(1.0)

print("BASE: UDP test started")

while True:
    try:
        send_time = time.time()
        sock.sendto(str(send_time).encode(), (PI_IP, PI_PORT))

        data, addr = sock.recvfrom(1024)
        recv_time = time.time()
        pi_time = float(data.decode())

        rtt = recv_time - send_time

        print(
            f"BASE: sent {send_time:.6f} | pi replied {pi_time:.6f} | RTT = {rtt * 1000:.2f} ms",
            end="\r",
            flush=True,
        )

        time.sleep(0.5)

    except socket.timeout:
        print("BASE: no response", end="\r", flush=True)
    except KeyboardInterrupt:
        break
