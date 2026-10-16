# pi_udp_test.py
import socket
import time

# !!! TURN OFF WIFI FOR ENSURING CONNECTION OVER ETHERNET NOT SAME WIFI

PI_IP = "0.0.0.0"
PI_PORT = 5006

# BASE_IP = "10.222.231.148"   # <-- base station IP
BASE_IP = socket.gethostbyname("mba.local")
BASE_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((PI_IP, PI_PORT))
sock.settimeout(1.0)

print("PI: UDP test started")

while True:
    try:
        data, addr = sock.recvfrom(1024)
        recv_time = time.time()
        sent_time = float(data.decode())

        print(f"PI: received {sent_time:.6f} from {addr}")

        reply = str(recv_time).encode()
        sock.sendto(reply, (BASE_IP, BASE_PORT))

    except socket.timeout:
        continue
    except KeyboardInterrupt:
        break
