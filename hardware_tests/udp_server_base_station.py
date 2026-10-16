"""UDP round-trip test, base-station side: measure RTT to the Pi.

Run udp_client_pi.py on the Pi at the same time.
Find the Pi with `ping auv.local` or `ifconfig`. Turn Wi-Fi off to make sure
the test runs over Ethernet.
"""

import os
import socket
import time

BIND_IP = "0.0.0.0"
BASE_PORT = 5005
PI_HOST = os.environ.get("ROV_PI_HOST", "auv.local")
PI_PORT = 5006
PROBE_PERIOD_S = 0.5
REPLY_TIMEOUT_S = 1.0


def main():
    pi_ip = socket.gethostbyname(PI_HOST)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((BIND_IP, BASE_PORT))
    print(f"BASE: UDP test started, probing {PI_HOST} ({pi_ip})")

    seq = 0
    try:
        while True:
            seq += 1
            send_time = time.monotonic()
            sock.sendto(f"{seq},{send_time}".encode(), (pi_ip, PI_PORT))

            # Wait for the reply to *this* probe; late replies to earlier
            # probes are discarded so they can't produce a wrong RTT.
            deadline = send_time + REPLY_TIMEOUT_S
            got_reply = False
            while not got_reply:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                sock.settimeout(remaining)
                try:
                    data, _ = sock.recvfrom(1024)
                except TimeoutError:
                    break
                try:
                    reply_seq, _, _ = data.decode().split(",")
                except ValueError:
                    continue
                if int(reply_seq) == seq:
                    rtt = time.monotonic() - send_time
                    print(f"BASE: probe {seq} | RTT = {rtt * 1000:7.2f} ms", end="\r", flush=True)
                    got_reply = True

            if not got_reply:
                print(f"BASE: probe {seq} | no response   ", end="\r", flush=True)

            time.sleep(max(0.0, send_time + PROBE_PERIOD_S - time.monotonic()))
    except KeyboardInterrupt:
        print("\nBASE: stopped.")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
