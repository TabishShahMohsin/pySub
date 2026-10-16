"""Print raw IMU data (accel, gyro, mag) from the Pixhawk."""

import time

from pymavlink import mavutil

DEVICE = "/dev/ttyACM0"  # Change if your port differs
BAUD = 115200  # Usually ignored for USB, but 115200 is standard
PRINT_PERIOD_S = 0.1


def main():
    print(f"Connecting to Pixhawk on {DEVICE}...")
    master = mavutil.mavlink_connection(DEVICE, baud=BAUD)

    # The first heartbeat confirms the connection is working
    master.wait_heartbeat()
    print(f"Heartbeat received! (system {master.target_system}, component {master.target_component})")

    last_print = 0.0
    try:
        while True:
            # RAW_IMU has accel, gyro and mag (SCALED_IMU / HIGHRES_IMU also exist).
            # Keep reading at the stream rate and only throttle the printing;
            # sleeping between reads would let a backlog build up so the
            # printed values fall further and further behind.
            msg = master.recv_match(type="RAW_IMU", blocking=True, timeout=1.0)
            if msg is None:
                print("No RAW_IMU message received... check stream rates.")
                continue

            now = time.monotonic()
            if now - last_print < PRINT_PERIOD_S:
                continue
            last_print = now

            print("\n--- IMU DATA ---")
            print(f"Accel: x={msg.xacc}, y={msg.yacc}, z={msg.zacc}")
            print(f"Gyro:  x={msg.xgyro}, y={msg.ygyro}, z={msg.zgyro}")
            print(f"Mag:   x={msg.xmag}, y={msg.ymag}, z={msg.zmag}")
    except KeyboardInterrupt:
        print("\nClosing connection.")
    finally:
        master.close()


if __name__ == "__main__":
    main()
