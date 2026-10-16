"""Print roll / pitch / yaw from the Pixhawk ATTITUDE message."""

import math
import os

# Must be set before pymavlink is imported, otherwise MAVLink 1 is selected.
os.environ["MAVLINK20"] = "1"

from pymavlink import mavutil  # noqa: E402

DEVICE = "/dev/ttyACM0"
BAUD = 115200


def main():
    master = mavutil.mavlink_connection(DEVICE, baud=BAUD)
    print("Waiting for heartbeat...")
    master.wait_heartbeat()
    print("Connected!")

    # Ask the Pixhawk to stream ATTITUDE (part of EXTRA1) at 10 Hz
    master.mav.request_data_stream_send(
        master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10, 1
    )

    try:
        while True:
            # Timeout so the script doesn't hang forever if a packet is lost
            msg = master.recv_match(type="ATTITUDE", blocking=True, timeout=1.0)
            if msg is None:
                print("No ATTITUDE message received... check stream rates.")
                continue
            roll = math.degrees(msg.roll)
            pitch = math.degrees(msg.pitch)
            yaw = math.degrees(msg.yaw)
            print(f"Roll: {roll:6.2f} | Pitch: {pitch:6.2f} | Yaw: {yaw:6.2f}")
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        master.close()


if __name__ == "__main__":
    main()
