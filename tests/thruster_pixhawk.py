"""Spin each Pixhawk output channel 1-8 in turn at low thrust."""

import time

from pymavlink import mavutil

NEUTRAL = 1500
TEST_PWM = 1560  # Low thrust to be safe
SPIN_TIME = 1.5
NEUTRAL_8 = [NEUTRAL] * 8


def send_rc(master, pwms):
    master.mav.rc_channels_override_send(master.target_system, master.target_component, *pwms)


def main():
    master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
    master.wait_heartbeat()
    print("Handshake successful!")

    master.set_mode("MANUAL")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    print("Armed! Starting sequential spin...")

    try:
        for i in range(1, 9):
            print(f"Testing motor {i}...")
            pwms = list(NEUTRAL_8)
            pwms[i - 1] = TEST_PWM

            # Re-send at 10 Hz so the override doesn't time out
            t_end = time.monotonic() + SPIN_TIME
            while time.monotonic() < t_end:
                send_rc(master, pwms)
                time.sleep(0.1)

            send_rc(master, NEUTRAL_8)
            time.sleep(0.5)
        print("Test complete.")
    finally:
        # Always neutral + disarm, even on Ctrl+C
        send_rc(master, NEUTRAL_8)
        print("Disarming.")
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )


if __name__ == "__main__":
    main()
