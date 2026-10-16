"""Spin each physical Pixhawk output pin 1-8 individually (passthrough mode)."""

import time

from pymavlink import mavutil

NEUTRAL = 1500
TEST_PWM = 1600
NEUTRAL_8 = [NEUTRAL] * 8


def set_parameter(master, name, value):
    """Set a parameter on the Pixhawk."""
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        name.encode("utf-8"),
        value,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )


def send_rc(master, pwms):
    master.mav.rc_channels_override_send(master.target_system, master.target_component, *pwms)


def arm_disarm(master, arm):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1 if arm else 0,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def main():
    master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
    master.wait_heartbeat()
    print("Connected!")

    # 1. Passthrough: pin N follows RC channel N (51 = RCIN1 ... 58 = RCIN8)
    print("Configuring pins for direct passthrough (no mixing)...")
    for i in range(1, 9):
        set_parameter(master, f"SERVO{i}_FUNCTION", 50 + i)
        time.sleep(0.1)

    # 2. Arm
    master.set_mode("MANUAL")
    arm_disarm(master, True)
    print("Armed! Testing pins 1-8 individually...")

    # 3. Test loop
    try:
        for i in range(1, 9):
            print(f"Spinning physical pin {i} ONLY")
            pwms = list(NEUTRAL_8)
            pwms[i - 1] = TEST_PWM

            t_end = time.monotonic() + 2
            while time.monotonic() < t_end:
                send_rc(master, pwms)
                time.sleep(0.1)

            send_rc(master, NEUTRAL_8)
            time.sleep(1)
    finally:
        # Set SERVOx_FUNCTION back to Motor1..8 (33-40) to use normal ROV modes later.
        send_rc(master, NEUTRAL_8)
        print("Disarming.")
        arm_disarm(master, False)


if __name__ == "__main__":
    main()
