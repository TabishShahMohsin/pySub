"""Spin each logical thruster T1-T8 with pin mapping and inversion applied."""

import time

from pymavlink import mavutil

NEUTRAL = 1500
TEST_PWM = 1600
NEUTRAL_8 = [NEUTRAL] * 8

# Logical thruster -> physical pin (same as pi/main.py THRUSTER_MAP)
LOGICAL_TO_PHYSICAL = {
    1: 2,
    2: 1,
    3: 4,
    4: 3,
    5: 6,
    6: 7,
    7: 5,
    8: 8,
}

# Logical thrusters that must be reversed. Matches I1-I8 in
# base_station/config.py (I2, I3, I4, I5, I6 = True).
INVERT_LIST = [2, 3, 4, 5, 6]


def set_parameter(master, name, value):
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


def get_mapped_pwm(t_num, pwm):
    """Mirror the PWM about neutral for inverted thrusters (1600 -> 1400)."""
    if t_num in INVERT_LIST:
        return NEUTRAL - (pwm - NEUTRAL)
    return pwm


def main():
    master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
    master.wait_heartbeat()
    print("Connected!")

    print("Setting pins to passthrough...")
    for i in range(1, 9):
        set_parameter(master, f"SERVO{i}_FUNCTION", 50 + i)
        time.sleep(0.05)

    master.set_mode("MANUAL")
    arm_disarm(master, True)
    print("Armed! Sequence: corrected order + inversions active.")

    try:
        for t_num in range(1, 9):
            physical_pin = LOGICAL_TO_PHYSICAL[t_num]
            test_pwm = get_mapped_pwm(t_num, TEST_PWM)

            status = "INVERTED" if t_num in INVERT_LIST else "NORMAL"
            print(f"Testing T{t_num} ({status}) -> physical pin {physical_pin} @ PWM {test_pwm}")

            pwms = list(NEUTRAL_8)
            pwms[physical_pin - 1] = test_pwm

            t_end = time.monotonic() + 2
            while time.monotonic() < t_end:
                send_rc(master, pwms)
                time.sleep(0.1)

            send_rc(master, NEUTRAL_8)
            time.sleep(0.5)
    finally:
        send_rc(master, NEUTRAL_8)
        print("Disarming.")
        arm_disarm(master, False)


if __name__ == "__main__":
    main()
