"""Step through a group of thrusters driven directly by pigpio (legacy wiring)."""

import sys
import time

import pigpio

LATERAL_PINS = [18, 27, 17, 22]  # T1-T4; T2, T3, T4 are to be inverted
VERTICAL_PINS = [19, 6, 20, 13]  # T5-T8; T5, T6 are to be inverted

THRUSTER_PINS = VERTICAL_PINS  # Pick the group to test

NEUTRAL = 1500
FORWARD = 1600
REVERSE = 1400


def main():
    pi = pigpio.pi()
    if not pi.connected:
        print("pigpiod not running! Run: sudo pigpiod")
        sys.exit(1)

    try:
        for pin in THRUSTER_PINS:
            print(f"Arming... GPIO {pin}")
            pi.set_servo_pulsewidth(pin, NEUTRAL)
            time.sleep(1)

            print(f"FWD...... GPIO {pin}")
            pi.set_servo_pulsewidth(pin, FORWARD)
            time.sleep(5)

            print(f"Neutral.. GPIO {pin}")
            pi.set_servo_pulsewidth(pin, NEUTRAL)
            time.sleep(1)

            print(f"RVS...... GPIO {pin}")
            pi.set_servo_pulsewidth(pin, REVERSE)
            time.sleep(1)

            print(f"Neutral.. GPIO {pin}")
            pi.set_servo_pulsewidth(pin, NEUTRAL)
            time.sleep(1)

            input("Enter to run next thruster: ")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        # pigpio keeps generating pulses after the script exits, so always
        # return every thruster to neutral.
        for pin in THRUSTER_PINS:
            pi.set_servo_pulsewidth(pin, NEUTRAL)
        pi.stop()


if __name__ == "__main__":
    main()
