"""Run each thruster forward then reverse, driven directly by pigpio (legacy wiring)."""

import sys
import time

import pigpio

# GPIO pins for all 8 thrusters
THRUSTER_PINS = [6, 5, 16, 26, 22, 23, 27, 17]

# PWM values (µs)
NEUTRAL = 1500
FORWARD = 1650  # 150 µs above neutral
REVERSE = 1350  # 150 µs below neutral

# Timing (s)
MOVE_TIME = 2.0
BREAK_TIME = 1.0
ARM_TIME = 3.0


def safe_stop(pi):
    """Set every pin to neutral and release pigpio."""
    print("\nNeutralizing all thrusters...")
    for pin in THRUSTER_PINS:
        pi.set_servo_pulsewidth(pin, NEUTRAL)
    pi.stop()


def arm_system(pi):
    print(f"Step 1: Arming ESCs (sending neutral for {ARM_TIME:.0f}s)...")
    for pin in THRUSTER_PINS:
        pi.set_servo_pulsewidth(pin, NEUTRAL)
    time.sleep(ARM_TIME)
    print("System armed and ready.")


def main():
    pi = pigpio.pi()
    if not pi.connected:
        print("pigpiod not running! Run: sudo pigpiod")
        sys.exit(1)

    try:
        arm_system(pi)

        for i, pin in enumerate(THRUSTER_PINS, start=1):
            print(f"\nTesting thruster {i} (GPIO {pin})")

            print("  Forward")
            pi.set_servo_pulsewidth(pin, FORWARD)
            time.sleep(MOVE_TIME)

            print("  Break")
            pi.set_servo_pulsewidth(pin, NEUTRAL)
            time.sleep(BREAK_TIME)

            print("  Backward")
            pi.set_servo_pulsewidth(pin, REVERSE)
            time.sleep(MOVE_TIME)

            pi.set_servo_pulsewidth(pin, NEUTRAL)
            print(f"Thruster {i} complete.")
            time.sleep(0.5)

        print("\nSequence finished successfully.")
    except KeyboardInterrupt:
        print("\nEmergency stop!")
    finally:
        safe_stop(pi)


if __name__ == "__main__":
    main()
