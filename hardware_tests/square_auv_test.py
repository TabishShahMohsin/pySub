"""ROV/AUV mission control - square pattern (open loop, legacy pigpio wiring).

Result: failed. Without the IMU and pressure sensor a floating vehicle isn't
controllable with simple timed commands; this needs PID control.

Executes a timed square movement pattern using an 8-thruster V-configuration
allocation, with PWM mapping for thruster non-linearity, a ramping engine to
prevent current spikes, and "calming" periods between manoeuvres to let
hydrodynamic momentum die down.

Parameters requiring tuning: CONST_HEAVE, MOVE_DURATION, BRAKE_DURATION,
SPEED, INVERSION_MASK.
"""

import subprocess
import sys
import threading
import time

import numpy as np
import pigpio

# --- 1. Configuration ---
THRUSTER_PINS = {
    "t1": 18,
    "t2": 23,
    "t3": 17,
    "t4": 27,
    "t5": 20,
    "t6": 13,
    "t7": 19,
    "t8": 6,
}

# 1 = normal, -1 = inverted (for any thruster spinning the wrong way)
INVERSION_MASK = [-1, 1, 1, -1, -1, -1, -1, -1]

NEUTRAL = 1500
RAMP_STEP = 15  # µs per ramp tick
RAMP_PERIOD = 0.05  # s (20 Hz)
CONST_HEAVE = -0.3

SPEED = 0.4
MOVE_DURATION = 4  # Time spent thrusting (s)
BRAKE_DURATION = 3  # Calming period to let momentum die down (s)

target_pwms = {key: NEUTRAL for key in THRUSTER_PINS}
current_pwms = {key: NEUTRAL for key in THRUSTER_PINS}
targets_lock = threading.Lock()
running = threading.Event()


# --- 2. Kinematics ---
def map_force_to_pwm(f):
    if abs(f) < 1e-2:
        return NEUTRAL
    if f < 0:
        return int(np.array([f**3, f**2, f, 1]) @ [4.58, 35.21, 169.73, 1464.33])
    return int(np.array([f**3, f**2, f, 1]) @ [2.22, -22.41, 135.44, 1535.90])


def get_pwms(surge, sway, yaw, heave, roll=0, pitch=0):
    t1 = surge + sway + yaw
    t2 = surge - sway - yaw
    t3 = surge - sway + yaw
    t4 = surge + sway - yaw
    t5 = heave - pitch - roll
    t6 = heave - pitch + roll
    t7 = heave + pitch - roll
    t8 = heave + pitch + roll

    # Apply the inversion mask before normalisation
    forces = [f * m for f, m in zip([t1, t2, t3, t4, t5, t6, t7, t8], INVERSION_MASK, strict=True)]

    max_f = max(abs(f) for f in forces)
    if max_f > 1.0:
        forces = [f / max_f for f in forces]

    return [map_force_to_pwm(f) for f in forces]


# --- 3. Hardware and ramping ---
def ramping_loop(pi):
    """Move each thruster towards its target by RAMP_STEP every RAMP_PERIOD."""
    next_tick = time.monotonic()
    while running.is_set():
        # Copy under the lock so all 8 targets come from the same command
        with targets_lock:
            targets = dict(target_pwms)
        for key, target in targets.items():
            curr = current_pwms[key]
            if curr < target:
                current_pwms[key] = min(curr + RAMP_STEP, target)
            elif curr > target:
                current_pwms[key] = max(curr - RAMP_STEP, target)
            pi.set_servo_pulsewidth(THRUSTER_PINS[key], current_pwms[key])

        next_tick += RAMP_PERIOD
        time.sleep(max(0.0, next_tick - time.monotonic()))


def update_motion(surge, sway, yaw, heave):
    pwms = get_pwms(surge, sway, yaw, heave)
    with targets_lock:
        for i, pwm in enumerate(pwms, start=1):
            target_pwms[f"t{i}"] = pwm


# --- 4. Mission: the square pattern ---
def stop_and_wait():
    print("Braking / waiting...")
    update_motion(0, 0, 0, CONST_HEAVE)
    time.sleep(BRAKE_DURATION)


def mission_logic():
    print("Mission starting in 5 seconds...")
    time.sleep(5)

    print("Side 1: Forward")
    update_motion(SPEED, 0, 0, CONST_HEAVE)
    time.sleep(MOVE_DURATION)
    stop_and_wait()

    print("Side 2: Right")
    update_motion(0, SPEED, 0, CONST_HEAVE)
    time.sleep(MOVE_DURATION)
    stop_and_wait()

    print("Side 3: Backward")
    update_motion(-SPEED, 0, 0, CONST_HEAVE)
    time.sleep(MOVE_DURATION)
    stop_and_wait()

    print("Side 4: Left")
    update_motion(0, -SPEED, 0, CONST_HEAVE)
    time.sleep(MOVE_DURATION)

    print("Mission complete. Surfacing.")
    update_motion(0, 0, 0, 0)
    time.sleep(2)


# --- 5. Main ---
def main():
    try:
        subprocess.run(["sudo", "pigpiod"], capture_output=True, check=False)
    except OSError:
        pass

    pi = pigpio.pi()
    if not pi.connected:
        print("pigpiod not running! Run: sudo pigpiod")
        sys.exit(1)

    running.set()
    t_ramp = threading.Thread(target=ramping_loop, args=(pi,), daemon=True)
    try:
        t_ramp.start()
        mission_logic()
    except KeyboardInterrupt:
        print("\nEmergency stop!")
    finally:
        # Stop the ramp thread *before* neutralising, otherwise it can write a
        # non-neutral pulse after we have set neutral.
        running.clear()
        t_ramp.join(timeout=1.0)
        for pin in THRUSTER_PINS.values():
            pi.set_servo_pulsewidth(pin, NEUTRAL)
        pi.stop()


if __name__ == "__main__":
    main()
