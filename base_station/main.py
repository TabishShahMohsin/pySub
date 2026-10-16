"""Base-station control loop.

Threads:
  * main             - input, control (PID / KF), allocation, dashboard, video display
  * command_sender   - sends thruster PWMs to the Pi at COMMAND_RATE_HZ
  * telemetry_listener - receives IMU / pressure telemetry from the Pi
  * video_receiver   - receives camera frames via imagezmq

All data shared between threads goes through SharedState, which guards it with
a lock and hands out copies so readers never see a half-updated sample.
"""

import json
import math
import socket
import threading
import time

import cv2
import imagezmq
import pygame
import zmq

from config import (
    ANGLE_SETPOINT_RATE,
    COMMAND_RATE_HZ,
    CONTROL_RATE_HZ,
    DEPTH_KD,
    DEPTH_KF,
    DEPTH_KI,
    DEPTH_KP,
    DEPTH_PID,
    DEPTH_SETPOINT_RATE,
    GRAVITY,
    INVERTED_THRUSTERS,
    PI_HOST,
    PITCH_KD,
    PITCH_KI,
    PITCH_KP,
    PITCH_OFFSET,
    PITCH_PID,
    PRESSURE_OFFSET,
    PWM_NEUTRAL,
    ROLL_KD,
    ROLL_KI,
    ROLL_KP,
    ROLL_OFFSET,
    ROLL_PID,
    SURFACE_PRESSURE_MBAR,
    TELEMETRY_TIMEOUT_S,
    UDP_PORT_CMD,
    UDP_PORT_DATA,
    VIDEO_PORT,
    WATER_DENSITY,
    YAW_KD,
    YAW_KI,
    YAW_KP,
    YAW_OFFSET,
    YAW_PID,
    invert_pwm,
)
from input_handler import JoystickController
from kf import DepthKalmanFilter
from pid import PID, wrap_angle
from rov_kinematics import compute_thruster_forces, map_force_to_pwm

DASHBOARD_RATE_HZ = 10


class SharedState:
    """Thread-safe container for data shared between the threads."""

    def __init__(self):
        self._lock = threading.Lock()
        self.running = threading.Event()
        self.running.set()

        # Base station -> Pi
        self._pwms = [PWM_NEUTRAL] * 8

        # Pi -> base station (offset-corrected)
        self._telemetry = {
            "timestamp": 0.0,  # Pi clock, heartbeat only
            "pressure": SURFACE_PRESSURE_MBAR,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        }
        self._telemetry_seq = 0  # Increments on every packet received
        self._telemetry_rx_time = None  # Local monotonic receive time

        self._frames = {}

    # --- PWMs -------------------------------------------------------------
    def set_pwms(self, pwms):
        with self._lock:
            self._pwms = list(pwms)

    def get_pwms(self):
        with self._lock:
            return list(self._pwms)

    # --- Telemetry --------------------------------------------------------
    def update_telemetry(self, telemetry):
        with self._lock:
            self._telemetry = telemetry
            self._telemetry_seq += 1
            self._telemetry_rx_time = time.monotonic()

    def get_telemetry(self):
        """Return (telemetry copy, sequence number, local receive time)."""
        with self._lock:
            return dict(self._telemetry), self._telemetry_seq, self._telemetry_rx_time

    # --- Video ------------------------------------------------------------
    def put_frame(self, cam_id, frame):
        with self._lock:
            self._frames[cam_id] = frame

    def get_frames(self):
        with self._lock:
            return dict(self._frames)


state = SharedState()


def pressure_to_depth(pressure_mbar):
    """Hydrostatic depth in metres from absolute pressure in mbar."""
    return max(0.0, (pressure_mbar - SURFACE_PRESSURE_MBAR) * 100 / (WATER_DENSITY * GRAVITY))


def build_command(pwms):
    """Apply per-thruster inversion and build the JSON command for the Pi."""
    return {
        f"t{i + 1}": invert_pwm(pwm, inv)
        for i, (pwm, inv) in enumerate(zip(pwms, INVERTED_THRUSTERS, strict=True))
    }


def parse_telemetry(raw):
    """Validate a telemetry packet and apply the calibration offsets.

    Raises ValueError / KeyError / TypeError for malformed packets. NaN and
    infinity are rejected: a single NaN would poison the PID integral forever.
    """
    values = {key: float(raw[key]) for key in ("timestamp", "pressure", "roll", "pitch", "yaw")}
    if not all(math.isfinite(v) for v in values.values()):
        raise ValueError(f"non-finite value in {values}")
    return {
        "timestamp": values["timestamp"],
        "pressure": values["pressure"] - PRESSURE_OFFSET,
        "roll": values["roll"] - ROLL_OFFSET,
        "pitch": values["pitch"] - PITCH_OFFSET,
        "yaw": wrap_angle(values["yaw"] - YAW_OFFSET),
    }


def resolve_host(host):
    try:
        return socket.gethostbyname(host)
    except OSError:
        return None


# ---------------------------------------------------------------------------
# Threads
# ---------------------------------------------------------------------------
def video_receiver(port=VIDEO_PORT):
    """Receive camera frames from the Pi and store the latest one per camera."""
    try:
        image_hub = imagezmq.ImageHub(open_port=f"tcp://*:{port}")
    except zmq.ZMQError as e:
        print(f"Video receiver disabled: cannot bind port {port} ({e})")
        return
    # Time out recv so the thread can notice shutdown instead of blocking forever.
    image_hub.zmq_socket.setsockopt(zmq.RCVTIMEO, 1000)
    print("[Thread] Video receiver started. Waiting for frames...")

    try:
        while state.running.is_set():
            try:
                cam_id, frame = image_hub.recv_image()
            except zmq.Again:
                continue
            except Exception as e:
                print(f"Video receiver error: {e}")
                time.sleep(1)
                continue

            state.put_frame(cam_id, frame)
            # REQ/REP: the sender blocks until it gets this reply.
            image_hub.send_reply(b"OK")
    finally:
        image_hub.close()


def command_sender(host=PI_HOST, port=UDP_PORT_CMD):
    """Send the latest PWM commands to the Pi at a fixed rate."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    period = 1.0 / COMMAND_RATE_HZ
    pi_ip = None
    warned = False

    print("[Thread] Command sender started.")
    next_tick = time.monotonic()
    while state.running.is_set():
        if pi_ip is None:
            pi_ip = resolve_host(host)
            if pi_ip is None:
                if not warned:
                    print(f"Sender: cannot resolve '{host}', retrying...")
                    warned = True
                time.sleep(1.0)
                next_tick = time.monotonic()
                continue
            print(f"Sender: sending commands to {host} ({pi_ip})")

        try:
            payload = json.dumps(build_command(state.get_pwms())).encode()
            sock.sendto(payload, (pi_ip, port))
        except OSError as e:
            print(f"Sender error: {e}")
            pi_ip = None  # Re-resolve: the Pi may have a new address

        # Fixed-rate schedule (no drift); resync if we fell more than a tick behind.
        next_tick += period
        delay = next_tick - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            next_tick = time.monotonic()

    # Shutdown: make sure the last thing the Pi hears is neutral.
    if pi_ip is not None:
        neutral = json.dumps(build_command([PWM_NEUTRAL] * 8)).encode()
        for _ in range(3):
            try:
                sock.sendto(neutral, (pi_ip, port))
            except OSError:
                break
            time.sleep(period)
    sock.close()


def telemetry_listener(port=UDP_PORT_DATA):
    """Receive telemetry from the Pi."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        sock.bind(("0.0.0.0", port))
    except OSError as e:
        print(f"Binding error: {e}")
        return
    sock.settimeout(1.0)  # Don't block forever so shutdown is noticed

    print("[Thread] Telemetry listener started.")
    while state.running.is_set():
        try:
            data, _ = sock.recvfrom(1024)
            state.update_telemetry(parse_telemetry(json.loads(data.decode())))
        except TimeoutError:
            continue
        except (ValueError, KeyError, TypeError) as e:
            print(f"Listener: bad packet ({e})")
        except OSError as e:
            print(f"Listener error: {e}")
            time.sleep(0.5)
    sock.close()


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
def render_dashboard(fps, link_ok, telem, measured_depth, targets, forces, pwms):
    target_depth, target_roll, target_pitch, target_yaw = targets
    f, p = forces, pwms
    link = "OK" if link_ok else "NO TELEMETRY"
    sep = "-" * 60
    return (
        "\033[H"  # Cursor home; the screen is cleared once at start-up
        "--- ROV_SEA-6.0 DASHBOARD ---\n"
        f"SYSTEM: Pressure: {telem['pressure']:>7.2f} mbar | Link: {link:<12}\n"
        f"{sep}\n"
        "THRUSTERS (Forces & PWMs):\n"
        f"  Horizontal: T1:{f[0]:>6.2f}({p[0]}) T2:{f[1]:>6.2f}({p[1]}) "
        f"T3:{f[2]:>6.2f}({p[2]}) T4:{f[3]:>6.2f}({p[3]})\n"
        f"  Vertical:   T5:{f[4]:>6.2f}({p[4]}) T6:{f[5]:>6.2f}({p[5]}) "
        f"T7:{f[6]:>6.2f}({p[6]}) T8:{f[7]:>6.2f}({p[7]})\n"
        f"{sep}\n"
        f"NAVIGATION:      {'[SETPOINT]':>15} {'[MEASURED]':>15}\n"
        f"  Depth (m):     {target_depth:>15.2f} {measured_depth:>15.2f}\n"
        f"  Roll  (°):     {target_roll:>15.2f} {telem['roll']:>15.2f}\n"
        f"  Pitch (°):     {target_pitch:>15.2f} {telem['pitch']:>15.2f}\n"
        f"  Yaw   (°):     {target_yaw:>15.2f} {telem['yaw']:>15.2f}\n"
        f"{sep}\n"
        f"Status: RUNNING | Frequency: {fps:5.1f} FPS\n"
    )


def main():
    pygame.init()
    # A window is needed for keyboard input focus.
    pygame.display.set_mode((400, 300))
    pygame.display.set_caption("ROV Base Station")
    clock = pygame.time.Clock()

    try:
        controller = JoystickController(deadzone=0.1)
    except RuntimeError as e:
        print(e)
        pygame.quit()
        return

    kf = DepthKalmanFilter()

    depth_pid = PID(DEPTH_KP, DEPTH_KI, DEPTH_KD, 1, -1)
    roll_pid = PID(ROLL_KP, ROLL_KI, ROLL_KD, 1, -1, is_angle=True)
    pitch_pid = PID(PITCH_KP, PITCH_KI, PITCH_KD, 1, -1, is_angle=True)
    yaw_pid = PID(YAW_KP, YAW_KI, YAW_KD, 1, -1, is_angle=True)
    pids = (depth_pid, roll_pid, pitch_pid, yaw_pid)

    target_depth = 0.0
    target_roll = 0.0
    target_pitch = 0.0
    target_yaw = 0.0

    threads = [
        threading.Thread(target=telemetry_listener, name="telemetry", daemon=True),
        threading.Thread(target=command_sender, name="sender", daemon=True),
        threading.Thread(target=video_receiver, name="video", daemon=True),
    ]
    for t in threads:
        t.start()

    last_seq = 0
    last_sample_time = None
    measured_depth = 0.0
    last_dashboard = 0.0

    print("\033[2J", end="")  # Clear the terminal once

    try:
        running = True
        while running:
            dt = clock.tick(CONTROL_RATE_HZ) / 1000

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            raw_surge, raw_sway, raw_heave, raw_roll, raw_pitch, raw_yaw = controller.get_input_vector()

            telem, seq, rx_time = state.get_telemetry()
            now = time.monotonic()
            link_ok = rx_time is not None and now - rx_time < TELEMETRY_TIMEOUT_S

            # Telemetry arrives at ~20 Hz while this loop runs at 30 Hz. Only
            # feed the filter / PIDs when a *new* sample arrives, using the time
            # between samples as dt; otherwise repeated samples would be counted
            # as fresh measurements and the D-term would spike.
            new_sample = seq != last_seq
            sample_dt = 0.0
            if new_sample:
                sample_dt = rx_time - last_sample_time if last_sample_time is not None else dt
                last_sample_time = rx_time
                last_seq = seq

            raw_depth = pressure_to_depth(telem["pressure"])
            if DEPTH_KF:
                # Without measurements the constant-velocity model would drift
                # without bound, so hold the estimate while the link is down.
                if link_ok:
                    measured_depth = kf.predict(dt)
                if new_sample:
                    measured_depth = kf.correct(raw_depth)
            else:
                measured_depth = raw_depth

            # Setpoints follow the sticks.
            if DEPTH_PID:
                target_depth = max(0.0, target_depth + raw_heave * DEPTH_SETPOINT_RATE * dt)
            if ROLL_PID:
                target_roll = wrap_angle(target_roll + raw_roll * ANGLE_SETPOINT_RATE * dt)
            if PITCH_PID:
                target_pitch = wrap_angle(target_pitch + raw_pitch * ANGLE_SETPOINT_RATE * dt)
            if YAW_PID:
                target_yaw = wrap_angle(target_yaw + raw_yaw * ANGLE_SETPOINT_RATE * dt)

            if not link_ok:
                # No feedback: don't let the PIDs act on stale data. Also forget
                # the last sample time, otherwise the first sample after the
                # link returns would use the whole outage as dt.
                for pid in pids:
                    pid.reset()
                last_sample_time = None
            elif new_sample:
                if DEPTH_PID:
                    depth_pid.compute(measured_depth, target_depth, sample_dt)
                if ROLL_PID:
                    roll_pid.compute(telem["roll"], target_roll, sample_dt)
                if PITCH_PID:
                    pitch_pid.compute(telem["pitch"], target_pitch, sample_dt)
                if YAW_PID:
                    yaw_pid.compute(telem["yaw"], target_yaw, sample_dt)

            heave_cmd = depth_pid.output if DEPTH_PID else raw_heave
            roll_cmd = roll_pid.output if ROLL_PID else raw_roll
            pitch_cmd = pitch_pid.output if PITCH_PID else raw_pitch
            yaw_cmd = yaw_pid.output if YAW_PID else raw_yaw

            forces = compute_thruster_forces(raw_surge, raw_sway, heave_cmd, roll_cmd, pitch_cmd, yaw_cmd)
            pwms = [map_force_to_pwm(f) for f in forces]
            state.set_pwms(pwms)

            if now - last_dashboard >= 1.0 / DASHBOARD_RATE_HZ:
                last_dashboard = now
                print(
                    render_dashboard(
                        clock.get_fps(),
                        link_ok,
                        telem,
                        measured_depth,
                        (target_depth, target_roll, target_pitch, target_yaw),
                        forces,
                        pwms,
                    ),
                    end="",
                    flush=True,
                )

            for cam_id, frame in state.get_frames().items():
                if frame is not None:
                    cv2.imshow(cam_id, frame[::-1])  # Flip vertically

            # waitKey is required for OpenCV windows to render.
            if cv2.waitKey(1) & 0xFF == ord("q"):
                running = False
    finally:
        # Stop thrusters safely: neutral first, then let the sender flush it.
        state.set_pwms([PWM_NEUTRAL] * 8)
        state.running.clear()
        for t in threads:
            t.join(timeout=1.5)
        cv2.destroyAllWindows()
        pygame.quit()
        print("\nBase station exited.")


if __name__ == "__main__":
    main()
