"""ROV-side program (Raspberry Pi + Pixhawk running ArduSub).

Threads:
  * main              - start-up, watchdog (neutral on lost link), dashboard
  * command_receiver  - receives PWM commands and forwards them as RC overrides
  * sensor_sender     - drains Pixhawk telemetry and sends it to the base station
  * video_stream_loop - streams RealSense + USB camera frames via imagezmq

The MAVLink connection is not thread-safe, so every access goes through
Pixhawk.lock. Thruster state is guarded by ThrusterState.lock. When both are
needed they are always taken in the order ThrusterState.lock -> Pixhawk.lock.
"""

import os

# Must be set before pymavlink is imported, otherwise MAVLink 1 is selected.
os.environ["MAVLINK20"] = "1"

import json  # noqa: E402
import math  # noqa: E402
import socket  # noqa: E402
import threading  # noqa: E402
import time  # noqa: E402

import cv2  # noqa: E402
import imagezmq  # noqa: E402
import numpy as np  # noqa: E402
import zmq  # noqa: E402
from pymavlink import mavutil  # noqa: E402

try:
    import pyrealsense2 as rs
except ImportError:  # Allow running without the RealSense SDK installed
    rs = None

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
PIXHAWK_DEVICE = "/dev/ttyACM0"
PIXHAWK_BAUD = 115200

BASE_HOST = os.environ.get("ROV_BASE_HOST", "mba.local")  # Base-station hostname/IP
BIND_IP = "0.0.0.0"
UDP_PORT_DATA = 5005  # Pi -> base station telemetry
UDP_PORT_CMD = 5006  # Base station -> Pi commands
VIDEO_PORT = 5555

USB_CAM_INDEX = 2
VIDEO_REPLY_TIMEOUT_MS = 2000  # Reconnect if the base station stops replying

TELEMETRY_RATE_HZ = 20
STREAM_RATE_HZ = 50  # Rate requested from the Pixhawk
WATCHDOG_RATE_HZ = 10
COMMAND_TIMEOUT_S = 1.0  # Go neutral if no command for this long

PWM_NEUTRAL = 1500
PWM_MIN = 1150
PWM_MAX = 1850

# Logical thruster number -> physical Pixhawk output pin
THRUSTER_MAP = {
    1: 2,
    2: 1,
    3: 4,
    4: 3,
    5: 6,
    6: 7,
    7: 5,
    8: 8,
}

HOSTNAME = socket.gethostname()
running = threading.Event()


def resolve_host(host):
    try:
        return socket.gethostbyname(host)
    except OSError:
        return None


# ---------------------------------------------------------------------------
# Pixhawk access
# ---------------------------------------------------------------------------
class Pixhawk:
    """Serialises all access to the MAVLink connection."""

    def __init__(self, device, baud):
        self.master = mavutil.mavlink_connection(device, baud=baud)
        self.lock = threading.Lock()

    def wait_heartbeat(self):
        self.master.wait_heartbeat()

    def _command_long(self, command, *params):
        params = list(params) + [0] * (7 - len(params))
        with self.lock:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component, command, 0, *params
            )

    def set_passthrough(self):
        """Set servo outputs 1-8 to RCPassThru (RCIN1..RCIN8 = 51..58)."""
        print("Configuring Pixhawk passthrough...")
        for i in range(1, 9):
            with self.lock:
                self.master.mav.param_set_send(
                    self.master.target_system,
                    self.master.target_component,
                    f"SERVO{i}_FUNCTION".encode(),
                    50 + i,
                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
                )
            time.sleep(0.05)  # Don't flood the link with parameter writes

    def request_streams(self, rate_hz):
        with self.lock:
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                rate_hz,
                1,
            )

    def arm(self):
        with self.lock:
            self.master.set_mode("MANUAL")
        time.sleep(0.5)
        self._command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        print("Vehicle ARMED")

    def disarm(self):
        self._command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)
        print("Vehicle DISARMED")

    def send_rc(self, pwms):
        """Send RC overrides for channels 1-8."""
        with self.lock:
            self.master.mav.rc_channels_override_send(
                self.master.target_system, self.master.target_component, *pwms
            )

    def drain(self, types, max_messages=500):
        """Read every pending message of the given types (non-blocking).

        recv_match() discards non-matching messages it reads, so asking for
        each type separately starves the others. Draining everything each
        cycle also stops a backlog (and growing latency) building up.
        """
        msgs = []
        with self.lock:
            for _ in range(max_messages):
                msg = self.master.recv_match(type=list(types), blocking=False)
                if msg is None:
                    break
                msgs.append(msg)
        return msgs

    def close(self):
        with self.lock:
            self.master.close()


# ---------------------------------------------------------------------------
# Shared thruster state
# ---------------------------------------------------------------------------
class ThrusterState:
    def __init__(self):
        self.lock = threading.Lock()
        self.pwms = {f"t{i}": PWM_NEUTRAL for i in range(1, 9)}
        self.last_command_time = None  # time.monotonic() of the last command

    def mapped_pwm_list(self):
        """Translate logical t1-t8 into physical output order (call with lock held)."""
        output = [PWM_NEUTRAL] * 8
        for t_num, physical_pin in THRUSTER_MAP.items():
            output[physical_pin - 1] = self.pwms[f"t{t_num}"]
        return output

    def snapshot(self):
        with self.lock:
            return dict(self.pwms)


def parse_pwm(value):
    """Validate one PWM value from a command packet and clamp it to the ESC range.

    Rejects bools, strings, NaN and infinity: int(float("inf")) would raise
    OverflowError and kill the receiver thread, and int(True) would be 1.
    """
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"invalid PWM value {value!r}")
    return max(PWM_MIN, min(PWM_MAX, int(value)))


def _neutral_locked(pixhawk, thrusters):
    """Set every thruster to neutral and send it (thrusters.lock must be held)."""
    for key in thrusters.pwms:
        thrusters.pwms[key] = PWM_NEUTRAL
    pixhawk.send_rc([PWM_NEUTRAL] * 8)


def stop_all_thrusters(pixhawk, thrusters, verbose=True):
    """Set every thruster to neutral and send it immediately."""
    if verbose:
        print("!!! STOPPING ALL THRUSTERS (NEUTRAL) !!!")
    with thrusters.lock:
        _neutral_locked(pixhawk, thrusters)


def watchdog_step(pixhawk, thrusters, now=None):
    """Go neutral if no command arrived within COMMAND_TIMEOUT_S. Returns True if lost.

    The check and the stop happen under one lock so a command arriving in
    between can't be overwritten by a stale "lost" decision. While the link is
    lost, neutral is re-sent on every call so the RC override stays fresh.
    """
    if now is None:
        now = time.monotonic()
    with thrusters.lock:
        last = thrusters.last_command_time
        lost = last is None or now - last > COMMAND_TIMEOUT_S
        if lost:
            _neutral_locked(pixhawk, thrusters)
    return lost


# ---------------------------------------------------------------------------
# Threads
# ---------------------------------------------------------------------------
def command_receiver(pixhawk, thrusters, port=UDP_PORT_CMD):
    """Receive PWM commands and forward them to the Pixhawk as RC overrides."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((BIND_IP, port))
    sock.settimeout(0.5)

    while running.is_set():
        try:
            data, _ = sock.recvfrom(1024)
        except TimeoutError:
            continue
        except OSError as e:
            print(f"\n[Cmd] Socket error: {e}")
            time.sleep(0.5)
            continue

        try:
            new_cmds = json.loads(data.decode())
            updates = {key: parse_pwm(val) for key, val in new_cmds.items() if key in thrusters.pwms}
        except (ValueError, TypeError, AttributeError) as e:
            print(f"\n[Cmd] Bad packet: {e}")
            continue

        with thrusters.lock:
            if not running.is_set():
                break  # Shutting down: don't override the final neutral
            thrusters.pwms.update(updates)
            pixhawk.send_rc(thrusters.mapped_pwm_list())
            thrusters.last_command_time = time.monotonic()

    sock.close()


def sensor_sender(pixhawk, telemetry, telemetry_lock, host=BASE_HOST, port=UDP_PORT_DATA):
    """Drain Pixhawk telemetry and send it to the base station at a fixed rate."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    period = 1.0 / TELEMETRY_RATE_HZ
    base_ip = None
    next_tick = time.monotonic()

    while running.is_set():
        for msg in pixhawk.drain(("ATTITUDE", "SCALED_PRESSURE2")):
            with telemetry_lock:
                if msg.get_type() == "ATTITUDE":
                    telemetry["roll"] = float(np.degrees(msg.roll))
                    telemetry["pitch"] = float(np.degrees(msg.pitch))
                    telemetry["yaw"] = float(np.degrees(msg.yaw))
                else:
                    telemetry["pressure"] = float(msg.press_abs)

        if base_ip is None:
            base_ip = resolve_host(host)

        if base_ip is not None:
            with telemetry_lock:
                telemetry["timestamp"] = time.time()
                payload = json.dumps(telemetry).encode()
            try:
                sock.sendto(payload, (base_ip, port))
            except OSError as e:
                print(f"\n[Telemetry] Send error: {e}")
                base_ip = None  # Re-resolve next time

        next_tick += period
        delay = next_tick - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            next_tick = time.monotonic()

    sock.close()


def start_realsense():
    if rs is None:
        print("[Video] pyrealsense2 not installed, skipping RealSense.")
        return None
    try:
        pipeline = rs.pipeline()
        rs_config = rs.config()
        rs_config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipeline.start(rs_config)
        return pipeline
    except Exception as e:
        print(f"[Video] RealSense unavailable: {e}")
        return None


def open_usb_camera():
    cam = cv2.VideoCapture(USB_CAM_INDEX)
    if not cam.isOpened():
        print(f"[Video] USB camera {USB_CAM_INDEX} unavailable.")
        cam.release()
        return None
    # Keep only the newest frame so the stream doesn't lag behind.
    cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cam


def video_stream_loop(host=BASE_HOST, port=VIDEO_PORT):
    """Stream camera frames, reconnecting cameras and the network on failure."""
    while running.is_set():
        pipeline = None
        usb_cam = None
        sender = None

        try:
            base_ip = resolve_host(host)
            if base_ip is None:
                raise RuntimeError(f"cannot resolve '{host}'")

            print(f"\n[Video] Connecting to base station at {base_ip}...")
            sender = imagezmq.ImageSender(connect_to=f"tcp://{base_ip}:{port}")
            # Without a timeout, send_image() blocks forever waiting for a
            # reply if the base station restarts (REQ/REP lock-step).
            sender.zmq_socket.setsockopt(zmq.RCVTIMEO, VIDEO_REPLY_TIMEOUT_MS)
            sender.zmq_socket.setsockopt(zmq.LINGER, 0)

            pipeline = start_realsense()
            usb_cam = open_usb_camera()
            if pipeline is None and usb_cam is None:
                raise RuntimeError("no cameras available")

            print("[Video] Streaming started.")
            while running.is_set():
                if pipeline is not None:
                    # Non-blocking, so a slow RealSense doesn't stall the USB cam
                    frames = pipeline.poll_for_frames()
                    if frames:
                        color_frame = frames.get_color_frame()
                        if color_frame:
                            rs_img = np.asanyarray(color_frame.get_data())
                            sender.send_image(f"{HOSTNAME}_realsense", rs_img)

                if usb_cam is not None:
                    # Blocks for at most one frame period of the camera
                    ret, usb_frame = usb_cam.read()
                    if not ret:
                        raise RuntimeError("USB camera read failed")
                    sender.send_image("usb_cam", usb_frame)
                else:
                    time.sleep(0.005)  # Avoid a busy loop when polling RealSense only

        except zmq.Again:
            print("\n[Video] Base station not replying. Reconnecting in 3s...")
        except Exception as e:
            print(f"\n[Video] Stream error: {e}. Retrying in 3s...")
        finally:
            if pipeline is not None:
                try:
                    pipeline.stop()
                except Exception:
                    pass
            if usb_cam is not None:
                usb_cam.release()
            if sender is not None:
                try:
                    sender.close()
                except Exception:
                    pass

        if running.is_set():
            time.sleep(3)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    pixhawk = Pixhawk(PIXHAWK_DEVICE, PIXHAWK_BAUD)
    print("Waiting for Pixhawk heartbeat...")
    pixhawk.wait_heartbeat()
    print("Pixhawk connected.")

    thrusters = ThrusterState()
    telemetry = {
        "pressure": 1013.25,
        "timestamp": time.time(),
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
    }
    telemetry_lock = threading.Lock()

    running.set()
    threads = [
        threading.Thread(target=sensor_sender, args=(pixhawk, telemetry, telemetry_lock), daemon=True),
        threading.Thread(target=command_receiver, args=(pixhawk, thrusters), daemon=True),
        threading.Thread(target=video_stream_loop, daemon=True),
    ]

    try:
        pixhawk.set_passthrough()
        pixhawk.request_streams(STREAM_RATE_HZ)
        pixhawk.arm()
        stop_all_thrusters(pixhawk, thrusters)

        for t in threads:
            t.start()

        link_lost = True
        print("Waiting for base station commands...")
        while True:
            lost = watchdog_step(pixhawk, thrusters)
            if lost:
                if not link_lost:
                    print("\nWarning: Connection lost. Idling (neutral)...")
            else:
                p = thrusters.snapshot()
                with telemetry_lock:
                    pressure = telemetry["pressure"]
                dashboard = " ".join(f"T{i}:{p[f't{i}']}" for i in range(1, 9))
                print(f"{dashboard} | P:{pressure:.2f}".ljust(100), end="\r", flush=True)
            link_lost = lost

            time.sleep(1.0 / WATCHDOG_RATE_HZ)

    except KeyboardInterrupt:
        print("\nShutting down.")
    finally:
        running.clear()
        stop_all_thrusters(pixhawk, thrusters)
        pixhawk.disarm()
        for t in threads:
            if t.is_alive():
                t.join(timeout=2.0)
        pixhawk.close()


if __name__ == "__main__":
    main()
