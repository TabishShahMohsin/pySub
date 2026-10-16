"""Pi program tested against a fake Pixhawk.

FakeMaster records every RC override and flags any concurrent access, which
would corrupt the real (non-thread-safe) MAVLink connection.
"""

import contextlib
import json
import socket
import threading
import time

import pytest


class Msg:
    def __init__(self, kind, **fields):
        self._kind = kind
        self.__dict__.update(fields)

    def get_type(self):
        return self._kind


class FakeMav:
    def __init__(self, master):
        self.master = master

    def rc_channels_override_send(self, target_system, target_component, *pwms):
        with self.master.guard():
            self.master.rc.append(list(pwms))

    def command_long_send(self, *args):
        with self.master.guard():
            self.master.commands.append(args)

    def param_set_send(self, *args):
        with self.master.guard():
            pass

    def request_data_stream_send(self, *args):
        with self.master.guard():
            pass


class FakeMaster:
    """Stands in for a pymavlink connection and detects concurrent use.

    Like pymavlink's recv_match, filtering by type *discards* the non-matching
    messages it reads.
    """

    target_system = 1
    target_component = 1

    def __init__(self, pending=()):
        self._busy = threading.Lock()
        self.concurrent_access = False
        self.rc = []
        self.commands = []
        self.pending = list(pending)
        self.mav = FakeMav(self)

    @contextlib.contextmanager
    def guard(self):
        acquired = self._busy.acquire(blocking=False)
        if not acquired:
            self.concurrent_access = True
        try:
            time.sleep(0.0005)  # Widen any race window
            yield
        finally:
            if acquired:
                self._busy.release()

    def recv_match(self, type=None, blocking=False):
        with self.guard():
            while self.pending:
                msg = self.pending.pop(0)
                if type is None or msg.get_type() in type:
                    return msg
            return None

    def set_mode(self, mode):
        with self.guard():
            pass

    def close(self):
        pass


def make_pixhawk(pi_main, pending=()):
    """A real Pixhawk wrapper around a fake MAVLink connection."""
    px = pi_main.Pixhawk.__new__(pi_main.Pixhawk)
    px.master = FakeMaster(pending)
    px.lock = threading.Lock()
    return px


@pytest.fixture
def running(pi_main):
    pi_main.running.set()
    yield pi_main.running
    pi_main.running.clear()


def test_mapping_follows_thruster_map(pi_main):
    thrusters = pi_main.ThrusterState()
    for i in range(1, 9):
        thrusters.pwms[f"t{i}"] = 1500 + i
    out = thrusters.mapped_pwm_list()
    for logical, physical in pi_main.THRUSTER_MAP.items():
        assert out[physical - 1] == 1500 + logical
    assert sorted(pi_main.THRUSTER_MAP.values()) == list(range(1, 9))


def test_watchdog_goes_neutral_only_after_timeout(pi_main):
    px, thrusters = make_pixhawk(pi_main), pi_main.ThrusterState()
    assert pi_main.watchdog_step(px, thrusters, now=0.0) is True  # No command yet

    thrusters.pwms["t1"] = 1700
    thrusters.last_command_time = 100.0
    assert pi_main.watchdog_step(px, thrusters, now=100.5) is False
    assert thrusters.pwms["t1"] == 1700

    px.master.rc.clear()
    assert pi_main.watchdog_step(px, thrusters, now=100.0 + pi_main.COMMAND_TIMEOUT_S + 0.01) is True
    assert set(thrusters.pwms.values()) == {pi_main.PWM_NEUTRAL}
    assert px.master.rc == [[pi_main.PWM_NEUTRAL] * 8]


def test_command_receiver_clamps_maps_and_ignores_bad_packets(pi_main, running, udp_port):
    px, thrusters = make_pixhawk(pi_main), pi_main.ThrusterState()
    receiver = threading.Thread(target=pi_main.command_receiver, args=(px, thrusters, udp_port), daemon=True)
    receiver.start()
    time.sleep(0.2)

    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        tx.sendto(b"garbage", ("127.0.0.1", udp_port))
        tx.sendto(json.dumps({"t1": "fast"}).encode(), ("127.0.0.1", udp_port))
        cmd = {f"t{i}": 1500 + 10 * i for i in range(1, 9)}
        cmd.update({"t1": 9999, "t2": -5, "bogus": 1600})
        tx.sendto(json.dumps(cmd).encode(), ("127.0.0.1", udp_port))

        deadline = time.monotonic() + 2
        while not px.master.rc and time.monotonic() < deadline:
            time.sleep(0.01)
    finally:
        tx.close()
        running.clear()
        receiver.join(timeout=2)

    assert not receiver.is_alive()
    assert len(px.master.rc) == 1  # Bad packets were dropped
    assert thrusters.pwms["t1"] == pi_main.PWM_MAX
    assert thrusters.pwms["t2"] == pi_main.PWM_MIN
    assert "bogus" not in thrusters.pwms
    assert px.master.rc[0] == thrusters.mapped_pwm_list()
    assert thrusters.last_command_time is not None


def test_sensor_sender_drains_all_message_types(pi_main, running, udp_port):
    pending = [
        Msg("HEARTBEAT"),
        Msg("ATTITUDE", roll=0.0, pitch=0.1, yaw=0.0),
        Msg("SCALED_PRESSURE2", press_abs=1100.0),
        Msg("ATTITUDE", roll=0.5, pitch=0.0, yaw=-0.5),  # Latest wins
    ]
    px = make_pixhawk(pi_main, pending)
    telemetry = {"pressure": 1013.25, "timestamp": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
    lock = threading.Lock()

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", udp_port))
    rx.settimeout(2)
    sender = threading.Thread(
        target=pi_main.sensor_sender, args=(px, telemetry, lock, "127.0.0.1", udp_port), daemon=True
    )
    sender.start()
    try:
        packet = json.loads(rx.recv(4096))
        count, t0 = 0, time.monotonic()
        while time.monotonic() - t0 < 1.0:
            rx.recv(4096)
            count += 1
    finally:
        running.clear()
        sender.join(timeout=2)
        rx.close()

    assert packet["pressure"] == 1100.0
    assert packet["roll"] == pytest.approx(28.6479, abs=1e-3)
    assert packet["yaw"] == pytest.approx(-28.6479, abs=1e-3)
    assert 17 <= count <= 23, f"expected ~20 Hz, got {count}"


def test_receiver_and_watchdog_never_access_mavlink_concurrently(pi_main, running, udp_port):
    px, thrusters = make_pixhawk(pi_main), pi_main.ThrusterState()
    receiver = threading.Thread(target=pi_main.command_receiver, args=(px, thrusters, udp_port), daemon=True)
    receiver.start()
    time.sleep(0.2)

    stop = threading.Event()

    def hammer_watchdog():
        while not stop.is_set():
            pi_main.watchdog_step(px, thrusters)
            px.drain(("ATTITUDE",))

    hammer = threading.Thread(target=hammer_watchdog, daemon=True)
    hammer.start()
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        for _ in range(200):
            tx.sendto(json.dumps({"t1": 1600}).encode(), ("127.0.0.1", udp_port))
            time.sleep(0.002)
    finally:
        stop.set()
        hammer.join(timeout=2)
        tx.close()
        running.clear()
        receiver.join(timeout=2)

    assert not px.master.concurrent_access


def test_video_loop_exits_cleanly_without_cameras(pi_main, monkeypatch, tcp_port):
    monkeypatch.setattr(pi_main, "start_realsense", lambda: None)
    monkeypatch.setattr(pi_main, "open_usb_camera", lambda: None)
    pi_main.running.set()
    loop = threading.Thread(target=pi_main.video_stream_loop, args=("127.0.0.1", tcp_port), daemon=True)
    loop.start()
    time.sleep(0.3)
    pi_main.running.clear()
    loop.join(timeout=5)
    assert not loop.is_alive()
