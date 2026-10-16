"""Branch-level tests for base_station/main.py and input_handler.py."""

import importlib
import json
import socket as real_socket
import types

import numpy as np
import pygame
import pytest
import zmq

import config
import input_handler
from fakes import FakeTime, ScriptedSocket, fake_time_module, socket_module
from pid import PID


@pytest.fixture
def clock(base_main, monkeypatch):
    fake = FakeTime()
    monkeypatch.setattr(base_main, "time", fake_time_module(fake))
    return fake


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
GOOD = {"timestamp": 1.0, "pressure": 1013.25, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -float("inf")])
@pytest.mark.parametrize("key", list(GOOD))
def test_parse_telemetry_rejects_non_finite(base_main, key, bad):
    with pytest.raises(ValueError):
        base_main.parse_telemetry({**GOOD, key: bad})


@pytest.mark.parametrize(
    ("raw", "error"),
    [({"timestamp": 1.0}, KeyError), ([1, 2, 3], TypeError), ({**GOOD, "roll": "abc"}, ValueError)],
)
def test_parse_telemetry_rejects_malformed(base_main, raw, error):
    with pytest.raises(error):
        base_main.parse_telemetry(raw)


def test_resolve_host(base_main):
    assert base_main.resolve_host("127.0.0.1") == "127.0.0.1"
    assert base_main.resolve_host("no-such-host.invalid") is None


def test_render_dashboard(base_main):
    text = base_main.render_dashboard(29.7, False, GOOD, 1.25, (1.0, 2.0, 3.0, 4.0), [0.5] * 8, [1550] * 8)
    assert "NO TELEMETRY" in text
    assert "T8:  0.50(1550)" in text
    assert "29.7 FPS" in text
    assert "OK" in base_main.render_dashboard(30, True, GOOD, 0, (0, 0, 0, 0), [0] * 8, [1500] * 8)


# ---------------------------------------------------------------------------
# video_receiver
# ---------------------------------------------------------------------------
def test_video_receiver_disabled_when_port_taken(base_main, monkeypatch, capsys):
    def busy(open_port):
        raise zmq.ZMQError(98, "Address already in use")

    monkeypatch.setattr(base_main.imagezmq, "ImageHub", busy)
    base_main.video_receiver(port=1)
    assert "Video receiver disabled" in capsys.readouterr().out


def test_video_receiver_survives_errors(base_main, monkeypatch, clock):
    frame = np.zeros((2, 2, 3), np.uint8)

    class Hub:
        def __init__(self, open_port):
            self.script = [zmq.Again(), RuntimeError("corrupt frame"), ("cam", frame)]
            self.replies = []
            self.closed = False
            self.zmq_socket = types.SimpleNamespace(setsockopt=lambda *a: None)
            hubs.append(self)

        def recv_image(self):
            item = self.script.pop(0)
            if isinstance(item, Exception):
                raise item
            base_main.state.running.clear()  # Stop after this frame
            return item

        def send_reply(self, msg):
            self.replies.append(msg)

        def close(self):
            self.closed = True

    hubs = []
    monkeypatch.setattr(base_main.imagezmq, "ImageHub", Hub)
    base_main.video_receiver()

    assert hubs[0].replies == [b"OK"]
    assert hubs[0].closed
    assert clock.sleeps == [1]  # Back-off after the error, not after Again
    assert base_main.state.get_frames()["cam"] is frame


# ---------------------------------------------------------------------------
# command_sender
# ---------------------------------------------------------------------------
def test_command_sender_resolution_errors_and_resync(base_main, monkeypatch, clock, capsys):
    sock = ScriptedSocket(send_errors=[None, OSError("unreachable"), None, None, OSError("down")])
    monkeypatch.setattr(base_main, "socket", socket_module(real_socket, sock))

    hosts = iter([None, None, "10.0.0.2", "10.0.0.3"])
    monkeypatch.setattr(base_main, "resolve_host", lambda h: next(hosts))

    def sleep(seconds):
        clock.sleeps.append(seconds)
        clock.now += seconds
        if len(clock.sleeps) == 4:
            clock.now += 1.0  # Fall behind once: must resync instead of bursting
        if len(clock.sleeps) == 5:
            base_main.state.running.clear()

    monkeypatch.setattr(base_main.time, "sleep", sleep)
    base_main.state.set_pwms([1600] * 8)
    base_main.command_sender(host="pi", port=7000)

    out = capsys.readouterr().out
    assert out.count("cannot resolve") == 1  # Warned once, not every retry
    assert clock.sleeps[:2] == [1.0, 1.0]
    sent = [(json.loads(p), addr) for p, addr in sock.sent]
    assert sent[0][1] == ("10.0.0.2", 7000)
    assert all(addr == ("10.0.0.3", 7000) for _, addr in sent[1:])  # Re-resolved after the error
    # Shutdown: the neutral flush stopped at the first send error
    assert sent[-1][0]["t1"] == 1600
    assert sock.closed


def test_command_sender_flushes_neutral_on_shutdown(base_main, monkeypatch, clock):
    sock = ScriptedSocket()
    monkeypatch.setattr(base_main, "socket", socket_module(real_socket, sock))
    monkeypatch.setattr(base_main, "resolve_host", lambda h: "10.0.0.2")
    monkeypatch.setattr(base_main.time, "sleep", lambda s: base_main.state.running.clear())

    base_main.command_sender()

    neutral = [json.loads(p) for p, _ in sock.sent[1:]]
    assert len(neutral) == 3 and all(set(n.values()) == {1500} for n in neutral)


def test_command_sender_without_resolved_host_sends_nothing(base_main, monkeypatch, clock):
    sock = ScriptedSocket()
    monkeypatch.setattr(base_main, "socket", socket_module(real_socket, sock))
    monkeypatch.setattr(base_main, "resolve_host", lambda h: None)
    monkeypatch.setattr(base_main.time, "sleep", lambda s: base_main.state.running.clear())

    base_main.command_sender()
    assert sock.sent == [] and sock.closed


# ---------------------------------------------------------------------------
# telemetry_listener
# ---------------------------------------------------------------------------
def test_telemetry_listener_bind_error(base_main, monkeypatch, capsys):
    sock = ScriptedSocket(bind_error=OSError("in use"))
    monkeypatch.setattr(base_main, "socket", socket_module(real_socket, sock))
    base_main.telemetry_listener()
    assert "Binding error" in capsys.readouterr().out


def test_telemetry_listener_socket_error_backs_off(base_main, monkeypatch, clock):
    sock = ScriptedSocket(
        recv=[OSError("reset"), json.dumps(GOOD).encode(), b'{"timestamp": NaN}'],
        on_empty=base_main.state.running.clear,
    )
    monkeypatch.setattr(base_main, "socket", socket_module(real_socket, sock))
    base_main.telemetry_listener()
    assert clock.sleeps == [0.5]
    assert base_main.state.get_telemetry()[1] == 1  # Only the good packet
    assert sock.closed


# ---------------------------------------------------------------------------
# main()
# ---------------------------------------------------------------------------
class FakeClock:
    def __init__(self, fake_time):
        self.fake_time = fake_time

    def tick(self, rate):
        ms = 1000 / rate
        self.fake_time.now += ms / 1000
        return ms

    def get_fps(self):
        return 30.0


class FakeCV2:
    def __init__(self, keys=()):
        self.shown = []
        self.keys = list(keys)
        self.destroyed = False

    def imshow(self, name, frame):
        self.shown.append((name, frame))

    def waitKey(self, delay):
        return self.keys.pop(0) if self.keys else -1

    def destroyAllWindows(self):
        self.destroyed = True


class ScriptedController:
    """Returns fixed stick inputs and runs a per-frame hook (for telemetry)."""

    def __init__(self, inputs, hook, frames):
        self.inputs = np.array(inputs, dtype=float)
        self.hook = hook
        self.frames = frames
        self.calls = 0

    def get_input_vector(self):
        self.calls += 1
        self.hook(self.calls)
        return self.inputs.copy()


@pytest.fixture
def run_main(base_main, monkeypatch):
    """Run base_main.main() for a scripted number of frames on a virtual clock."""

    def run(
        frames,
        inputs=(0, 0, 0, 0, 0, 0),
        hook=lambda i: None,
        keys=(),
        flags=None,
        controller_error=None,
        dashboard_hz=1e9,
    ):
        fake_time = FakeTime()
        monkeypatch.setattr(base_main, "time", fake_time_module(fake_time))
        monkeypatch.setattr(base_main.pygame.time, "Clock", lambda: FakeClock(fake_time))
        monkeypatch.setattr(base_main, "DASHBOARD_RATE_HZ", dashboard_hz)

        cv2 = FakeCV2(keys)
        monkeypatch.setattr(base_main, "cv2", cv2)

        controller = ScriptedController(inputs, hook, frames)

        def make_controller(deadzone):
            if controller_error is not None:
                raise controller_error
            return controller

        monkeypatch.setattr(base_main, "JoystickController", make_controller)

        quit_event = types.SimpleNamespace(type=pygame.QUIT)
        other_event = types.SimpleNamespace(type=pygame.KEYDOWN)
        monkeypatch.setattr(
            base_main.pygame.event,
            "get",
            lambda: [other_event, quit_event] if controller.calls >= frames - 1 else [other_event],
        )

        started = []

        def idle(*args, **kwargs):
            started.append(True)

        for name in ("telemetry_listener", "command_sender", "video_receiver"):
            monkeypatch.setattr(base_main, name, idle)

        for name, value in (flags or {}).items():
            monkeypatch.setattr(base_main, name, value)

        base_main.main()
        return types.SimpleNamespace(cv2=cv2, controller=controller, started=started, time=fake_time)

    return run


ALL_OFF = {"DEPTH_PID": False, "ROLL_PID": False, "PITCH_PID": False, "YAW_PID": False, "DEPTH_KF": False}
ALL_ON = {"DEPTH_PID": True, "ROLL_PID": True, "PITCH_PID": True, "YAW_PID": True, "DEPTH_KF": True}


def test_main_exits_cleanly_without_controller(run_main, base_main, capsys):
    result = run_main(1, controller_error=RuntimeError("Error: No Xbox controller detected."))
    assert "No Xbox controller" in capsys.readouterr().out
    assert result.started == []


def test_main_manual_mode_and_quit_key(run_main, base_main, capsys):
    frame = np.arange(12, dtype=np.uint8).reshape(2, 2, 3)
    base_main.state.put_frame("cam", frame)
    base_main.state.put_frame("dead", None)
    pwms_seen = []

    def hook(i):
        pwms_seen.append(base_main.state.get_pwms())

    result = run_main(100, inputs=(1, 0, 0, 0, 0, 0), hook=hook, keys=[-1, -1, ord("q")], flags=ALL_OFF)

    assert result.controller.calls == 3  # Stopped by the q key, not the frame limit
    assert len(result.started) == 3
    assert pwms_seen[-1] != [1500] * 8  # Full surge was being sent
    assert base_main.state.get_pwms() == [1500] * 8  # Neutral on exit
    assert not base_main.state.running.is_set()
    assert result.cv2.destroyed
    assert [name for name, _ in result.cv2.shown] == ["cam"] * 3  # None frames skipped
    assert np.array_equal(result.cv2.shown[0][1], frame[::-1])
    out = capsys.readouterr().out
    assert "NO TELEMETRY" in out and "Base station exited" in out


def test_main_neutral_even_if_loop_crashes(run_main, base_main):
    def hook(i):
        if i == 2:
            raise RuntimeError("controller unplugged")

    with pytest.raises(RuntimeError):
        run_main(10, inputs=(1, 0, 0, 0, 0, 0), hook=hook, flags=ALL_OFF)
    assert base_main.state.get_pwms() == [1500] * 8
    assert not base_main.state.running.is_set()


def test_main_closed_loop_control(run_main, base_main, monkeypatch):
    """PIDs + KF on; telemetry at 20 Hz, a 2 s outage, then recovery."""
    pid_calls = []
    predicts = []

    class SpyPID(PID):
        def compute(self, measurement, setpoint, dt):
            pid_calls.append((id(self), dt))
            return super().compute(measurement, setpoint, dt)

    class SpyKF(base_main.DepthKalmanFilter):
        def predict(self, dt):
            predicts.append(frame_no[0])
            return super().predict(dt)

    monkeypatch.setattr(base_main, "PID", SpyPID)
    monkeypatch.setattr(base_main, "DepthKalmanFilter", SpyKF)

    frame_no = [0]
    outage = range(20, 80)  # 60 frames = 2 s without telemetry
    snapshots = {}
    samples = []

    def hook(i):
        frame_no[0] = i
        snapshots[i] = list(base_main.state.get_pwms())
        if i not in outage and i % 3 != 0:  # ~20 Hz of new samples at 30 Hz
            samples.append(i)
            base_main.state.update_telemetry(
                {"timestamp": i, "pressure": 1013.25 + 50, "roll": 10.0, "pitch": -5.0, "yaw": 170.0}
            )

    run_main(100, inputs=(0, 0, 1, 1, 1, 1), hook=hook, flags=ALL_ON)

    # PIDs ran once per new sample (4 PIDs), never on repeated samples
    assert len(pid_calls) == 4 * len(samples)
    # After the outage the first sample uses one frame as dt, not the 2 s gap
    assert max(dt for _, dt in pid_calls) < 0.2
    # The KF holds its estimate while the link is down
    lost_frames = [i for i in predicts if 20 + 31 < i < 80]
    assert lost_frames == []
    # PID output (not the raw stick) drives the thrusters while connected
    assert snapshots[15] != snapshots[60]
    assert base_main.state.get_pwms() == [1500] * 8


# ---------------------------------------------------------------------------
# input_handler
# ---------------------------------------------------------------------------
class FakePygameJoystick:
    def __init__(self, index):
        self.initialised = False

    def init(self):
        self.initialised = True

    def get_name(self):
        return "Fake Pad"

    def get_axis(self, i):
        return [0.0, 0.0, -1.0, 0.0, 0.0, -1.0][i]

    def get_button(self, i):
        return {4: 1, 9: 1}.get(i, 0)  # Xbox LB / PS L1 held


@pytest.mark.parametrize("cls", [input_handler.XboxController, input_handler.PSController])
def test_gamepad_construction_and_down_buttons(monkeypatch, cls):
    monkeypatch.setattr(pygame.joystick, "init", lambda: None)
    monkeypatch.setattr(pygame.joystick, "get_count", lambda: 1)
    monkeypatch.setattr(pygame.joystick, "Joystick", FakePygameJoystick)
    pad = cls(deadzone=0.1)
    assert pad.joystick.initialised
    assert pad.get_input_vector()[2] == -1.0  # Heave down


def test_gamepad_missing_raises(monkeypatch):
    monkeypatch.setattr(pygame.joystick, "init", lambda: None)
    monkeypatch.setattr(pygame.joystick, "get_count", lambda: 0)
    with pytest.raises(RuntimeError, match="No PS controller"):
        input_handler.PSController()


@pytest.mark.parametrize(
    ("setting", "expected"),
    [
        ("xbox", "XboxController"),
        ("PS", "PSController"),
        ("keyboard", "KeyboardController"),
        ("wii", "KeyboardController"),
    ],
)
def test_controller_selection(monkeypatch, capsys, setting, expected):
    monkeypatch.setattr(config, "CONTROLLER_TYPE", setting)
    try:
        module = importlib.reload(input_handler)
        assert module.JoystickController.__name__ == expected
        assert ("Unknown controller" in capsys.readouterr().out) == (setting == "wii")
    finally:
        monkeypatch.undo()
        importlib.reload(input_handler)


def test_main_manual_mode_with_live_telemetry(run_main, base_main, capsys):
    """Link up but PIDs off: sticks drive the thrusters; dashboard is throttled."""

    def hook(i):
        base_main.state.update_telemetry({**GOOD, "pressure": 1013.25 + 100})

    run_main(12, inputs=(0, 0, 0.5, 0, 0, 0), hook=hook, flags=ALL_OFF, dashboard_hz=10)
    out = capsys.readouterr().out
    # 12 frames at 30 Hz = 0.4 s -> about 4 dashboards at 10 Hz, not 12
    assert 3 <= out.count("DASHBOARD") <= 5
    assert "Link: OK" in out
