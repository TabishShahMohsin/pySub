"""Branch-level tests for pi/main.py using fakes for sockets, time, cameras and MAVLink."""

import json
import threading
import types

import numpy as np
import pytest
import zmq

from fakes import FakeMaster, FakeTime, Msg, ScriptedSocket, fake_time_module, patch_mavlink, socket_module


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def make_pixhawk(pi_main, master=None):
    px = pi_main.Pixhawk.__new__(pi_main.Pixhawk)
    px.master = master or FakeMaster()
    px.lock = threading.Lock()
    return px


@pytest.fixture
def clock(pi_main, monkeypatch):
    fake = FakeTime()
    monkeypatch.setattr(pi_main, "time", fake_time_module(fake))
    return fake


@pytest.fixture
def running(pi_main):
    pi_main.running.set()
    yield pi_main.running
    pi_main.running.clear()


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------
def test_resolve_host(pi_main):
    assert pi_main.resolve_host("127.0.0.1") == "127.0.0.1"
    assert pi_main.resolve_host("no-such-host.invalid") is None


@pytest.mark.parametrize(
    ("value", "expected"),
    [(1600, 1600), (1600.7, 1600), (9999, 1850), (-3, 1150), (1150, 1150), (1850, 1850)],
)
def test_parse_pwm_clamps(pi_main, value, expected):
    assert pi_main.parse_pwm(value) == expected


@pytest.mark.parametrize("value", [True, False, "1600", None, float("nan"), float("inf"), -float("inf"), [1]])
def test_parse_pwm_rejects_invalid(pi_main, value):
    with pytest.raises(ValueError):
        pi_main.parse_pwm(value)


def test_stop_all_thrusters(pi_main, capsys):
    px, thrusters = make_pixhawk(pi_main), pi_main.ThrusterState()
    thrusters.pwms["t3"] = 1700
    pi_main.stop_all_thrusters(px, thrusters)
    assert "STOPPING" in capsys.readouterr().out
    pi_main.stop_all_thrusters(px, thrusters, verbose=False)
    assert capsys.readouterr().out == ""
    assert thrusters.snapshot() == {f"t{i}": 1500 for i in range(1, 9)}
    assert px.master.rc == [[1500] * 8] * 2


def test_watchdog_uses_monotonic_clock_by_default(pi_main, clock):
    px, thrusters = make_pixhawk(pi_main), pi_main.ThrusterState()
    thrusters.last_command_time = clock.monotonic()
    assert pi_main.watchdog_step(px, thrusters) is False
    clock.sleep(pi_main.COMMAND_TIMEOUT_S + 0.01)
    assert pi_main.watchdog_step(px, thrusters) is True


# ---------------------------------------------------------------------------
# Pixhawk wrapper
# ---------------------------------------------------------------------------
def test_pixhawk_wrapper_commands(pi_main, monkeypatch, clock):
    master = patch_mavlink(
        monkeypatch, FakeMaster([Msg("ATTITUDE"), Msg("HEARTBEAT"), Msg("SCALED_PRESSURE2")])
    )
    px = pi_main.Pixhawk("/dev/null", 115200)

    px.wait_heartbeat()
    px.set_passthrough()
    px.request_streams(25)
    px.arm()
    assert master.armed
    px.send_rc([1600] * 8)
    msgs = px.drain(("ATTITUDE", "SCALED_PRESSURE2"))
    px.disarm()
    px.close()

    assert master.heartbeat_waited
    assert master.params == {f"SERVO{i}_FUNCTION": 50 + i for i in range(1, 9)}
    assert master.streams[0][3] == 25
    assert master.modes == ["MANUAL"]
    assert [m.get_type() for m in msgs] == ["ATTITUDE", "SCALED_PRESSURE2"]
    assert master.rc == [[1600] * 8]
    assert not master.armed
    assert master.closed
    # Parameter writes are spaced out, arming waits for the mode change
    assert clock.sleeps.count(0.05) == 8 and 0.5 in clock.sleeps


def test_drain_stops_at_max_messages(pi_main):
    px = make_pixhawk(pi_main, FakeMaster([Msg("ATTITUDE")] * 10))
    assert len(px.drain(("ATTITUDE",), max_messages=3)) == 3
    assert len(px.master.pending) == 7


# ---------------------------------------------------------------------------
# command_receiver
# ---------------------------------------------------------------------------
def test_command_receiver_survives_socket_errors_and_bad_packets(pi_main, monkeypatch, running, clock):
    import socket as real_socket

    sock = ScriptedSocket(
        recv=[
            TimeoutError(),
            OSError("network down"),
            b"[1, 2, 3]",  # Not an object
            b'{"t1": Infinity}',  # Used to kill the thread with OverflowError
            b'{"t1": true}',
            b'{"t1": "1600"}',
            b'{"t1": 1700, "t9": 1800}',
        ],
        on_empty=running.clear,
    )
    monkeypatch.setattr(pi_main, "socket", socket_module(real_socket, sock))
    px, thrusters = make_pixhawk(pi_main), pi_main.ThrusterState()

    pi_main.command_receiver(px, thrusters, port=6000)

    assert sock.bound == ("0.0.0.0", 6000)
    assert sock.closed
    assert 0.5 in clock.sleeps  # Back-off after the socket error
    assert px.master.rc == [thrusters.mapped_pwm_list()]
    assert thrusters.pwms["t1"] == 1700
    assert "t9" not in thrusters.pwms


def test_command_receiver_does_not_override_final_neutral(pi_main, monkeypatch, running):
    import socket as real_socket

    def packet_arriving_during_shutdown():
        running.clear()
        return b'{"t1": 1700}'

    sock = ScriptedSocket(recv=[packet_arriving_during_shutdown])
    monkeypatch.setattr(pi_main, "socket", socket_module(real_socket, sock))
    px, thrusters = make_pixhawk(pi_main), pi_main.ThrusterState()

    pi_main.command_receiver(px, thrusters)

    assert px.master.rc == []
    assert thrusters.pwms["t1"] == 1500
    assert sock.closed


# ---------------------------------------------------------------------------
# sensor_sender
# ---------------------------------------------------------------------------
def test_sensor_sender_retries_resolution_and_send_errors(pi_main, monkeypatch, running, clock):
    import socket as real_socket

    sock = ScriptedSocket(send_errors=[OSError("unreachable"), None, None])
    monkeypatch.setattr(pi_main, "socket", socket_module(real_socket, sock))

    hosts = iter([None, "10.0.0.2", "10.0.0.3"])
    monkeypatch.setattr(pi_main, "resolve_host", lambda host: next(hosts))

    px = make_pixhawk(pi_main, FakeMaster([Msg("SCALED_PRESSURE2", press_abs=1050.0)]))
    telemetry = {"pressure": 0.0, "timestamp": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}

    real_sleep = clock.sleep

    def sleep(seconds):
        real_sleep(seconds)
        if len(clock.sleeps) == 2:
            clock.now += 1.0  # Fall behind schedule once: must resync, not burst
        if len(clock.sleeps) >= 4:
            running.clear()

    monkeypatch.setattr(pi_main.time, "sleep", sleep)
    pi_main.sensor_sender(px, telemetry, threading.Lock(), host="base", port=7000)

    # Cycle 1: unresolved, nothing sent. Cycle 2: send fails -> re-resolve.
    # Later cycles: sent to the newly resolved address.
    assert [addr for _, addr in sock.sent] == [("10.0.0.3", 7000)] * len(sock.sent)
    assert len(sock.sent) >= 1
    assert json.loads(sock.sent[0][0])["pressure"] == 1050.0
    assert sock.closed


# ---------------------------------------------------------------------------
# Cameras
# ---------------------------------------------------------------------------
def test_start_realsense_without_sdk(pi_main, monkeypatch):
    monkeypatch.setattr(pi_main, "rs", None)
    assert pi_main.start_realsense() is None


def test_start_realsense_success_and_failure(pi_main, monkeypatch):
    started = []

    class Pipeline:
        def start(self, cfg):
            started.append(cfg)

    class Config:
        def enable_stream(self, *args):
            self.args = args

    fake_rs = types.SimpleNamespace(
        pipeline=Pipeline,
        config=Config,
        stream=types.SimpleNamespace(color="color"),
        format=types.SimpleNamespace(bgr8="bgr8"),
    )
    monkeypatch.setattr(pi_main, "rs", fake_rs)
    assert isinstance(pi_main.start_realsense(), Pipeline)
    assert started[0].args == ("color", 640, 480, "bgr8", 30)

    def broken():
        raise RuntimeError("No device connected")

    monkeypatch.setattr(fake_rs, "pipeline", broken)
    assert pi_main.start_realsense() is None


class FakeCapture:
    def __init__(self, opened=True, frames=()):
        self.opened = opened
        self.frames = list(frames)
        self.released = False
        self.props = {}

    def isOpened(self):
        return self.opened

    def set(self, prop, value):
        self.props[prop] = value

    def read(self):
        if self.frames:
            return True, self.frames.pop(0)
        return False, None

    def release(self):
        self.released = True


def test_open_usb_camera(pi_main, monkeypatch):
    good = FakeCapture(opened=True)
    monkeypatch.setattr(pi_main.cv2, "VideoCapture", lambda index: good)
    assert pi_main.open_usb_camera() is good
    assert good.props == {pi_main.cv2.CAP_PROP_BUFFERSIZE: 1}

    bad = FakeCapture(opened=False)
    monkeypatch.setattr(pi_main.cv2, "VideoCapture", lambda index: bad)
    assert pi_main.open_usb_camera() is None
    assert bad.released


# ---------------------------------------------------------------------------
# video_stream_loop
# ---------------------------------------------------------------------------
class FakeSender:
    instances = []

    def __init__(self, connect_to, fail_on=None, close_error=False, on_send=None):
        self.connect_to = connect_to
        self.sent = []
        self.options = {}
        self.fail_on = fail_on  # Raise zmq.Again on this send number
        self.close_error = close_error
        self.on_send = on_send  # Called with the send count after every send
        self.closed = False
        self.zmq_socket = types.SimpleNamespace(setsockopt=self.options.__setitem__)
        FakeSender.instances.append(self)

    def send_image(self, name, image):
        self.sent.append(name)
        if self.fail_on is not None and len(self.sent) == self.fail_on:
            raise zmq.Again()
        if self.on_send is not None:
            self.on_send(len(self.sent))

    def close(self):
        self.closed = True
        if self.close_error:
            raise RuntimeError("close failed")


def stop_on_retry_delay(pi_main, monkeypatch, clock):
    """Stop the loop when it starts its 3 s retry delay."""

    def sleep(seconds):
        clock.sleep(seconds)
        if seconds == 3:
            pi_main.running.clear()

    monkeypatch.setattr(pi_main.time, "sleep", sleep)


class Frame:
    def __init__(self, data):
        self.data = data

    def get_data(self):
        return self.data

    def __bool__(self):
        return self.data is not None


class FrameSet:
    def __init__(self, color):
        self.color = color

    def get_color_frame(self):
        return Frame(self.color)

    def __bool__(self):
        return True


class FakePipeline:
    def __init__(self, polls, stop_error=False):
        self.polls = list(polls)
        self.stop_error = stop_error
        self.stopped = False

    def poll_for_frames(self):
        return self.polls.pop(0) if self.polls else FrameSet(np.zeros((2, 2, 3), np.uint8))

    def stop(self):
        self.stopped = True
        if self.stop_error:
            raise RuntimeError("stop failed")


def run_video_loop(pi_main, monkeypatch, sender_kwargs, pipeline=None, usb=None, hosts=("127.0.0.1",)):
    FakeSender.instances = []
    monkeypatch.setattr(
        pi_main.imagezmq, "ImageSender", lambda connect_to: FakeSender(connect_to, **sender_kwargs)
    )
    monkeypatch.setattr(pi_main, "start_realsense", lambda: pipeline)
    monkeypatch.setattr(pi_main, "open_usb_camera", lambda: usb)
    host_iter = iter(hosts)
    monkeypatch.setattr(pi_main, "resolve_host", lambda h: next(host_iter, "127.0.0.1"))
    pi_main.running.set()
    pi_main.video_stream_loop(host="base", port=6555)


def test_video_streams_both_cameras_until_camera_fails(pi_main, monkeypatch, clock):
    stop_on_retry_delay(pi_main, monkeypatch, clock)
    pipeline = FakePipeline([None, FrameSet(None), FrameSet(np.ones((2, 2, 3), np.uint8))])
    usb = FakeCapture(frames=[np.zeros((2, 2, 3), np.uint8)] * 10)
    run_video_loop(pi_main, monkeypatch, {}, pipeline=pipeline, usb=usb)

    # The USB camera runs out of frames -> error -> cleanup -> retry delay
    sender = FakeSender.instances[0]
    assert sender.connect_to == "tcp://127.0.0.1:6555"
    assert sender.options == {zmq.RCVTIMEO: pi_main.VIDEO_REPLY_TIMEOUT_MS, zmq.LINGER: 0}
    # Polls 1-2 have no usable frame, then one RealSense frame per loop
    assert sender.sent.count(f"{pi_main.HOSTNAME}_realsense") == 9
    assert sender.sent.count("usb_cam") == 10
    assert pipeline.stopped and usb.released and sender.closed
    assert len(FakeSender.instances) == 1


def test_video_reconnects_when_base_station_stops_replying(pi_main, monkeypatch, clock):
    pipeline = FakePipeline([], stop_error=True)

    def stop():
        pi_main.running.clear()

    # First sender times out (zmq.Again); the reconnect then streams until stopped.
    calls = iter([{"fail_on": 1, "close_error": True}, {"on_send": lambda n: stop()}])
    FakeSender.instances = []
    monkeypatch.setattr(
        pi_main.imagezmq, "ImageSender", lambda connect_to: FakeSender(connect_to, **next(calls))
    )
    monkeypatch.setattr(pi_main, "start_realsense", lambda: pipeline)
    monkeypatch.setattr(pi_main, "open_usb_camera", lambda: None)
    monkeypatch.setattr(pi_main, "resolve_host", lambda h: "127.0.0.1")
    pi_main.running.set()

    pi_main.video_stream_loop()

    assert len(FakeSender.instances) == 2
    assert 3 in clock.sleeps  # Retry delay after the timeout
    assert 0.005 in clock.sleeps  # RealSense-only loop doesn't busy-spin
    assert all(s.closed for s in FakeSender.instances)


def test_video_retries_when_host_unresolved_or_no_cameras(pi_main, monkeypatch, clock):
    attempts = []

    def resolve(host):
        attempts.append(host)
        if len(attempts) == 1:
            return None  # First attempt: cannot resolve
        if len(attempts) >= 3:
            pi_main.running.clear()
        return "127.0.0.1"

    FakeSender.instances = []
    monkeypatch.setattr(pi_main.imagezmq, "ImageSender", lambda connect_to: FakeSender(connect_to))
    monkeypatch.setattr(pi_main, "start_realsense", lambda: None)
    monkeypatch.setattr(pi_main, "open_usb_camera", lambda: None)
    monkeypatch.setattr(pi_main, "resolve_host", resolve)
    pi_main.running.set()

    pi_main.video_stream_loop()

    assert len(attempts) == 3
    assert clock.sleeps.count(3) == 2  # No sleep after the final, stopped attempt
    assert all(s.closed and not s.sent for s in FakeSender.instances)


# ---------------------------------------------------------------------------
# main()
# ---------------------------------------------------------------------------
@pytest.fixture
def fake_threads(pi_main, monkeypatch):
    """Replace the worker threads with ones that just wait for shutdown."""
    started = []

    def idle(*args, **kwargs):
        started.append(args)
        pi_main.running.wait(5)
        while pi_main.running.is_set():
            threading.Event().wait(0.01)

    for name in ("sensor_sender", "command_receiver", "video_stream_loop"):
        monkeypatch.setattr(pi_main, name, idle)
    return started


def test_main_runs_watchdog_and_shuts_down_safely(pi_main, monkeypatch, fake_threads, capsys):
    master = patch_mavlink(monkeypatch, FakeMaster())
    # 8 parameter writes + 1 arm delay, then 5 watchdog ticks before Ctrl+C
    clock = FakeTime(interrupt_after=14)
    monkeypatch.setattr(pi_main, "time", fake_time_module(clock))
    states = iter([True, False, False, True, True])
    monkeypatch.setattr(pi_main, "watchdog_step", lambda px, th: next(states))

    pi_main.main()

    out = capsys.readouterr().out
    assert "Connection lost" in out  # Printed once when the link drops
    assert out.count("Connection lost") == 1
    assert "T1:1500" in out  # Dashboard while connected
    assert len(fake_threads) == 3
    assert not pi_main.running.is_set()

    events = [e for e, _ in master.log]
    assert events[0] == "arm"
    assert events[-2:] == ["rc", "disarm"]
    assert master.log[-2][1] == [1500] * 8
    assert master.closed
    assert not master.concurrent_access


def test_main_disarms_even_if_startup_fails(pi_main, monkeypatch, fake_threads):
    master = patch_mavlink(monkeypatch, FakeMaster())
    monkeypatch.setattr(pi_main, "time", fake_time_module(FakeTime()))

    def broken_request_streams(self, rate):
        raise RuntimeError("serial link lost")

    monkeypatch.setattr(pi_main.Pixhawk, "request_streams", broken_request_streams)

    with pytest.raises(RuntimeError, match="serial link lost"):
        pi_main.main()

    assert fake_threads == []  # Threads never started
    assert [e for e, _ in master.log][-2:] == ["rc", "disarm"]
    assert master.closed


def test_video_streams_usb_camera_alone(pi_main, monkeypatch, clock):
    stop_on_retry_delay(pi_main, monkeypatch, clock)
    usb = FakeCapture(frames=[np.zeros((2, 2, 3), np.uint8)] * 3)
    run_video_loop(pi_main, monkeypatch, {}, pipeline=None, usb=usb)
    assert FakeSender.instances[0].sent == ["usb_cam"] * 3
    assert usb.released
