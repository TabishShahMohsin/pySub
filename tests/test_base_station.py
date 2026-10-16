"""Base-station threads exercised over real localhost sockets."""

import json
import socket
import threading
import time

import imagezmq
import numpy as np
import pytest

from config import PRESSURE_OFFSET, PWM_NEUTRAL, SURFACE_PRESSURE_MBAR, YAW_OFFSET
from pid import wrap_angle


def start(target, **kwargs):
    t = threading.Thread(target=target, kwargs=kwargs, daemon=True)
    t.start()
    return t


def drain(sock):
    sock.setblocking(False)
    try:
        while True:
            sock.recv(4096)
    except BlockingIOError:
        pass
    finally:
        sock.setblocking(True)


def test_pressure_to_depth(base_main):
    assert base_main.pressure_to_depth(SURFACE_PRESSURE_MBAR) == 0.0
    assert base_main.pressure_to_depth(SURFACE_PRESSURE_MBAR - 50) == 0.0  # Never negative
    assert base_main.pressure_to_depth(SURFACE_PRESSURE_MBAR + 100.5525) == pytest.approx(1.0, rel=1e-3)


def test_build_command_applies_inversion(base_main):
    cmd = base_main.build_command([1600] * 8)
    # I2-I6 are inverted in config.py
    assert cmd == {
        "t1": 1600,
        "t2": 1400,
        "t3": 1400,
        "t4": 1400,
        "t5": 1400,
        "t6": 1400,
        "t7": 1600,
        "t8": 1600,
    }
    assert set(base_main.build_command([PWM_NEUTRAL] * 8).values()) == {PWM_NEUTRAL}


def test_command_sender_rate_and_neutral_on_shutdown(base_main, udp_port):
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", udp_port))
    rx.settimeout(2)
    try:
        base_main.state.set_pwms([1600] * 8)
        sender = start(base_main.command_sender, host="127.0.0.1", port=udp_port)
        rx.recv(4096)  # Wait for the first packet
        drain(rx)

        count, t0 = 0, time.monotonic()
        while time.monotonic() - t0 < 1.0:
            packet = json.loads(rx.recv(4096))
            count += 1
        assert 17 <= count <= 23, f"expected ~20 Hz, got {count}"
        assert packet["t1"] == 1600 and packet["t2"] == 1400

        base_main.state.running.clear()
        sender.join(timeout=2)
        assert not sender.is_alive()

        last = None
        rx.settimeout(0.3)
        try:
            while True:
                last = json.loads(rx.recv(4096))
        except TimeoutError:
            pass
        assert last is not None and set(last.values()) == {PWM_NEUTRAL}
    finally:
        rx.close()


def test_telemetry_listener_applies_offsets_and_rejects_garbage(base_main, udp_port):
    listener = start(base_main.telemetry_listener, port=udp_port)
    time.sleep(0.2)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        tx.sendto(b"not json", ("127.0.0.1", udp_port))
        tx.sendto(json.dumps({"timestamp": 1}).encode(), ("127.0.0.1", udp_port))  # Missing keys
        tx.sendto(
            json.dumps(
                {"timestamp": 5.0, "pressure": 1100.0, "roll": 1.0, "pitch": 2.0, "yaw": -170.0}
            ).encode(),
            ("127.0.0.1", udp_port),
        )
        deadline = time.monotonic() + 2
        while base_main.state.get_telemetry()[1] == 0 and time.monotonic() < deadline:
            time.sleep(0.01)

        telem, seq, rx_time = base_main.state.get_telemetry()
        assert seq == 1  # Only the valid packet counted
        assert rx_time is not None
        assert telem["pressure"] == pytest.approx(1100.0 - PRESSURE_OFFSET)
        assert telem["yaw"] == pytest.approx(wrap_angle(-170.0 - YAW_OFFSET))
    finally:
        tx.close()
        base_main.state.running.clear()
        listener.join(timeout=2)
    assert not listener.is_alive()


def test_video_receiver_stores_frames_and_stops(base_main, tcp_port):
    receiver = start(base_main.video_receiver, port=tcp_port)
    sender = imagezmq.ImageSender(connect_to=f"tcp://127.0.0.1:{tcp_port}")
    try:
        frame = np.arange(48, dtype=np.uint8).reshape(4, 4, 3)
        sender.send_image("cam", frame)
        assert np.array_equal(base_main.state.get_frames()["cam"], frame)
    finally:
        sender.close()
        base_main.state.running.clear()
        receiver.join(timeout=3)
    assert not receiver.is_alive()


def test_shared_state_snapshots_are_copies(base_main):
    base_main.state.update_telemetry(
        {"timestamp": 0.0, "pressure": 1.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
    )
    telem, _, _ = base_main.state.get_telemetry()
    telem["pressure"] = 999
    assert base_main.state.get_telemetry()[0]["pressure"] == 1.0

    pwms = base_main.state.get_pwms()
    pwms[0] = 0
    assert base_main.state.get_pwms()[0] == PWM_NEUTRAL
