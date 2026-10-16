"""White-box tests for the manual bring-up scripts in hardware_tests/.

Every piece of hardware (Pixhawk, pigpio, serial IMU, Bar30, cameras, network)
is replaced by a fake, so each script's logic and especially its safety paths
(Ctrl+C mid-spin, start-up failures) can be exercised in CI.
"""

import builtins
import json
import socket as real_socket
import struct
import types

import cv2 as real_cv2
import numpy as np
import pygame
import pytest

from fakes import (
    FakeMaster,
    FakePigpioPi,
    FakeTime,
    Msg,
    ScriptedSocket,
    fake_pigpio_module,
    fake_time_module,
    load_script,
    patch_mavlink,
    socket_module,
)


def interrupt():
    raise KeyboardInterrupt


class FakeCV2:
    """cv2 with real image maths but fake windows."""

    def __init__(self, keys=()):
        self.keys = list(keys)
        self.shown = []
        self.destroyed = False
        for name in ("cvtColor", "resize", "COLOR_GRAY2BGR", "COLOR_RGB2BGR"):
            setattr(self, name, getattr(real_cv2, name))

    def imshow(self, name, frame):
        self.shown.append(name)

    def waitKey(self, delay):
        return self.keys.pop(0) if self.keys else -1

    def destroyAllWindows(self):
        self.destroyed = True


# ---------------------------------------------------------------------------
# Pixhawk thruster scripts
# ---------------------------------------------------------------------------
PIXHAWK_THRUSTER_SCRIPTS = [
    "hardware_tests/thruster_pixhawk.py",
    "hardware_tests/indi_thruster_pix.py",
    "hardware_tests/indi_pix_inv_tester.py",
]


def ends_neutral_and_disarmed(master):
    events = [e for e, _ in master.log]
    return events[-2:] == ["rc", "disarm"] and master.log[-2][1] == [1500] * 8 and not master.armed


@pytest.mark.parametrize("path", PIXHAWK_THRUSTER_SCRIPTS)
def test_pixhawk_thruster_script_full_run(monkeypatch, path):
    master = patch_mavlink(monkeypatch, FakeMaster())
    script = load_script(monkeypatch, path)
    monkeypatch.setattr(script, "time", fake_time_module(FakeTime()))

    script.main()

    assert master.modes == ["MANUAL"]
    assert ends_neutral_and_disarmed(master)
    # Each of the 8 outputs was driven on its own at some point
    active = {tuple(i for i, v in enumerate(rc) if v != 1500) for rc in master.rc}
    assert {(i,) for i in range(8)} <= active


@pytest.mark.parametrize("path", PIXHAWK_THRUSTER_SCRIPTS)
def test_pixhawk_thruster_script_ctrl_c_mid_spin(monkeypatch, path):
    master = patch_mavlink(monkeypatch, FakeMaster())
    script = load_script(monkeypatch, path)
    # Interrupt while a thruster is spinning (after parameter writes, if any)
    monkeypatch.setattr(script, "time", fake_time_module(FakeTime(interrupt_after=12)))

    with pytest.raises(KeyboardInterrupt):
        script.main()

    assert any(rc != [1500] * 8 for rc in master.rc)  # Was spinning
    assert ends_neutral_and_disarmed(master)  # ...and was stopped safely


def test_passthrough_scripts_configure_servo_functions(monkeypatch):
    for path in PIXHAWK_THRUSTER_SCRIPTS[1:]:
        master = patch_mavlink(monkeypatch, FakeMaster())
        script = load_script(monkeypatch, path)
        monkeypatch.setattr(script, "time", fake_time_module(FakeTime()))
        script.main()
        assert master.params == {f"SERVO{i}_FUNCTION": 50 + i for i in range(1, 9)}


def test_inversion_tester_mapping(monkeypatch):
    script = load_script(monkeypatch, "hardware_tests/indi_pix_inv_tester.py")
    assert script.get_mapped_pwm(1, 1600) == 1600
    assert script.get_mapped_pwm(2, 1600) == 1400
    assert sorted(script.LOGICAL_TO_PHYSICAL.values()) == list(range(1, 9))

    # Matches the inversion flags used by the base station
    import config

    flags = [config.I1, config.I2, config.I3, config.I4, config.I5, config.I6, config.I7, config.I8]
    assert [i + 1 for i, inverted in enumerate(flags) if inverted] == script.INVERT_LIST


# ---------------------------------------------------------------------------
# pigpio thruster scripts
# ---------------------------------------------------------------------------
PIGPIO_SCRIPTS = ["hardware_tests/thruster_test.py", "hardware_tests/thruster_test_simple.py"]


def load_pigpio_script(monkeypatch, path, pi, interrupt_after=None):
    script = load_script(monkeypatch, path, {"pigpio": fake_pigpio_module(pi)})
    monkeypatch.setattr(script, "time", fake_time_module(FakeTime(interrupt_after=interrupt_after)))
    monkeypatch.setattr(builtins, "input", lambda prompt="": "")
    return script


@pytest.mark.parametrize("path", PIGPIO_SCRIPTS)
def test_pigpio_script_full_run(monkeypatch, path):
    pi = FakePigpioPi()
    script = load_pigpio_script(monkeypatch, path, pi)
    script.main()
    pins = set(script.THRUSTER_PINS)
    assert {pin for pin, width in pi.history if width not in (1500,)} == pins  # Every pin spun
    assert all(pi.pulses[pin] == 1500 for pin in pins)
    assert pi.stopped


@pytest.mark.parametrize("path", PIGPIO_SCRIPTS)
def test_pigpio_script_ctrl_c_mid_spin(monkeypatch, path):
    pi = FakePigpioPi()
    script = load_pigpio_script(monkeypatch, path, pi, interrupt_after=3)
    script.main()  # Interrupt is handled inside the script
    assert any(width != 1500 for _, width in pi.history)
    assert all(pi.pulses[pin] == 1500 for pin in script.THRUSTER_PINS)
    assert pi.stopped


@pytest.mark.parametrize("path", [*PIGPIO_SCRIPTS, "hardware_tests/square_auv_test.py"])
def test_pigpio_script_without_daemon(monkeypatch, path):
    pi = FakePigpioPi(connected=False)
    script = load_pigpio_script(monkeypatch, path, pi)
    if hasattr(script, "subprocess"):
        monkeypatch.setattr(script, "subprocess", types.SimpleNamespace(run=lambda *a, **k: None))
    with pytest.raises(SystemExit) as exc:
        script.main()
    assert exc.value.code == 1
    assert pi.history == []


# ---------------------------------------------------------------------------
# square_auv_test
# ---------------------------------------------------------------------------
@pytest.fixture
def square(monkeypatch):
    pi = FakePigpioPi()
    script = load_script(monkeypatch, "hardware_tests/square_auv_test.py", {"pigpio": fake_pigpio_module(pi)})
    return script, pi


def test_square_allocation_and_pwm_mapping(square):
    script, _ = square
    assert script.map_force_to_pwm(0.0) == 1500
    assert script.map_force_to_pwm(0.5) > 1500 > script.map_force_to_pwm(-0.5)
    assert script.get_pwms(0, 0, 0, 0) == [1500] * 8
    # Large demands are normalised so no force exceeds 1
    big = script.get_pwms(1, 1, 1, 1, roll=1, pitch=1)
    assert max(big) <= script.map_force_to_pwm(1.0)
    assert min(big) >= script.map_force_to_pwm(-1.0)
    # Inversion mask flips T1 relative to T2 for pure surge
    surge = script.get_pwms(0.4, 0, 0, 0)
    assert surge[0] < 1500 < surge[1]


def test_square_ramp_moves_gradually(square, monkeypatch):
    script, pi = square
    fake = FakeTime()
    ticks = []

    def sleep(seconds):
        fake.sleep(seconds)
        ticks.append(dict(script.current_pwms))
        if len(ticks) == 10:
            script.running.clear()

    monkeypatch.setattr(script, "time", types.SimpleNamespace(monotonic=fake.monotonic, sleep=sleep))
    script.update_motion(0.4, 0, 0, 0)
    target = dict(script.target_pwms)
    script.running.set()
    script.ramping_loop(pi)

    first = ticks[0]
    for key in target:
        assert abs(first[key] - 1500) <= script.RAMP_STEP  # One step per tick
    # Never jumps by more than one step, and converges on the target
    for before, after in zip(ticks, ticks[1:], strict=False):
        assert all(abs(after[k] - before[k]) <= script.RAMP_STEP for k in target)
    assert ticks[-1] == target


def test_square_mission_completes_and_neutralises(square, monkeypatch):
    script, pi = square
    monkeypatch.setattr(script, "time", fake_time_module(FakeTime()))
    started = []
    monkeypatch.setattr(script, "subprocess", types.SimpleNamespace(run=lambda *a, **k: started.append(a)))
    script.main()
    assert started  # Tried to start pigpiod
    assert all(pi.pulses.get(pin, 1500) == 1500 for pin in script.THRUSTER_PINS.values())
    assert pi.stopped
    assert not script.running.is_set()


def test_square_emergency_stop(square, monkeypatch, capsys):
    script, pi = square
    monkeypatch.setattr(script, "time", fake_time_module(FakeTime(interrupt_after=2)))

    def no_sudo(*a, **k):
        raise OSError("sudo not found")

    monkeypatch.setattr(script, "subprocess", types.SimpleNamespace(run=no_sudo))
    script.main()
    assert "Emergency stop" in capsys.readouterr().out
    assert all(pi.pulses.get(pin, 1500) == 1500 for pin in script.THRUSTER_PINS.values())
    assert pi.stopped


# ---------------------------------------------------------------------------
# MAVLink sensor scripts
# ---------------------------------------------------------------------------
def test_imu_pixhawk_prints_throttled(monkeypatch, capsys):
    imu = {f"{k}{a}": 1 for k in ("x", "y", "z") for a in ("acc", "gyro", "mag")}
    master = patch_mavlink(
        monkeypatch,
        FakeMaster([Msg("RAW_IMU", **imu), Msg("RAW_IMU", **imu), None], interrupt_when_empty=True),
    )
    script = load_script(monkeypatch, "hardware_tests/imu_pixhawk.py")
    script.main()
    out = capsys.readouterr().out
    assert out.count("IMU DATA") == 1  # Second message within 0.1 s is skipped
    assert "No RAW_IMU" in out
    assert master.closed


def test_imu_roll_pitch_yaw(monkeypatch, capsys):
    master = patch_mavlink(
        monkeypatch,
        FakeMaster([Msg("ATTITUDE", roll=0.0, pitch=np.pi / 2, yaw=-np.pi), None], interrupt_when_empty=True),
    )
    script = load_script(monkeypatch, "hardware_tests/imu_roll_pitch_yaw.py")
    script.main()
    out = capsys.readouterr().out
    assert "Pitch:  90.00" in out and "Yaw: -180.00" in out
    assert "No ATTITUDE" in out
    assert master.streams  # Requested the ATTITUDE stream
    assert master.closed


def test_pix_pressure_sensor(monkeypatch, capsys):
    master = patch_mavlink(
        monkeypatch,
        FakeMaster(
            [Msg("SCALED_PRESSURE2", press_abs=1111.32, temperature=2150), None], interrupt_when_empty=True
        ),
    )
    script = load_script(monkeypatch, "hardware_tests/pix_pressure_sensor.py")
    script.main()
    out = capsys.readouterr().out
    assert "Temp: 21.50 C" in out
    assert "Approx depth: 1.00 m" in out  # 98.07 hPa of fresh water = 1 m
    assert "No SCALED_PRESSURE2" in out
    assert master.closed


# ---------------------------------------------------------------------------
# Serial IMU (HFI-A9)
# ---------------------------------------------------------------------------
class SerialException(Exception):
    pass


def fake_serial_modules(ports=(), serial_factory=None):
    serial = types.ModuleType("serial")
    tools = types.ModuleType("serial.tools")
    list_ports = types.ModuleType("serial.tools.list_ports")
    list_ports.comports = lambda: [types.SimpleNamespace(device=d, description=desc) for d, desc in ports]
    serial.SerialException = SerialException
    serial.Serial = serial_factory
    serial.tools = tools
    tools.list_ports = list_ports
    return {"serial": serial, "serial.tools": tools, "serial.tools.list_ports": list_ports}


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def make_packet(length, floats, corrupt=False):
    body = bytes([0xAA, 0x55, length, 0, 0, 0, 0]) + struct.pack(f"<{len(floats)}f", *floats)
    crc = crc16(body[2:])
    if corrupt:
        crc ^= 0xFFFF
    return body + bytes([crc & 0xFF, crc >> 8])


IMU_FLOATS = [0, 0.1, 0.2, 0.3, 0, 0, -1.0, 5, 6, 7]  # [?, gyro x3, accel x3, mag x3]
EULER_FLOATS = [0, 10.0, 20.0, 30.0]


@pytest.fixture
def imu_script(monkeypatch):
    return load_script(monkeypatch, "hardware_tests/imu_test.py", fake_serial_modules())


def test_imu_packet_layout(imu_script):
    assert len(make_packet(0x2C, IMU_FLOATS)) == 0x2C + 5
    assert len(make_packet(0x14, EULER_FLOATS)) == 0x14 + 5
    assert imu_script.check_sum(make_packet(0x14, EULER_FLOATS)[2:23], make_packet(0x14, EULER_FLOATS)[23:25])
    assert imu_script.bytes_to_floats(struct.pack("<2f", 1.5, -2.0)) == [1.5, -2.0]


def test_imu_parser_decodes_both_packets(imu_script, capsys):
    parser = imu_script.ImuParser()
    for byte in make_packet(0x2C, IMU_FLOATS):
        parser.feed(byte)
    assert capsys.readouterr().out == ""  # Waits for the Euler packet too
    for byte in make_packet(0x14, EULER_FLOATS):
        parser.feed(byte)

    assert parser.angular_velocity == pytest.approx([0.1, 0.2, 0.3])
    assert parser.angle_degree == pytest.approx([10.0, 20.0, 30.0])
    assert parser.magnetometer == pytest.approx([5, 6, 7])
    out = capsys.readouterr().out
    assert "\033[2J" in out  # Screen cleared on first display only
    assert "z-axis: 9.80" in out  # Accel normalised to gravity
    assert "roll : 10.00" in out

    for byte in make_packet(0x2C, IMU_FLOATS) + make_packet(0x14, EULER_FLOATS):
        parser.feed(byte)
    assert "\033[2J" not in capsys.readouterr().out


def test_imu_parser_rejects_bad_data(imu_script, capsys):
    parser = imu_script.ImuParser()
    for byte in b"\x00\x11" + bytes([0xAA, 0x00, 0x00]):  # Junk, then wrong 2nd header byte
        parser.feed(byte)
    assert parser.buff == []

    for byte in make_packet(0x2C, IMU_FLOATS, corrupt=True) + make_packet(0x14, EULER_FLOATS, corrupt=True):
        parser.feed(byte)
    assert capsys.readouterr().out.count("Checksum failed") == 2
    assert parser.angle_degree == [0.0, 0.0, 0.0]  # Not updated from corrupt data

    for byte in bytes([0xAA, 0x55, 0x01, 0, 0, 0]):  # Unknown packet type
        parser.feed(byte)
    assert "No parser implemented for packet length: 1" in capsys.readouterr().out

    # A repeated IMU packet before the Euler one is ignored
    parser.pending = [False, True]
    for byte in make_packet(0x2C, [9.0] * 10):
        parser.feed(byte)
    assert parser.angular_velocity != [9.0, 9.0, 9.0]


def test_imu_display_handles_zero_acceleration(imu_script, capsys):
    parser = imu_script.ImuParser()
    parser.display()  # All zeros: must not divide by zero
    assert "x-axis: -0.00" in capsys.readouterr().out


class FakeSerialPort:
    def __init__(self, chunks, end, is_open=True):
        self.chunks = list(chunks)
        self.end = end
        self.is_open = is_open
        self.opened = False
        self.closed = False

    @property
    def in_waiting(self):
        return len(self.chunks[0]) if self.chunks else 0

    def open(self):
        self.opened = True

    def read(self, n):
        if not self.chunks:
            raise self.end
        return self.chunks.pop(0)

    def close(self):
        self.closed = True


def test_imu_main_reads_until_ctrl_c(monkeypatch, capsys):
    port = FakeSerialPort(
        [make_packet(0x2C, IMU_FLOATS), make_packet(0x14, EULER_FLOATS)], KeyboardInterrupt(), False
    )
    script = load_script(
        monkeypatch,
        "hardware_tests/imu_test.py",
        fake_serial_modules([("/dev/ttyUSB0", "CP2102"), ("/dev/ttyS0", "serial")], lambda **k: port),
    )
    script.main()
    out = capsys.readouterr().out
    assert "['/dev/ttyUSB0']" in out  # Only USB devices listed
    assert port.opened and port.closed
    assert "roll : 10.00" in out


def test_imu_main_serial_errors(monkeypatch):
    def cannot_open(**kwargs):
        raise SerialException("no such port")

    script = load_script(
        monkeypatch, "hardware_tests/imu_test.py", fake_serial_modules(serial_factory=cannot_open)
    )
    with pytest.raises(SystemExit) as exc:
        script.main()
    assert exc.value.code == 1

    port = FakeSerialPort([b"\xaa"], SerialException("unplugged"))
    script = load_script(
        monkeypatch, "hardware_tests/imu_test.py", fake_serial_modules(serial_factory=lambda **k: port)
    )
    with pytest.raises(SystemExit) as exc:
        script.main()
    assert exc.value.code == 1
    assert port.closed


@pytest.mark.parametrize(
    ("ports", "found"),
    [
        ([("/dev/ttyS0", "Built-in"), ("/dev/ttyUSB0", "CH340 adapter")], "/dev/ttyUSB0"),
        ([("/dev/ttyACM0", "USB Serial Device")], "/dev/ttyACM0"),
        ([("/dev/cu.SLAB", "CP210x UART")], "/dev/cu.SLAB"),
        ([("/dev/ttyS0", "Built-in")], None),
    ],
)
def test_list_ports_imu(monkeypatch, capsys, ports, found):
    script = load_script(monkeypatch, "hardware_tests/list_ports_imu.py", fake_serial_modules(ports))
    assert script.get_imu_port() == found
    script.main()
    out = capsys.readouterr().out
    assert ("Connecting to" in out) == (found is not None)


# ---------------------------------------------------------------------------
# Bar30 over I2C
# ---------------------------------------------------------------------------
def fake_ms5837(init_ok=True, reads=(True, False)):
    module = types.ModuleType("ms5837")
    module.DENSITY_FRESHWATER = 997
    module.UNITS_mbar = "mbar"

    class Sensor:
        def __init__(self):
            self.reads = list(reads)
            self.density = None

        def init(self):
            return init_ok

        def setFluidDensity(self, density):
            self.density = density

        def read(self):
            return self.reads.pop(0) if self.reads else True

        def depth(self):
            return 1.5

        def temperature(self):
            return 21.0

        def pressure(self, units):
            assert units == "mbar"
            return 1160.0

    module.MS5837_30BA = Sensor
    return module


def test_bar30_reads_until_ctrl_c(monkeypatch, capsys):
    script = load_script(
        monkeypatch, "hardware_tests/pressure_sensor_bar30_test.py", {"ms5837": fake_ms5837()}
    )
    monkeypatch.setattr(script, "time", fake_time_module(FakeTime(interrupt_after=2)))
    script.main()
    out = capsys.readouterr().out
    assert "Depth:   1.50 m" in out
    assert "sensor read failed" in out
    assert "Stopping" in out


def test_bar30_init_failure(monkeypatch):
    script = load_script(
        monkeypatch, "hardware_tests/pressure_sensor_bar30_test.py", {"ms5837": fake_ms5837(init_ok=False)}
    )
    with pytest.raises(SystemExit) as exc:
        script.main()
    assert exc.value.code == 1


# ---------------------------------------------------------------------------
# Cameras
# ---------------------------------------------------------------------------
def fake_picamera2(frames):
    module = types.ModuleType("picamera2")

    class Picamera2:
        instance = None

        def __init__(self):
            self.frames = list(frames)
            self.started = self.stopped = False
            Picamera2.instance = self

        def create_preview_configuration(self, main):
            return main

        def configure(self, cfg):
            self.cfg = cfg

        def start(self):
            self.started = True

        def capture_array(self):
            if not self.frames:
                raise KeyboardInterrupt
            return self.frames.pop(0)

        def stop(self):
            self.stopped = True

    module.Picamera2 = Picamera2
    return module


@pytest.mark.parametrize(("keys", "frames", "shown"), [([-1, ord("q")], 5, 2), ([], 3, 3)])
def test_pi_cam(monkeypatch, keys, frames, shown):
    rgb = np.zeros((4, 4, 3), np.uint8)
    rgb[..., 0] = 255  # Pure red in RGB
    module = fake_picamera2([rgb] * frames)
    script = load_script(monkeypatch, "hardware_tests/pi_cam_test.py", {"picamera2": module})
    cv2 = FakeCV2(keys)
    monkeypatch.setattr(script, "cv2", cv2)

    script.main()

    cam = module.Picamera2.instance
    assert cam.cfg == {"size": (640, 480)}
    assert len(cv2.shown) == shown  # q quits early; otherwise Ctrl+C ends it
    assert cam.stopped and cv2.destroyed


def fake_realsense(framesets, has_emitter=True):
    rs = types.ModuleType("pyrealsense2")
    rs.stream = types.SimpleNamespace(depth="depth", color="color", infrared="ir", accel="accel", gyro="gyro")
    rs.format = types.SimpleNamespace(z16="z16", bgr8="bgr8", y8="y8")
    rs.option = types.SimpleNamespace(emitter_enabled="emitter")

    class Sensor:
        def __init__(self):
            self.options = {}

        def supports(self, option):
            return has_emitter

        def set_option(self, option, value):
            self.options[option] = value

    sensor = Sensor()

    class Pipeline:
        def __init__(self):
            self.framesets = list(framesets)
            self.stopped = False
            rs.pipeline_instance = self

        def start(self, config):
            device = types.SimpleNamespace(first_depth_sensor=lambda: sensor)
            return types.SimpleNamespace(get_device=lambda: device)

        def wait_for_frames(self):
            if not self.framesets:
                raise KeyboardInterrupt
            return self.framesets.pop(0)

        def stop(self):
            self.stopped = True

    class Config:
        def __init__(self):
            self.streams = []

        def enable_stream(self, *args):
            self.streams.append(args)

    rs.pipeline = Pipeline
    rs.config = Config
    rs.colorizer = lambda: types.SimpleNamespace(colorize=lambda f: Img(np.zeros((4, 4, 3), np.uint8)))
    rs.sensor = sensor
    return rs


class Img:
    def __init__(self, data):
        self.data = data

    def get_data(self):
        return self.data

    def __bool__(self):
        return self.data is not None


class Motion:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z

    def as_motion_frame(self):
        return self

    def get_motion_data(self):
        return self

    def __bool__(self):
        return True


class FrameSet:
    def __init__(self, complete=True, accel=None, gyro=None):
        color = np.zeros((4, 4, 3), np.uint8)
        ir = np.zeros((4, 4), np.uint8)
        self.frames = {
            "depth": Img(np.zeros((4, 4), np.uint16)),
            "color": Img(color if complete else None),
            1: Img(ir),
            2: Img(ir),
        }
        self.motion = {"accel": accel, "gyro": gyro}

    def get_depth_frame(self):
        return self.frames["depth"]

    def get_color_frame(self):
        return self.frames["color"]

    def get_infrared_frame(self, i):
        return self.frames[i]

    def first_or_default(self, stream):
        return self.motion[stream] or Img(None)

    def get_timestamp(self):
        return 12.5


@pytest.mark.parametrize("quit_with_q", [True, False])
def test_realsense(monkeypatch, capsys, quit_with_q):
    framesets = [
        FrameSet(complete=False, accel=Motion(0, 0, 9.8)),  # Missing color -> skipped
        FrameSet(),  # No motion yet -> no IMU line
        FrameSet(gyro=Motion(0.1, 0.2, 0.3)),
    ]
    rs = fake_realsense(framesets)
    script = load_script(monkeypatch, "hardware_tests/real_sense_test.py", {"pyrealsense2": rs})
    cv2 = FakeCV2([-1, ord("q")] if quit_with_q else [])
    monkeypatch.setattr(script, "cv2", cv2)

    script.main()

    out = capsys.readouterr().out
    assert len(cv2.shown) == 2
    assert out.count("ACCEL") == 1 and "GYRO: x: 0.10" in out
    assert rs.sensor.options == {"emitter": 0}
    assert rs.pipeline_instance.stopped and cv2.destroyed


# ---------------------------------------------------------------------------
# Controller viewer
# ---------------------------------------------------------------------------
class FakeJoystick:
    def __init__(self, index):
        pass

    def init(self):
        pass

    def get_name(self):
        return "Pad"

    def get_numaxes(self):
        return 2

    def get_axis(self, i):
        return [-1.0, 0.5][i]

    def get_numbuttons(self):
        return 5

    def get_button(self, i):
        return i == 1


def test_controller_viewer(monkeypatch, capsys):
    script = load_script(monkeypatch, "hardware_tests/controller_test.py")
    monkeypatch.setattr(pygame.joystick, "get_count", lambda: 1)
    monkeypatch.setattr(pygame.joystick, "Joystick", FakeJoystick)
    waits = []

    def wait(ms):
        waits.append(ms)
        if len(waits) == 2:
            raise KeyboardInterrupt

    monkeypatch.setattr(pygame.time, "wait", wait)
    script.main()
    out = capsys.readouterr().out
    assert "Axis 0: [                    ] -1.000" in out
    assert "Axis 1: [###############     ]  0.500" in out
    assert "B1: ON " in out and "B4: OFF" in out
    assert "Exiting" in out


def test_controller_viewer_without_controller(monkeypatch, capsys):
    script = load_script(monkeypatch, "hardware_tests/controller_test.py")
    monkeypatch.setattr(pygame.joystick, "get_count", lambda: 0)
    script.main()
    assert "No controller found" in capsys.readouterr().out


# ---------------------------------------------------------------------------
# UDP round-trip scripts
# ---------------------------------------------------------------------------
def test_udp_client_echoes_probes(monkeypatch):
    sock = ScriptedSocket(recv=[TimeoutError(), b"7,123.5"], on_empty=interrupt)
    script = load_script(monkeypatch, "hardware_tests/udp_client_pi.py")
    monkeypatch.setattr(script, "socket", socket_module(real_socket, sock))
    monkeypatch.setattr(script, "time", fake_time_module(FakeTime()))

    script.main()

    assert sock.bound == ("0.0.0.0", 5006)
    reply, addr = sock.sent[0]
    seq, sent_time, pi_time = reply.decode().split(",")
    assert (seq, sent_time) == ("7", "123.5") and float(pi_time) > 0
    assert addr == ("127.0.0.1", 5005)
    assert sock.closed


def test_udp_server_matches_replies_by_sequence(monkeypatch, capsys):
    fake = FakeTime()
    script = load_script(monkeypatch, "hardware_tests/udp_server_base_station.py")

    def late_reply():
        fake.now += 5  # Reply arrives after the deadline
        return b"2,0,0"

    sock = ScriptedSocket(
        recv=[
            b"0,0,0",  # Stale reply to an older probe: ignored
            b"garbage",  # Malformed: ignored
            b"1,0,0",  # Probe 1 answered
            TimeoutError(),  # Probe 2: no response
            late_reply,  # Probe 3: deadline passes while waiting
        ],
        on_empty=interrupt,
    )
    monkeypatch.setattr(script, "socket", socket_module(real_socket, sock))
    monkeypatch.setattr(script, "time", fake_time_module(fake))

    script.main()

    out = capsys.readouterr().out
    assert "probe 1 | RTT" in out
    assert "probe 2 | no response" in out
    assert "probe 3 | no response" in out
    assert [json.dumps(p.decode().split(",")[0]) for p, _ in sock.sent][:3] == ['"1"', '"2"', '"3"']
    assert sock.closed


def test_realsense_without_emitter(monkeypatch):
    rs = fake_realsense([FrameSet()], has_emitter=False)
    script = load_script(monkeypatch, "hardware_tests/real_sense_test.py", {"pyrealsense2": rs})
    monkeypatch.setattr(script, "cv2", FakeCV2())
    script.main()
    assert rs.sensor.options == {}  # Not touched when unsupported
