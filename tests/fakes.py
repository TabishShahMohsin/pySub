"""Test doubles for the hardware and libraries the programs talk to."""

import contextlib
import importlib.util
import sys
import threading
import time
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


# ---------------------------------------------------------------------------
# MAVLink
# ---------------------------------------------------------------------------
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
            self.master.rc.append(list(pwms[:8]))
            self.master.log.append(("rc", list(pwms[:8])))

    def command_long_send(self, *args):
        # (target_system, target_component, command, confirmation, param1, ...)
        with self.master.guard():
            self.master.commands.append(args)
            self.master.log.append(("arm" if args[4] == 1 else "disarm", None))

    def param_set_send(self, target_system, target_component, name, value, param_type):
        with self.master.guard():
            self.master.params[name.decode() if isinstance(name, bytes) else name] = value

    def request_data_stream_send(self, *args):
        with self.master.guard():
            self.master.streams.append(args)


class FakeMaster:
    """Stands in for a pymavlink connection and detects concurrent use.

    ``pending`` is the queue of incoming messages (``None`` entries simulate a
    recv timeout). Like pymavlink, filtering by type *discards* non-matching
    messages. With ``interrupt_when_empty`` a read from an empty queue raises
    KeyboardInterrupt, which is how the endless script loops are ended.
    """

    target_system = 1
    target_component = 1

    def __init__(self, pending=(), interrupt_when_empty=False):
        self._busy = threading.Lock()
        self.concurrent_access = False
        self.rc = []
        self.commands = []
        self.params = {}
        self.streams = []
        self.modes = []
        self.log = []
        self.closed = False
        self.heartbeat_waited = False
        self.pending = list(pending)
        self.interrupt_when_empty = interrupt_when_empty
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

    @property
    def armed(self):
        arms = [c[4] for c in self.commands]
        return bool(arms) and arms[-1] == 1

    def wait_heartbeat(self):
        self.heartbeat_waited = True

    def set_mode(self, mode):
        with self.guard():
            self.modes.append(mode)

    def recv_match(self, type=None, blocking=False, timeout=None):
        with self.guard():
            while self.pending:
                msg = self.pending.pop(0)
                if msg is None:
                    return None
                if type is None or msg.get_type() == type or msg.get_type() in type:
                    return msg
            if self.interrupt_when_empty:
                raise KeyboardInterrupt
            return None

    def close(self):
        self.closed = True


def patch_mavlink(monkeypatch, master):
    """Make mavutil.mavlink_connection() return ``master``."""
    from pymavlink import mavutil

    monkeypatch.setattr(mavutil, "mavlink_connection", lambda *a, **k: master)
    return master


# ---------------------------------------------------------------------------
# Sockets
# ---------------------------------------------------------------------------
class ScriptedSocket:
    """UDP socket whose recvfrom() / sendto() follow a script.

    recv items: bytes -> returned; an exception instance -> raised; a callable
    -> called then its return value is used. When the script runs out, the
    ``on_empty`` callback runs (normally clearing the running flag) and a
    timeout is raised.
    """

    def __init__(self, recv=(), send_errors=(), on_empty=None, bind_error=None):
        self.recv = list(recv)
        self.bind_error = bind_error
        self.send_errors = list(send_errors)
        self.on_empty = on_empty
        self.sent = []
        self.bound = None
        self.closed = False

    def setsockopt(self, *args):
        pass

    def bind(self, addr):
        if self.bind_error is not None:
            raise self.bind_error
        self.bound = addr

    def settimeout(self, timeout):
        pass

    def recvfrom(self, size):
        if not self.recv:
            if self.on_empty:
                self.on_empty()
            raise TimeoutError
        item = self.recv.pop(0)
        if callable(item):
            item = item()
        if isinstance(item, BaseException):
            raise item
        return item, ("127.0.0.1", 1234)

    def sendto(self, payload, addr):
        if self.send_errors:
            err = self.send_errors.pop(0)
            if err is not None:
                raise err
        self.sent.append((payload, addr))

    def close(self):
        self.closed = True


def socket_module(real_socket, sock):
    """A stand-in for the socket module whose socket() returns ``sock``."""
    return types.SimpleNamespace(
        socket=lambda *a, **k: sock,
        AF_INET=real_socket.AF_INET,
        SOCK_DGRAM=real_socket.SOCK_DGRAM,
        SOL_SOCKET=real_socket.SOL_SOCKET,
        SO_REUSEADDR=real_socket.SO_REUSEADDR,
        gethostbyname=real_socket.gethostbyname,
        gethostname=real_socket.gethostname,
    )


# ---------------------------------------------------------------------------
# Time
# ---------------------------------------------------------------------------
class FakeTime:
    """Virtual clock: sleep() advances time instantly.

    ``interrupt_after`` raises KeyboardInterrupt on that many-th sleep call
    (counting all threads, raised in the main thread), simulating Ctrl+C in
    the middle of a run.
    """

    def __init__(self, interrupt_after=None):
        self.now = 1000.0
        self.sleeps = []
        self.interrupt_after = interrupt_after
        self._lock = threading.Lock()

    def monotonic(self):
        with self._lock:
            return self.now

    def time(self):
        return self.monotonic() + 1.7e9

    def sleep(self, seconds):
        with self._lock:
            self.sleeps.append(seconds)
            self.now += max(0.0, seconds)
            count = len(self.sleeps)
        # Like a real Ctrl+C, the interrupt is only delivered to the main thread.
        is_main = threading.current_thread() is threading.main_thread()
        if self.interrupt_after is not None and count >= self.interrupt_after and is_main:
            raise KeyboardInterrupt


def fake_time_module(fake):
    return types.SimpleNamespace(monotonic=fake.monotonic, time=fake.time, sleep=fake.sleep)


# ---------------------------------------------------------------------------
# pigpio
# ---------------------------------------------------------------------------
class FakePigpioPi:
    def __init__(self, connected=True):
        self.connected = connected
        self.pulses = {}
        self.history = []
        self.stopped = False

    def set_servo_pulsewidth(self, pin, width):
        self.pulses[pin] = width
        self.history.append((pin, width))

    def stop(self):
        self.stopped = True


def fake_pigpio_module(pi):
    return types.SimpleNamespace(pi=lambda: pi)


# ---------------------------------------------------------------------------
# Loading scripts
# ---------------------------------------------------------------------------
_counter = 0


def load_script(monkeypatch, relative_path, modules=None):
    """Import a script by path with some modules replaced by fakes."""
    global _counter
    for name, module in (modules or {}).items():
        monkeypatch.setitem(sys.modules, name, module)
    _counter += 1
    name = f"script_{Path(relative_path).stem}_{_counter}"
    spec = importlib.util.spec_from_file_location(name, ROOT / relative_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module
