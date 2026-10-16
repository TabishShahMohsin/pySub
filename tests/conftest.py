"""Shared test setup.

The base station and Pi programs are plain scripts (each with a main.py), so
they are put on sys.path / loaded by file path here instead of being packages.
"""

import importlib.util
import os
import socket
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
BASE_STATION = ROOT / "base_station"
PI_MAIN = ROOT / "pi" / "main.py"

# Headless pygame / no mDNS lookups during tests
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
os.environ.setdefault("ROV_PI_HOST", "127.0.0.1")
os.environ.setdefault("ROV_BASE_HOST", "127.0.0.1")

sys.path.insert(0, str(BASE_STATION))
sys.path.insert(0, str(Path(__file__).resolve().parent))


def _load_pi_main():
    spec = importlib.util.spec_from_file_location("pi_main", PI_MAIN)
    module = importlib.util.module_from_spec(spec)
    sys.modules["pi_main"] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="session")
def pi_main():
    return _load_pi_main()


@pytest.fixture
def base_main():
    import main as base_main_module

    # Fresh shared state for every test
    base_main_module.state = base_main_module.SharedState()
    yield base_main_module
    base_main_module.state.running.clear()


def _free_port(kind):
    with socket.socket(socket.AF_INET, kind) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture
def udp_port():
    return _free_port(socket.SOCK_DGRAM)


@pytest.fixture
def tcp_port():
    return _free_port(socket.SOCK_STREAM)
