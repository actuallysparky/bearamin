#!/usr/bin/env python3

import contextlib
import importlib.util
import io
import pathlib
import sys


PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[1]
WATCHER_PATH = PROJECT_ROOT / "tools" / "watch_sidecar_wifi.py"

spec = importlib.util.spec_from_file_location("watch_sidecar_wifi", WATCHER_PATH)
watcher = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = watcher
spec.loader.exec_module(watcher)


class FakeResponse:
    status = 204

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback):
        return False

    def read(self, size):
        assert size == 128
        return b'{"ok":true}'


class FakeSerial:
    def __init__(self, chunks):
        self.chunks = list(chunks)

    def read(self, size):
        assert size == 256
        if not self.chunks:
            return b""
        return self.chunks.pop(0)


def test_poll_http_reports_status_and_short_failure():
    def fake_urlopen(url, timeout):
        assert url == "http://bearamin.lan/status.json"
        assert timeout == 2
        return FakeResponse()

    def failing_urlopen(url, timeout):
        raise TimeoutError("sidecar did not answer within the polling window")

    original_urlopen = watcher.urllib.request.urlopen
    try:
        watcher.urllib.request.urlopen = fake_urlopen
        assert watcher.poll_http("http://bearamin.lan/status.json") == "ok http_204"

        watcher.urllib.request.urlopen = failing_urlopen
        state = watcher.poll_http("http://bearamin.lan/status.json")
        assert state.startswith("fail TimeoutError:sidecar did not answer")
    finally:
        watcher.urllib.request.urlopen = original_urlopen


def test_poll_serial_logs_complete_and_overlong_lines():
    original_stamp = watcher.stamp
    watcher.stamp = lambda: "2026-07-21 12:34:56"
    log = io.StringIO()
    serial_buffer = bytearray()

    try:
        with contextlib.redirect_stdout(io.StringIO()):
            watcher.poll_serial(FakeSerial([b"wifi_diag connect_begin\rpartial"]), serial_buffer, log)
            assert "2026-07-21 12:34:56 serial wifi_diag connect_begin\n" in log.getvalue()
            assert serial_buffer == bytearray(b"partial")

            watcher.poll_serial(FakeSerial([b" line\n"]), serial_buffer, log)
            assert "2026-07-21 12:34:56 serial partial line\n" in log.getvalue()
            assert serial_buffer == bytearray()

            watcher.poll_serial(FakeSerial([b"x" * 512]), serial_buffer, log)
            assert "2026-07-21 12:34:56 serial " + ("x" * 512) + "\n" in log.getvalue()
            assert serial_buffer == bytearray()
    finally:
        watcher.stamp = original_stamp


if __name__ == "__main__":
    test_poll_http_reports_status_and_short_failure()
    test_poll_serial_logs_complete_and_overlong_lines()
