#!/usr/bin/env python3
import argparse
import time
import urllib.request

try:
    import serial
except ImportError:
    serial = None


def stamp():
    return time.strftime("%Y-%m-%d %H:%M:%S")


def log_line(log, text):
    line = f"{stamp()} {text}"
    print(line, flush=True)
    log.write(line + "\n")
    log.flush()


def poll_http(url):
    try:
        with urllib.request.urlopen(url, timeout=2) as response:
            response.read(128)
            return f"ok http_{response.status}"
    except Exception as exc:
        return f"fail {type(exc).__name__}:{str(exc)[:120]}"


def poll_serial(ser, serial_buffer, log):
    chunk = ser.read(256)
    if not chunk:
        return
    for byte in chunk:
        if byte in (10, 13):
            if serial_buffer:
                text = serial_buffer.decode("utf-8", errors="replace")
                log_line(log, "serial " + text)
                serial_buffer.clear()
        else:
            serial_buffer.append(byte)
            if len(serial_buffer) >= 512:
                text = serial_buffer.decode("utf-8", errors="replace")
                log_line(log, "serial " + text)
                serial_buffer.clear()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/cu.usbmodem211101")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--url", default="http://bearamin.lan/status.json")
    parser.add_argument("--log", required=True)
    parser.add_argument("--duration", type=int, default=7200)
    parser.add_argument("--http-interval", type=float, default=5.0)
    parser.add_argument("--no-serial", action="store_true")
    args = parser.parse_args()

    deadline = time.monotonic() + args.duration
    next_http = 0.0
    last_http = None
    serial_buffer = bytearray()

    with open(args.log, "a", buffering=1) as log:
        log_line(
            log,
            f"watch_start port={args.port} baud={args.baud} "
            f"url={args.url} duration={args.duration}s",
        )

        ser = None
        if not args.no_serial:
            if serial is None:
                parser.error("pyserial is required unless --no-serial is used")
            ser = serial.Serial(args.port, args.baud, timeout=0.2)

        try:
            while time.monotonic() < deadline:
                if ser is not None:
                    poll_serial(ser, serial_buffer, log)
                else:
                    time.sleep(0.1)

                now = time.monotonic()
                if now >= next_http:
                    state = poll_http(args.url)
                    if state != last_http or state.startswith("fail"):
                        log_line(log, "http " + state)
                        last_http = state
                    next_http = now + args.http_interval
        finally:
            if ser is not None:
                ser.close()

        if serial_buffer:
            text = serial_buffer.decode("utf-8", errors="replace")
            log_line(log, "serial " + text)
        log_line(log, "watch_end")


if __name__ == "__main__":
    main()
