#!/usr/bin/env python3
"""Optional Mac MIDI/JSON field reader and local OBS Browser Source server."""
import argparse
import json
import math
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

PREFIX = [0x7d, 0x42, 0x41, 1]

def encode(value):
    raw = json.dumps(value, ensure_ascii=False, separators=(",", ":")).encode()
    if len(raw) > 4096:
        raise ValueError("Message too large")
    data = PREFIX.copy()
    for start in range(0, len(raw), 7):
        part = raw[start:start + 7]
        data.append(sum((b >> 7) << i for i, b in enumerate(part)))
        data.extend(b & 127 for b in part)
    return data

def decode(data):
    if list(data[:4]) != PREFIX or len(data) > 4690:
        return None
    raw = bytearray()
    for start in range(4, len(data), 8):
        part = data[start:start + 8]
        if len(part) < 2 or any(b < 0 or b > 127 for b in part):
            return None
        raw.extend(b | (((part[0] >> i) & 1) << 7) for i, b in enumerate(part[1:]))
    try:
        value = json.loads(raw.decode())
        return value if isinstance(value, dict) and len(raw) <= 4096 else None
    except (ValueError, UnicodeError):
        return None

def valid_frame(value):
    if not isinstance(value, dict):
        return None
    mm = value.get("mm")
    if not isinstance(mm, list) or len(mm) != 64:
        return None
    if any(v is not None and (type(v) not in (int, float) or not math.isfinite(v) or not 0 < v <= 4000) for v in mm):
        return None
    return {"mm": mm}

class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.frame = None
        self.updated = 0
    def put(self, value):
        frame = valid_frame(value)
        if frame:
            with self.lock:
                self.frame, self.updated = frame, time.monotonic()
    def get(self):
        with self.lock:
            return self.frame if time.monotonic() - self.updated < 1.5 else {"mm": [None] * 64}

def midi_reader(state, stop, input_name, output_name):
    import mido
    with mido.open_input(input_name) as inp, mido.open_output(output_name) as out:
        request = 1
        uid = None
        last_probe = 0
        def send(op, data):
            nonlocal request
            out.send(mido.Message("sysex", data=encode({"op": op, "request": request, "data": data})))
            request += 1
        try:
            while not stop.wait(0.02):
                now = time.monotonic()
                if now - last_probe > (30 if uid else 5):
                    send("field.subscribe" if uid else "identity", {"hz": 5} if uid else {"protocol": 1})
                    last_probe = now
                for message in inp.iter_pending():
                    if message.type != "sysex":
                        continue
                    value = decode(message.data)
                    if not value:
                        continue
                    data = value.get("data")
                    if value.get("op") == "identity" and isinstance(data, dict) and data.get("protocol") == 1 and data.get("model") in ("lite", "virtual") and data.get("capabilities", {}).get("field") and isinstance(data.get("uid"), str):
                        uid = data["uid"]
                        send("field.subscribe", {"hz": 5})
                        last_probe = now
                    elif uid and value.get("uid") == uid and value.get("op") == "field":
                        state.put(data)
        finally:
            if uid:
                send("field.subscribe", {"hz": 0})

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--stdin", action="store_true", help="Read 8x8 mm JSON frames, one per line")
    parser.add_argument("--input", help="Exact MIDI input name")
    parser.add_argument("--output", help="Exact MIDI output name")
    parser.add_argument("--port", type=int, default=8767)
    args = parser.parse_args()
    if not args.stdin:
        import mido
        if args.list:
            print("Inputs:", *mido.get_input_names(), "Outputs:", *mido.get_output_names(), sep="\n")
            return
        inputs = [n for n in mido.get_input_names() if "bearamin" in n.lower()]
        outputs = [n for n in mido.get_output_names() if "bearamin" in n.lower()]
        if not args.input and len(inputs) == 1:
            args.input = inputs[0]
        if not args.output and len(outputs) == 1:
            args.output = outputs[0]
        if not args.input or not args.output:
            parser.error("Connect one Bearamin, or use --list and choose --input and --output.")
    state, stop = State(), threading.Event()
    page = Path(__file__).with_name("overlay.html").read_bytes()
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path in ("/", "/overlay.html"):
                body, mime = page, "text/html; charset=utf-8"
            elif self.path == "/state":
                body, mime = json.dumps(state.get(), allow_nan=False).encode(), "application/json"
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", mime)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        def log_message(self, *args):
            pass
    def reader():
        try:
            if args.stdin:
                for line in sys.stdin:
                    if len(line) <= 8192:
                        try:
                            value = json.loads(line)
                            state.put(value.get("data") if value.get("op") == "field" else value)
                        except (ValueError, AttributeError):
                            pass
            else:
                midi_reader(state, stop, args.input, args.output)
        except Exception as error:
            print("Overlay input stopped:", error, file=sys.stderr)
        finally:
            stop.set()
    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    server.timeout = 0.25
    worker = threading.Thread(target=reader, daemon=True)
    worker.start()
    print(f"OBS Browser Source: http://127.0.0.1:{args.port}/", flush=True)
    try:
        while not stop.is_set():
            server.handle_request()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        worker.join(timeout=2)
        server.server_close()

if __name__ == "__main__":
    main()
