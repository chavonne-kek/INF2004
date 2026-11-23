#!/usr/bin/env python3
"""IS25 web dashboard: the robot's MQTT telemetry, live, in your browser.

Put this file next to is25_dashboard.py (INF2004-main/tools/) and run:

    py tools\\is25_web.py                       (live: the broker on this laptop)
    py tools\\is25_web.py --broker 10.222.14.184   (live, a given broker address)
    py tools\\is25_web.py --demo                (fake data, no robot)

Then open http://localhost:8025 (it opens by itself). Works with the car
firmware and with the barcode bench image (only the barcode panel fills in).

Options:
    --port-web 8025     web page port
    --log run1.csv      also save every message (evidence for the report)
    --ssid "The network"  the hotspot the laptop and robot should be on
    --broker auto       (default) this laptop's own address on the hotspot
    --car-host 184      the last number the car's firmware looks for
    --no-browser        do not open the browser automatically

The "Hotspot & MQTT link" panel shows, hop by hop, that the link works:
the laptop's Wi-Fi network (read from Windows / macOS / Linux), the broker
(connected, and a loopback ping through it every 2 s), the robot (online
status, heartbeat age, its Wi-Fi signal) and the command path ("Test"
sends one invalid byte; the robot counts it as a bad command, which proves
commands reach it without moving the car).  The "Demo rubric" panel ticks
each of the 10 demo items when the robot's own telemetry shows it.

It uses the decoders of is25_dashboard.py, so every value is decoded exactly
as the terminal dashboard does. Needs: pip install paho-mqtt (not for --demo).
"""

import argparse
import collections
import csv
import json
import os
import platform
import queue
import random
import re
import socket
import subprocess
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import is25_dashboard as d  # noqa: E402  (decoders, command encoders)

MOVE_NAMES = {1: "TURN LEFT", 2: "TURN RIGHT", 3: "STRAIGHT", 4: "U-TURN"}
# Code 39 patterns (subsystem1/barcode.c g_patterns), for --demo traces only.
CODE39_PATTERNS = [
    0x034, 0x121, 0x061, 0x160, 0x031, 0x130, 0x070, 0x025, 0x124, 0x064,
    0x109, 0x049, 0x148, 0x019, 0x118, 0x058, 0x00D, 0x10C, 0x04C, 0x01C,
    0x103, 0x043, 0x142, 0x013, 0x112, 0x052, 0x007, 0x106, 0x046, 0x016,
    0x181, 0x0C1, 0x1C0, 0x091, 0x190, 0x0D0, 0x085, 0x184, 0x0C4, 0x0A8,
    0x0A2, 0x08A, 0x02A]
MOVE_ICONS = {1: "left", 2: "right", 3: "straight", 4: "uturn"}
DEFAULT_MAP = {"A": 1, "B": 2, "C": 3, "D": 4}     # firmware default
HISTORY_MAX = 40
EXPECTED_SSID = "The network"        # the team's hotspot (--ssid to change)
CAR_HOST = 184                       # the car looks for <its subnet>.184
PING_TOPIC = "is25/web/ping/"        # broker loopback, never robot/...
PING_EVERY_S = 2.0
PROBE_TIMEOUT_S = 5.0
RUBRIC = [
    ("detect", "1. Detect barcode", "a barcode (or a rejected read) arrived"),
    ("decode", "2. Decode barcode", "a letter was decoded"),
    ("hump", "3. Measure hump height", "a hump record arrived"),
    ("follow", "4. Follow line", "LINE_FOLLOW with the line under the sensors"),
    ("instr", "5. Follow barcode instruction", "TURN_EXECUTE after a decoded letter"),
    ("obst", "6. Detect obstacle", "OBSTACLE_AVOID or an obstacle record"),
    ("width", "7. Measure obstacle width", "an obstacle width arrived"),
    ("bypass", "8. Go around, rejoin line", "OBSTACLE_AVOID, then LINE_FOLLOW"),
    ("mqtt", "9. Telemetry via MQTT", "heartbeats arriving from the robot"),
    ("lift", "10. Stop when picked up", "fault LIFTED, motors stopped"),
]


# --------------------------------------------------------------------------
# The laptop's own network: which Wi-Fi it is on, and its address toward the
# broker.  Read from the operating system; nothing here needs admin rights.
# --------------------------------------------------------------------------
def _run(cmd, timeout=4):
    kw = {}
    if os.name == "nt":
        kw["creationflags"] = 0x08000000          # CREATE_NO_WINDOW
    out = subprocess.run(cmd, capture_output=True, timeout=timeout, **kw)
    return out.stdout.decode("utf-8", "replace")


def _windows_profile_name():
    """Windows network-profile name of the adapter that holds this laptop's
    hotspot address.  For a Wi-Fi network it is the SSID (Windows may add
    " 2" for a second network of the same name).  Used when netsh is
    blocked: Windows 11 24H2 refuses "netsh wlan show interfaces" until
    location access is allowed for desktop apps."""
    ip = own_hotspot_ip()
    if not ip or not re.fullmatch(r"\d{1,3}(\.\d{1,3}){3}", ip):
        return None
    ps = ("$i=(Get-NetIPAddress -AddressFamily IPv4 -IPAddress '%s' "
          "-ErrorAction Stop).InterfaceIndex;"
          "(Get-NetConnectionProfile -InterfaceIndex $i "
          "-ErrorAction Stop).Name" % ip)
    text = _run(["powershell", "-NoProfile", "-NonInteractive",
                 "-Command", ps], timeout=10).strip()
    return text.splitlines()[0].strip() if text else None


def wifi_info():
    """{'ssid', 'signal' (%), 'how'} or {'error'}; never raises."""
    system = platform.system()
    try:
        if system == "Windows":
            text = _run(["netsh", "wlan", "show", "interfaces"])
            ssid = re.search(r"^\s*SSID\s*:\s*(.+?)\s*$", text, re.M)
            sig = re.search(r"^\s*Signal\s*:\s*(\d+)\s*%", text, re.M)
            state = re.search(r"^\s*State\s*:\s*(.+?)\s*$", text, re.M)
            if ssid:
                return {"ssid": ssid.group(1),
                        "signal": int(sig.group(1)) if sig else None,
                        "how": "netsh wlan show interfaces"}
            blocked = re.search(r"location", text, re.I) is not None
            name = _windows_profile_name()
            if name:
                return {"ssid": name, "signal": None,
                        "how": "network profile" + (
                            " (netsh needs location access)" if blocked
                            else "")}
            if blocked:
                return {"ssid": None, "error": "Windows hides the Wi-Fi "
                        "name (location access is off)"}
            return {"ssid": None, "error": "Wi-Fi %s" % (
                state.group(1) if state else "not connected")}
        if system == "Darwin":
            for dev in ("en0", "en1"):
                text = _run(["networksetup", "-getairportnetwork", dev])
                m = re.search(r"Current Wi-Fi Network:\s*(.+)", text)
                if m:
                    return {"ssid": m.group(1).strip(), "signal": None,
                            "how": "networksetup"}
            text = _run(["ipconfig", "getsummary", "en0"])
            m = re.search(r"^\s*SSID\s*:\s*(.+)$", text, re.M)
            if m:
                return {"ssid": m.group(1).strip(), "signal": None,
                        "how": "ipconfig getsummary"}
            return {"ssid": None, "error": "Wi-Fi not connected"}
        text = _run(["nmcli", "-t", "-f", "ACTIVE,SSID,SIGNAL", "dev", "wifi"])
        for line in text.splitlines():
            parts = line.split(":")
            if len(parts) >= 3 and parts[0] == "yes":
                return {"ssid": ":".join(parts[1:-1]),
                        "signal": int(parts[-1]) if parts[-1].isdigit()
                        else None, "how": "nmcli"}
        text = _run(["iwgetid", "-r"]).strip()
        if text:
            return {"ssid": text, "signal": None, "how": "iwgetid"}
        return {"ssid": None, "error": "Wi-Fi not connected"}
    except FileNotFoundError:
        return {"ssid": None, "error": "cannot read Wi-Fi on this computer"}
    except Exception as exc:                       # timeout, odd output
        return {"ssid": None, "error": str(exc)[:80]}


def own_hotspot_ip():
    """This laptop's address on the network with the default route (the
    hotspot), or None.  No packet is sent."""
    ip = local_ip_toward("8.8.8.8", 53)
    return None if (ip is None or ip.startswith("127.")) else ip


def local_ip_toward(host, port):
    """This computer's IP on the route to host (no packet is sent)."""
    try:
        sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sk.connect((host, port))
            return sk.getsockname()[0]
        finally:
            sk.close()
    except OSError:
        return None


# --------------------------------------------------------------------------
# Hub: decodes every message once and fans it out to all open pages.
# --------------------------------------------------------------------------
class Hub:
    def __init__(self, log_path=None, expected_ssid=EXPECTED_SSID,
                 broker=None, port=1883, car_host=CAR_HOST):
        self.lock = threading.Lock()
        self.expected_ssid = expected_ssid
        self.car_host = car_host
        self.broker_host = broker
        self.broker_port = port
        self.broker_since = None
        self.reconnects = 0
        self.topic_stats = {}            # topic -> [count, first t, last t]
        self.hb_times = collections.deque(maxlen=30)
        self.robot_status = None
        self.robot_status_t = None
        self.ping = {"sent": 0, "got": 0, "rtt_ms": None, "last_ok": None}
        self.ping_out = {}               # id -> send time
        self.probe = None
        self.wifi = {"ssid": None, "error": "checking..."}
        self.faults_prev = None
        self.check = {k: None for k, _, _ in RUBRIC}
        self.decoded_once = False
        self.prev_state = None
        self.clients = []
        self.latest = {}                 # topic -> decoded dict
        self.history = []                # barcode reads and rejects
        self.traces = []                 # 18 Nov: what the sensor saw, per read
        self.bc_parts = {}               # seq -> {part: decoded part}
        self.events = []                 # state / hump / obstacle / commands
        self.map = dict(DEFAULT_MAP)
        self.counts = {"reads": 0, "rejects": 0}
        self.broker = "connecting"
        self.last_msg = 0.0
        self.log_fh = None
        self.log = None
        if log_path:
            self.log_fh = open(log_path, "w", newline="")
            self.log = csv.writer(self.log_fh)
            self.log.writerow(["time", "topic", "hex", "decoded"])

    # -- incoming -----------------------------------------------------------
    def on_message(self, topic, payload):
        now = time.time()
        try:
            data = d.decode(topic, payload)
        except Exception as exc:          # malformed payload: show, not crash
            data = {"error": str(exc), "raw": payload.hex()}
        if self.log:
            self.log.writerow(["%.3f" % now, topic, payload.hex(),
                               json.dumps(data)])
            self.log_fh.flush()
        msg = {"type": "msg", "topic": topic, "data": data, "t": now}
        notes = []
        with self.lock:
            self.last_msg = now
            self.latest[topic] = data
            st = self.topic_stats.setdefault(topic, [0, now, now])
            st[0] += 1
            st[2] = now
            notes = self._track(now, topic, data)
            if topic == "robot/telemetry/barcode":
                entry = dict(data, t=now)
                if data.get("rejected"):
                    self.counts["rejects"] += 1
                else:
                    self.counts["reads"] += 1
                    mv = self.map.get(data.get("char"))
                    entry["move"] = MOVE_NAMES.get(mv, "not mapped")
                    entry["icon"] = MOVE_ICONS.get(mv, "none")
                self.history.insert(0, entry)
                del self.history[HISTORY_MAX:]
                msg["entry"] = entry
                msg["counts"] = dict(self.counts)
            elif topic in d.EVENT_TOPICS or topic == d.TOPIC_STATUS:
                self._event(now, topic, data)
            elif topic == "robot/telemetry/bctrace" and "part" in data:
                trace = self._trace_part(now, data)
                if trace:
                    msg["trace"] = trace
        self.broadcast(msg)

    def _trace_part(self, now, data):
        """Collect the parts of one barcode trace; return it when complete.
        Called with the lock held."""
        seq, part = data["seq"], data["part"]
        if part == 0:
            self.bc_parts[seq] = {0: data}
        elif seq in self.bc_parts:
            self.bc_parts[seq][part] = data
        parts = self.bc_parts.get(seq)
        if not parts or 0 not in parts:
            return None
        head = parts[0]
        n = head["count"]
        need = 1 + (max(0, n - 1) + 6) // 7
        if any(k not in parts for k in range(need)):
            return None
        raw = list(head["w"])
        for k in range(1, need):
            raw += parts[k]["w"]
        raw = raw[:n]
        unit = max(1, head["unit"])
        if head["travel"]:
            widths = [round(w * unit / 16.0, 2) for w in raw]
            units = "mm"
        else:
            widths = [w * unit for w in raw]
            units = "ms"
        trace = {"t": now, "n": n, "result": head["result"], "units": units,
                 "widths": widths, "arm_hi": head["arm_hi"],
                 "arm_lo": head["arm_lo"]}
        del self.bc_parts[seq]
        self.traces.insert(0, trace)
        del self.traces[6:]
        return trace
        for text in notes:
            self.note(text)

    def _tick(self, key, now):
        if self.check.get(key) is None:
            self.check[key] = now

    def _track(self, now, topic, data):
        """Link statistics, fault edges, the command probe and the rubric
        checklist.  Called with the lock held; returns notes to log."""
        notes = []
        if topic == "robot/heartbeat":
            self.hb_times.append(now)
            if len(self.hb_times) >= 3:
                self._tick("mqtt", now)
            faults = set(data.get("fault_names", []))
            if self.faults_prev is not None:
                for f in sorted(faults - self.faults_prev):
                    notes.append("fault %s raised" % f)
                for f in sorted(self.faults_prev - faults):
                    notes.append("fault %s cleared" % f)
                if "LIFTED" in faults and "LIFTED" not in self.faults_prev:
                    notes.append("ROBOT LIFTED - motors stopped, run over")
            self.faults_prev = faults
            if "LIFTED" in faults:
                self._tick("lift", now)
            self._state(now, data.get("state_name"))
        elif topic == d.TOPIC_STATUS:
            self.robot_status = data.get("status")
            self.robot_status_t = now
        elif topic == "robot/telemetry/state":
            self._state(now, data.get("state_name"))
        elif topic == "robot/telemetry/barcode":
            self._tick("detect", now)
            if not data.get("rejected"):
                self._tick("decode", now)
                self.decoded_once = True
        elif topic == "robot/telemetry/hump":
            if data.get("last_mm"):
                self._tick("hump", now)
        elif topic == "robot/telemetry/obstacle":
            if data.get("detected"):
                self._tick("obst", now)
            if data.get("width_cm") is not None:
                self._tick("width", now)
        elif topic == "robot/telemetry/line":
            if self.prev_state == "LINE_FOLLOW" and data.get("valid"):
                self._tick("follow", now)
        elif topic == "robot/telemetry/comms" and self.probe:
            pr = self.probe
            bad = data.get("bad_commands")
            if pr["result"] is None and bad is not None:
                if pr["base"] is None:
                    pr["base"] = bad
                elif bad > pr["base"] or (bad == 255 and pr["base"] == 255):
                    pr["result"] = "ok"
                    pr["ms"] = int(1000 * (now - pr["t"]))
                    notes.append("command path OK: robot counted the test "
                                 "byte within %.1f s" % (now - pr["t"]))
        return notes

    def _state(self, now, name):
        if not name or name == self.prev_state:
            return
        prev, self.prev_state = self.prev_state, name
        if name == "TURN_EXECUTE" and self.decoded_once:
            self._tick("instr", now)
        if name == "OBSTACLE_AVOID":
            self._tick("obst", now)
        if prev == "OBSTACLE_AVOID" and name == "LINE_FOLLOW":
            self._tick("bypass", now)

    # -- link status --------------------------------------------------------
    def on_ping(self, payload):
        now = time.time()
        with self.lock:
            t = self.ping_out.pop(payload.decode("ascii", "replace"), None)
            if t is not None:
                self.ping["got"] += 1
                self.ping["rtt_ms"] = round(1000 * (now - t), 1)
                self.ping["last_ok"] = now

    def new_ping(self):
        now = time.time()
        with self.lock:
            self.ping["sent"] += 1
            pid = "%d-%d" % (os.getpid(), self.ping["sent"])
            self.ping_out[pid] = now
            for k in [k for k, t in self.ping_out.items() if now - t > 10]:
                del self.ping_out[k]
        return pid

    def start_probe(self):
        with self.lock:
            cm = self.latest.get("robot/telemetry/comms", {})
            self.probe = {"t": time.time(), "result": None, "ms": None,
                          "base": cm.get("bad_commands")}

    def reset_checklist(self):
        with self.lock:
            self.check = {k: None for k, _, _ in RUBRIC}
            self.decoded_once = False

    def net_status(self):
        now = time.time()
        with self.lock:
            wifi = dict(self.wifi)
            hb = list(self.hb_times)
            hb_age = (now - hb[-1]) if hb else None
            hb_rate = ((len(hb) - 1) / (hb[-1] - hb[0])
                       if len(hb) >= 2 and hb[-1] > hb[0] else None)
            tele = [now - last for tp, (n, first, last)
                    in self.topic_stats.items()
                    if tp.startswith("robot/telemetry/")]
            tele_age = min(tele) if tele else None
            beat = self.latest.get("robot/heartbeat", {})
            comms = self.latest.get("robot/telemetry/comms", {})
            topics = []
            total_rate = 0.0
            for tp, (n, first, last) in sorted(self.topic_stats.items()):
                span = max(1.0, min(now, last) - first)
                rate = (n - 1) / span if n > 1 and now - last < 5 else 0.0
                total_rate += rate
                topics.append({"topic": tp, "n": n, "age": now - last,
                               "rate": round(rate, 1)})
            ping = dict(self.ping)
            ping["age"] = (now - ping["last_ok"]) if ping["last_ok"] else None
            probe = dict(self.probe) if self.probe else None
            if probe and probe["result"] is None and \
                    now - probe["t"] > PROBE_TIMEOUT_S:
                self.probe["result"] = probe["result"] = "timeout"
            check = [{"key": k, "name": n, "how": h, "t": self.check[k]}
                     for k, n, h in RUBRIC]
            status = self.robot_status
            broker = self.broker
            since = self.broker_since
            reconnects = self.reconnects
        laptop_ip = (local_ip_toward(self.broker_host, self.broker_port)
                     if self.broker_host else None)
        if laptop_ip is None or laptop_ip.startswith("127."):
            laptop_ip = own_hotspot_ip() or laptop_ip
        car_ok = (None if not laptop_ip or laptop_ip.startswith("127.") else
                  laptop_ip.rsplit(".", 1)[-1] == str(self.car_host))
        ssid = wifi.get("ssid")
        want = self.expected_ssid.strip().lower()
        on_hotspot = (None if not ssid else
                      ssid.strip().lower() == want or
                      # A Windows profile name: "The network 2" is the same
                      # Wi-Fi joined a second time.
                      (wifi.get("how", "").startswith("network profile") and
                       re.fullmatch(re.escape(want) + r"( \d+)?",
                                    ssid.strip().lower()) is not None))
        broker_local = bool(self.broker_host) and (
            self.broker_host in ("127.0.0.1", "localhost") or
            (laptop_ip is not None and laptop_ip == self.broker_host))
        robot_up = status != "offline" and hb_age is not None and hb_age < 3.5
        return {"type": "net", "now": now,
                "wifi": wifi, "expected_ssid": self.expected_ssid,
                "on_hotspot": on_hotspot, "laptop_ip": laptop_ip,
                "car_host": self.car_host, "car_ok": car_ok,
                "broker": {"host": self.broker_host, "port": self.broker_port,
                           "status": broker, "since": since,
                           "reconnects": reconnects, "local": broker_local},
                "ping": ping,
                "robot": {"status": status, "hb_age": hb_age,
                          "tele_age": tele_age,
                          "hb_rate": hb_rate, "online": robot_up,
                          "signal_dbm": beat.get("signal_dbm"),
                          "uptime_s": beat.get("uptime_s"),
                          "drops": comms.get("drops"),
                          "bad_commands": comms.get("bad_commands")},
                "probe": probe, "topics": topics,
                "msg_rate": round(total_rate, 1), "check": check}

    def _event(self, now, topic, data):
        self.events.insert(0, {"t": now, "topic": topic, "data": data})
        del self.events[HISTORY_MAX:]

    def note(self, text):
        now = time.time()
        with self.lock:
            self._event(now, "note", {"text": text})
        self.broadcast({"type": "note", "text": text, "t": now})

    def set_broker(self, status):
        with self.lock:
            if status == "connected":
                if self.broker_since is not None:
                    self.reconnects += 1
                self.broker_since = time.time()
            elif not status.startswith("demo"):
                self.ping_out.clear()
        self.broker = status
        self.broadcast({"type": "broker", "status": status})

    # -- outgoing -----------------------------------------------------------
    def snapshot(self):
        with self.lock:
            return {"type": "snapshot", "latest": self.latest,
                    "history": self.history, "events": self.events,
                    "traces": self.traces,
                    "map": {k: MOVE_NAMES[v] for k, v in self.map.items()},
                    "counts": self.counts, "broker": self.broker,
                    "now": time.time()}

    def subscribe(self):
        q = queue.Queue(maxsize=500)
        with self.lock:
            self.clients.append(q)
        return q

    def unsubscribe(self, q):
        with self.lock:
            if q in self.clients:
                self.clients.remove(q)

    def broadcast(self, msg):
        text = json.dumps(msg)
        with self.lock:
            clients = list(self.clients)
        for q in clients:
            try:
                q.put_nowait(text)
            except queue.Full:
                pass                      # a stalled tab just misses data

    def set_map(self, letter, move):
        with self.lock:
            self.map[letter] = move
            mp = {k: MOVE_NAMES[v] for k, v in self.map.items()}
        self.broadcast({"type": "map", "map": mp})


# --------------------------------------------------------------------------
# Commands typed or clicked on the page, encoded like the terminal dashboard.
# --------------------------------------------------------------------------
def parse_command(text, hub):
    """Return (raw bytes, description) or raise ValueError."""
    words = text.split()
    if not words:
        raise ValueError("empty command")
    cmd = words[0].lower()
    if cmd == "probe":
        hub.start_probe()
        return b"\x00", "command-path test (one invalid byte 0x00; the " \
                         "robot counts it as a bad command and does nothing)"
    if cmd == "checklist-reset":
        hub.reset_checklist()
        return None, "demo checklist cleared"
    if cmd == "map":
        if len(words) < 3:
            raise ValueError("use: map <A..Z> <left|right|forward|uturn>")
        raw = d.encode_map(words[1], words[2])
        letter = words[1].strip().upper()[:1]
        hub.set_map(letter, raw[1] >> 5)
        return raw, "map %s -> %s" % (letter, MOVE_NAMES[raw[1] >> 5])
    if cmd not in d.RCMD:
        raise ValueError("unknown command '%s'" % cmd)
    arg = 0
    if cmd in ("speed", "gain"):
        if len(words) < 2:
            raise ValueError("%s needs a value" % cmd)
        arg = int(words[1])
    raw = d.encode_command(cmd, arg)
    return raw, "%s%s (0x%s)" % (cmd, " %d" % arg if arg else "", raw.hex())


# --------------------------------------------------------------------------
# Web server
# --------------------------------------------------------------------------
def make_handler(hub, send):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):     # keep the console quiet
            pass

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                body = PAGE.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == "/events":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                q = hub.subscribe()
                try:
                    self._send(json.dumps(hub.snapshot()))
                    while True:
                        try:
                            self._send(q.get(timeout=10))
                        except queue.Empty:
                            self.wfile.write(b": keep-alive\n\n")
                            self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError,
                        ConnectionAbortedError, OSError):
                    pass
                finally:
                    hub.unsubscribe(q)
            else:
                self.send_error(404)

        def _send(self, text):
            self.wfile.write(("data: %s\n\n" % text).encode("utf-8"))
            self.wfile.flush()

        def do_POST(self):
            if self.path != "/cmd":
                self.send_error(404)
                return
            n = int(self.headers.get("Content-Length", "0") or 0)
            text = self.rfile.read(n).decode("utf-8", "replace")
            try:
                raw, desc = parse_command(text, hub)
                if raw is not None:
                    send(raw)
                    hub.note("sent " + desc)
                else:
                    hub.note(desc)
                reply = {"ok": True, "sent": desc}
            except Exception as exc:
                reply = {"ok": False, "error": str(exc)}
            body = json.dumps(reply).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    return Handler


# --------------------------------------------------------------------------
# Live MQTT
# --------------------------------------------------------------------------
def start_mqtt(args, hub):
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        print("paho-mqtt is not installed:  py -m pip install paho-mqtt")
        sys.exit(1)

    def on_connect(c, userdata, flags, rc, properties=None):
        ok = (rc == 0) or (str(rc) == "Success")
        hub.set_broker("connected" if ok else "refused (%s)" % rc)
        print("broker %s:%d: %s" % (args.broker, args.port,
                                    "connected" if ok else rc))
        c.subscribe(d.TOPIC_PREFIX + "#", qos=1)
        c.subscribe(PING_TOPIC + "#", qos=0)

    def on_disconnect(c, userdata, *rest):
        hub.set_broker("disconnected, retrying")

    def on_message(c, userdata, msg):
        if msg.topic.startswith(PING_TOPIC):
            hub.on_ping(msg.payload)
        elif msg.topic != d.TOPIC_COMMAND:
            hub.on_message(msg.topic, msg.payload)

    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                             client_id="is25-web-%d" % os.getpid())
    except AttributeError:
        client = mqtt.Client(client_id="is25-web-%d" % os.getpid())
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    client.reconnect_delay_set(1, 5)
    try:
        client.connect_async(args.broker, args.port, keepalive=30)
    except Exception as exc:
        hub.set_broker("error: %s" % exc)
    client.loop_start()

    def send(raw):
        info = client.publish(d.TOPIC_COMMAND, raw, qos=1)
        if info.rc != 0:
            raise ValueError("not connected to the broker")

    def pinger():
        # Broker loopback: publish to our own ping topic, time the echo.
        while True:
            time.sleep(PING_EVERY_S)
            if hub.broker == "connected":
                pid = hub.new_ping()
                client.publish(PING_TOPIC + str(os.getpid()),
                               pid.encode(), qos=0)

    threading.Thread(target=pinger, daemon=True).start()
    return send


# --------------------------------------------------------------------------
# Demo: fake robot that sends payloads in the firmware's binary format, so
# the page is exercised through exactly the same decode path as live data.
# --------------------------------------------------------------------------
def start_demo(hub):
    hub.set_broker("demo (no robot)")
    sim = {"state": 0, "speed": 3, "running": False, "t0": time.time(),
           "dist": 0.0, "next_bc": time.time() + 4, "faults": 0,
           "run_t": 0.0, "bad": 0}

    def put(topic, value, size):
        hub.on_message(topic, int(value).to_bytes(size, "little"))

    seq = [0]

    def put_trace(rnd, result_byte, ch=None, rejected_n=0):
        """Fake robot/telemetry/bctrace parts like line_task.c post_trace:
        *ch* at 3.5 mm narrow / 10.5 mm wide, with some jitter."""
        if ch is not None:
            widths = []
            for k, c in enumerate("*" + ch + "*"):
                pat = CODE39_PATTERNS[d.CODE39.index(c)] if c != "*" \
                    else 0x094
                for e in range(9):
                    wide = (pat >> (8 - e)) & 1
                    widths.append((10.5 if wide else 3.5) *
                                  rnd.uniform(0.85, 1.15))
                if k < 2:
                    widths.append(3.5 * rnd.uniform(0.85, 1.15))
        else:
            widths = [rnd.uniform(2.5, 12) for _ in range(rejected_n)]
        raw = [int(w * 16) for w in widths]
        unit = max(1, (max(raw) + 254) // 255)
        q = [min(255, (w + unit // 2) // unit) for w in raw]
        seq[0] = (seq[0] + 1) & 0x0F
        n = len(q)
        parts = [bytes([seq[0] << 4, n, result_byte, 1, unit,
                        rnd.randint(220, 250), rnd.randint(0, 5), q[0]])]
        for k in range(1, 5):
            chunk = q[7 * k - 6:7 * k + 1]
            if not chunk:
                break
            parts.append(bytes([(seq[0] << 4) | k] + chunk +
                               [0] * (7 - len(chunk))))
        for part in parts:
            hub.on_message("robot/telemetry/bctrace", part)

    def send(raw):
        op = raw[0] & 0x07
        if raw[0] == d.RCMD["map"] and len(raw) == 2:
            return
        if not d.command_is_valid(raw):
            sim["bad"] = min(255, sim.get("bad", 0) + 1)
            return
        if op == 1 and sim["state"] in (1, 6):
            sim["running"] = True
            sim["state"] = 2
            sim["run_t"] = time.time()
        elif op in (2, 4):
            sim["running"] = False
            sim["state"] = 6 if op == 2 else 7
        elif op == 3:
            sim["running"] = True
            sim["state"] = 2
        elif op == 5:
            sim["running"] = False
            sim["state"] = 1
            sim["faults"] = 0
        elif op == 6:
            sim["speed"] = raw[0] >> 3
        put("robot/telemetry/state", sim["state"], 1)

    def loop():
        rnd = random.Random(7)
        put("robot/telemetry/state", 0, 1)
        hub.on_message(d.TOPIC_STATUS, b"online")
        tick = 0
        while True:
            time.sleep(0.2)
            tick += 1
            now = time.time()
            up = int(now - sim["t0"])
            if sim["state"] == 0 and up >= 4:
                sim["state"] = 1
                put("robot/telemetry/state", 1, 1)
            run = sim["running"]
            if run:
                # A demo run: a barcode turn, an obstacle bypass, then the
                # examiner lifts the car after 60 s (fault LIFTED, FINISHED).
                k = int((now - sim["run_t"]) * 5)
                new = sim["state"]
                if k in (60, 160):
                    new = 3                               # TURN_EXECUTE
                elif k in (66, 166):
                    new = 2
                elif k == 110:
                    new = 4                               # OBSTACLE_AVOID
                elif k == 135:
                    new = 2
                elif k >= 300:
                    new, run = 9, False                   # FINISHED
                    sim["running"] = False
                    sim["faults"] |= 1 << 5               # LIFTED
                if new != sim["state"]:
                    sim["state"] = new
                    put("robot/telemetry/state", new, 1)
            v = sim["speed"] * 10 if run else 0
            sim["dist"] += v * 0.2
            err = int(round(6 * rnd.uniform(-1, 1))) if run else 0
            arm = rnd.randint(250, 400)
            if run and (tick % 25) in range(10, 14):
                arm = rnd.choice([380, 2900, 3000, 420])   # passing bars
            if tick % 5 == 0:
                put("robot/heartbeat", sim["state"] | (sim["faults"] << 4) |
                    (48 << 10) | (min(up, 4095) << 17), 4)
            put("robot/telemetry/line", (err + 40) | (1 << 7), 2)
            lw = v - 4 * err
            rw = v + 4 * err
            put("robot/telemetry/motion",
                (int(sim["dist"]) & 0x7FFF) |
                ((max(0, min(127, lw // 5 + 64))) << 15) |
                ((max(0, min(127, rw // 5 + 64))) << 22) |
                ((int(sim["dist"] * 6) & 0xFFFF) << 32) |
                ((int(sim["dist"] * 6) & 0xFFFF) << 48), 8)
            pitch = rnd.randint(-2, 2)
            put("robot/telemetry/imu", (pitch + 128) | (90 << 8) |
                (0 << 17) | ((7 if run else 0) << 20), 3)
            put("robot/telemetry/ir", arm | (rnd.randint(1500, 1800) << 16) |
                (rnd.randint(250, 350) << 32), 6)
            if tick % 5 == 0:
                put("robot/telemetry/comms", (sim.get("bad", 0) << 8) |
                    (hub.counts["rejects"] << 16), 3)
                pid = hub.new_ping()
                hub.on_ping(pid.encode())            # pretend broker echo
            if now >= sim["next_bc"]:
                sim["next_bc"] = now + rnd.uniform(4, 7)
                if rnd.random() < 0.8:
                    ch = rnd.choice("ABCDX")
                    b = d.CODE39.index(ch) | ((rnd.random() < 0.3) << 6)
                    put("robot/telemetry/barcode", b, 1)
                    put_trace(rnd, b, ch)
                else:
                    n = rnd.randint(9, 15)
                    b = 0x80 | (n << 3) | 5
                    put("robot/telemetry/barcode", b, 1)
                    put_trace(rnd, b, None, n)
            if tick % 150 == 75:
                put("robot/telemetry/hump", 23 | (31 << 7), 2)
            if tick % 150 == 120:
                put("robot/telemetry/obstacle", 14 | (22 << 7) | (8 << 14) |
                    (35 << 19) | (60 << 26) | (1 << 33), 5)

    threading.Thread(target=loop, daemon=True).start()
    return send


# --------------------------------------------------------------------------
# The page (one file, no external libraries, works offline)
# --------------------------------------------------------------------------
PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>IS25 Live</title>
<style>
:root{--bg:#f4f5f7;--card:#fff;--ink:#16181d;--mute:#6b7280;--line:#e3e5ea;
--ok:#15803d;--okbg:#e7f6ec;--bad:#b91c1c;--badbg:#fdeaea;--warn:#a16207;--warnbg:#fdf3dc;
--acc:#2457d6;--accbg:#e8eefc;--bar:#1f2937}
@media (prefers-color-scheme:dark){:root{--bg:#0f1115;--card:#181b21;--ink:#e8eaee;--mute:#9aa1ad;
--line:#2a2f38;--okbg:#12301d;--badbg:#3a1616;--warnbg:#3a2e10;--accbg:#18264a;--bar:#d6dae1;--ok:#4ade80;--bad:#f87171;--warn:#fbbf24;--acc:#7aa2ff}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);
font:14px/1.4 system-ui,-apple-system,Segoe UI,Roboto,sans-serif}
header{display:flex;flex-wrap:wrap;gap:10px;align-items:center;padding:12px 16px;background:var(--card);
border-bottom:1px solid var(--line);position:sticky;top:0;z-index:2}
header h1{font-size:16px;margin:0 8px 0 0}
.pill{padding:3px 10px;border-radius:999px;font-weight:600;font-size:12px;background:var(--line);color:var(--ink)}
.pill.ok{background:var(--okbg);color:var(--ok)}.pill.bad{background:var(--badbg);color:var(--bad)}
.pill.warn{background:var(--warnbg);color:var(--warn)}.pill.acc{background:var(--accbg);color:var(--acc)}
.sp{flex:1}.mute{color:var(--mute)}
main{display:grid;gap:12px;padding:12px 16px;grid-template-columns:repeat(12,1fr);max-width:1400px;margin:0 auto}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px;min-width:0}
.card h2{font-size:12px;letter-spacing:.06em;text-transform:uppercase;color:var(--mute);margin:0 0 10px}
.c4{grid-column:span 4}.c5{grid-column:span 5}.c7{grid-column:span 7}.c8{grid-column:span 8}.c3{grid-column:span 3}.c6{grid-column:span 6}.c12{grid-column:span 12}
@media (max-width:1000px){.c3,.c4,.c5,.c6,.c7,.c8{grid-column:span 12}}
#bcbox{display:flex;gap:18px;align-items:center;border-radius:10px;padding:14px;transition:background .3s}
#bcbox.flash-ok{background:var(--okbg)}#bcbox.flash-bad{background:var(--badbg)}
#bcchar{font-size:96px;font-weight:800;line-height:1;width:120px;text-align:center;font-variant-numeric:tabular-nums}
#bcmove{font-size:24px;font-weight:700}#bcsub{color:var(--mute);margin-top:4px}
#bcicon svg{width:64px;height:64px}
.stats{display:flex;gap:16px;margin-top:12px;flex-wrap:wrap}
.stat b{display:block;font-size:22px;font-variant-numeric:tabular-nums}.stat span{font-size:12px;color:var(--mute)}
table{width:100%;border-collapse:collapse;font-size:13px}td,th{padding:5px 6px;border-bottom:1px solid var(--line);text-align:left}
th{color:var(--mute);font-weight:600;font-size:12px}
.hist{max-height:260px;overflow:auto}
.ir{display:flex;gap:14px;align-items:flex-end;height:170px}
.irc{flex:1;display:flex;flex-direction:column;align-items:center;gap:6px}
.irt{position:relative;width:100%;max-width:70px;height:130px;background:var(--line);border-radius:8px;overflow:hidden}
.irf{position:absolute;bottom:0;left:0;right:0;background:var(--bar);transition:height .15s}
.irv{font-weight:700;font-variant-numeric:tabular-nums}
.thr{position:absolute;left:0;right:0;border-top:2px dashed var(--acc);opacity:.6}
canvas{width:100%;height:90px;display:block}
#spark,#trace{height:auto}
.gauge{position:relative;height:28px;background:linear-gradient(90deg,var(--badbg),var(--okbg) 40%,var(--okbg) 60%,var(--badbg));border-radius:8px}
.gauge .mk{position:absolute;top:-4px;width:4px;height:36px;background:var(--ink);border-radius:2px;transition:left .15s}
.gauge .mid{position:absolute;left:50%;top:0;bottom:0;border-left:1px dashed var(--mute)}
.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin-top:8px}
.kv{display:grid;grid-template-columns:auto 1fr;gap:4px 12px;font-variant-numeric:tabular-nums}
.kv div:nth-child(odd){color:var(--mute)}
.wheels{display:flex;gap:12px}.wh{flex:1}.wht{height:10px;background:var(--line);border-radius:6px;overflow:hidden}
.whf{height:100%;background:var(--acc);transition:width .15s}
button{font:inherit;padding:7px 12px;border-radius:8px;border:1px solid var(--line);background:var(--card);color:var(--ink);cursor:pointer}
button:hover{border-color:var(--acc)}button.go{background:var(--ok);color:#fff;border:0}button.stop{background:var(--bad);color:#fff;border:0}
input,select{font:inherit;padding:6px 8px;border-radius:8px;border:1px solid var(--line);background:var(--card);color:var(--ink)}
#cmdmsg{min-height:20px;margin-top:6px;font-size:13px}
.log{max-height:220px;overflow:auto;font-size:12px;font-family:ui-monospace,Consolas,monospace}
.log div{padding:2px 0;border-bottom:1px solid var(--line)}
.big{font-size:28px;font-weight:800;font-variant-numeric:tabular-nums}
.lights{display:flex;gap:6px;flex-wrap:wrap}
#verdict{display:flex;align-items:center;gap:10px;padding:10px 12px;border-radius:10px;font-weight:700;font-size:15px;background:var(--line)}
#verdict.ok{background:var(--okbg);color:var(--ok)}#verdict.bad{background:var(--badbg);color:var(--bad)}#verdict.warn{background:var(--warnbg);color:var(--warn)}
.chain{display:grid;grid-template-columns:1fr auto 1fr auto 1fr auto 1fr;gap:8px;align-items:stretch;margin-top:12px}
@media (max-width:1000px){.chain{grid-template-columns:1fr}.arrow{display:none}}
.hop{border:1px solid var(--line);border-radius:10px;padding:10px 12px;display:flex;gap:10px;min-width:0}
.hop b{font-size:13px}.hv{font-size:15px;font-weight:700;margin-top:2px;overflow-wrap:anywhere}.hs{font-size:12px;color:var(--mute);margin-top:2px;overflow-wrap:anywhere}
.dot{flex:none;width:12px;height:12px;border-radius:50%;margin-top:3px;background:var(--line)}
.dot.ok{background:var(--ok)}.dot.bad{background:var(--bad)}.dot.warn{background:var(--warn)}
.arrow{align-self:center;color:var(--mute);font-size:18px}
.topics{max-height:170px;overflow:auto;margin-top:10px}
.rub{display:grid;grid-template-columns:1fr 1fr;gap:6px 18px}
@media (max-width:700px){.rub{grid-template-columns:1fr}}
.ri{display:flex;gap:8px;align-items:flex-start;padding:6px 8px;border-radius:8px;border:1px solid var(--line)}
.ri.done{background:var(--okbg);border-color:transparent}
.ri .tk{flex:none;width:18px;height:18px;border-radius:50%;border:2px solid var(--mute);margin-top:1px}
.ri.done .tk{background:var(--ok);border-color:var(--ok)}
.ri small{display:block;color:var(--mute)}
#liftbanner{display:none;grid-column:span 12;background:var(--badbg);color:var(--bad);border-radius:12px;padding:14px 16px;font-weight:800;font-size:18px}
</style></head><body>
<header>
 <h1>IS25 Live</h1>
 <span id="broker" class="pill">broker …</span>
 <span id="robot" class="pill">robot …</span>
 <span id="state" class="pill acc">state –</span>
 <span id="faults"></span>
 <span class="sp"></span>
 <span class="mute" id="hb">–</span>
</header>
<main>
 <div id="liftbanner">Robot LIFTED: motors stopped, the run is over (press GP21 or Reset for the next run)</div>
 <section class="card c12" id="netcard">
  <h2>Hotspot &amp; MQTT link</h2>
  <div id="verdict">Checking the link…</div>
  <div class="chain">
   <div class="hop"><div class="dot" id="d-wifi"></div><div><b>Laptop Wi-Fi</b><div class="hv" id="h-wifi">–</div><div class="hs" id="s-wifi">–</div></div></div>
   <div class="arrow">→</div>
   <div class="hop"><div class="dot" id="d-broker"></div><div><b>MQTT broker</b><div class="hv" id="h-broker">–</div><div class="hs" id="s-broker">–</div></div></div>
   <div class="arrow">→</div>
   <div class="hop"><div class="dot" id="d-robot"></div><div><b>Robot (Pico W)</b><div class="hv" id="h-robot">–</div><div class="hs" id="s-robot">–</div></div></div>
   <div class="arrow">→</div>
   <div class="hop"><div class="dot" id="d-cmd"></div><div><b>Commands to robot</b><div class="hv" id="h-cmd">not tested</div><div class="hs" id="s-cmd">Sends one invalid byte: the car does not move</div><div class="row" style="margin-top:6px"><button id="probe">Test command path</button></div></div></div>
  </div>
  <div class="topics"><table><thead><tr><th>Topic</th><th>Messages</th><th>Rate</th><th>Last</th></tr></thead><tbody id="topics"></tbody></table></div>
 </section>
 <section class="card c7">
  <h2>Barcode</h2>
  <div id="bcbox">
   <div id="bcchar">–</div>
   <div><div id="bcicon"></div><div id="bcmove">Waiting for a barcode</div><div id="bcsub">Swipe or drive over a label</div></div>
  </div>
  <div class="stats">
   <div class="stat"><b id="nread">0</b><span>read</span></div>
   <div class="stat"><b id="nrej">0</b><span>rejected</span></div>
   <div class="stat"><b id="rate">–</b><span>success</span></div>
   <div class="stat"><b id="ago">–</b><span>since last</span></div>
  </div>
 </section>
 <section class="card c5">
  <h2>Barcode history</h2>
  <div class="hist"><table><thead><tr><th>Time</th><th>Result</th><th>Detail</th></tr></thead><tbody id="hist"></tbody></table></div>
 </section>

 <section class="card c4">
  <h2>IR sensors (raw 0–4095)</h2>
  <div class="ir">
   <div class="irc"><div class="irt"><div class="irf" id="f-arm"></div></div><div class="irv" id="v-arm">–</div><div class="mute">arm / barcode</div></div>
   <div class="irc"><div class="irt"><div class="irf" id="f-centre"></div></div><div class="irv" id="v-centre">–</div><div class="mute">centre</div></div>
   <div class="irc"><div class="irt"><div class="irf" id="f-right"></div></div><div class="irv" id="v-right">–</div><div class="mute">right</div></div>
  </div>
 </section>
 <section class="card c8">
  <h2>Barcode sensor, last 30 s: black or white</h2>
  <canvas id="spark" width="900" height="70"></canvas>
  <div class="row mute" style="font-size:12px;margin-top:6px;gap:8px">
   <span>Black above</span><input id="thr" type="number" min="0" max="4095" step="50" placeholder="auto" style="width:80px"><span id="thrinfo"></span>
  </div>
  <div class="mute" style="font-size:12px;margin-top:4px">A few readings a second: shows when the sensor is over black, but is too slow to draw every bar. Every bar is in the panel below.</div>
 </section>
 <section class="card c12">
  <h2>What the barcode sensor saw on each read (1000 readings a second, the robot's own black / white)</h2>
  <canvas id="trace" width="1200" height="300"></canvas>
  <div class="mute" style="font-size:12px;margin-top:6px" id="traceinfo">Waiting for the robot to read a barcode. A good read is 29 bars and spaces: 3 characters of 5 bars and 4 spaces, with a narrow space between characters. Fewer means bars were missed or merged; "arm" is how dark the darkest bar and how light the lightest space looked (0 white – 1000 black: bars must reach about 600).</div>
 </section>

 <section class="card c4">
  <h2>Line</h2>
  <div class="big" id="lerr">–</div>
  <div class="gauge"><div class="mid"></div><div class="mk" id="lmk" style="left:50%"></div></div>
  <div class="row mute" style="justify-content:space-between;font-size:12px"><span>−40 (steer right)</span><span>+40 (steer left)</span></div>
  <div class="lights row"><span id="lvalid" class="pill">valid</span><span id="llost" class="pill">lost</span><span id="ljunc" class="pill">junction</span></div>
 </section>
 <section class="card c4">
  <h2>Motion</h2>
  <div class="wheels">
   <div class="wh"><div class="mute">Left <b id="wl">–</b> mm/s</div><div class="wht"><div class="whf" id="wlf" style="width:0"></div></div></div>
   <div class="wh"><div class="mute">Right <b id="wr">–</b> mm/s</div><div class="wht"><div class="whf" id="wrf" style="width:0"></div></div></div>
  </div>
  <div class="kv" style="margin-top:12px"><div>Distance</div><div id="dist">–</div><div>Turn rate</div><div id="turn">–</div><div>Encoder L / R</div><div id="enc">–</div></div>
 </section>
 <section class="card c4">
  <h2>IMU and hump</h2>
  <div class="kv"><div>Pitch</div><div id="pitch">–</div><div>Heading</div><div id="head">–</div><div>Hump state</div><div id="hump">–</div><div>Motion</div><div id="imot">–</div><div>Last hump</div><div id="hlast">–</div><div>Max hump</div><div id="hmax">–</div></div>
 </section>

 <section class="card c4">
  <h2>Obstacle</h2>
  <div class="kv"><div>Detected</div><div id="odet">–</div><div>Width</div><div id="owid">–</div><div>Closest</div><div id="oclo">–</div><div>Left clear</div><div id="olc">–</div><div>Right clear</div><div id="orc">–</div></div>
 </section>
 <section class="card c4">
  <h2>Control</h2>
  <div class="row"><button class="go" data-c="start">Start</button><button data-c="pause">Pause</button><button data-c="resume">Resume</button><button class="stop" data-c="estop">E-stop</button><button data-c="reset">Reset</button></div>
  <div class="row"><label>Speed <input id="spd" type="number" min="2" max="25" value="3" style="width:64px"> cm/s</label><button id="spdb">Set</button>
   <label>Gain <input id="gn" type="number" min="100" max="2500" step="100" value="500" style="width:80px"></label><button id="gnb">Set</button></div>
  <div class="row"><label>Map <select id="ml"></select></label><span>→</span>
   <select id="mm"><option value="left">Turn left</option><option value="right">Turn right</option><option value="forward">Straight</option><option value="uturn">U-turn</option></select><button id="mb">Send</button></div>
  <div id="cmdmsg" class="mute"></div>
 </section>
 <section class="card c4">
  <h2>Barcode map (sent from this page)</h2>
  <table><tbody id="maptab"></tbody></table>
  <div class="mute" style="font-size:12px;margin-top:6px">The robot forgets the map at power-up: send it again before each run.</div>
 </section>

 <section class="card c12">
  <h2>Demo rubric (ticked from the robot's own telemetry)</h2>
  <div class="rub" id="rubric"></div>
  <div class="row"><button id="rubreset">Clear for a new run</button><span class="mute" style="font-size:12px">Only the latest of the 3 runs counts: clear it before each run. Save the page or the --log CSV as evidence.</span></div>
 </section>
 <section class="card c12">
  <h2>Events</h2>
  <div class="log" id="log"></div>
 </section>
</main>
<script>
const $=id=>document.getElementById(id);
const IR_MAX=4095, SPARK_S=30, spark=[];
let counts={reads:0,rejects:0}, lastBc=0, lastHb=0, lastTele=0, robotStatus='';
const ICON={left:'<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.4" stroke-linecap="round" stroke-linejoin="round"><path d="M19 20v-7a4 4 0 0 0-4-4H5"/><path d="M9 5 5 9l4 4"/></svg>',
right:'<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.4" stroke-linecap="round" stroke-linejoin="round"><path d="M5 20v-7a4 4 0 0 1 4-4h10"/><path d="m15 5 4 4-4 4"/></svg>',
straight:'<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.4" stroke-linecap="round" stroke-linejoin="round"><path d="M12 20V4"/><path d="m6 10 6-6 6 6"/></svg>',
uturn:'<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.4" stroke-linecap="round" stroke-linejoin="round"><path d="M8 20V9a5 5 0 0 1 10 0v4"/><path d="m14 10 4 4 4-4" transform="translate(-4 0)"/></svg>',
none:'',bad:'<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.4" stroke-linecap="round"><circle cx="12" cy="12" r="9"/><path d="m9 9 6 6M15 9l-6 6"/></svg>'};
const tstr=t=>new Date(t*1000).toLocaleTimeString();
function pill(el,text,cls){el.textContent=text;el.className='pill '+(cls||'');}
function log(text,t){const d=document.createElement('div');d.textContent=tstr(t||Date.now()/1000)+'  '+text;$('log').prepend(d);while($('log').children.length>200)$('log').lastChild.remove();}
function setCounts(c){counts=c;$('nread').textContent=c.reads;$('nrej').textContent=c.rejects;
 const n=c.reads+c.rejects;$('rate').textContent=n?Math.round(100*c.reads/n)+'%':'–';}
function histRow(e,top){const tr=document.createElement('tr');
 if(e.rejected){tr.innerHTML=`<td>${tstr(e.t)}</td><td><span class="pill bad">REJECT r=${e.reason}</span></td><td>${e.why} (n=${e.elements})</td>`;}
 else{tr.innerHTML=`<td>${tstr(e.t)}</td><td><span class="pill ok">'${e.char}'</span></td><td>${e.move||''}${e.reversed?' · reversed':''}</td>`;}
 const tb=$('hist'); top?tb.prepend(tr):tb.append(tr); while(tb.children.length>40)tb.lastChild.remove();}
function showBarcode(e,flash){lastBc=e.t;const box=$('bcbox');
 if(e.rejected){$('bcchar').textContent='✕';$('bcicon').innerHTML=ICON.bad;$('bcicon').style.color='var(--bad)';
  $('bcmove').textContent='Rejected';$('bcsub').textContent=e.why+' (reason '+e.reason+')';}
 else{$('bcchar').textContent=e.char;$('bcicon').innerHTML=ICON[e.icon]||'';$('bcicon').style.color='var(--ok)';
  $('bcmove').textContent=e.move||'';$('bcsub').textContent=(e.reversed?'Read in reverse':'Read forwards')+' · '+tstr(e.t);}
 if(flash){box.className=e.rejected?'flash-bad':'flash-ok';setTimeout(()=>box.className='',1200);}}
function bar(id,v){const el=$('f-'+id);el.style.height=Math.max(0,Math.min(100,100*v/IR_MAX))+'%';$('v-'+id).textContent=v;}
function drawSpark(){const c=$('spark'),g=c.getContext('2d'),W=c.width,H=c.height;g.clearRect(0,0,W,H);
 const now=Date.now()/1000, css=getComputedStyle(document.documentElement);
 const pts=spark.filter(([t])=>now-t<=SPARK_S);
 let thr=parseInt($('thr').value,10), auto=isNaN(thr);
 if(auto){let lo=Infinity,hi=-Infinity;pts.forEach(([,v])=>{lo=Math.min(lo,v);hi=Math.max(hi,v);});
  thr=(hi-lo>=600)?Math.round((lo+hi)/2):2000;}
 $('thrinfo').textContent=`raw ${thr}${auto?' (auto: half way between the lightest and darkest of the last 30 s)':''}`;
 g.fillStyle='#fff';g.fillRect(0,0,W,H);g.strokeStyle=css.getPropertyValue('--line');g.lineWidth=2;g.strokeRect(1,1,W-2,H-2);
 g.fillStyle='#111';
 for(let i=0;i<pts.length;i++){const [t,v]=pts[i];if(v<=thr)continue;
  const tEnd=(i+1<pts.length)?pts[i+1][0]:now;const x0=W-(now-t)/SPARK_S*W,x1=W-(now-tEnd)/SPARK_S*W;
  g.fillRect(Math.max(0,x0),2,Math.max(2,x1-x0),H-4);}
 g.fillStyle=css.getPropertyValue('--mute');g.font='16px system-ui';g.fillText('30 s ago',8,H-8);const tw=g.measureText('now').width;g.fillText('now',W-tw-8,H-8);}
const traces=[];
function drawTraces(){const c=$('trace'),g=c.getContext('2d'),W=c.width,H=c.height;g.clearRect(0,0,W,H);
 const css=getComputedStyle(document.documentElement);
 if(!traces.length){g.fillStyle=css.getPropertyValue('--mute');g.font='18px system-ui';g.fillText('No barcode read yet',10,30);return;}
 const rows=Math.min(traces.length,5),rowH=H/rows,lab=170;
 traces.slice(0,rows).forEach((tr,r)=>{const y=r*rowH+4,h=rowH-30;
  const tot=tr.widths.reduce((a,b)=>a+b,0)||1,scale=(W-lab-10)/tot;
  g.fillStyle='#fff';g.fillRect(lab,y,W-lab-10,h);
  let x=lab;tr.widths.forEach((w,i)=>{const ww=w*scale;if(i%2===0){g.fillStyle='#111';g.fillRect(x,y,Math.max(1,ww),h);}x+=ww;});
  g.strokeStyle=css.getPropertyValue('--line');g.lineWidth=1;g.strokeRect(lab,y,W-lab-10,h);
  const res=tr.result||{};const ok=!res.rejected;
  g.fillStyle=ok?css.getPropertyValue('--ok'):css.getPropertyValue('--bad');g.font='bold 22px system-ui';
  g.fillText(ok?`'${res.char}'${res.reversed?' rev':''}`:'REJECT',8,y+h/2+4);
  g.fillStyle=css.getPropertyValue('--mute');g.font='14px system-ui';
  g.fillText(`${tr.n} of 29 · ${tstr(tr.t)}`,8,y+h/2+24);
  const nar=Math.min(...tr.widths),wid=Math.max(...tr.widths);
  g.fillText(`arm ${tr.arm_lo}–${tr.arm_hi} of 1000 · narrowest ${nar} ${tr.units}, widest ${wid} ${tr.units}`+(ok?'':` · ${res.why||''}`),lab,y+h+16);});
 const t=traces[0];$('traceinfo').textContent=`Last read: ${t.n} of 29 elements, widths in ${t.units==='mm'?'mm the sensor travelled (from the wheel encoders)':'ms (the car was not moving)'}: ${t.widths.join(', ')}.`;}
function addTrace(t){traces.unshift(t);traces.length=Math.min(traces.length,6);drawTraces();}
setInterval(drawSpark,250);
setInterval(()=>{const now=Date.now()/1000;$('ago').textContent=lastBc?Math.round(now-lastBc)+' s':'–';
 if(lastHb){const age=now-lastHb;if(age>3&&robotStatus!=='offline')pill($('robot'),(now-lastTele<2?'heartbeat late ':'robot silent ')+Math.round(age)+' s','warn');}},500);
function setMap(m){const tb=$('maptab');tb.innerHTML='';const sel=$('ml'),cur=sel.value;sel.innerHTML='';
 Object.keys(m).sort().forEach(k=>{tb.insertAdjacentHTML('beforeend',`<tr><td><b>${k}</b></td><td>${m[k]}</td></tr>`);});
 'ABCDEFGHIJKLMNOPQRSTUVWXYZ'.split('').forEach(k=>sel.insertAdjacentHTML('beforeend',`<option>${k}</option>`));if(cur)sel.value=cur;}
function apply(topic,v,t){
 if(topic.startsWith('robot/telemetry/'))lastTele=t;
 switch(topic){
 case 'robot/heartbeat': lastHb=t;pill($('state'),v.state_name,'acc');
  $('faults').innerHTML=v.fault_names.map(f=>`<span class="pill bad">${f}</span>`).join(' ');
  $('liftbanner').style.display=v.fault_names.includes('LIFTED')?'block':'none';
  $('hb').textContent=`signal ${v.signal_dbm??'–'} dBm · up ${v.uptime_s} s`;
  if(robotStatus!=='offline')pill($('robot'),'robot online','ok');break;
 case 'robot/status': robotStatus=v.status;pill($('robot'),'robot '+v.status,v.status==='online'?'ok':'bad');break;
 case 'robot/telemetry/state': pill($('state'),v.state_name,'acc');break;
 case 'robot/telemetry/ir': bar('arm',v.arm);bar('centre',v.centre);bar('right',v.right);spark.push([t,v.arm]);while(spark.length&&t-spark[0][0]>SPARK_S+1)spark.shift();break;
 case 'robot/telemetry/line': $('lerr').textContent=(v.error_mm>0?'+':'')+v.error_mm+' mm';$('lmk').style.left=(50+v.error_mm*1.25)+'%';
  pill($('lvalid'),'valid',v.valid?'ok':'');pill($('llost'),'lost',v.lost?'bad':'');pill($('ljunc'),'junction',v.junction?'warn':'');break;
 case 'robot/telemetry/motion': $('wl').textContent=v.left_mm_s;$('wr').textContent=v.right_mm_s;
  $('wlf').style.width=Math.min(100,Math.abs(v.left_mm_s)/3)+'%';$('wrf').style.width=Math.min(100,Math.abs(v.right_mm_s)/3)+'%';
  $('dist').textContent=v.distance_mm+' mm';$('turn').textContent=Math.round((v.right_mm_s-v.left_mm_s)/120*57.3)+' °/s';$('enc').textContent=v.left_count+' / '+v.right_count;break;
 case 'robot/telemetry/imu': $('pitch').textContent=v.pitch_deg.toFixed(2)+' °';$('head').textContent=v.heading_deg==null?'n/a':v.heading_deg+' °';
  $('hump').textContent=v.hump_name;$('imot').textContent=v.motion_name;break;
 case 'robot/telemetry/hump': $('hlast').textContent=v.last_mm+' mm';$('hmax').textContent=v.max_mm+' mm';break;
 case 'robot/telemetry/obstacle': $('odet').innerHTML=v.detected?'<span class="pill bad">YES</span>':'no';
  const cm=x=>x==null?'out of range':x+' cm';$('owid').textContent=cm(v.width_cm);$('oclo').textContent=cm(v.closest_cm)+' at '+v.closest_angle_deg+'°';
  $('olc').textContent=cm(v.left_clear_cm);$('orc').textContent=cm(v.right_clear_cm);break;
 }}
function describe(topic,v){
 if(topic==='robot/telemetry/barcode')return v.rejected?`barcode REJECTED: ${v.why}`:`barcode '${v.char}'${v.reversed?' (reversed)':''}`;
 if(topic==='robot/telemetry/state')return 'state → '+v.state_name;
 if(topic==='robot/telemetry/hump')return `hump: last ${v.last_mm} mm, max ${v.max_mm} mm`;
 if(topic==='robot/telemetry/obstacle')return `obstacle: width ${v.width_cm??'?'} cm, closest ${v.closest_cm??'?'} cm`;
 if(topic==='robot/status')return 'robot '+v.status;
 if(topic==='note')return v.text;return topic;}
const es=new EventSource('/events');
es.onopen=()=>{};
es.onerror=()=>pill($('broker'),'page lost the server (is is25_web.py running?)','bad');
es.onmessage=ev=>{const m=JSON.parse(ev.data);
 if(m.type==='snapshot'){pill($('broker'),'broker '+m.broker,m.broker==='connected'||m.broker.startsWith('demo')?'ok':'warn');
  for(const [tp,v] of Object.entries(m.latest))apply(tp,v,m.now);
  $('hist').innerHTML='';m.history.forEach(e=>histRow(e,false));if(m.history.length)showBarcode(m.history[0],false);
  setCounts(m.counts);setMap(m.map);$('log').innerHTML='';m.events.slice().reverse().forEach(e=>log(describe(e.topic,e.data),e.t));
  traces.length=0;(m.traces||[]).forEach(t=>traces.push(t));drawTraces();return;}
 if(m.type==='broker'){pill($('broker'),'broker '+m.status,m.status==='connected'?'ok':'warn');log('broker '+m.status);return;}
 if(m.type==='net'){renderNet(m);return;}
 if(m.type==='map'){setMap(m.map);return;}
 if(m.type==='note'){log(m.text,m.t);return;}
 if(m.type==='msg'){apply(m.topic,m.data,m.t);
  if(m.trace)addTrace(m.trace);
  if(m.entry){showBarcode(m.entry,true);histRow(m.entry,true);setCounts(m.counts);log(describe(m.topic,m.data),m.t);}
  else if(['robot/telemetry/state','robot/telemetry/hump','robot/telemetry/obstacle','robot/status'].includes(m.topic))log(describe(m.topic,m.data),m.t);}};
async function cmd(text){const r=await fetch('/cmd',{method:'POST',body:text});const j=await r.json();
 $('cmdmsg').textContent=j.ok?'Sent: '+j.sent:'Error: '+j.error;$('cmdmsg').style.color=j.ok?'var(--ok)':'var(--bad)';}
document.querySelectorAll('button[data-c]').forEach(b=>b.onclick=()=>cmd(b.dataset.c));
$('spdb').onclick=()=>cmd('speed '+$('spd').value);$('gnb').onclick=()=>cmd('gain '+$('gn').value);
$('mb').onclick=()=>cmd('map '+$('ml').value+' '+$('mm').value);
$('probe').onclick=()=>{cmd('probe');pill2('d-cmd','warn');$('h-cmd').textContent='testing…';};
$('rubreset').onclick=()=>cmd('checklist-reset');
function pill2(id,cls){$(id).className='dot '+(cls||'');}
function ago(s){if(s==null)return 'never';if(s<1)return 'just now';if(s<90)return Math.round(s)+' s ago';return Math.round(s/60)+' min ago';}
function renderNet(n){
 const w=n.wifi||{},b=n.broker||{},r=n.robot||{},p=n.ping||{};
 // 1. laptop Wi-Fi
 if(w.ssid){$('h-wifi').textContent=w.ssid;
  pill2('d-wifi',n.on_hotspot?'ok':'warn');
  $('s-wifi').textContent=(n.on_hotspot?'the team hotspot':'expected "'+n.expected_ssid+'"')+(w.signal!=null?' · signal '+w.signal+'%':'')+(n.laptop_ip?' · laptop '+n.laptop_ip:'');}
 else{$('h-wifi').textContent=w.error||'unknown';pill2('d-wifi',n.laptop_ip?'warn':'bad');
  $('s-wifi').textContent='expected "'+n.expected_ssid+'"'+(n.laptop_ip?' · laptop '+n.laptop_ip:'');}
 if(n.car_ok===false){pill2('d-wifi','warn');$('s-wifi').textContent+=' · the car looks for .'+n.car_host+' on this hotspot, the laptop is .'+n.laptop_ip.split('.').pop();}
 // 2. broker
 const bok=b.status==='connected'||String(b.status).startsWith('demo');
 $('h-broker').textContent=(b.host?b.host+':'+b.port:'demo')+' · '+b.status;
 const pingOk=p.age!=null&&p.age<6;
 pill2('d-broker',bok?(pingOk?'ok':'warn'):'bad');
 $('s-broker').textContent=(b.local?'running on this laptop · ':'')+(pingOk?'loopback '+p.rtt_ms+' ms':'no loopback reply yet')+(b.since?' · up '+ago(n.now-b.since).replace(' ago',''):'')+(b.reconnects?' · '+b.reconnects+' reconnects':'');
 // 3. robot
 let rcls='bad',rtxt='no messages yet';
 if(r.status==='offline'){rtxt='offline (last will)';}
 else if(r.online){rcls='ok';rtxt='online · heartbeat '+ago(r.hb_age);}
 else if(r.tele_age!=null&&r.tele_age<2&&r.status!=='offline'){rcls='warn';rtxt='sending telemetry, heartbeat late'+(r.hb_age!=null?' ('+ago(r.hb_age)+')':'');}
 else if(r.hb_age!=null){rcls='warn';rtxt='silent · heartbeat '+ago(r.hb_age);}
 else if(r.status==='online'){rcls='warn';rtxt='online (retained), no heartbeat yet';}
 pill2('d-robot',rcls);$('h-robot').textContent=rtxt;
 const sig=r.signal_dbm==null?'signal n/a':'Wi-Fi '+r.signal_dbm+' dBm ('+(r.signal_dbm>=-60?'good':r.signal_dbm>=-72?'ok':'weak')+')';
 $('s-robot').textContent=sig+(r.uptime_s!=null?' · up '+r.uptime_s+' s':'')+' · '+n.msg_rate+' msg/s'+(r.drops!=null?' · dropped '+r.drops+', bad cmds '+r.bad_commands:'');
 // 4. command path
 const pr=n.probe;
 if(pr){if(pr.result==='ok'){pill2('d-cmd','ok');$('h-cmd').textContent='reaches the robot';$('s-cmd').textContent='the robot counted the test byte (tested '+ago(n.now-pr.t)+')';}
  else if(pr.result==='timeout'){pill2('d-cmd','bad');$('h-cmd').textContent='no answer';$('s-cmd').textContent='the robot did not count the test byte in 5 s';}
  else{pill2('d-cmd','warn');$('h-cmd').textContent='testing…';}}
 // verdict
 const v=$('verdict');let cls='bad',txt;
 if(!bok){txt='Broker not connected: start Mosquitto on the laptop ('+(b.host||'?')+') and check the firewall';}
 else if(rcls==='ok'&&n.on_hotspot!==false){cls='ok';txt='MQTT working: '+(w.ssid?'"'+w.ssid+'" → ':'')+'broker → robot online'+(pr&&pr.result==='ok'?' → commands reach the robot':'');}
 else if(rcls==='ok'){cls='warn';txt='MQTT working, but the laptop is on "'+w.ssid+'", not "'+n.expected_ssid+'"';}
 else if(r.status==='offline'){txt='Robot went offline: check its battery and that it is near the hotspot';}
 else{cls='warn';txt='Broker OK, waiting for the robot: power it on (MQTT image) on "'+n.expected_ssid+'"';}
 v.className=cls;v.textContent=txt;
 // topics
 $('topics').innerHTML=n.topics.map(t=>`<tr><td>${t.topic}</td><td>${t.n}</td><td>${t.rate}/s</td><td>${ago(t.age)}</td></tr>`).join('');
 // rubric
 $('rubric').innerHTML=n.check.map(c=>`<div class="ri${c.t?' done':''}"><div class="tk"></div><div><b>${c.name}</b><small>${c.t?'seen at '+tstr(c.t):c.how}</small></div></div>`).join('');
}
</script></body></html>
"""


def start_status(hub):
    """Laptop Wi-Fi every 5 s; the link panel to every page every second."""
    def wifi_loop():
        while True:
            info = wifi_info()
            with hub.lock:
                hub.wifi = info
            time.sleep(5)

    def push_loop():
        while True:
            time.sleep(1)
            try:
                hub.broadcast(hub.net_status())
            except Exception as exc:       # never let the panel kill the app
                print("status error:", exc)

    threading.Thread(target=wifi_loop, daemon=True).start()
    threading.Thread(target=push_loop, daemon=True).start()


def main():
    ap = argparse.ArgumentParser(description="IS25 web dashboard")
    ap.add_argument("--broker", default="auto",
                    help="broker IP; default auto = this laptop on the hotspot")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--port-web", type=int, default=8025)
    ap.add_argument("--log", help="CSV file for every message (evidence)")
    ap.add_argument("--demo", action="store_true", help="fake robot data")
    ap.add_argument("--no-browser", action="store_true")
    ap.add_argument("--ssid", default=EXPECTED_SSID,
                    help="hotspot name the laptop should be on")
    ap.add_argument("--car-host", type=int, default=CAR_HOST,
                    help="last number of the broker address the car uses")
    args = ap.parse_args()
    if args.demo:
        args.broker = None
    elif args.broker == "auto":
        # The broker runs on this laptop (tools/mosquitto.conf listens on
        # every address): connect to the laptop's hotspot address, which
        # is also what the car connects to.
        args.broker = own_hotspot_ip() or "127.0.0.1"
        print("broker: this laptop, %s:%d" % (args.broker, args.port))

    hub = Hub(args.log, args.ssid, args.broker, args.port, args.car_host)
    send = start_demo(hub) if args.demo else start_mqtt(args, hub)
    start_status(hub)
    server = ThreadingHTTPServer(("127.0.0.1", args.port_web),
                                 make_handler(hub, send))
    server.daemon_threads = True
    url = "http://localhost:%d" % args.port_web
    print("IS25 web dashboard on %s  (Ctrl+C to stop)" % url)
    if not args.no_browser:
        threading.Timer(0.8, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    if hub.log_fh:
        hub.log_fh.close()
        print("log written to", args.log)


if __name__ == "__main__":
    main()
