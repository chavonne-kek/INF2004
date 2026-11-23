#!/usr/bin/env python3
"""
is25_dashboard.py -- IS25 robot dashboard: decodes the binary MQTT payloads
and sends remote commands.  Subsystem 5 (Buddy 1, Chavonne).

Every payload layout here mirrors the firmware's pack functions exactly
(see app_program/subsystem5/MQTT_PROTOCOL.md).  `--selftest` checks this
decoder against vectors produced by the firmware's own C pack functions
(testing/subsystem5/roundtrip), so robot and dashboard cannot drift apart
silently (Tribunal 1, risk R2).

Usage:
    pip install paho-mqtt
    python tools/is25_dashboard.py --broker 192.168.1.10
    python tools/is25_dashboard.py --broker test.mosquitto.org --log run1.csv
    python tools/is25_dashboard.py --selftest vectors.jsonl

While running, type a command and press Enter:
    start | pause | resume | estop | reset | speed <2..25> | gain <100..2500>
    map <A..Z> <f|l|r|u>   (char -> movement, sent to the car over MQTT)
    bad | quit
("gain" sets the steering gain live in mm/s of wheel difference per mm of
line error: higher corners harder but weaves, lower is smoother but runs wide.)
("bad" sends 20 malformed commands: the robot must ignore all of them and
count them in robot/telemetry/comms -- Tribunal 1 interface test.)
"""

import argparse
import csv
import json
import struct
import sys
import threading
import time

# --------------------------------------------------------------------------
# Constants mirrored from the firmware headers.
# --------------------------------------------------------------------------
STATES = ["INIT_CALIBRATE", "IDLE", "LINE_FOLLOW", "TURN_EXECUTE",
          "OBSTACLE_AVOID", "LINE_RECOVERY", "PAUSED", "EMERGENCY_STOP",
          "FAULT", "FINISHED"]                              # subsystem4.h
# Nov 2026: bit 5 was COLLISION; it is now LIFTED (robot lifted or
# overturned: the run is over, motors stopped).
FAULTS = ["STALL", "ENCODER", "IMU", "NO_ECHO", "WIFI", "LIFTED"]
HUMPS = ["FLAT", "CLIMBING", "CRESTING", "DESCENDING", "INVALID"]  # S3
MOTIONS = ["STATIONARY", "ACCELERATING", "TURNING", "CLIMBING",
           "DESCENDING", "LIFTED", "DECELERATING", "CRUISING"]     # S3
CODE39 = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%"           # S1
BARCODE_CMDS = {"A": "TURN_LEFT", "B": "TURN_RIGHT", "C": "STRAIGHT",
                "D": "U_TURN"}
LINKS = ["CONNECTING", "ONLINE", "OFFLINE", "FLUSH"]

RCMD = {"start": 1, "pause": 2, "resume": 3, "estop": 4, "reset": 5,
        "speed": 6, "gain": 7, "map": 8}                  # S4_RCMD_* opcodes

# Barcode character -> movement (subsystem1_cmd_t): the four characters are
# announced by the instructor and sent from here, never fixed in the firmware.
MAP_MOVES = {"f": 3, "forward": 3, "straight": 3,
             "l": 1, "left": 1,
             "r": 2, "right": 2,
             "u": 4, "uturn": 4, "u-turn": 4}
SPEED_MIN, SPEED_MAX = 2, 25
GAIN_MIN, GAIN_MAX = 100, 2500        # mm/s per mm (the robot rejects above)

TOPIC_PREFIX = "robot/"
TOPIC_COMMAND = "robot/command"
TOPIC_STATUS = "robot/status"


def _bits(value, shift, width):
    return (value >> shift) & ((1 << width) - 1)


def _u(data, size):
    """Little-endian unsigned integer of the first `size` bytes."""
    if len(data) < size:
        raise ValueError("payload too short: %d < %d bytes" % (len(data), size))
    return int.from_bytes(data[:size], "little")


# --------------------------------------------------------------------------
# Decoders: one per topic.  Each returns a dict of plain values.
# --------------------------------------------------------------------------
def decode_heartbeat(data):
    w = _u(data, 4)
    faults = _bits(w, 4, 6)
    signal = _bits(w, 10, 7)
    state = _bits(w, 0, 4)
    return {
        "state": state,
        "state_name": STATES[state] if state < len(STATES) else "?",
        "faults": faults,
        "fault_names": [n for i, n in enumerate(FAULTS) if faults & (1 << i)],
        "signal_dbm": -signal if signal else None,
        "uptime_s": _bits(w, 17, 12),
    }


def decode_line(data):
    w = _u(data, 2)
    return {
        "error_mm": _bits(w, 0, 7) - 40,
        "valid": bool(w & (1 << 7)),
        "lost": bool(w & (1 << 8)),
        "junction": bool(w & (1 << 9)),
    }


def decode_motion(data):
    w = _u(data, 4)
    return {
        "distance_mm": _bits(w, 0, 15),
        "left_mm_s": (_bits(w, 15, 7) - 64) * 5,
        "right_mm_s": (_bits(w, 22, 7) - 64) * 5,
        "left_count": _u(data[4:], 2),
        "right_count": _u(data[6:], 2),
    }


def decode_imu(data):
    w = _u(data, 3)
    heading = _bits(w, 8, 9)
    hump = _bits(w, 17, 3)
    motion = _bits(w, 20, 3)
    return {
        "pitch_q": _bits(w, 0, 8) - 128,
        "pitch_deg": (_bits(w, 0, 8) - 128) * 0.25,
        "heading_deg": None if heading == 511 else heading,
        "hump": hump,
        "hump_name": HUMPS[hump] if hump < len(HUMPS) else "?",
        "motion": motion,
        "motion_name": MOTIONS[motion],
        "hump_active": bool(w & (1 << 23)),
    }


REJECT_REASONS = {1: "too many bars (noise/smudge)",
                  2: "wide/narrow unclear (blur: lower the sensor)",
                  3: "29 elements but not Code 39",
                  4: "black too long (sensor on black)",
                  5: "stopped before 29 (swipe the whole label)"}


def decode_barcode(data):
    b = _u(data, 1)
    if b & 0x80:
        # A rejected read (bit 7), not a character: count in bits 6..3,
        # reason in bits 2..0 (line_task.c, 2 Nov).
        reason = b & 0x07
        return {"rejected": True, "elements": (b >> 3) & 0x0F,
                "reason": reason,
                "why": REJECT_REASONS.get(reason, "?")}
    index = _bits(b, 0, 6)
    char = CODE39[index] if index < len(CODE39) else "?"
    return {
        "index": index,
        "char": char,
        "reversed": bool(b & (1 << 6)),
        "command": BARCODE_CMDS.get(char, "NONE"),
    }


def decode_hump(data):
    w = _u(data, 2)
    return {"last_mm": _bits(w, 0, 7), "max_mm": _bits(w, 7, 7)}


def decode_obstacle(data):
    w = _u(data, 5)

    def rng(v):
        return None if v == 127 else v

    index = _bits(w, 14, 5)
    return {
        "width_cm": rng(_bits(w, 0, 7)),
        "closest_cm": rng(_bits(w, 7, 7)),
        "closest_index": index,
        "closest_angle_deg": 30 + 5 * index,
        "left_clear_cm": rng(_bits(w, 19, 7)),
        "right_clear_cm": rng(_bits(w, 26, 7)),
        "detected": bool(w & (1 << 33)),
    }


def decode_state(data):
    s = _bits(_u(data, 1), 0, 4)
    return {"state": s, "state_name": STATES[s] if s < len(STATES) else "?"}


def decode_ir(data):
    """Raw IR levels: arm (barcode), centre, right."""
    w = _u(data, 6)
    return {"arm": w & 0xFFFF, "centre": (w >> 16) & 0xFFFF,
            "right": (w >> 32) & 0xFFFF}


def decode_range(data):
    """22 Nov: the obstacle watch's live view - front distance and hits."""
    cm = data[0]
    return {"front_cm": None if cm == 255 else cm, "no_echo": cm == 255,
            "hits": data[1]}


def decode_bctrace(data):
    """One 8-byte part of a barcode trace (18 Nov, line_task.c post_trace):
    the element widths the robot measured on its last barcode read.
    Part 0: seq<<4|0, count, result byte (as robot/telemetry/barcode),
    flags (bit 0: widths in travel, unit x 1/16 mm, else time, unit x 1 ms),
    unit, darkest/4, lightest/4 (arm 0..1000), width 0.
    Part k (1..4): seq<<4|k, widths 7k-6 .. 7k (width / unit, rounded)."""
    b = bytes(data)
    if len(b) != 8:
        raise ValueError("bctrace part must be 8 bytes")
    seq, part = b[0] >> 4, b[0] & 0x0F
    if part == 0:
        return {"seq": seq, "part": 0, "count": b[1],
                "result": decode_barcode(bytes([b[2]])),
                "travel": bool(b[3] & 1), "unit": b[4],
                "arm_hi": b[5] * 4, "arm_lo": b[6] * 4, "w": [b[7]]}
    return {"seq": seq, "part": part, "w": list(b[1:8])}


def decode_comms(data):
    w = _u(data, 3)
    return {"drops": w & 0xFF, "bad_commands": (w >> 8) & 0xFF,
            "barcode_rejects": w >> 16}


TRACK_MM = 120   # MOTION_TRACK_MM in subsystem2/motion_calc.h


def turn_rate(motion):
    """Turn rate (deg/s, + = left) from the two wheel speeds."""
    if "left_mm_s" not in motion:
        return "-"
    rad_s = (motion["right_mm_s"] - motion["left_mm_s"]) / float(TRACK_MM)
    return int(round(rad_s * 57.2958))


DECODERS = {
    "robot/heartbeat": decode_heartbeat,
    "robot/telemetry/line": decode_line,
    "robot/telemetry/motion": decode_motion,
    "robot/telemetry/imu": decode_imu,
    "robot/telemetry/barcode": decode_barcode,
    "robot/telemetry/hump": decode_hump,
    "robot/telemetry/obstacle": decode_obstacle,
    "robot/telemetry/state": decode_state,
    "robot/telemetry/comms": decode_comms,
    "robot/telemetry/ir": decode_ir,
    "robot/telemetry/range": decode_range,
    "robot/telemetry/bctrace": decode_bctrace,
}
EVENT_TOPICS = {"robot/telemetry/barcode", "robot/telemetry/hump",
                "robot/telemetry/obstacle", "robot/telemetry/state"}


def decode(topic, payload):
    if topic == TOPIC_STATUS:
        return {"status": payload.decode("ascii", "replace")}
    fn = DECODERS.get(topic)
    if fn is None:
        return {"raw": payload.hex()}
    return fn(payload)


# --------------------------------------------------------------------------
# Commands: 1 byte, opcode bits 0-2, speed (cm/s) bits 3-7 (subsystem4.h).
# --------------------------------------------------------------------------
def encode_command(name, speed=0):
    op = RCMD[name]
    if name == "speed":
        if not SPEED_MIN <= speed <= SPEED_MAX:
            raise ValueError("speed must be %d..%d cm/s" % (SPEED_MIN, SPEED_MAX))
        return bytes([op | (speed << 3)])
    if name == "gain":
        if not GAIN_MIN <= speed <= GAIN_MAX:
            raise ValueError("gain must be %d..%d" % (GAIN_MIN, GAIN_MAX))
        return bytes([op | ((speed // 100) << 3)])
    return bytes([op])


def encode_map(letter, movement):
    """2-byte frame: [S4_RCMD_MAP, (movement << 5) | letter index]."""
    idx = ord(str(letter).strip().upper()[:1] or "?") - ord("A")
    if not 0 <= idx < 26:
        raise ValueError("letter must be A..Z")
    mv = MAP_MOVES.get(str(movement).strip().lower())
    if mv is None:
        raise ValueError("movement must be f/forward, l/left, r/right, u/uturn")
    return bytes([RCMD["map"], (mv << 5) | idx])


def command_is_valid(raw):
    """Same rule as comms_command_decode() in the firmware."""
    if len(raw) == 2:
        if raw[0] != RCMD["map"]:
            return False
        return 1 <= (raw[1] >> 5) <= 4 and (raw[1] & 0x1F) < 26
    if len(raw) != 1:
        return False
    op, arg = raw[0] & 7, raw[0] >> 3
    if op == RCMD["speed"]:
        return SPEED_MIN <= arg <= SPEED_MAX
    if op == RCMD["gain"]:
        return 1 <= arg <= GAIN_MAX // 100
    return RCMD["start"] <= op <= RCMD["reset"] and arg == 0


BAD_COMMANDS = [b"", b"\x00", b"\x07", b"\x0e", b"\x08", b"\x09", b"\xfe",
                b"\x06", b"\xd6", b"\x01\x01", b"start", b"\xff",
                b"\x80", b"\x29", b"\x31", b"\x02\x00\x00", b"\x40",
                b"\xf8", b"\x07\x07", b"STOP"]


# --------------------------------------------------------------------------
# Self-test against the firmware's own vectors.
# --------------------------------------------------------------------------
def selftest(path):
    fails = checks = 0
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            vec = json.loads(line)
            raw = bytes.fromhex(vec["hex"])
            checks += 1
            if vec["topic"] == TOPIC_COMMAND:
                got = command_is_valid(raw)
                ok = got == vec["valid"]
                if ok and vec.get("name") == "map":
                    move = {v: k for k, v in MAP_MOVES.items() if len(k) == 1}
                    ok = encode_map(vec["letter"], move[vec["move"]]) == raw
                elif ok and "name" in vec:
                    ok = encode_command(vec["name"], vec.get("speed", 0)) == raw
                what = "valid=%s" % got
            else:
                got = decode(vec["topic"], raw)
                ok = all(got.get(k) == v for k, v in vec["expect"].items())
                what = json.dumps({k: got.get(k) for k in vec["expect"]})
            if not ok:
                fails += 1
                print("FAIL %s %s: got %s, want %s" % (
                    vec["topic"], vec["hex"], what,
                    vec.get("expect", vec.get("valid"))))
    for bad in BAD_COMMANDS:
        checks += 1
        if command_is_valid(bad):
            fails += 1
            print("FAIL bad command %r accepted" % bad)
    print("dashboard selftest: %d checks, %d failed" % (checks, fails))
    return 1 if fails else 0


# --------------------------------------------------------------------------
# Live dashboard.
# --------------------------------------------------------------------------
class Dashboard:
    def __init__(self, log_path, show_periodic):
        self.latest = {}
        self.counts = {}
        self.lock = threading.Lock()
        self.show_periodic = show_periodic
        self.started = time.time()
        self.log_fh = open(log_path, "w", newline="") if log_path else None
        self.log = csv.writer(self.log_fh) if self.log_fh else None
        if self.log:
            self.log.writerow(["t_s", "topic", "hex", "decoded"])

    def on_message(self, topic, payload):
        try:
            fields = decode(topic, payload)
        except ValueError as exc:
            fields = {"error": str(exc), "raw": payload.hex()}
        now = time.time() - self.started
        with self.lock:
            self.latest[topic] = fields
            self.counts[topic] = self.counts.get(topic, 0) + 1
        if self.log:
            self.log.writerow(["%.3f" % now, topic, payload.hex(),
                               json.dumps(fields)])
        if topic in EVENT_TOPICS or topic == TOPIC_STATUS or self.show_periodic:
            print("%8.2f  %-26s %s" % (now, topic, fields))

    def summary(self):
        with self.lock:
            hb = self.latest.get("robot/heartbeat", {})
            ln = self.latest.get("robot/telemetry/line", {})
            mo = self.latest.get("robot/telemetry/motion", {})
            im = self.latest.get("robot/telemetry/imu", {})
            hu = self.latest.get("robot/telemetry/hump", {})
            cm = self.latest.get("robot/telemetry/comms", {})
            ir = self.latest.get("robot/telemetry/ir", {})
        return ("[%s faults=%s sig=%s up=%ss] line %s mm%s | %s mm L%s R%s mm/s"
                " | IR arm %s centre %s right %s"
                " | pitch %s hump %s %s | turn %s deg/s | peak %s mm | drop %s bad %s" % (
                    hb.get("state_name", "-"), hb.get("fault_names", "-"),
                    hb.get("signal_dbm", "-"), hb.get("uptime_s", "-"),
                    ln.get("error_mm", "-"), " LOST" if ln.get("lost") else "",
                    mo.get("distance_mm", "-"), mo.get("left_mm_s", "-"),
                    mo.get("right_mm_s", "-"),
                    ir.get("arm", "-"), ir.get("centre", "-"), ir.get("right", "-"),
                    im.get("pitch_deg", "-"),
                    im.get("hump_name", "-"), im.get("motion_name", "-"),
                    turn_rate(mo),
                    hu.get("max_mm", "-"), cm.get("drops", "-"),
                    cm.get("bad_commands", "-")))

    def rates(self):
        elapsed = max(time.time() - self.started, 1e-3)
        with self.lock:
            return {t: round(n / elapsed, 2) for t, n in sorted(self.counts.items())}


def run_live(args):
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        print("paho-mqtt is not installed:  pip install paho-mqtt")
        return 1

    dash = Dashboard(args.log, args.periodic)

    def on_connect(c, userdata, flags, rc, properties=None):
        print("connected to %s:%d (rc=%s); subscribing to robot/#" %
              (args.broker, args.port, rc))
        c.subscribe(TOPIC_PREFIX + "#", qos=1)

    def on_message(c, userdata, msg):
        if msg.topic != TOPIC_COMMAND:      # our own commands echo back
            dash.on_message(msg.topic, msg.payload)

    try:                                    # paho-mqtt 2.x
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                             client_id="is25-dashboard")
    except AttributeError:                  # paho-mqtt 1.x
        client = mqtt.Client(client_id="is25-dashboard")

    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(args.broker, args.port, keepalive=30)
    client.loop_start()

    stop = threading.Event()

    def ticker():
        while not stop.wait(args.every):
            print(dash.summary())

    threading.Thread(target=ticker, daemon=True).start()
    print(__doc__.split("While running,")[1].strip())

    try:
        for line in sys.stdin:
            words = line.split()
            if not words:
                continue
            cmd = words[0].lower()
            if cmd in ("quit", "exit"):
                break
            if cmd == "rates":
                print("messages per second:", dash.rates())
                continue
            if cmd == "bad":
                for raw in BAD_COMMANDS:
                    client.publish(TOPIC_COMMAND, raw, qos=1)
                print("sent %d malformed commands" % len(BAD_COMMANDS))
                continue
            if cmd == "map":
                try:
                    raw = encode_map(words[1], words[2])
                except (ValueError, IndexError) as exc:
                    print("bad map:", exc)
                    continue
                client.publish(TOPIC_COMMAND, raw, qos=1)
                print("sent map %s -> %s" % (words[1].strip().upper()[:1],
                                             words[2].strip().lower()))
                continue
            if cmd not in RCMD:
                print("unknown command; use start pause resume estop reset "
                      "speed <2..25> gain <100..2500> map <A..Z> <f|l|r|u> "
                      "bad rates quit")
                continue
            try:
                arg = int(words[1]) if cmd in ("speed", "gain") else 0
                raw = encode_command(cmd, arg)
            except (ValueError, IndexError) as exc:
                print("bad value:", exc)
                continue
            client.publish(TOPIC_COMMAND, raw, qos=1)
            print("sent %s = 0x%02x" % (cmd, raw[0]))
    except KeyboardInterrupt:
        pass

    stop.set()
    client.loop_stop()
    client.disconnect()
    print("messages per second:", dash.rates())
    if dash.log_fh:
        dash.log_fh.close()
        print("log written to", args.log)
    return 0


def main():
    ap = argparse.ArgumentParser(description="IS25 MQTT dashboard")
    ap.add_argument("--broker", help="MQTT broker host (same as MQTT_BROKER_IP)")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--log", help="CSV file for every message (evidence)")
    ap.add_argument("--periodic", action="store_true",
                    help="also print every periodic message (5 Hz)")
    ap.add_argument("--every", type=float, default=1.0,
                    help="seconds between summary lines")
    ap.add_argument("--selftest", metavar="VECTORS",
                    help="check the decoder against firmware vectors")
    args = ap.parse_args()

    if args.selftest:
        return selftest(args.selftest)
    if not args.broker:
        ap.error("--broker is required (or use --selftest)")
    return run_live(args)


if __name__ == "__main__":
    sys.exit(main())
