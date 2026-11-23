"""Start the IS25 broker + dashboard for a test run.

Run this with the laptop already on the car's network (phone hotspot).

    python start_dashboard.py

It finds the laptop's Wi-Fi address, writes a broker config bound to it, starts
mosquitto, starts the web dashboard on http://localhost:8025 and prints the URL.
Telemetry is logged to dashboard/run_<date>_<time>.csv.

Why the address matters: the Windows Mosquitto *service* owns 127.0.0.1:1883, so
this broker has to bind the specific interface address. When Windows roams to
another network that address changes, the dashboard sits in SYN_SENT and the car
reconnects forever -- rerun this script and it is fixed.

Stop everything later with:  python stop_dashboard.py
"""
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
RUN_DIR = os.path.join(HERE, "_run")            # broker config + log live here
CONF = os.path.join(RUN_DIR, "mosquitto.conf")
BROKER_LOG = os.path.join(RUN_DIR, "broker.log")
BROKER = r"C:\Program Files\mosquitto\mosquitto.exe"
WEB_PORT = 8025
BROKER_PORT = 1883


def wifi_ip():
    """Return the laptop's address on the car's network (the 10.x one)."""
    out = subprocess.run(["ipconfig"], capture_output=True, text=True).stdout
    addrs = re.findall(r"IPv4 Address[^:]*:\s*(\d+\.\d+\.\d+\.\d+)", out)
    for a in addrs:
        if a.startswith("10."):
            return a
    return addrs[-1] if addrs else None


def kill_listener(port):
    out = subprocess.run(["netstat", "-ano"], capture_output=True, text=True).stdout
    pids = set()
    for line in out.splitlines():
        if ":%d" % port in line and "LISTENING" in line:
            pids.add(line.split()[-1])
    for pid in pids:
        subprocess.run(["powershell", "-NoProfile", "-Command",
                        "Stop-Process -Id %s -Force" % pid],
                       capture_output=True)
    return len(pids)


def listening(port):
    out = subprocess.run(["netstat", "-ano"], capture_output=True, text=True).stdout
    for line in out.splitlines():
        if ":%d" % port in line and "LISTENING" in line:
            return line.split()[1]
    return None


def main():
    if not os.path.exists(BROKER):
        print("mosquitto not found at", BROKER)
        return 1
    ip = wifi_ip()
    if not ip:
        print("no IPv4 address - connect the laptop to the hotspot first")
        return 1
    print("laptop address:", ip)

    os.makedirs(RUN_DIR, exist_ok=True)
    kill_listener(BROKER_PORT)
    kill_listener(WEB_PORT)
    time.sleep(1)

    with open(CONF, "w", encoding="utf-8") as fh:
        fh.write("listener %d %s\n" % (BROKER_PORT, ip))
        fh.write("allow_anonymous true\n")
        fh.write("persistence false\n")
        fh.write("max_keepalive 60\n")
        fh.write("log_dest file %s\n" % BROKER_LOG.replace("\\", "/"))
        fh.write("log_type all\n")
        fh.write("connection_messages true\n")

    subprocess.Popen([BROKER, "-c", CONF, "-v"],
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2)

    stamp = time.strftime("%Y%m%d_%H%M%S")
    log = os.path.join(HERE, "run_%s.csv" % stamp)
    subprocess.Popen([sys.executable, "is25_web.py",
                      "--broker", ip, "--no-browser", "--log", log],
                     cwd=HERE,
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(4)

    b = listening(BROKER_PORT)
    w = listening(WEB_PORT)
    print("broker  :", b or "NOT LISTENING")
    print("web     :", w or "NOT LISTENING")
    print("open    : http://localhost:%d" % WEB_PORT)
    print("logfile :", log)
    if not b or not w:
        print("\nsomething did not start - try  python is25_web.py --broker %s" % ip)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
