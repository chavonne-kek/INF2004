"""Stop the IS25 broker + dashboard started by start_dashboard.py."""
import subprocess

for port in (1883, 8025):
    out = subprocess.run(["netstat", "-ano"], capture_output=True, text=True).stdout
    pids = set()
    for line in out.splitlines():
        if ":%d" % port in line and "LISTENING" in line:
            pids.add(line.split()[-1])
    for pid in pids:
        subprocess.run(["powershell", "-NoProfile", "-Command",
                        "Stop-Process -Id %s -Force" % pid], capture_output=True)
    print("port %d: stopped %d process(es)" % (port, len(pids)))
