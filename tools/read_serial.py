"""
read_serial.py -- show what the Pico W is printing, and save it as your test log.

Why this exists: the console output IS the evidence for the test log, so the
same command that shows it on screen writes it to testing/<subsystem>/logs/ (default subsystem3).

No extra packages needed (plain Windows Python + the Pico's USB port).

Usage:
    python read_serial.py            find the Pico automatically
    python read_serial.py COM7       use a port you know
    python read_serial.py COM7 subsystem2   save under testing/subsystem2/logs
    python read_serial.py auto subsystem2   same, port found automatically

Press Ctrl-C to stop. The log file is written as you go, so an interrupted
run is still a usable log.
"""

import sys
import time
import datetime
import pathlib

BAUD_IGNORED = True     # the Pico's USB console ignores baud rate settings


def list_ports():
    """All COM ports Windows knows about, via the registry (no dependencies)."""
    ports = []
    try:
        import winreg
        key = winreg.OpenKey(
            winreg.HKEY_LOCAL_MACHINE,
            r"HARDWARE\DEVICEMAP\SERIALCOMM")
        i = 0
        while True:
            try:
                name, value, _ = winreg.EnumValue(key, i)
            except OSError:
                break
            ports.append(value)
            i += 1
    except Exception:
        pass
    return ports


def open_port(name):
    """Open a COM port as a plain file: works in CPython on Windows."""
    return open(name, "rb", buffering=0)


def find_pico(candidates, seconds=2.0):
    """Return the first port that is actually sending something."""
    for name in candidates:
        try:
            fh = open_port(name)
        except OSError:
            continue
        deadline = time.time() + seconds
        chunk = b""
        try:
            while time.time() < deadline and not chunk:
                chunk = fh.read(256) or b""
        except OSError:
            chunk = b""
        if chunk:
            return name, fh, chunk
        fh.close()
    return None, None, b""


def main():
    here = pathlib.Path(__file__).resolve().parent
    # IS25: any subsystem's log folder (2nd argument), default subsystem3.
    sub = sys.argv[2] if len(sys.argv) > 2 else "subsystem3"
    log_dir = here.parent / "testing" / sub / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H%M")
    log_path = log_dir / (stamp + "_pico.txt")

    explicit = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] != "auto" else None

    if explicit:
        try:
            fh = open_port(explicit)
            first = b""
        except OSError as exc:
            print("cannot open %s: %s" % (explicit, exc))
            return 1
    else:
        ports = list_ports()
        print("COM ports found: %s" % (", ".join(ports) if ports else "none"))
        print("looking for the Pico W (press its RESET button if it is silent) ...")
        name, fh, first = find_pico(ports)
        if fh is None:
            print("")
            print("No Pico W sending anything. Checklist:")
            print("  1. Is it plugged in with a DATA usb cable (not charge-only)?")
            print("  2. Is it flashed with one of the .uf2 files?")
            print("  3. Press RESET / re-plug, then run this again.")
            print("  4. Still nothing? Run:  python read_serial.py COM7")
            print("     (use whichever port shows up in Device Manager)")
            return 1
        explicit = name
        print("Pico W is talking on %s" % explicit)

    print("logging to %s" % log_path)
    print("-" * 60)

    written = 0
    with open(log_path, "wb") as log:
        if first:
            log.write(first)
            sys.stdout.write(first.decode("utf-8", "replace"))
            written += len(first)
        try:
            while True:
                data = fh.read(512)
                if data:
                    log.write(data)
                    log.flush()
                    written += len(data)
                    sys.stdout.write(data.decode("utf-8", "replace"))
                    sys.stdout.flush()
                else:
                    time.sleep(0.05)
        except KeyboardInterrupt:
            pass
        except OSError as exc:
            print("\nport closed: %s" % exc)

    print("-" * 60)
    print("saved %d bytes to %s" % (written, log_path))
    print("Next: open that file, count the [TEST] PASS lines, and copy the")
    print("date into testing/%s/TEST_LOG.md." % sub)
    return 0


if __name__ == "__main__":
    sys.exit(main())
