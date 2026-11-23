# Running the dashboard (demo runbook)

## Once, before the demo

1. Make sure the laptop is on the **same hotspot the car is** (the phone hotspot the
   firmware was compiled with).
2. Check the laptop's address: **it must end in `.41`**.

Why: the car's broker address is compiled as `0.0.0.41`, which means "whatever
subnet I'm on, but the broker is at .41". The phone hotspot hands the laptop
`10.173.252.41`, so it matches. On any other network the car publishes into
nothing and the dashboard stays empty.

Check it with `ipconfig` — look for `IPv4 Address . . . : 10.x.x.41`.

If it is not `.41`, tell me before the demo: the car has to be reflashed with the
right number, and that needs a cable.

## During the demo

```bat
cd C:\Users\szisi\Downloads\IS25_21nov\v4\IS25_group_2000_fixed\dashboard
python start_dashboard.py
```

Expected output:

```
laptop address: 10.173.252.41
broker  : 10.173.252.41:1883
web     : 127.0.0.1:8025
open    : http://localhost:8025
logfile : ...\dashboard\run_20261122_211500.csv
```

Then open **http://localhost:8025** in a browser.

## What you should see

- **Link ONLINE** with a signal figure (-40 to -70 dBm) — that means the car reached
  the broker.
- **State** changing as the car runs: IDLE, LINE_FOLLOW, OBSTACLE_AVOID, TURN_EXECUTE.
- **Obstacle panel** filling in after a scan: width, closest point, left and right
  clearance. This is stage 3 of the rubric, so it is worth showing the marker.
- **`robot/telemetry/range`** updating about 5×/s as the car approaches an object:
  `front_cm: 17, hits: 3`. `front_cm: null` means no echo (nothing in range).

Each run writes its own file, `run_<date>_<time>.csv`, so nothing is overwritten.

## If something is wrong

| Symptom | Fix |
|---|---|
| No "laptop address" line | Laptop is not on a network — connect to the hotspot |
| Link stays CONNECTING | Address does not end in `.41`, or the broker is not running |
| Link went ONLINE then dropped | The laptop changed network. Rerun `start_dashboard.py` |
| Page will not open | Port 8025 taken — the script kills the old one; if not, rerun it |
| Dashboard shows nothing at all | Car is off, or its Wi-Fi failed (check the car's own screen/log) |

## To stop it

```bat
python stop_dashboard.py
```

## If it fails completely

The demo still works without the dashboard — the car runs from the buttons. But
the telemetry/rubric evidence comes from the dashboard, so keep a screenshot of a
successful run (the obstacle panel with width and clearances) as a fallback.
