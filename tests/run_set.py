#!/usr/bin/env python3
"""Run a set of scenarios, a few at a time, and say what parked.

    python tests/run_set.py cameras                     the 60 runs with the camera rig (docs/results.md)
    python tests/run_set.py cameras --seeds 10,11,12    the same layouts on other seeds, 15 runs a seed
    python tests/run_set.py standin                     the 74 runs without simulated sensors
    python tests/run_set.py list.txt                    your own: one set of parking_sim.py arguments a line

What follows `--` is added to every run, for example `-- --depth-python ~/venv/bin/python`.

A line is printed for every run when it ends, in the order of the set, and a summary after the
last. A run with sensors takes a few minutes and several gigabytes, and all of them share one
graphics card, so only --workers of them run at a time (3 unless told otherwise). A run that
grows beyond --max-gb or takes longer than --max-minutes is stopped and counts as not parked."""

import argparse
import os
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TYPES = ("perpendicular", "angled", "parallel")


def cameras(seeds=None):
    """The runs with the rig. Without seeds, the 60 of docs/results.md: cameras on 3 stall types
    x 4 car layouts x seeds 1 to 3, each type twice at double noise, and cameras with the lidar
    on 3 types x 3 layouts x 2 seeds. With seeds, 15 runs for each: the 12 layouts with cameras
    and the 3 types with random cars and the lidar."""
    if seeds:
        return ["--sensors camera --type %s --cars %s --seed %d" % (t, c, s)
                for s in seeds for t in TYPES for c in ("both", "left", "right", "none")] + \
               ["--sensors camera+lidar --type %s --cars random --seed %d" % (t, s) for s in seeds for t in TYPES]
    runs = []
    for s in (1, 2, 3):
        for t in TYPES:
            runs += ["--sensors camera --type %s --cars %s --seed %d" % (t, c, s) for c in ("both", "left", "right", "none")]
            if s < 3:
                runs.append("--sensors camera --type %s --cars both --noise 2 --seed %d" % (t, s))
                runs += ["--sensors camera+lidar --type %s --cars %s --seed %d" % (t, c, s) for c in ("both", "none", "random")]
    return runs


def standin(seeds=None):
    """The runs without simulated sensors: every layout on four seeds, and with seeds 1 to 4
    also the other side of the lane, the other tire model, double noise, nose-in parking,
    another stall angle and targets placed by hand."""
    runs = ["--sensors sim --type %s --cars %s --seed %d" % (t, c, s) for s in seeds or (1, 2, 3, 4)
            for t in TYPES for c in ("both", "left", "right", "none", "random")]
    if not seeds:
        for t in TYPES:
            runs += ["--sensors sim --type %s --cars both %s" % (t, more) for more in ("--side left", "--tire pac02", "--noise 2 --seed 2")]
        runs += ["--sensors sim --type perpendicular --cars both --park forward",
                 "--sensors sim --type angled --angle 45 --cars both",
                 "--sensors sim --type perpendicular --cars random --seed 4 --target 17.6,-6.3,90",
                 "--sensors sim --type parallel --cars both --target 18.0,-3.0,0",
                 "--sensors sim --type perpendicular --cars none --target 6.0,0.5,25 --no-snap"]
    return runs


def run(line, extra, max_gb, max_seconds, live):
    """One run. Returns (its arguments, its result as a dictionary or None, what to print)."""
    cmd = [sys.executable, os.path.join(HERE, "parking_sim.py"), "--headless"] + line.split() + extra
    # (a session of its own: the simulator may start again with another Python, and it starts
    # the depth networks, and stopping a run has to stop all of that)
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    live.add(proc.pid)
    text = []
    reader = threading.Thread(target=lambda: text.append(proc.stdout.read()), daemon=True)
    reader.start()
    began, peak, why = time.time(), 0.0, None
    while proc.poll() is None:
        seen = subprocess.run(["ps", "-o", "rss=", "-p", str(proc.pid)], capture_output=True, text=True).stdout.strip()
        size = float(seen) / 1048576.0 if seen else 0.0
        peak = max(peak, size)
        if size > max_gb:
            why = "stopped at %.1f GB" % size
        elif time.time() - began > max_seconds:
            why = "stopped after %d minutes" % round(max_seconds / 60.0)
        if why:
            os.killpg(proc.pid, 9)
            break
        time.sleep(0.5)
    reader.join(10.0)
    live.discard(proc.pid)
    out = (text[0] if text else "").splitlines()
    found = [row for row in out if row.startswith("[result]")]
    took = "%.0f s, %.1f GB" % (time.time() - began, peak)
    if why or not found:
        return line, None, "%s :: %s (%s)" % (line, why or "no result: " + " | ".join(out[-2:]), took)
    res = dict(part.split("=", 1) for part in found[-1].split()[1:])
    return line, res, "%s :: %s  (%s)" % (line, found[-1], took)


def main():
    argv = sys.argv[1:]
    extra = argv[argv.index("--") + 1:] if "--" in argv else []
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("set", help="cameras, standin, or a file with one set of parking_sim.py arguments a line")
    ap.add_argument("--seeds", default=None, help="for cameras and standin: these seeds, like 10,11,12")
    ap.add_argument("--workers", type=int, default=3, help="runs at a time (3)")
    ap.add_argument("--max-gb", type=float, default=7.0, help="stop a run whose simulator grows beyond this (7)")
    ap.add_argument("--max-minutes", type=float, default=30.0, help="stop a run that takes longer (30)")
    args = ap.parse_args(argv[:argv.index("--")] if "--" in argv else argv)
    seeds = [int(v) for v in args.seeds.split(",")] if args.seeds else None
    if args.set in ("cameras", "standin"):
        lines = (cameras if args.set == "cameras" else standin)(seeds)
    else:
        with open(args.set) as f:
            lines = [row.strip() for row in f if row.strip() and not row.startswith("#")]
    live, results = set(), []
    try:
        with ThreadPoolExecutor(args.workers) as pool:
            for line, res, text in pool.map(lambda q: run(q, extra, args.max_gb, 60.0 * args.max_minutes, live), lines):
                results.append((line, res, text))
                print(text, flush=True)
    except KeyboardInterrupt:
        for pid in list(live):
            os.killpg(pid, 9)
        os._exit(1)
    parked = [res for _, res, _ in results if res and res.get("ok") == "True"]
    print("\nparked %d of %d" % (len(parked), len(results)))
    for line, res, text in results:
        if not (res and res.get("ok") == "True"):
            print("  not parked: " + text[:200])
    for key, name, scale, unit in (("lateral", "sideways", 100.0, "cm"), ("depth", "along the stall", 100.0, "cm"),
                                   ("heading_deg", "heading", 1.0, "deg")):
        vals = [abs(float(res[key])) * scale for res in parked if key in res]
        if vals:
            print("  %s: %.1f %s on average, %.1f at worst" % (name, sum(vals) / len(vals), unit, max(vals)))
    sys.exit(0 if len(parked) == len(results) else 1)


if __name__ == "__main__":
    main()
