#!/usr/bin/env python3
"""Does the perception depend on how often the networks run? A check on recorded drives.

    python docs/rate_check.py record drive.pkl --type perpendicular --cars both --seed 6
    python docs/rate_check.py replay drive.pkl [more.pkl ...] [--counts]

`record` drives the car along the lane once, without parking, with both networks on every camera
frame (10 per second), and stores what the sensors gave on every tick. It needs a PyChrono with
the ray-traced sensors and the depth networks set up (see docs/sensors.md). The arguments after
the file name are those of parking_sim.py.

`replay` needs neither. Keeping every k-th answer of each network turns a recording into the same
drive at 10/k answers per second, in k ways (which answer is the first one kept). For each of
them the maps are rebuilt, and the agent's own stall choice runs on them up to the moment it
would brake. If the method does not depend on the rate, that moment and the stall are the same.

With --counts the maps are run the way an earlier version worked, where every answer counted as
one, whatever time it stood for. That is the comparison in docs/sensors.md."""

import glob
import math
import os
import pickle
import sys
import types

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

RATES = ((1, 10.0), (2, 5.0), (4, 2.5))          # keep every k-th answer: answers per second


def record(out, sim_args):
    sys.argv = ["parking_sim.py", "--headless", "--stereo-hz", "10", "--mono-hz", "10"] + sim_args
    from parking.agent import ParkingSim
    from parking.cli import parse_args
    from parking.sensors import SensorRig
    from parking.vehicle import EGO

    sim = ParkingSim(parse_args())
    if not isinstance(sim.sensor, SensorRig):
        sys.exit("recording needs the sensor rig (--sensors camera or camera+lidar)")
    sim._decide = lambda: None                   # drive past: the choice is made in the replay
    labels, frames = [], []
    camera = SensorRig._camera

    def camera_noted(self, cam, *rest):
        res = camera(self, cam, *rest)
        labels.append((cam["role"], len(res[2])))        # which network, and how many line pieces
        return res

    SensorRig._camera = camera_noted
    sense = sim.sensor.sense

    def sense_kept(pose):
        labels.clear()
        scans, dets = sense(pose)
        frames.append(dict(t=sim.time, pose=tuple(pose), scans=scans, dets=dets, cams=list(labels)))
        return scans, dets

    sim.sensor.sense = sense_kept
    while sim.result is None and sim.time < 90.0:
        sim.advance(50)
    scn = sim.scn
    ego = {k: v for k, v in vars(EGO).items() if isinstance(v, (int, float, tuple))}
    with open(out, "wb") as f:
        pickle.dump(dict(args=sim_args, frames=frames, bounds=scn.bounds, start=scn.start, ego=ego,
                         stalls=[dict(center=np.array(s["center"]), occupied=bool(s["occupied"])) for s in scn.stalls]), f)
    print("recorded %d ticks to %s" % (len(frames), out), flush=True)
    os._exit(0)             # (see the note on leaving in parking/cli.py)


def count_frames():
    """The rules as they were: one per answer. An obstacle or free ground after 2 answers, far
    ground after 3, a line after 5 detections, a stall after 8 of its weaker line."""
    import parking.mapping as mapping
    from parking.mapping import GridMap, LineMap, LineTrack

    mapping.FRAME_MAX = 1.0
    GridMap.SURE, GridMap.SURE_FAR, LineTrack.SHOWN, LineMap.MIN_WATCH = 2.0, 3.0, 150.0, 5.0
    init, add = LineTrack.__init__, LineTrack.add

    def init_counting(self, det, t, keep=False):
        init(self, det, t, keep)
        self.watched = 1.0

    def add_counting(self, det, t):
        add(self, det, t)
        self.watched = float(len(self.dets))

    LineTrack.__init__, LineTrack.add = init_counting, add_counting


def replay(rec, k, phase, counts=False):
    """The drive with every k-th answer of each network. Returns (time, metres to go to the stall,
    the scenario's stall that was chosen) at the moment the agent commits, or None if it never does."""
    from parking.agent import ParkingSim
    from parking.config import PERCEPTION_DT
    from parking.mapping import GridMap, LineMap
    from parking.stalls import find_slots
    from parking.vehicle import EGO

    for key, val in rec["ego"].items():
        setattr(EGO, key, val)
    grid, lines = GridMap(rec["bounds"]), LineMap(keep=True)
    trail = [np.array(rec["start"][:2])]
    # the part of the agent that chooses a stall, with nothing behind it that drives
    agent = types.SimpleNamespace(rejected=[], watch=[], target=None, args=types.SimpleNamespace(park="auto"),
                                  tracker=types.SimpleNamespace(stop=lambda: None), say=lambda text: None)
    seen = dict(stereo=0, mono=0)
    for f in rec["frames"]:
        roles = [role for role, _ in f["cams"]]
        use = {role: role in roles and seen[role] % k == phase for role in seen}       # (both on the same ticks)
        for role in seen:
            seen[role] += role in roles
        dets, first = [], 0
        for i, scan in enumerate(f["scans"]):
            if i < len(f["cams"]):
                role, n = f["cams"][i]
                pieces, first = f["dets"][first:first + n], first + n
                if not use[role]:
                    continue
                dt = 1.0 if counts else scan[0] * k          # an answer stands for the time since the one before
                dets += [tuple(p[:5]) + (dt,) for p in pieces]
            else:                                            # the lidar: every scan
                dt = 1.0 if counts else scan[0]
            grid.update(*scan[1:], dt=dt)
        grid.mark_free(f["pose"], 1.0 if counts else PERCEPTION_DT)
        lines.update(dets, f["t"])
        x, y, th = f["pose"]
        if np.hypot(x - trail[-1][0], y - trail[-1][1]) > 0.5:
            trail.append(np.array([x, y]))
        agent.slots = find_slots(lines.markers(), trail, grid, True)
        if counts:
            for s in agent.slots:
                s.watched = 0.65 if s.watched >= 8.0 else 0.0
        if f["t"] <= 1.0:
            continue
        agent.pose, agent.time = f["pose"], f["t"]
        agent.travel_dir = np.array([math.cos(rec["start"][2]), math.sin(rec["start"][2])])
        ParkingSim._decide(agent)
        if agent.target is not None:
            c = agent.target.center
            true = min(rec["stalls"], key=lambda q: np.hypot(*(q["center"] - c)))
            ahead = (c - np.array([x, y]) - EGO.center * np.array([math.cos(th), math.sin(th)])) @ agent.travel_dir
            return f["t"], float(ahead), true, float(np.hypot(*(true["center"] - c)))
    return None


def main():
    if len(sys.argv) >= 3 and sys.argv[1] == "record":
        record(sys.argv[2], sys.argv[3:])
    if len(sys.argv) < 3 or sys.argv[1] != "replay":
        sys.exit(__doc__)
    counts = "--counts" in sys.argv
    if counts:
        count_frames()
    files = sorted(sum((glob.glob(a) for a in sys.argv[2:] if not a.startswith("--")), []))
    total = {hz: [0, 0, 0] for _, hz in RATES}
    spread = []
    print("the moment the car commits to a stall [s], for every way of keeping the answers%s" %
          (" (every answer counts as one)" if counts else ""))
    print("%-28s %s" % ("drive", "".join("%-34s" % ("%g per second" % hz) for _, hz in RATES)))
    for fn in files:
        with open(fn, "rb") as f:
            rec = pickle.load(f)
        row, times = [], {}
        for k, hz in RATES:
            res = [replay(rec, k, ph, counts) for ph in range(k)]
            good = [r for r in res if r and not r[2]["occupied"] and r[3] < 1.0]
            total[hz][0] += len(good)
            total[hz][1] += len(res)
            total[hz][2] += sum(1 for r in res if r) - len(good)
            times[hz] = [r[0] for r in good]
            row.append(" ".join("%.1f" % r[0] + ("" if r in good else "!") if r else "none" for r in res))
        if all(len(times[hz]) == k for k, hz in RATES):
            spread.append((max(max(v) for v in times.values()) - min(min(v) for v in times.values()),
                           abs(float(np.mean(times[5.0])) - times[10.0][0])))
        print("%-28s %s" % (os.path.basename(fn)[:-4], "".join("%-34s" % c for c in row)), flush=True)
    for hz in total:
        print("%4g per second: a free stall chosen in %d of %d replays, another one in %d" % (hz, *total[hz]))
    if spread:
        print("drives with a choice at every rate: %d. Earliest to latest choice: %.2f s on average, %.2f s at most. "
              "Between 10 and 5 per second: %.2f s on average" %
              (len(spread), np.mean([s[0] for s in spread]), max(s[0] for s in spread), np.mean([s[1] for s in spread])))


if __name__ == "__main__":
    main()
