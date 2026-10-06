"""The parking agent: perception, map, stall decision, plan and tracking, as one state machine."""

import math
import os
import sys
import threading
import time

import numpy as np

from .chrono_env import veh
from .config import A_BRAKE, CONTROL_DT, PERCEPTION_DT, STEER_RATE, STEP, V_SEARCH
from .control import MpcTracker
from .geometry import ego_poly, footprint_hits, poly_distance, wrap
from .mapping import GridMap, LineMap
from .localization import Localization
from .perception import Perception
from .planner import CSpace, Planner, Segment, holonomic_distance, split_segments
from .scenario import make_scenario
from .networks import DepthWorker, find_depth_python
from .sensors import SensorRig
from .stalls import Slot, find_slots
from .scene_net import NAME as SCENE_NET
from .vehicle import EGO
from .world import World


def start_depth_worker(args):
    """Start the process with the networks, or exit with what is missing."""
    scene = SCENE_NET if args.scene == "net" else args.scene          # (auto: the worker knows its device)
    if args.depth_host:
        # on another machine, which has a copy of this repository and a Python with PyTorch
        there = "%s %s/parking/stereo_worker.py --repo %s/third_party/IGEV-plusplus --model %s --scene %s" % (
            args.depth_python or "python3", args.depth_dir, args.depth_dir, args.stereo, scene)
        try:
            return DepthWorker(["ssh", "-T", "-o", "BatchMode=yes", args.depth_host, there], remote=True)
        except RuntimeError as exc:
            sys.exit("[parking] %s" % exc)
    python = find_depth_python(args.depth_python)
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    repo = args.igev or os.environ.get("IGEV_ROOT") or os.path.join(root, "third_party", "IGEV-plusplus")
    if python is None:
        sys.exit("--sensors %s computes depth with neural networks and needs a Python with torch, timm and "
                 "transformers. None was found: name one with --depth-python, or see docs/sensors.md. "
                 "--sensors sim runs without sensors." % args.sensors)
    try:
        script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "stereo_worker.py")
        return DepthWorker([python, script, "--repo", repo, "--model", args.stereo, "--scene", scene])
    except RuntimeError as exc:
        sys.exit("[parking] %s" % exc)


class ParkingSim:
    MAX_REPLANS = 6
    SEARCH_REACH = 70.0      # how far along the lane the car looks for a stall [m]
    MAP = (20.0, 80.0, 15.0)     # the map covers this much behind, ahead and to each side of where the car starts [m]
    MAX_CORRECTIONS = 2

    def __init__(self, args):
        self.args = args
        self.scn = make_scenario(args.type, args.cars, args.side, args.angle, args.seed)
        rng = np.random.default_rng(args.seed + 7919)
        rig = args.sensors != "sim"
        self.world = World(self.scn, visual=rig or not args.headless, tire=args.tire, wear=args.wear,
                           bumps=0.01 * args.bumps)        # (sensors render the visual assets)
        self.sensor = SensorRig(self.world, args.sensors, args.noise, rng, start_depth_worker(args), args.sky,
                                args.stereo_hz, args.mono_hz, args.stereo_rows, args.give) if rig else \
            Perception(self.scn, args.noise, rng)
        self.grid = None             # (made once the car knows where it is: see below)
        self.lines = LineMap(keep=rig)
        self.planner = Planner()
        self.planner.table()          # tabulate the Reeds-Shepp heuristic up front (~1 s)
        self.tracker = MpcTracker()
        self.obstacles = self.scn.obstacle_polys()

        self.state = "SETTLE"
        self.message = ""
        self.slots, self.target, self.goal = [], None, None
        self.path, self.seg_i = [], 0
        self.rejected = []
        self.watch = []                # recent estimates of the leading candidate stall
        self.manual = None             # user-requested pose of the car's middle (x, y, heading)
        self.nose_in = False
        self.trail = None
        self.dets, self.scan = [], np.zeros((0, 2))
        self.cmd = (0.0, 0.0, EGO.torque(A_BRAKE))     # steering angle, drive torque, brake torque
        self.steps = 0
        self.t_still = None
        self.replans = self.corrections = self.blocked = self.lane_blocked = 0
        self.must_replan = False
        self.watch_margin = True       # whether the path monitor still asks for more margin on this plan
        self.plan_info = None          # statistics and search tree of the last plan
        self.plan_thread = self.plan_result = None
        self.plan_time = 0.0
        self.min_clearance = float("inf")
        self.gear_changes = 0
        self.result = None
        # The car goes by where it thinks it is (pose). Where it really is (true_pose) is for the score.
        self.true_pose, self.speed = self.world.state()
        self.odo = Localization(self.true_pose, np.random.default_rng(args.seed + 4441), args.pose,
                                0.0 if "pose" in args.give else args.pose_noise, "speed" in args.give)
        self.pose = self.odo.pose
        # What the car assumes about the place: it starts on a lane and aligned with it, and
        # stalls may be on either side. It is not told where the lane ends or how big the lot
        # is (unless --give lane, map): it looks for a stall straight ahead for SEARCH_REACH,
        # and maps a fixed area around where it started.
        self.origin = self.pose
        self.travel_dir = np.array([math.cos(self.pose[2]), math.sin(self.pose[2])])
        self.trail = [np.array(self.pose[:2])]
        if "map" in args.give:
            self.grid = GridMap(self.scn.bounds)
        else:
            back, ahead, side = self.MAP
            fwd, left = self.travel_dir, np.array([-self.travel_dir[1], self.travel_dir[0]])
            box = np.array([np.array(self.pose[:2]) + a * fwd + b * left for a in (-back, ahead) for b in (-side, side)])
            self.grid = GridMap((box[:, 0].min(), box[:, 1].min(), box[:, 0].max(), box[:, 1].max()))
        self.park_dir = self.travel_dir          # which way a parallel-parked car should face
        car = self.world.car
        print("[parking] vehicle: Chrono::Vehicle Sedan, full multibody model (%s front / %s rear suspension, "
              "%s steering, %s driveline, %s)" % (
                  car.GetSuspension(0).GetTemplateName(), car.GetSuspension(1).GetTemplateName(),
                  car.GetSteering(0).GetTemplateName(), car.GetDriveline().GetTemplateName(),
                  car.GetTire(0, veh.LEFT).GetTemplateName()), flush=True)
        print("[parking] perception: %s" % self.sensor.name, flush=True)
        if rig:
            info = self.sensor.depth.info
            rows = "" if args.stereo_rows is None else " on rows %d to %d of the images" % self.sensor.rows
            print("[parking] depth from images: %s (%s) for the stereo pair at %.3g Hz%s, %s for the single cameras at "
                  "%.3g Hz, on %s of %s; sky: %s" % (
                      info["model"], info["weights"], 1.0 / (self.sensor.stereo_every * PERCEPTION_DT), rows, info["mono"],
                      1.0 / (self.sensor.mono_every * PERCEPTION_DT), info["device"], info["where"], args.sky), flush=True)
            if self.sensor.scene:
                print("[parking] markings, kerbs and the car's own body in each image: %s" % info["scene"], flush=True)

    @property
    def time(self):
        return self.world.system.GetChTime()

    def say(self, text):
        self.message = text
        print("[%6.2f s] %s" % (self.time, text), flush=True)

    # ---- simulation loop -----------------------------------------------------

    def advance(self, n_steps):
        """Advance the simulation; returns False while the planner is busy (time is frozen)."""
        if self.state == "PLAN":
            if self.plan_thread.is_alive():
                return False
            self._plan_done()
        for _ in range(n_steps):
            if self.state == "PLAN":
                break
            if self.steps % int(round(PERCEPTION_DT / STEP)) == 0:
                self._locate()
                self._perceive()
            if self.steps % int(round(CONTROL_DT / STEP)) == 0:
                self._locate()
                self._control()
            self.world.step(*self.cmd)
            self.steps += 1
        return True

    def _locate(self):
        self.true_pose, v = self.world.state()
        self.pose, self.speed = self.odo.update(self.true_pose, self.time, self.world.wheel_travel()), self.odo.speed(v)

    def _follow(self, seg, presteer=True):
        self.tracker.start(seg, self.time, presteer)

    def _search_route(self):
        """Straight on along the lane the car started on, from where it is now."""
        x, y, th = self.pose
        if "lane" in self.args.give:         # the scenario's own line and end
            y0, n = self.scn.start[1], max(2, int((self.scn.route_end - x) / 0.1))
            xs = np.linspace(x, self.scn.route_end, n)
            return Segment(np.stack([xs, np.full(n, y0), np.zeros(n)], axis=1), np.zeros(n), 1, V_SEARCH)
        o, fwd = np.array(self.origin[:2]), self.travel_dir
        s0 = (np.array([x, y]) - o) @ fwd
        s = np.linspace(s0, max(self.SEARCH_REACH, s0 + 0.2), max(2, int((self.SEARCH_REACH - s0) / 0.1)))
        return Segment(np.stack([o[0] + s * fwd[0], o[1] + s * fwd[1], np.full(len(s), self.origin[2])], axis=1),
                       np.zeros(len(s)), 1, V_SEARCH)

    def _lane_blocked(self):
        """Is something in the way within the next 6 m of the lane? (Nothing told the car that
        the lane is clear.)"""
        pts = self.grid.occupied_points()
        if len(pts) == 0:
            return False
        x, y, th = self.pose
        fwd = self.travel_dir
        pts = pts[np.hypot(pts[:, 0] - x, pts[:, 1] - y) < 12.0]
        poses = np.array([(x + d * fwd[0], y + d * fwd[1], th) for d in np.arange(1.0, 6.01, 0.5)])
        return bool(len(pts)) and bool(footprint_hits(poses, pts, 0.10).any())

    def _control(self):
        t, v = self.time, self.speed
        if self.state == "SETTLE":
            self.cmd = (0.0, 0.0, EGO.torque(A_BRAKE))
            if t > 1.0 and self.args.target == "drag":
                self.state = "WAIT"
                self.say("drag the box onto a spot and press GO")
            elif t > 1.0 and self.args.target is not None:
                self.go_to(self.args.target)
            elif t > 1.0:
                self.state = "SEARCH"
                self._follow(self._search_route(), presteer=False)
                self.say("searching for a free stall (%s, cars: %s)" % (self.scn.name, self.args.cars))
        elif self.state in ("SEARCH", "DRIVE"):
            self.cmd = self.tracker.update(self.pose, v, t, CONTROL_DT)
            if self.tracker.done:
                if self.state == "SEARCH" and self.manual is not None:
                    self.state, self.t_still = "BRAKE", None
                elif self.state == "SEARCH":
                    self._finish(False, "reached the end of the route without finding a usable stall" if "lane" in self.args.give
                                 else "found no usable stall in %.0f m of lane" % self.SEARCH_REACH)
                else:
                    self._segment_done()
        elif self.state == "BRAKE":
            self.cmd = (self.tracker.delta, 0.0, EGO.torque(A_BRAKE))
            if abs(v) < 0.03:
                if self.t_still is None:
                    self.t_still = t
                elif t - self.t_still > 0.3:
                    self._request_plan()
            else:
                self.t_still = None
        else:   # WAIT / PARKED / FAILED: straighten the wheels and hold the brake
            a = self.tracker.delta
            self.tracker.delta = a - max(-STEER_RATE * CONTROL_DT, min(STEER_RATE * CONTROL_DT, a))
            self.cmd = (self.tracker.delta, 0.0, EGO.torque(A_BRAKE))

    def _perceive(self):
        if isinstance(self.sensor, SensorRig):
            scans, dets = self.sensor.sense(self.pose, self.odo.tilt)
        else:      # (the stand-in works out what a sensor at the true pose would give)
            scans, dets = self.odo.as_believed(*self.sensor.sense(self.true_pose))
        hits = [np.zeros((0, 2))]
        for dt, origin, ang, r_hit, r_free, *more in scans:
            self.grid.update(origin, ang, r_hit, r_free, *more, dt=dt)
            ok = np.isfinite(r_hit)
            hits.append(np.stack([origin[0] + r_hit[ok] * np.cos(ang[ok]), origin[1] + r_hit[ok] * np.sin(ang[ok])], axis=1))
        for dt, xy in getattr(self.sensor, "kerbs", ()):          # (kerbs that the scene network pointed out)
            self.grid.add_kerb(xy, dt)
        self.grid.mark_free(self.pose)
        self.lines.update(dets, self.time)
        self.scan = np.concatenate(hits)
        self.dets = dets
        # The trail tells the stall inference which end of a stall faces the lane. Only the drive
        # along the lane counts: once the car is parking, its track runs inside the stall.
        p = np.array(self.pose[:2])
        if self.state in ("SETTLE", "WAIT", "SEARCH") and np.hypot(*(p - self.trail[-1])) > 0.5:
            self.trail.append(p)
        rig = isinstance(self.sensor, SensorRig)
        self.slots = find_slots(self.lines.markers() if rig else self.lines.confirmed(), self.trail, self.grid, rig)

        ego = ego_poly(self.true_pose)
        ctr = ego.mean(axis=0)
        for poly in self.obstacles:
            if np.hypot(*(poly.mean(axis=0) - ctr)) < np.ptp(poly, axis=0).max() + 6.0:
                self.min_clearance = min(self.min_clearance, poly_distance(ego, poly))

        if self.state == "SEARCH":
            if self.manual is None:
                self._decide()
            else:
                self._approach()
            if self.state == "SEARCH" and "lane" not in self.args.give:
                self.lane_blocked = self.lane_blocked + 1 if self._lane_blocked() else 0
                if self.lane_blocked >= 2:
                    self.tracker.stop()
                    self._finish(False, "something is in the way on the lane, stopping")
        elif self.state == "BRAKE":
            if self.target is not None:
                self._retarget()
            elif self.manual is not None:
                self._snap()
        elif self.state == "DRIVE":
            if self.target is not None:
                self._refine()
            self._monitor()

    # ---- decision ------------------------------------------------------------

    def _decide(self):
        """Pick a stall: confirmed free, close enough to be well observed, best score."""
        fwd = self.travel_dir
        left = np.array([-fwd[1], fwd[0]])
        ctr = np.array(self.pose[:2]) + EGO.center * fwd
        best = None
        for s in self.slots:
            if s.status != Slot.FREE or s.watched < 0.65:        # (both lines watched for that long)
                continue
            # a stall the planner turned down is left alone until the car has moved on a few metres
            # (its estimate may have been poor), and for good after three attempts
            if any(np.hypot(*(s.center - c)) < 1.5 and (n >= 3 or np.hypot(*(ctr - at)) < 3.0)
                   for c, at, n in self.rejected):
                continue
            rel = s.center - ctr
            ahead, lat = rel @ fwd, rel @ left
            if not -8.0 < ahead < 10.0:
                continue
            # A stall known from two stubs and its row is not taken from afar. Until the car is
            # level with it, more of its far line is still coming into view, and a stall taken
            # on less is placed worse: the car then needs two or three goes to get into it.
            if s.by_row and ahead > 1.0:
                continue
            score = abs(ahead) + 0.3 * abs(lat) + 1.5 * sum(s.neighbors)
            if best is None or score < best[0]:
                best = (score, s)
        if best is None:
            self.watch = []
            return
        s = best[1]
        axis = math.atan2(s.u_in[1], s.u_in[0])
        if self.watch and np.hypot(*(s.center - self.watch[-1][1])) > 1.2:
            self.watch = []                    # a different stall took the lead
        self.watch = [w for w in self.watch if self.time - w[0] < 0.75] + [(self.time, s.center, axis)]
        if (self.time - self.watch[0][0] < 0.55 or
                max(np.hypot(*(w[1] - s.center)) for w in self.watch) > 0.12 or
                max(abs(wrap(w[2] - axis)) for w in self.watch) > math.radians(1.2)):
            return                             # wait until its estimate has settled
        self.target, self.park_dir = s, fwd
        if self.args.park == "auto":
            self.nose_in = s.kind == "angled" and float(s.u_in @ fwd) > 0.17
        else:
            self.nose_in = self.args.park == "forward"
        nb = {(False, False): "no neighbours", (True, True): "cars on both sides"}.get(
            tuple(s.neighbors), "a car on one side")
        how = "parallel park" if s.kind == "parallel" else ("drive in forwards" if self.nose_in else "back in")
        self.say("chose a free %s stall %.1f m ahead with %s -> %s" %
                 (s.kind, (s.center - ctr) @ fwd, nb, how))
        self.tracker.stop()
        self.state, self.t_still = "BRAKE", None

    # ---- manual target ---------------------------------------------------------

    def go_to(self, box):
        """Park with the middle of the car at box = (x, y, heading), instead of choosing a stall.
        If the box sits on a stall the map knows about, that stall becomes the target."""
        self.manual = (box[0], box[1], wrap(box[2]))
        self.target, self.path, self.seg_i, self.result = None, [], 0, None
        self.replans = self.corrections = 0
        self.must_replan = False
        self.rejected = []
        self._snap()
        x, y, th = self.pose
        o, fwd = np.array(self.origin[:2]), self.travel_dir
        off = np.array([x, y]) - o
        on_route = abs(off[0] * fwd[1] - off[1] * fwd[0]) < 0.6 and abs(wrap(th - self.origin[2])) < 0.2
        ahead = (np.array(box[:2]) - np.array([x, y])) @ self.travel_dir
        if on_route and ahead > 12.0 and self.SEARCH_REACH - off @ fwd > 3.0:
            self.say("driving up to the requested spot")
            self.state = "SEARCH"
            self._follow(self._search_route(), presteer=abs(self.speed) < 0.1)
        else:
            self.tracker.stop()
            self.state, self.t_still = "BRAKE", None

    def _snap(self):
        """Adopt the detected stall under the user's box, if there is one."""
        if self.args.no_snap:
            return False
        x, y, th = self.manual
        h = np.array([math.cos(th), math.sin(th)])
        for s in self.slots:
            axis = s.along if s.kind == "parallel" else s.u_in
            if np.hypot(s.center[0] - x, s.center[1] - y) < 1.2 and abs(axis @ h) > 0.9:
                self.target = s
                self.nose_in = float(s.u_in @ h) > 0.0
                self.park_dir = h
                self.say("the box is on a %s %s stall, using its lines" % (s.status, s.kind))
                return True
        return False

    def _box_cells(self, grow=0.0):
        """Grid cells under the user's box: (mask, slices), or None if it is off the map."""
        x, y, th = self.manual
        win = self.grid.window(x - 3.5, y - 3.5, x + 3.5, y + 3.5)
        if win is None:
            return None
        X, Y, sl = win
        c, s = math.cos(th), math.sin(th)
        inside = ((np.abs((X - x) * c + (Y - y) * s) < 0.5 * EGO.length + grow) &
                  (np.abs(-(X - x) * s + (Y - y) * c) < EGO.half_width + grow))
        return inside, sl

    def _approach(self):
        """Manual target further down the lane: cruise until it is close and has been looked at."""
        if self.target is None:
            self._snap()
        if self.target is not None:
            ref, near, seen = self.target.center, 7.0, self.target.status != Slot.UNKNOWN
        else:      # no stall recognised under the box (yet): go right up to it before planning
            ref, near, cells = np.array(self.manual[:2]), 1.5, self._box_cells()
            seen = cells is None or self.grid.seen_free()[cells[1]][cells[0]].mean() > 0.6 or \
                self.grid.occupied()[cells[1]][cells[0]].sum() >= 6
        ahead = (ref - np.array(self.pose[:2])) @ self.travel_dir
        if (ahead < near and seen) or ahead < -1.5:
            self.tracker.stop()
            self.state, self.t_still = "BRAKE", None

    # ---- planning --------------------------------------------------------------

    def _goal_spec(self):
        """What the planner should aim for: the chosen stall, or the pose the user asked for."""
        if self.target is not None:
            s = self.target
            nominal = s.goal(self.nose_in, self.park_dir)
            if s.kind == "parallel":     # reverse in, then pull forward to the middle of the stall
                return dict(kind=s.kind, nominal=nominal, u=s.u_in, signs=(1.0,), runs=(1.6, 1.1, 0.6, 0.0),
                            trials=sorted((abs(a) + 2.0 * b, a, b) for a in np.arange(-0.6, 0.61, 0.1)
                                          for b in np.arange(0.0, 0.31, 0.05)),
                            margins=((0.20, 0.30), (0.12, 0.22), (0.10, 0.18)) if isinstance(self.sensor, SensorRig)
                            else ((0.12, 0.22), (0.08, 0.15), (0.06, 0.12)))
            return dict(kind=s.kind, nominal=nominal, u=s.u_in, signs=(1.0 if self.nose_in else -1.0,),
                        runs=(3.5, 2.5, 1.5, 0.8),
                        trials=sorted((abs(a) + b, a, b) for a in np.arange(-0.3, 0.31, 0.05)
                                      for b in np.arange(0.0, 0.61, 0.15)),
                        margins=((0.30, 0.35), (0.20, 0.25), (0.12, 0.15)) if isinstance(self.sensor, SensorRig)
                        else ((0.25, 0.30), (0.15, 0.20), (0.08, 0.12)))
        x, y, th = self.manual
        nominal = (x - EGO.center * math.cos(th), y - EGO.center * math.sin(th), th)
        return dict(kind="manual", nominal=nominal, u=np.array([math.cos(th), math.sin(th)]),
                    signs=(1.0, -1.0), runs=(3.0, 2.0, 1.2, 0.6, 0.0),
                    trials=sorted((abs(a) + abs(b), a, b) for a in np.arange(-0.2, 0.21, 0.1)
                                  for b in np.arange(-0.2, 0.21, 0.1)),
                    margins=((0.25, 0.30), (0.15, 0.20), (0.08, 0.12)))

    def _request_plan(self):
        occ = self.grid.blocked()
        region = self.target.region if self.target is not None else self._box_cells(0.3)
        if region is not None:       # the chosen spot was judged free (by the map, or by the user)
            mask, sl = region
            if isinstance(self.sensor, SensorRig) and self.target is not None and self.target.kind == "parallel":
                # A parallel stall lies open to the lane: what a camera has not seen of it is small.
                # Only fill gaps, within 0.5 m of ground seen to be free. Its far side is the kerb,
                # which a camera cannot tell from the road until it is close.
                mask = mask & self.grid.grow(self.grid.seen_free(), 5)[sl]
            occ[sl] &= ~mask | self.grid.occupied()[sl]
        self.state = "PLAN"
        self.say("planning ...")
        self.plan_result = None
        self._plan_wall = time.time()
        job = (self.pose, self._goal_spec(), occ)
        self.plan_thread = threading.Thread(target=self._plan_job, args=job, daemon=True)
        self.plan_thread.start()
        if self.args.headless:
            self.plan_thread.join()

    def _plan_job(self, start, spec, occ):
        try:
            for k, (m_lat, m_lon) in enumerate(spec["margins"]):
                self.planner.max_iter = 30000 if k == len(spec["margins"]) - 1 else 12000
                res = self._plan_once(start, spec, occ, m_lat, m_lon)
                if res is not None:
                    self.plan_result = res
                    return
        except Exception:             # never leave the main loop waiting on a dead thread
            import traceback
            traceback.print_exc()

    def _plan_once(self, start, spec, occ, m_lat, m_lon):
        g, res = self.grid, self.grid.RES
        nominal = spec["nominal"]
        win = 13.0
        i0 = max(0, int((min(start[0], nominal[0]) - win - g.x0) / res))
        i1 = min(g.nx, int((max(start[0], nominal[0]) + win - g.x0) / res))
        j0 = max(0, int((min(start[1], nominal[1]) - win - g.y0) / res))
        j1 = min(g.ny, int((max(start[1], nominal[1]) + win - g.y0) / res))
        sub = occ[j0:j1, i0:i1]
        wx0, wy0 = g.x0 + i0 * res, g.y0 + j0 * res
        cs = CSpace(sub, wx0, wy0, res, m_lat, m_lon)
        if cs.query(*start) == CSpace.HARD:
            return None
        edge = sub.copy()                      # only the rim of the blocked regions matters
        edge[1:-1, 1:-1] &= ~(sub[:-2, 1:-1] & sub[2:, 1:-1] & sub[1:-1, :-2] & sub[1:-1, 2:])
        jy, ix = np.nonzero(edge)
        pts = np.stack([wx0 + (ix + 0.5) * res, wy0 + (jy + 0.5) * res], axis=1)
        tight = min(m_lat, 0.12)

        # Final pose: the nominal one, nudged just enough to clear what is actually there. The
        # maneuver ends with a straight run along the stall axis (the docking run), so the
        # search aims at the start of that run.
        th = nominal[2]
        h = np.array([math.cos(th), math.sin(th)])
        if spec["kind"] == "manual":
            u, nu = -h, np.array([-h[1], h[0]])            # trials: a sideways, b along the box
        else:
            u, nu = spec["u"], np.array([-spec["u"][1], spec["u"][0]])   # b: back out of the stall
        best = None
        for sign in spec["signs"]:
            goal = None
            for _, a, b in spec["trials"]:
                shift = (a, b)
                p = (nominal[0] + a * nu[0] - b * u[0], nominal[1] + a * nu[1] - b * u[1], th)
                if footprint_hits(np.array([p]), pts, tight)[0] or sub[
                        min(max(int((p[1] - wy0) / res), 0), sub.shape[0] - 1),
                        min(max(int((p[0] - wx0) / res), 0), sub.shape[1] - 1)]:
                    continue
                for dock in spec["runs"]:
                    q = (p[0] - sign * dock * h[0], p[1] - sign * dock * h[1], th)
                    n = int(dock / 0.2) + 1
                    line = np.stack([np.linspace(q[0], p[0], n), np.linspace(q[1], p[1], n), np.full(n, th)], axis=1)
                    if cs.query(*q) != CSpace.HARD and not footprint_hits(line, pts, tight).any():
                        goal, pre = p, q
                        break
                if goal is not None:
                    break
            if goal is None:
                continue
            h2d, cell = holonomic_distance(sub, res, pre[:2], wx0, wy0)
            docking = dict(goal=goal, sign=sign, length=dock) if dock > 0.0 else None
            rows = self.planner.search(start, pre, cs, h2d, cell, docking)
            if rows is None:
                continue
            stats = dict(self.planner.stats)
            segs = split_segments(start, rows) if len(rows) else []
            if any(footprint_hits(sg.poses(), pts, 0.03).any() for sg in segs):
                continue
            if best is None or stats["cost"] < best["stats"]["cost"]:
                best = dict(segments=segs, goal=goal, nominal=nominal, margin=m_lat, stats=stats, shift=shift,
                            explored=self.planner.explored)
        return best

    def _plan_done(self):
        self.plan_time += time.time() - self._plan_wall
        res = self.plan_result
        if res is None:
            if self.must_replan:
                self._finish(False, "the way is blocked and there is no other maneuver")
                return
            if self.path and self.seg_i < len(self.path):
                self.say("no better plan found, continuing with the current one")
                self.state = "DRIVE"
                self.gear_changes += 1
                self._follow(self.path[self.seg_i])
                return
            if self.path:      # a final correction was not possible: stay where we are
                self._finish(True, "parked (no room to correct further)")
                return
            if self.manual is not None:
                self._finish(False, "cannot reach that spot (blocked, or not seen to be free yet)")
                return
            here = np.array(self.pose[:2]) + EGO.center * self.travel_dir
            old = [r for r in self.rejected if np.hypot(*(self.target.center - r[0])) < 1.5]
            self.rejected = [r for r in self.rejected if r not in old]
            self.rejected.append((self.target.center.copy(), here, 1 + sum(r[2] for r in old)))
            self.target = None
            self.say("no feasible maneuver into that stall, searching on")
            self.state = "SEARCH"
            self._follow(self._search_route(), presteer=False)
            return
        segs = res["segments"]
        self.goal, self.nominal = res["goal"], res["nominal"]
        self.blocked, self.must_replan, self.watch_margin = 0, False, True
        self.plan_info = dict(res["stats"], margin=res["margin"], explored=res["explored"])
        st = res["stats"]
        a, b = res["shift"]
        moved = "" if abs(a) < 0.01 and abs(b) < 0.01 else \
            ", goal shifted %.2f m sideways and %.2f m outward to stay clear" % (abs(a), b)
        self.say("plan: %s  (%d expansions, %.1f s, margin %.2f m)" % (
            " + ".join("%s %.1f m" % ("fwd" if s.dir > 0 else "rev", s.length) for s in segs) or "already there",
            st["iterations"], time.time() - self._plan_wall, res["margin"]) + moved)
        self.path, self.seg_i = segs, 0
        if not segs:
            self._finish(True, "parked")
            return
        self.state = "DRIVE"
        self._follow(segs[0])

    # ---- execution -------------------------------------------------------------

    def _retarget(self):
        """Follow the chosen stall through the stream of fresh stall estimates."""
        cand = [s for s in self.slots if s.kind == self.target.kind and
                np.hypot(*(s.center - self.target.center)) < 1.2]
        if cand:
            self.target = min(cand, key=lambda s: np.hypot(*(s.center - self.target.center)))
        return bool(cand)

    def _refine(self):
        """Keep the plan attached to the stall as its line estimates improve."""
        if not self._retarget():
            return
        new = self.target.goal(self.nose_in, self.park_dir)
        old = self.nominal
        d, dth = math.hypot(new[0] - old[0], new[1] - old[1]), abs(wrap(new[2] - old[2]))
        if d < 0.01 and dth < 0.003:
            self._recentre()
            return
        if d > 0.5 or dth > 0.1:
            if self.replans < self.MAX_REPLANS:
                self.replans += 1
                self.say("the stall estimate jumped, replanning")
                self.tracker.stop()
                self.state, self.t_still = "BRAKE", None
            return
        a = 0.3                       # move the plan gradually so the tracker is not jerked around
        new = (old[0] + a * (new[0] - old[0]), old[1] + a * (new[1] - old[1]),
               old[2] + a * wrap(new[2] - old[2]))
        beyond = 0.0
        for seg in reversed(self.path[self.seg_i:]):
            seg.reanchor(old, new, beyond)
            beyond += seg.length
        c, s = math.cos(new[2] - old[2]), math.sin(new[2] - old[2])
        dx, dy = self.goal[0] - old[0], self.goal[1] - old[1]
        self.goal = (new[0] + dx * c - dy * s, new[1] + dx * s + dy * c, wrap(self.goal[2] + new[2] - old[2]))
        self.nominal = new
        self._recentre()

    def _recentre(self):
        """A goal that was planned off the stall centre, to stay clear of something the map showed
        there, goes back towards the centre once the map says the way is clear. From a distance a
        camera places the side of a parked car to a decimetre or two. Up close it knows better."""
        off = np.array(self.goal[:2]) - np.array(self.nominal[:2])
        cur = self.path[self.seg_i]
        rest = cur.s[-1] - cur.s[min(self.tracker.i, len(cur.s) - 1)] + sum(sg.length for sg in self.path[self.seg_i + 1:])
        if np.hypot(*off) < 0.02 or rest < 3.0:          # (the car needs some distance to follow the move)
            return
        step = off * max(0.3, min(1.0, 0.02 / np.hypot(*off)))
        cand = (self.goal[0] - step[0], self.goal[1] - step[1], self.goal[2])
        pts = self.grid.occupied_points()
        if len(pts):
            pts = pts[np.hypot(pts[:, 0] - cand[0], pts[:, 1] - cand[1]) < 8.0]
            if len(pts) and footprint_hits(np.array([cand]), pts, 0.12)[0]:
                return
        beyond = 0.0
        for seg in reversed(self.path[self.seg_i:]):
            seg.reanchor(self.goal, cand, beyond)
            beyond += seg.length
        self.goal = cand

    def _monitor(self):
        """Stop and replan if newly seen obstacles are in the way of the remaining path."""
        pts = self.grid.occupied_points()
        if len(pts) == 0:
            return
        poses = [self.path[self.seg_i].poses(self.tracker.i)] + [s.poses() for s in self.path[self.seg_i + 1:]]
        poses = np.concatenate(poses)
        near = pts[np.hypot(pts[:, 0] - self.pose[0], pts[:, 1] - self.pose[1]) < 25.0]
        hard = bool(footprint_hits(poses, near, 0.0).any())
        # With cameras, an obstacle is only placed exactly once it is close, often after the plan
        # was made: also react when it turns out to be nearer to the path than the plan allowed
        # for. That asks for a better plan. If there is none, the current one is still drivable.
        margin = min(0.10, self.plan_info["margin"] - 0.03) if isinstance(self.sensor, SensorRig) else 0.0
        close = not hard and margin > 0.0 and self.watch_margin and self.replans < self.MAX_REPLANS and \
            bool(footprint_hits(poses, near, margin).any())
        self.blocked = self.blocked + 1 if hard or close else 0
        if self.blocked >= 2:
            self.replans += 1
            self.must_replan = hard            # the current plan cannot be driven any further
            self.watch_margin = hard           # (asked once per plan)
            self.say("the path is blocked by something newly seen, replanning" if hard else
                     "an obstacle is nearer to the path than planned for, replanning")
            self.tracker.stop()
            self.state, self.t_still = "BRAKE", None

    def _segment_done(self):
        self.seg_i += 1
        if self.seg_i < len(self.path):
            self.gear_changes += 1
            self._follow(self.path[self.seg_i])
            return
        lon, lat, dth = self._pose_error(self.goal)
        if (abs(lat) > 0.08 or abs(dth) > math.radians(1.5) or abs(lon) > 0.3) and \
                self.corrections < self.MAX_CORRECTIONS:
            self.corrections += 1
            self.say("off the stall centre by %.2f m / %.1f deg, correcting" % (lat, math.degrees(dth)))
            self._request_plan()
            return
        self._finish(True, "parked")

    def _pose_error(self, ref, pose=None):
        pose = self.pose if pose is None else pose
        dx, dy = pose[0] - ref[0], pose[1] - ref[1]
        c, s = math.cos(ref[2]), math.sin(ref[2])
        return dx * c + dy * s, -dx * s + dy * c, wrap(pose[2] - ref[2])

    def _finish(self, ok, text):
        self.state = "PARKED" if ok else "FAILED"
        self.say(text)
        res = dict(ok=ok, time=self.time, plan_time=self.plan_time, gear_changes=self.gear_changes,
                   replans=self.replans, corrections=self.corrections, min_clearance=self.min_clearance)
        if ok and self.manual is not None and self.target is None:
            lon, lat, dth = self._pose_error((self.manual[0] - EGO.center * math.cos(self.manual[2]),
                                              self.manual[1] - EGO.center * math.sin(self.manual[2]), self.manual[2]),
                                             self.true_pose)
            res.update(kind="manual", lateral=lat, depth=lon, heading_deg=math.degrees(abs(dth)))
            res["ok"] = bool(self.min_clearance > 0.0 and math.hypot(lon, lat) < 0.5)
        elif ok:
            # score against the ground-truth stall the car ended up in, with the pose it really has
            true = self.true_pose
            c = np.array(true[:2]) + EGO.center * np.array([math.cos(true[2]), math.sin(true[2])])
            stall = min(self.scn.stalls, key=lambda s: np.hypot(*(s["center"] - c)))
            u = stall["u_in"]
            nu = np.array([-u[1], u[0]])
            axis = math.atan2(u[1], u[0]) + (0.5 * math.pi if stall["kind"] == "parallel" else 0.0)
            head = wrap(true[2] - axis)
            head = min(abs(head), abs(wrap(head - math.pi)))
            poly = stall["corners"]
            e = np.roll(poly, -1, axis=0) - poly
            cr = lambda p: e[:, 0] * (p[1] - poly[:, 1]) - e[:, 1] * (p[0] - poly[:, 0])
            inside = all(np.all(cr(p) >= -0.02) or np.all(cr(p) <= 0.02) for p in ego_poly(true))
            res.update(kind=stall["kind"], gt_occupied=stall["occupied"], inside_lines=bool(inside),
                       lateral=float((c - stall["center"]) @ (u if stall["kind"] == "parallel" else nu)),
                       depth=float((c - stall["center"]) @ (nu if stall["kind"] == "parallel" else u)),
                       heading_deg=math.degrees(head))
            res["ok"] = bool(inside and not stall["occupied"] and self.min_clearance > 0.0)
        if "lateral" in res:
            self.message = "%s: %.0f cm off centre, %.1f deg off axis, %d gear changes, %.0f s" % (
                text, 100.0 * abs(res["lateral"]), res["heading_deg"], self.gear_changes, self.time)
        self.result = res
        print("[result] " + "  ".join("%s=%s" % (k, ("%.3f" % v) if isinstance(v, float) else v)
                                        for k, v in res.items()), flush=True)
