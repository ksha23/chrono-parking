"""The plan into the chosen stall: what the planner is asked for, and what is done with its answer."""

import math
import threading
import time

import numpy as np

from .geometry import footprint_hits, wrap
from .planner import CSpace, holonomic_distance, split_segments
from .sensors import SensorRig
from .vehicle import EGO


class PlanStall:
    """The part of ParkingSim (agent.py) that plans the way into a stall, or to where the user pointed."""
    # Parallel parking: the car may not come nearer to the kerb than its side will be when it
    # is parked, plus this much with its front corners and this much with its rear ones [m].
    # The room for the maneuver is on the street. What lies beyond the stall is a kerb whether
    # or not anything has seen it, and a camera cannot tell it from the road until it is close.
    # Backing in between two cars takes the rear corner 10 cm beyond where the side ends up.
    # The nose needs nothing there, and it is where an error in heading shows: 3 degrees are
    # 20 cm at the front corner. So the car backs in, and does not drive in nose first.
    KERB_SIDE = (0.05, 0.15)

    def _goal_spec(self):
        """What the planner should aim for: the chosen stall, or the pose the user asked for."""
        if self.target is not None:
            s = self.target
            nominal = self._held_goal(s)
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
        self.odo.hold(True)            # (also for a spot the user asked for)
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
            best = None
            for k, (m_lat, m_lon) in enumerate(spec["margins"]):
                self.planner.max_iter = 30000 if k == len(spec["margins"]) - 1 else 12000
                res = self._plan_once(start, spec, occ, m_lat, m_lon)
                if res is None:
                    if best is not None:
                        break
                    continue
                if best is None:
                    # A wider margin comes first, but not at any price. With 0.30 m one stall
                    # was to be had by 36 m of driving in five pieces, and with 0.20 m by 18 m
                    # in four: a plan of more than three pieces is held against the next
                    # margin, once, and gives way if that one is simpler and much the cheaper.
                    best = res
                    if len(res["segments"]) <= 3:
                        break
                    continue
                if len(res["segments"]) < len(best["segments"]) and res["stats"]["cost"] < 0.7 * best["stats"]["cost"]:
                    best = res
                break
            if best is not None:
                self.plan_result = best
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
        if spec["kind"] == "parallel":
            # Not towards the kerb (KERB_SIDE). The line is that of the kerb-side edge of the
            # car where it will stand. A car that is beyond the allowance already, after a
            # docking run that ended a few degrees off, may not go further.
            th = nominal[2]
            at = (nominal[0] + EGO.center * math.cos(th), nominal[1] + EGO.center * math.sin(th))
            cs.keep_off(at, spec["u"], 0.0, 0.0)
            f0, r0 = cs.reach(*start)
            cs.keep_off(at, spec["u"], max(EGO.half_width + self.KERB_SIDE[0], f0 + 0.02),
                        max(EGO.half_width + self.KERB_SIDE[1], r0 + 0.02))
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
                # If the car has not left the lane yet, this stall is lost and the others are
                # not: in one run a single map cell at the mouth of the stall turned into an
                # obstacle while the car stood and planned, with 13 free stalls further on.
                x, y, th = self.pose
                off = np.array([x, y]) - np.array(self.origin[:2])
                fwd = self.travel_dir
                if self.manual is None and self.target is not None and abs(wrap(th - self.origin[2])) < 0.2 and \
                        abs(off[0] * fwd[1] - off[1] * fwd[0]) < 0.6 and self.SEARCH_REACH - off @ fwd > 3.0:
                    self.path, self.seg_i, self.must_replan, self.blocked = [], 0, False, 0
                    self._search_on("the way into that stall is blocked, searching on")
                    return
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
            self._search_on("no feasible maneuver into that stall, searching on")
            return
        segs = res["segments"]
        self.goal, self.nominal = res["goal"], res["nominal"]
        if self.target is not None and self.planned is None:
            self.planned = (self.target.center.copy(), math.atan2(self.target.u_in[1], self.target.u_in[0]))
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
        # What the monitor will hold the plan to: 3 cm less than the margin it was planned with,
        # 10 cm at most, and no more than the plan has. The collision table knows a pose to a
        # cell and 3 degrees, so a fresh plan can pass a cell nearer than its margin says, and
        # asked for the full margin the monitor sent such a plan back at once, six times in a row.
        self.near_margin = 0.0
        if isinstance(self.sensor, SensorRig):
            self.near_margin = min(0.10, res["margin"] - 0.03)
            pts = self.grid.occupied_points()
            pts = pts[np.hypot(pts[:, 0] - self.pose[0], pts[:, 1] - self.pose[1]) < 25.0] if len(pts) else pts
            poses = np.concatenate([sg.poses() for sg in segs])
            while self.near_margin > 0.0 and len(pts) and footprint_hits(poses, pts, self.near_margin).any():
                self.near_margin = max(self.near_margin - 0.03, 0.0)
        self.state = "DRIVE"
        self._follow(segs[0])

    def _search_on(self, why):
        """Give up the chosen stall for now and drive on along the lane."""
        here = np.array(self.pose[:2]) + EGO.center * self.travel_dir
        old = [r for r in self.rejected if np.hypot(*(self.target.center - r[0])) < 1.5]
        self.rejected = [r for r in self.rejected if r not in old]
        self.rejected.append((self.target.center.copy(), here, 1 + sum(r[2] for r in old)))
        self.target = None
        self.planned = None
        self.say(why)
        self.state = "SEARCH"
        self.odo.hold(False)
        self.pose = self.odo.update(self.true_pose, self.time, self.world.wheel_travel())
        self._follow(self._search_route(), presteer=False)
