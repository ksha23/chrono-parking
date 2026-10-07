"""The chosen stall while the car drives into it: how far the plan follows the stall's estimate,
and the watch for obstacles on what is left of the path."""

import math

import numpy as np

from .geometry import footprint_hits, wrap
from .slot import Slot


class DriveIn:
    """The part of ParkingSim (agent.py) that keeps a plan on its stall while it is driven."""
    MAX_REPLANS = 6
    SETTLED = 2.0            # for the last metres of its way in, the stall is where it was taken to be [m]
    # The estimate of the chosen stall is followed while it stays this near to where the stall
    # was when the plan was made, across the stall [m] and in direction [rad]. An estimate
    # further off than that is another reading of the paint, not a better one of the same: of
    # those that were looked at, each was wrong.
    #
    # Along the stall it is followed one way only: towards the lane. Paint that is seen is
    # there. Paint that is not seen may be worn, in a shadow or behind something, so a stall
    # may turn out to begin nearer to the lane than it was taken to, and not further in. In 21
    # runs the estimate moved by up to 30 cm either way while the car drove in. Followed
    # wherever it went, the car ended 12 cm off in depth on average and 34 cm at worst.
    # Followed towards the lane only, by the same numbers, 7 cm and 18 cm. Not followed at
    # all, 9 cm and 21 cm, and in one run 84 cm: that stall had been taken 0.9 m too deep.
    # (A parallel stall is not followed at all across the kerb: see _toward_lane.)
    #
    # And an estimate that the map shows as taken is not one of the stall the car is driving
    # into. One stall was estimated right, to 3 cm, until the car was half-way in. Then one of
    # its lines was paired with a stub of something beside the next car, 0.7 m further on: a
    # stall 3.4 m wide, 0.33 m to the side, with the next car in it. For that moment it was
    # the only estimate of its kind there, the car followed it, and from then on it was the
    # nearest to the last one. The car parked 0.30 m off the centre. (Two more tests were
    # tried with it and dropped. The same width as at the plan, to 0.3 m: the width of an
    # angled stall moves by more than that as the direction of its lines gets known, and a car
    # that was kept from following ended 16 cm and 2.4 degrees off. The estimate nearest to
    # the plan and not to the last one: three cars ended 6 to 10 cm further along their
    # parallel stalls, and none nearer to the middle.)
    FOLLOW = (0.5, 0.1)

    def _held_goal(self, s):
        """The pose to park at in stall s, as it is estimated now: across the stall and in
        direction by that estimate, and along the stall no further in than the first plan had
        it or any estimate since (FOLLOW)."""
        goal = s.goal(self.nose_in, self.park_dir)
        if self.planned is None:
            return goal
        u = np.array([math.cos(self.planned[1]), math.sin(self.planned[1])])
        back = float((s.center - self.planned[0]) @ u)
        return (goal[0] - back * u[0], goal[1] - back * u[1], goal[2])

    def _toward_lane(self):
        """Where along the stall the car is to stand moves with the estimate if that puts the
        stall nearer to the lane, a third of the way per look, and stays if it does not. Not
        beside a kerb: how far a parallel stall is from the kerb was seen while the car drove
        up, and nothing sees the kerb while it backs in. (Followed towards the street, one such
        estimate took the car 0.58 m out of its stall.)"""
        if self.planned is None or self.target is None or self.target.kind == "parallel":
            return
        u = np.array([math.cos(self.planned[1]), math.sin(self.planned[1])])
        back = float((self.target.center - self.planned[0]) @ u)
        if back < 0.0:
            self.planned = (self.planned[0] + 0.3 * back * u, self.planned[1])

    def _retarget(self):
        """Follow the chosen stall through the stream of fresh stall estimates. Once a plan has
        been made for it, only as far as FOLLOW from where it was then."""
        cand = [s for s in self.slots if s.kind == self.target.kind and
                np.hypot(*(s.center - self.target.center)) < 1.2]
        if self.planned is not None and self.path:
            across = np.array([-math.sin(self.planned[1]), math.cos(self.planned[1])])
            cand = [s for s in cand if abs((s.center - self.planned[0]) @ across) < self.FOLLOW[0] and
                    abs(wrap(math.atan2(s.u_in[1], s.u_in[0]) - self.planned[1])) < self.FOLLOW[1] and
                    s.status != Slot.OCCUPIED]
        if cand:
            self.target = min(cand, key=lambda s: np.hypot(*(s.center - self.target.center)))
            if self.path:
                self._toward_lane()
        return bool(cand)

    def _refine(self):
        """Keep the plan attached to the stall as its line estimates improve."""
        # Not on the last metres. What the cameras show of a stall from inside it is little, and
        # an estimate that changes there changes for the worse as often as not: the car had
        # parked well, the estimate moved, and two corrections later it stood 0.6 m off.
        cur = self.path[self.seg_i]
        rest = cur.s[-1] - cur.s[min(self.tracker.i, len(cur.s) - 1)] + sum(sg.length for sg in self.path[self.seg_i + 1:])
        if rest < self.SETTLED:
            return
        if not self._retarget():
            return
        new = self._held_goal(self.target)
        old = self.nominal
        d, dth = math.hypot(new[0] - old[0], new[1] - old[1]), abs(wrap(new[2] - old[2]))
        if d < 0.01 and dth < 0.003:
            self._recentre()
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
        left = self.path[self.seg_i:]
        was = [(seg.x.copy(), seg.y.copy(), seg.th.copy()) for seg in left]
        beyond = 0.0
        for seg in reversed(left):
            seg.reanchor(self.goal, cand, beyond)
            beyond += seg.length
        # Not if that brings any of what is left of the path within 12 cm of an obstacle, or
        # nearer than the monitor allows. (Only the goal used to be looked at. The goal can
        # have been put off centre for the sake of the run up to it: moved back, the path was
        # nearer to an obstacle than the monitor allows, and the car stopped to plan again.)
        pts = self.grid.occupied_points()
        if len(pts):
            pts = pts[np.hypot(pts[:, 0] - cand[0], pts[:, 1] - cand[1]) < 25.0]
            poses = np.concatenate([left[0].poses(self.tracker.i)] + [seg.poses() for seg in left[1:]])
            if len(pts) and footprint_hits(poses, pts, max(0.12, self.near_margin + 0.02)).any():
                for seg, (x, y, th) in zip(left, was):
                    seg.x, seg.y, seg.th = x, y, th
                return
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
        margin = self.near_margin
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
