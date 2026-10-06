"""Stall inference: stalls from pairs of line tracks, and whether each is free."""

import numpy as np

from .one_line import one_line_stalls
from .rows import lane_rows
from .slot import KERB_GAP, TICK, Slot, _align_with_kerb, _classify, _kerb_behind, _seen_along
from .vehicle import EGO

LATE = 2.0           # a line may be seen to start this much further in, or out, than its row does [m]


def _runs_with(track, row):
    """Is this a line of that row by its direction? A piece of paint too short to have a
    direction is one only if it starts near the row's mouth (LATE). A line that runs the way
    the row does is one wherever in the stall it begins: while the car drives in, its cameras
    find the far half of a line whose first half is worn away or hidden."""
    return track.length >= 1.5 and row.d is not None and abs(track.d[0] * row.d[1] - track.d[1] * row.d[0]) < 0.1


class JoinedLine:
    """Pieces of one painted line with gaps between them, taken as one line. It has what a
    LineTrack has that the stall inference reads."""

    def __init__(self, parts):
        d = max(parts, key=lambda t: t.length).d
        n = np.array([-d[1], d[0]])
        w = np.array([t.length for t in parts])
        across = float(sum(wi * (t.c @ n) for wi, t in zip(w, parts)) / w.sum())
        s = np.array([q @ d for t in parts for q in t.ends()])
        self.d, self.watched, self.length = d, max(t.watched for t in parts), float(s.max() - s.min())
        self.c = 0.5 * (s.max() + s.min()) * d + across * n
        self._ends = (s.min() * d + across * n, s.max() * d + across * n)

    def ends(self):
        return self._ends


def join_collinear(tracks, gap=1.5):
    """Line tracks that lie on one straight line less than 'gap' apart, each such group as one
    line. A camera often sees a stripe in pieces: paint wears off, and where the edge of a shadow
    runs along a stripe, the stripe is not lighter than the ground on both sides and goes
    unseen. Two half-metre pieces of a tick are not a tick, but they are the same tick."""
    group = list(range(len(tracks)))

    def root(i):
        while group[i] != i:
            i = group[i]
        return i

    for i in range(len(tracks)):
        for j in range(i + 1, len(tracks)):
            a, b = (tracks[i], tracks[j]) if tracks[i].length >= tracks[j].length else (tracks[j], tracks[i])
            n = np.array([-a.d[1], a.d[0]])
            if max(abs((q - a.c) @ n) for q in b.ends()) > 0.15:           # not on the same line
                continue
            if b.length >= 1.2 and abs(a.d[0] * b.d[1] - a.d[1] * b.d[0]) > 0.09:
                continue
            sa, sb = sorted(q @ a.d for q in a.ends()), sorted(q @ a.d for q in b.ends())
            between = max(sa[0], sb[0]) - min(sa[1], sb[1])              # the gap, negative if they overlap
            if -0.3 < between < gap:
                group[root(j)] = root(i)
    parts = {}
    for i, t in enumerate(tracks):
        parts.setdefault(root(i), []).append(t)
    return [v[0] if len(v) == 1 else JoinedLine(v) for v in parts.values()]


def find_slots(tracks, trail, grid, stubs=False):
    """Pair up line tracks into stalls and classify them with the occupancy grid.
    trail: recent ego positions (N,2), used to tell the open end of a stall from its back.
    stubs: also accept a stall of which the lines are only partly seen. Looking along the lane, a
    camera sees the far line of an empty stall through the stall, but of the near line only the
    end that sticks out between the cars. Such a stall is one line of some length and a stub.
    Between two parked cars it sees two stubs and nothing else: such a stall is taken from the
    row it stands in (rows.py), which says where the stalls open and which way they point."""
    slots = []
    rows = {}
    if stubs:
        tracks = join_collinear(tracks)
        rows = lane_rows(tracks, trail)
    # The shortest piece of paint that counts as the tick of a parallel stall. A camera that looks
    # along the lane sees less of a tick next to a parked car: the car hides the part beside it.
    tick = 1.2 if stubs else 1.5
    n = len(tracks)
    trail = np.asarray(trail)
    # the direction across the lane, from the way the car has come: stalls open onto the lane
    lane = along = None
    if len(trail) >= 2 and np.hypot(*(trail[-1] - trail[0])) > 2.0:
        along = (trail[-1] - trail[0]) / np.hypot(*(trail[-1] - trail[0]))
        lane = np.array([-along[1], along[0]])
    for i in range(n):
        for j in range(i + 1, n):
            a, b = tracks[i], tracks[j]
            db = b.d if a.d @ b.d >= 0.0 else -b.d
            # the row the two lines stand in, if they are on one side of the lane
            row = None
            if rows and ((a.c - trail[0]) @ lane > 0.0) == ((b.c - trail[0]) @ lane > 0.0):
                row = rows.get(1 if (a.c - trail[0]) @ lane > 0.0 else -1)
            by_row = None
            if row is not None and row.count >= 3 and \
                    all(abs(t.d @ lane) > 0.3 and -0.4 < row.start(t) - row.mouth < 1.0 for t in (a, b)):
                # Two pieces of paint at the mouth of a row, at least one of them a stub that
                # says nothing of which way the stall points. How far apart they are along the
                # lane says which kind of stall it is.
                apart = abs((b.c - a.c) @ np.array([lane[1], -lane[0]]))
                if 5.0 <= apart <= 7.8 and row.longest <= 3.6 and min(a.length, b.length) < tick:
                    by_row, d = "parallel", row.inward
                elif max(a.length, b.length) < 2.0 and row.d is not None and \
                        2.2 <= abs((b.c - a.c) @ np.array([-row.d[1], row.d[0]])) <= 3.5:
                    by_row, d = "stall", row.d
            if by_row:
                pass
            elif stubs and min(a.length, b.length) < 1.2 and max(a.length, b.length) >= 2.0:
                d = a.d if a.length > b.length else db       # a stub has no direction of its own
            elif abs(a.d[0] * db[1] - a.d[1] * db[0]) > 0.14:
                continue
            else:
                d = a.d * a.length + db * b.length
                d = d / np.hypot(*d)
            nv = np.array([-d[1], d[0]])
            sep = (b.c - a.c) @ nv
            sa = np.sort(np.array(a.ends()) @ d)
            sb = np.sort(np.array(b.ends()) @ d)
            overlap = min(sa[1], sb[1]) - max(sa[0], sb[0])
            # partly seen: anything short of two long lines side by side. That includes two lines
            # of an angled stall of which only the halves near the lane were in view, which lie
            # too far apart lengthwise to overlap much.
            whole = min(a.length, b.length) >= 3.5 and overlap >= 3.0
            partial = stubs and not whole and max(a.length, b.length) >= 2.0
            if by_row == "stall":
                # each line from where it meets the mouth of the row, as deep as a stall has to be
                sa, sb = (row.entrance(t, d) + np.array([0.0, EGO.length + 0.7]) for t in (a, b))
                partial, parallel = True, False
            elif by_row == "parallel":
                sa, sb = (row.entrance(t, d) + np.array([0.0, TICK]) for t in (a, b))
                na, nb = sorted((a.c @ nv, b.c @ nv))
                if any(abs(k.d @ nv) < 0.5 and na + 1.0 < k.c @ nv < nb - 1.0 and sa[0] - 2.0 < k.c @ d < sa[1] + 2.0
                       for k in tracks if k is not a and k is not b):
                    continue                   # (a line in between: these two are not neighbours)
                # Two stubs this far apart are also what is seen of a wider stall with its
                # middle line hidden. Parallel stalls lie along a kerb: it has to be there.
                if _seen_along(grid, d, nv, sa[0] + 1.8, sa[0] + 3.8, na + 0.5, nb - 0.5) < 1.5:
                    continue
                parallel = True
            elif partial and 2.2 <= abs(sep) <= 3.5:
                # the shorter piece marks the mouth of the stall: one of its ends lies near the
                # matching end of the longer one (further off the more the stalls are angled)
                lng, sht = (sa, sb) if a.length > b.length else (sb, sa)
                reach = 1.2 * abs(sep) + 0.5
                end = None
                if lane is not None and abs(d @ lane) > 0.3:
                    # The ends of the two lines lie on a line along the lane, at the mouth and at
                    # the back. That says how far apart they are lengthwise, whatever the angle
                    # of the stalls: 1.56 m at 60 degrees, where a fixed window just misses it.
                    side = sep if a.length > b.length else -sep           # from the longer line to the shorter
                    shift = -side * (nv @ lane) / (d @ lane)
                    end = 0 if abs(sht[0] - lng[0] - shift) < 0.75 else 1 if abs(sht[1] - lng[1] - shift) < 0.75 else None
                if end is None:
                    end = 0 if lng[0] - reach < sht[0] < lng[0] + 1.5 else 1 if lng[1] - 1.5 < sht[1] < lng[1] + reach else None
                if end is None:
                    continue
                if end == 0:
                    sht = np.array([sht[0], sht[0] + lng[1] - lng[0]])      # as deep as the longer one
                else:
                    sht = np.array([sht[1] - lng[1] + lng[0], sht[1]])
                sa, sb = (lng, sht) if a.length > b.length else (sht, lng)
                parallel = False
            elif 2.2 <= abs(sep) <= 3.5 and whole:
                parallel = False
            elif (5.0 <= abs(sep) <= 7.8 and tick <= a.length <= 3.6 and tick <= b.length <= 3.6
                  and overlap >= tick - 0.3):
                # the two ticks must be neighbours: no third tick in between
                na, nb = sorted((a.c @ nv, b.c @ nv))
                if any(abs(k.d @ nv) < 0.2 and na + 1.0 < k.c @ nv < nb - 1.0 and
                       sa[0] - 2.0 < k.c @ d < sa[1] + 2.0 and 1.2 <= k.length < 3.6
                       for k in tracks if k is not a and k is not b):
                    continue
                parallel = True
            else:
                continue

            # lateral position of each line and of the open (lane-side) end
            n_a, n_b = a.c @ nv, b.c @ nv
            n_mid = 0.5 * (n_a + n_b)
            p_lo = d * 0.5 * (sa[0] + sb[0]) + nv * n_mid
            p_hi = d * 0.5 * (sa[1] + sb[1]) + nv * n_mid
            d_lo = np.hypot(*(trail - p_lo).T).min()
            d_hi = np.hypot(*(trail - p_hi).T).min()
            if d_lo <= d_hi:
                u_in, in_a, in_b, bk_a, bk_b = d, sa[0], sb[0], sa[1], sb[1]
            else:
                u_in, in_a, in_b, bk_a, bk_b = -d, -sa[1], -sb[1], -sa[0], -sb[0]
            nu = np.array([-u_in[1], u_in[0]])
            la, lb = a.c @ nu, b.c @ nu
            if by_row:
                pass                               # (taken from the mouth of the row already)
            elif row is not None and row.count >= 3 and not parallel and u_in @ row.inward > 0.3:
                # The lines of a row start on one line along the lane. A line that was seen to
                # start further in than that has lost its first piece to wear or to a shadow.
                # One that was seen to start further out has been carried on past its end by
                # something that looked like paint: in one run a line grew 0.7 m into the lane
                # while the car turned in, and took the stall with it. Either way the stall
                # starts where the row does. (This goes by where the paint was seen, not by the
                # extent assumed above for a partly seen line.) A line that runs the way the
                # row does may begin anywhere in the stall (_runs_with).
                oa, ob = (min(q @ u_in for q in t.ends()) for t in (a, b))
                ea, eb = row.entrance(a, u_in), row.entrance(b, u_in)
                if 0.25 < abs(oa - ea) < LATE:
                    in_a = ea
                elif _runs_with(a, row) and LATE <= oa - ea < EGO.length:
                    bk_a, in_a = bk_a - (in_a - ea), ea    # (its far end was assumed from where it began)
                if 0.25 < abs(ob - eb) < LATE:
                    in_b = eb
                elif _runs_with(b, row) and LATE <= ob - eb < EGO.length:
                    bk_b, in_b = bk_b - (in_b - eb), eb
            elif stubs and lane is not None and abs(u_in @ lane) > 0.3:
                # The two lines of a stall start on one line along the lane. If one was seen to
                # start up to 0.75 m further in than the other, its first piece is worn off or was
                # lost in a shadow, and the stall starts where the other one does. (This goes by
                # where the paint was seen, not by the extent assumed above for a partly seen line.)
                oa, ob = (min(q @ u_in for q in t.ends()) for t in (a, b))
                late = ((u_in * oa + nu * la) - (u_in * ob + nu * lb)) @ lane / (u_in @ lane)
                if 0.1 < late < 0.75:
                    in_a = oa - late
                elif -0.75 < late < -0.1:
                    in_b = ob + late
            s0, s1 = max(in_a, in_b), min(bk_a, bk_b)
            if partial and not parallel and s1 < s0 + EGO.length + 0.7:
                # the far end of the lines is out of sight: take the stall to be deep enough for the car
                s1 = s0 + EGO.length + 0.7
                bk_a, bk_b = max(bk_a, s1), max(bk_b, s1)
            corners = np.array([u_in * in_a + nu * la, u_in * in_b + nu * lb,
                                u_in * bk_b + nu * lb, u_in * bk_a + nu * la])
            if parallel:
                s_c = 0.5 * (in_a + in_b) + min(max(0.5 * (s1 - s0), 1.2), 1.4)
                slot = Slot("parallel", u_in * s_c + nu * 0.5 * (la + lb), u_in, nu,
                            abs(sep), s1 - s0, corners)
            else:
                s_c = s0 + min(max(0.5 * (s1 - s0), 2.65), 2.95)
                kerb = _kerb_behind(grid, u_in, nu, s0, min(la, lb), max(la, lb), along)
                if kerb is not None:           # however deep the lines say the stall is, the car ends short of the kerb
                    s_c = min(s_c, kerb - 0.5 * EGO.length - KERB_GAP)
                skew = abs(in_a - in_b) / max(abs(la - lb), 1e-6)
                slot = Slot("perpendicular" if skew < 0.2 else "angled",
                            u_in * s_c + nu * 0.5 * (la + lb), u_in, nu, abs(sep), s1 - s0, corners)
            slot.watched, slot.by_row = min(a.watched, b.watched), bool(by_row)
            _classify(slot, grid, s0, s1, min(la, lb), max(la, lb), deep=stubs and not parallel,
                      stagger=(in_b - in_a) * (1.0 if la < lb else -1.0))
            if parallel:
                _align_with_kerb(slot, grid, s1, min(la, lb), max(la, lb), along)
            slots.append(slot)
    if rows:
        slots += one_line_stalls(tracks, rows, slots, trail, grid, lane, along, tick)
    return slots
