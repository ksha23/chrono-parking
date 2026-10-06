"""Stall inference: stalls from pairs of line tracks, and whether each is free."""

import math

import numpy as np

from .geometry import wrap
from .rows import lane_rows
from .vehicle import EGO


class Slot:
    FREE, OCCUPIED, UNKNOWN = "free", "occupied", "unknown"

    def __init__(self, kind, center, u_in, along, width, depth, corners):
        self.kind = kind            # 'perpendicular' | 'angled' | 'parallel'
        self.center = center        # where the middle of the parked car should be
        self.u_in = u_in            # unit vector from the lane into the stall
        self.along = along          # parallel stalls: unit vector along the kerb
        self.width = width
        self.depth = depth
        self.corners = corners      # entrance A, entrance B, back B, back A
        self.status = Slot.UNKNOWN
        self.neighbors = [False, False]
        self.watched = 0.0          # for how long the less watched of its two lines has been watched [s]
        self.by_row = False         # known from two stubs and the row they stand in, not from a line of its own
        self.region = None          # (mask, slices) of the grid cells between the two lines

    def goal(self, nose_in, travel_dir):
        """Rear-axle pose of the parked car."""
        if self.kind == "parallel":
            t = self.along if self.along @ travel_dir >= 0.0 else -self.along
            th = math.atan2(t[1], t[0])
        else:
            th = math.atan2(self.u_in[1], self.u_in[0]) + (0.0 if nose_in else math.pi)
        return (self.center[0] - EGO.center * math.cos(th),
                self.center[1] - EGO.center * math.sin(th), wrap(th))


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
                # extent assumed above for a partly seen line.)
                oa, ob = (min(q @ u_in for q in t.ends()) for t in (a, b))
                ea, eb = row.entrance(a, u_in), row.entrance(b, u_in)
                if 0.25 < abs(oa - ea) < LATE:
                    in_a = ea
                if 0.25 < abs(ob - eb) < LATE:
                    in_b = eb
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
                kerb = _kerb_behind(grid, u_in, nu, s0, min(la, lb), max(la, lb))
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
        slots += _one_line_stalls(tracks, rows, slots, trail, grid, lane, along, tick)
    return slots


def _one_line_stalls(tracks, rows, slots, trail, grid, lane, along, tick):
    """Stalls of which one line was found and the other was not: worn away, or under the wheels
    of the car parked next to it. The row says where the other line has to be, one stall's width
    on. Such a stall counts if its ground was seen to be free and there is a reason to take the
    place for a stall: a parked car beyond the line that was not found, or a parked car beyond
    the one that was, as long as the row across the lane reaches as far (beyond the last line of
    a row there is free ground too, and no stall)."""
    origin, out = trail[0], []

    def anchors(r):                            # the lines of a row that start at its mouth
        return [t for t in tracks if abs(t.d @ lane) > 0.3 and (t.c - origin) @ r.inward > 0.5
                and -0.4 < r.start(t) - r.mouth < 1.0]

    def where(r, p, dr):                       # where a point, carried along dr to the mouth of row r, is along the lane
        return float((p + (r.mouth - r.off_path(p)) / (dr @ r.inward) * dr) @ along)

    for side, row in rows.items():
        own = anchors(row)
        if len(own) < 3:
            continue
        # Which kind of stall the row is made of. The ticks of parallel stalls are short and a
        # car's length apart. Lines of other stalls are a car's width apart, and although some
        # are missed, most neighbours are not. (The stalls already made out do not say: two line
        # ends two stalls apart pass for the ticks of a parallel stall, and a tick with a bit
        # of something beside it for a narrow stall.)
        at = np.sort([min(t.ends(), key=row.off_path) @ along for t in own])
        gaps = np.diff(at)
        gaps = gaps[gaps > 0.6]                # (one line found as two pieces)
        if not len(gaps):
            continue
        parallel = row.longest <= 3.6 and float(np.median(gaps)) >= 4.5
        mine = [s for s in slots if (s.center - origin) @ row.inward > 0.5 and (s.kind == "parallel") == parallel]
        d = row.inward if parallel else row.d
        if d is None:
            continue
        nu = np.array([-d[1], d[0]])
        slant = abs(d @ row.inward)
        xs = [where(row, t.c, d) for t in own]
        known = [where(row, s.center, d) for s in mine]
        # the width of a stall along the lane: from the stalls already made out on this side, or
        # from how far apart the lines are, which is a whole number of stalls
        lo, hi = (5.0, 7.8) if parallel else (2.2 / slant, 3.5 / slant)
        if mine:
            pitch = float(np.median([s.width for s in mine])) / (1.0 if parallel else slant)
        else:
            cand = [abs(a - b) / k for i, a in enumerate(xs) for b in xs[i + 1:] for k in (1, 2, 3, 4) if lo <= abs(a - b) / k <= hi]
            if not cand:
                continue
            fits = lambda p: sum(abs((x - xs[0]) / p - round((x - xs[0]) / p)) * p < 0.4 for x in xs)
            pitch = max(cand, key=lambda p: (fits(p), p))
        if not lo <= pitch <= hi:
            continue
        other = rows.get(-side)
        far = []
        if other is not None:
            do = other.inward if other.d is None else other.d
            far = [where(other, t.c, do) for t in anchors(other)]
        for t, x in zip(own, xs):
            if t.length < (tick if parallel else 1.5):
                continue
            for sg in (1.0, -1.0):
                if any(abs(x + sg * pitch - q) < 0.6 for q in xs):
                    continue                   # (there is a line where the other one belongs: a pair, above)
                step = sg * pitch * along
                if any(abs(k - (x + 0.5 * sg * pitch)) < 0.5 * pitch for k in known):
                    continue                   # (a stall is known there already)
                in_a = row.entrance(t, d)
                in_b = in_a + step @ d
                la = t.c @ nu
                lb = la + step @ nu
                deep = TICK if parallel else EGO.length + 0.7
                s0, s1 = max(in_a, in_b), min(in_a, in_b) + deep
                corners = np.array([d * in_a + nu * la, d * in_b + nu * lb, d * (in_b + deep) + nu * lb, d * (in_a + deep) + nu * la])
                if parallel:
                    s_c = 0.5 * (in_a + in_b) + min(max(0.5 * (s1 - s0), 1.2), 1.4)
                    slot = Slot("parallel", d * s_c + nu * 0.5 * (la + lb), d, nu, abs(la - lb), s1 - s0, corners)
                else:
                    s_c = s0 + min(max(0.5 * (s1 - s0), 2.65), 2.95)
                    kerb = _kerb_behind(grid, d, nu, s0, min(la, lb), max(la, lb))
                    if kerb is not None:
                        if kerb - s0 < 4.0:
                            continue           # (no room for a car: whatever this is, it is not such a stall)
                        s_c = min(s_c, kerb - 0.5 * EGO.length - KERB_GAP)
                    skew = abs(in_a - in_b) / max(abs(la - lb), 1e-6)
                    slot = Slot("perpendicular" if skew < 0.2 else "angled", d * s_c + nu * 0.5 * (la + lb), d, nu,
                                abs(la - lb), s1 - s0, corners)
                slot.watched, slot.by_row = t.watched, True
                _classify(slot, grid, s0, s1, min(la, lb), max(la, lb), deep=not parallel,
                          stagger=(in_b - in_a) * (1.0 if la < lb else -1.0))
                if slot.status != Slot.FREE:
                    continue
                beyond_found, beyond_other = slot.neighbors if la < lb else slot.neighbors[::-1]
                reaches = len(far) >= 2 and min(far) - 0.5 * pitch <= x + sg * pitch <= max(far) + 0.5 * pitch
                if not (beyond_other or (beyond_found and reaches)):
                    continue
                if parallel:
                    if _seen_along(grid, d, nu, s0 + 1.8, s0 + 3.8, min(la, lb) + 0.5, max(la, lb) - 0.5) < 1.5:
                        continue
                    _align_with_kerb(slot, grid, s1, min(la, lb), max(la, lb), along)
                out.append(slot)
    return out


KERB_GAP = 0.40      # between the end of the parked car and a kerb that a camera network saw [m]
TICK = 2.5           # how deep a parallel stall is taken to be when only the ends of its ticks were seen [m]
LATE = 2.0           # a line may be seen to start this much further in, or out, than its row does [m]


def _seen_along(grid, u, nu, s_lo, s_hi, l0, l1):
    """Over what length along nu, between l0 and l1, something stands at a distance s_lo to s_hi
    along u: an obstacle in the map, or a kerb that a camera network saw [m]."""
    box = np.array([u * s + nu * l for s in (s_lo, s_hi) for l in (l0, l1)])
    win = grid.window(box[:, 0].min(), box[:, 1].min(), box[:, 0].max(), box[:, 1].max())
    if win is None:
        return 0.0
    X, Y, sl = win
    S, Lc = X * u[0] + Y * u[1], X * nu[0] + Y * nu[1]
    there = grid.occupied()[sl]
    if getattr(grid, "kerb", None) is not None:
        there = there | (grid.kerb[sl] >= 0.35)
    m = there & (S > s_lo) & (S < s_hi) & (Lc > l0) & (Lc < l1)
    return 0.1 * len(np.unique(np.floor(Lc[m] / 0.1))) if m.any() else 0.0


def _kerb_behind(grid, u, nu, s0, l0, l1):
    """Where the kerb at the back of a stall is: the distance along u (into the stall) at which the
    car would meet it, or None if no kerb was seen there. It is read from the map's kerb layer
    between the two lines, from 2.5 to 9.5 m in from the mouth.

    Behind an angled stall the kerb lies at a slant, so a line is fitted to what was seen, and
    the answer is for the side of the car that the kerb is nearer to. A kerb seen from far away is
    smeared over half a metre in range, most of it beyond its face: the line is put where the
    nearest fifth of what was seen begins to count."""
    kerb = getattr(grid, "kerb", None)
    if kerb is None or not kerb.any():
        return None
    box = np.array([u * s + nu * l for s in (s0 + 2.5, s0 + 9.5) for l in (l0, l1)])
    win = grid.window(box[:, 0].min(), box[:, 1].min(), box[:, 0].max(), box[:, 1].max())
    if win is None:
        return None
    X, Y, sl = win
    S, Lc, K = X * u[0] + Y * u[1], X * nu[0] + Y * nu[1], kerb[sl]
    m = (K >= 0.35) & (S > s0 + 2.5) & (S < s0 + 9.5) & (Lc > l0 + 0.2) & (Lc < l1 - 0.2)
    if m.sum() < 8 or np.ptp(Lc[m]) < 1.0:
        return None
    S, Lc, K = S[m], Lc[m] - 0.5 * (l0 + l1), K[m]
    slant = float(np.polyfit(Lc, S, 1, w=np.sqrt(K))[0]) if np.ptp(Lc) >= 1.5 else 0.0
    slant = 0.0 if abs(slant) < 0.25 else max(-1.2, min(1.2, slant))       # (square to the stall, or up to 50 degrees off)
    level = S - slant * Lc                                # as far in as the kerb is at the middle of the stall
    order = np.argsort(level)
    share = np.cumsum(K[order]) / K.sum()
    return float(level[order][np.searchsorted(share, 0.2)]) - abs(slant) * (EGO.half_width + 0.1)


def _align_with_kerb(slot, grid, s1, l0, l1, lane=None):
    """Parallel stalls: the two short ticks give a poor heading. If the range scan shows the
    kerb behind the stall, park parallel to it and 0.3 m off it, like a driver would.
    lane: the direction of the lane, if the car has driven enough of it to tell."""
    c = slot.corners
    win = grid.window(c[:, 0].min() - 1.5, c[:, 1].min() - 1.5, c[:, 0].max() + 1.5, c[:, 1].max() + 1.5)
    if win is None:
        return
    X, Y, sl = win
    u, nu = slot.u_in, slot.along
    S, Lc = X * u[0] + Y * u[1], X * nu[0] + Y * nu[1]
    m = grid.occupied()[sl] & (S > s1 - 0.6) & (S < s1 + 0.9) & (Lc > l0 + 0.2) & (Lc < l1 - 0.2)
    if m.sum() < 25 or np.ptp(Lc[m]) < 3.5:
        return
    mid = 0.5 * (l0 + l1)
    # the face of the kerb is the lane-side edge of what was hit: a sensor that looks down on the
    # kerb also returns points from its top. Per 0.2 m along the kerb, keep the nearest cells.
    col = np.floor((Lc[m] - l0) / 0.2).astype(int)
    first = np.full(col.max() + 1, np.inf)
    np.minimum.at(first, col, S[m])
    edge = S[m] < first[col] + 0.12
    Le, Se = Lc[m][edge], S[m][edge]
    # A kerb runs along the lane. Something else that was hit in the window, like a corner of the
    # car parked next to the stall, is not on a line along the lane through the middle of the
    # hits, and a line that turns out more than 4 degrees off the lane is not the kerb. (A fit to
    # all the hits let such a corner tilt it by 7 degrees, and the car then set out to correct a
    # heading that was right.)
    b0 = float((lane @ u) / (lane @ nu)) if lane is not None and abs(lane @ nu) > 0.5 else 0.0
    level = Se - b0 * (Le - mid)
    keep = np.abs(level - np.median(level)) < 0.15
    if keep.sum() < 15 or np.ptp(Le[keep]) < 3.5:
        return
    b, a = np.polyfit(Le[keep] - mid, Se[keep], 1)
    if abs(math.atan(b) - math.atan(b0)) > math.radians(4.0):
        return
    along = (nu + b * u) / math.hypot(1.0, b)
    u_k = np.array([along[1], -along[0]])
    if u_k @ u < 0.0:
        u_k = -u_k
    # the fitted line runs through the middle of the kerb's hit cells; its face is ~half a cell nearer
    slot.center = u * (a - 0.05) + nu * mid - u_k * (EGO.half_width + 0.30)
    slot.u_in, slot.along = u_k, along


def _classify(slot, grid, s0, s1, l0, l1, deep=False, stagger=0.0):
    """Free / occupied / unknown from the grid cells inside the stall, plus neighbours.
    deep: the far end of the stall may be out of sight (a sensor rig that looks along the lane).
    stagger: how much further in the stall starts at l1 than at l0 (angled stalls) [m]."""
    c = slot.corners
    win = grid.window(c[:, 0].min() - 3.0, c[:, 1].min() - 3.0, c[:, 0].max() + 3.0, c[:, 1].max() + 3.0)
    if win is None:
        return
    X, Y, sl = win
    u, nu = slot.u_in, np.array([-slot.u_in[1], slot.u_in[0]])
    S = X * u[0] + Y * u[1]
    Lc = X * nu[0] + Y * nu[1]
    occ = grid.occupied()[sl]
    free = grid.clear()[sl]
    lat = 0.35 if slot.kind != "parallel" else 0.6
    back = 0.45
    inside = (S > s0 + 0.1) & (S < s1 - back) & (Lc > l0 + lat) & (Lc < l1 - lat)
    cells = max(int(inside.sum()), 1)
    n_occ = int((occ & inside).sum())
    frac_free = (free & inside).sum() / cells
    # Seen from along the lane, the far end of a stall between two cars lies in the shadow of the
    # nearer car. A parked car would show at the mouth of the stall, so a stall whose mouth is seen
    # to be empty counts as free; what is deeper in comes into view while backing in.
    mouth = inside & (S < s0 + 2.5)
    frac_mouth = (free & mouth).sum() / max(int(mouth.sum()), 1)
    # A camera that looks forward sees into a stall only at a slant, past the corner of the car
    # before it: of an empty stall it sees a wedge, about two thirds of the mouth. A car in the
    # stall would stand in that wedge. So with such a rig a stall is free if most of its mouth is
    # seen empty and the empty ground is seen at least 2 m into it.
    wedge = deep and frac_mouth >= 0.55 and frac_free >= 0.25 and \
        (free & inside).any() and float(S[free & inside].max()) - s0 >= 2.0
    # A stall that is known from its row is taken when the car is level with it (agent.py), and
    # by then the cameras have seen all of it that they are going to. Behind a wide car the
    # wedge is narrower than that. What is left to ask is whether a car could stand in the
    # stall: its front would be in the first 1.2 m, across most of the width.
    front = inside & (S < s0 + 1.2)
    level = deep and slot.by_row and front.any() and (free & front).sum() >= 0.75 * front.sum() and \
        float(S[free & inside].max()) - s0 >= 2.0
    if n_occ >= 4:
        slot.status = Slot.OCCUPIED
    elif n_occ <= 1 and (frac_free >= 0.6 or (frac_mouth >= 0.8 and frac_free >= 0.3) or wedge or level):
        slot.status = Slot.FREE
    # The stalls of an angled row are staggered: the one next to this one starts further in,
    # or further out, and so does the car in it. (A band straight across found the car on one
    # side by its front, and of the car on the other side only what the cameras do not see.)
    enter = s0 + (Lc - (l1 if stagger > 0.0 else l0)) * stagger / max(l1 - l0, 1e-6) if slot.kind == "angled" else s0
    side = (S > enter - 0.3) & (S < enter + s1 - s0 - back)
    reach = 2.4 if slot.kind != "parallel" else 3.0
    slot.neighbors = [bool((occ & side & (Lc < l0 - 0.05) & (Lc > l0 - reach)).sum() >= 4),
                      bool((occ & side & (Lc > l1 + 0.05) & (Lc < l1 + reach)).sum() >= 4)]
    # The cells the planner may take to be free once this stall is chosen. If the far end is out
    # of sight, that has to reach as far as the parked car does, plus the planner's margin: a
    # stall is a place that holds a car. What is really there shows up on the way in.
    end = s1 - 0.2
    if deep:
        end = max(end, slot.center @ u + 0.5 * EGO.length + 0.35)
    slot.region = (S > s0 - 1.0) & (S < end) & (Lc > l0 + 0.1) & (Lc < l1 - 0.1), sl
