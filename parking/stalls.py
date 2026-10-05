"""Stall inference: stalls from pairs of line tracks, and whether each is free."""

import math

import numpy as np

from .geometry import wrap
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
        self.hits = 0               # detections behind the weaker of the two lines
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
        self.d, self.hits, self.length = d, sum(t.hits for t in parts), float(s.max() - s.min())
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
    end that sticks out between the cars; such a stall is one line of some length and a stub."""
    slots = []
    if stubs:
        tracks = join_collinear(tracks)
    # The shortest piece of paint that counts as the tick of a parallel stall. A camera that looks
    # along the lane sees less of a tick next to a parked car: the car hides the part beside it.
    tick = 1.2 if stubs else 1.5
    n = len(tracks)
    trail = np.asarray(trail)
    for i in range(n):
        for j in range(i + 1, n):
            a, b = tracks[i], tracks[j]
            db = b.d if a.d @ b.d >= 0.0 else -b.d
            if stubs and min(a.length, b.length) < 1.2 and max(a.length, b.length) >= 2.0:
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
            if partial and 2.2 <= abs(sep) <= 3.5:
                # the shorter piece marks the mouth of the stall: one of its ends lies near the
                # matching end of the longer one (further off the more the stalls are angled)
                lng, sht = (sa, sb) if a.length > b.length else (sb, sa)
                reach = 1.2 * abs(sep) + 0.5
                if lng[0] - reach < sht[0] < lng[0] + 1.5:
                    sht = np.array([sht[0], sht[0] + lng[1] - lng[0]])      # as deep as the longer one
                elif lng[1] - 1.5 < sht[1] < lng[1] + reach:
                    sht = np.array([sht[1] - lng[1] + lng[0], sht[1]])
                else:
                    continue
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
                skew = abs(in_a - in_b) / max(abs(la - lb), 1e-6)
                slot = Slot("perpendicular" if skew < 0.2 else "angled",
                            u_in * s_c + nu * 0.5 * (la + lb), u_in, nu, abs(sep), s1 - s0, corners)
            slot.hits = min(a.hits, b.hits)
            _classify(slot, grid, s0, s1, min(la, lb), max(la, lb), deep=stubs and not parallel)
            if parallel:
                _align_with_kerb(slot, grid, s1, min(la, lb), max(la, lb))
            slots.append(slot)
    return slots


def _align_with_kerb(slot, grid, s1, l0, l1):
    """Parallel stalls: the two short ticks give a poor heading. If the range scan shows the
    kerb behind the stall, park parallel to it and 0.3 m off it, like a driver would."""
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
    b, a = np.polyfit(Le - mid, Se, 1)
    keep = np.abs(Se - (a + b * (Le - mid))) < 0.15
    if keep.sum() < 15:
        return
    b, a = np.polyfit(Le[keep] - mid, Se[keep], 1)
    along = (nu + b * u) / math.hypot(1.0, b)
    u_k = np.array([along[1], -along[0]])
    if u_k @ u < 0.0:
        u_k = -u_k
    # the fitted line runs through the middle of the kerb's hit cells; its face is ~half a cell nearer
    slot.center = u * (a - 0.05) + nu * mid - u_k * (EGO.half_width + 0.30)
    slot.u_in, slot.along = u_k, along


def _classify(slot, grid, s0, s1, l0, l1, deep=False):
    """Free / occupied / unknown from the grid cells inside the stall, plus neighbours.
    deep: the far end of the stall may be out of sight (a sensor rig that looks along the lane)."""
    c = slot.corners
    win = grid.window(c[:, 0].min() - 3.0, c[:, 1].min() - 3.0, c[:, 0].max() + 3.0, c[:, 1].max() + 3.0)
    if win is None:
        return
    X, Y, sl = win
    u, nu = slot.u_in, np.array([-slot.u_in[1], slot.u_in[0]])
    S = X * u[0] + Y * u[1]
    Lc = X * nu[0] + Y * nu[1]
    occ = grid.occupied()[sl]
    # (ground seen from too far to rule out a kerb also counts here: a parked car stands tall
    # enough to be recognised at that range, and it is a parked car this test is about)
    free = ((grid.free[sl] >= 2) | ((grid.far[sl] >= 3) & (grid.stop[sl] == 0))) & ~occ
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
    if n_occ >= 4:
        slot.status = Slot.OCCUPIED
    elif n_occ <= 1 and (frac_free >= 0.6 or (frac_mouth >= 0.8 and frac_free >= 0.3) or wedge):
        slot.status = Slot.FREE
    side = (S > s0 - 0.3) & (S < s1 - back)
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
