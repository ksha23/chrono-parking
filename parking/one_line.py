"""Stalls of which only one line was found.

A line can be worn away altogether, or lie under the side of the car parked next to it. The
row the stall stands in (rows.py) says where the missing line has to be."""

import numpy as np

from .slot import KERB_GAP, TICK, Slot, _align_with_kerb, _classify, _kerb_behind, _seen_along
from .vehicle import EGO


def one_line_stalls(tracks, rows, slots, trail, grid, lane, along, tick):
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
